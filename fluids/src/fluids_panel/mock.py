"""A simulated fluids panel behind the Backend interface.

The model is deliberately coarse -- first-order lags and linear ramps -- but follows the
panel's operations sheet closely enough to drive every visual state on the canvas:

- Supply: pump_1 enables the compressor, which fills the accumulator between its own
  pressure switch's cut-in and cut-out. pr_1 regulates the header to ~85 psi, but only
  while v_1 (normally open) is held closed; otherwise the header vents. rv_1 caps it at 100.
- Leg 1: fm_1 follows the flow commanded on v_2, scaled down when the header is starved.
- Leg 2: v_3 drives the actuator between sw_1 (retracted) and sw_2 (extended).
- Leg 3: with v_5 (normally open) open, p_2 follows pr_2's setpoint; once v_5 closes, the
  vessel decays through the orifice.

The simulation advances on every read or write by however much the injected clock moved,
so tests can drive it with a fake clock and stay deterministic.
"""

import math
import random
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field

from fluids_panel.backend import SAFE_OUTPUTS, Backend, RawValue, check_outputs
from fluids_panel.signals import (
    LOOP_MIN_MA,
    LOOP_SATURATION_HIGH_MA,
    LOOP_SATURATION_LOW_MA,
    LOOP_SPAN_MA,
    SIGNALS_BY_REF,
    IoKind,
    analog_full_scale,
    ma_to_eng,
)


@dataclass(frozen=True)
class MockParams:
    """Tunable constants of the simulated plant."""

    compressor_fill_psi_s: float = 4.0
    compressor_cut_in_psi: float = 95.0
    compressor_cut_out_psi: float = 125.0
    vent_drain_psi_s: float = 6.0  # accumulator loss while v_1 is open
    flow_drain_psi_s_per_lpm: float = 0.1  # accumulator loss per L/min through leg 1
    pr_1_setpoint_psi: float = 85.0
    rv_1_relief_psi: float = 100.0
    header_tau_s: float = 1.0
    flow_tau_s: float = 0.5
    actuator_min_psi: float = 20.0
    actuator_travel_s: float = 1.5
    vessel_fill_tau_s: float = 2.0
    vessel_leak_tau_s: float = 600.0
    noise_fraction: float = 0.002  # 1-sigma analog noise as a fraction of full scale
    max_step_s: float = 0.05


@dataclass
class MockFaults:
    """Faults that can be switched on and off while the mock runs."""

    estop: bool = False  # cuts output power and vents, as the hardware E-stop does
    compressor_failed: bool = (
        False  # pump_1 relay energizes but the compressor never runs
    )
    regulator_failed: bool = False  # pr_1 passes accumulator pressure, so rv_1 relieves
    actuator_stuck: bool = False  # the actuator stops wherever it is
    vessel_leak_tau_s: float | None = (
        None  # overrides the vessel's leak-down time constant
    )
    broken_wires: set[str] = field(
        default_factory=set
    )  # inputs that read as an open loop


@dataclass
class PlantState:
    """The simulated plant's physical state."""

    accumulator_psi: float = 0.0
    compressor_running: bool = False
    p_1_psi: float = 0.0
    fm_1_lpm: float = 0.0
    actuator_position: float = 0.0  # 0 is retracted, 1 is extended
    p_2_psi: float = 0.0


def _approach(value: float, target: float, tau_s: float, dt: float) -> float:
    """Advance a first-order lag exactly by dt."""
    return target + (value - target) * math.exp(-dt / tau_s)


class MockBackend(Backend):
    """A simulated panel. Inspect `plant` for ground truth; toggle `faults` to inject failures."""

    def __init__(
        self,
        params: MockParams | None = None,
        clock: Callable[[], float] = time.monotonic,
        seed: int | None = None,
        noise: bool = False,
    ) -> None:
        self.params = params or MockParams()
        # Whether analog inputs carry `params.noise_fraction` of noise. Switchable while running.
        self.noise = noise
        self.faults = MockFaults()
        self.plant = PlantState()
        self._clock = clock
        self._rng = random.Random(seed)
        self._outputs: dict[str, RawValue] = dict(SAFE_OUTPUTS)
        self._last_time: float | None = None

    def open(self) -> None:
        self._outputs = dict(SAFE_OUTPUTS)
        self._last_time = self._clock()

    def close(self) -> None:
        if self._last_time is None:
            return
        self._advance()
        self._outputs = dict(SAFE_OUTPUTS)
        self._last_time = None

    def read(self) -> dict[str, RawValue]:
        self._advance()
        position = self.plant.actuator_position
        inputs: dict[str, RawValue] = {
            "sw_1": position <= 0.0,
            "sw_2": position >= 1.0,
            "p_1": self._analog_input("p_1", self.plant.p_1_psi),
            "p_2": self._analog_input("p_2", self.plant.p_2_psi),
            "fm_1": self._analog_input("fm_1", self.plant.fm_1_lpm),
        }
        for ref in self.faults.broken_wires:
            if ref not in inputs:
                raise ValueError(f"{ref!r} is not an input signal")
            inputs[ref] = 0.0 if SIGNALS_BY_REF[ref].kind == IoKind.AI else False
        return inputs

    def write(self, outputs: Mapping[str, RawValue]) -> None:
        check_outputs(outputs)
        # Settle the plant under the old outputs before the new ones take effect.
        self._advance()
        self._outputs.update(outputs)

    @property
    def outputs(self) -> dict[str, RawValue]:
        """The outputs as last commanded, regardless of whether an E-stop is cutting them."""
        return dict(self._outputs)

    def _analog_input(self, ref: str, value: float) -> float:
        """What a healthy transmitter would put on the loop for `value`, noise included.

        Unlike `eng_to_ma`, which clamps a command to 4-20 mA, this saturates at the NE 43
        measuring range, so noise around zero reads both sides of it instead of only above.
        """
        signal = SIGNALS_BY_REF[ref]
        full_scale = analog_full_scale(signal)
        if self.noise and self.params.noise_fraction:
            value += self._rng.gauss(0.0, self.params.noise_fraction * full_scale)
        ma = LOOP_MIN_MA + value / full_scale * LOOP_SPAN_MA
        return min(max(ma, LOOP_SATURATION_LOW_MA), LOOP_SATURATION_HIGH_MA)

    def _advance(self) -> None:
        if self._last_time is None:
            raise RuntimeError("MockBackend is not open")
        now = self._clock()
        elapsed = now - self._last_time
        self._last_time = now
        steps = math.ceil(elapsed / self.params.max_step_s)
        for _ in range(steps):
            self._step(elapsed / steps)

    def _step(self, dt: float) -> None:
        params, faults, plant = self.params, self.faults, self.plant
        outputs = SAFE_OUTPUTS if faults.estop else self._outputs

        v_1_closed = bool(outputs["v_1"])  # normally open: energized is closed
        v_5_closed = bool(outputs["v_5"])

        # Supply: the compressor's own pressure switch cycles it between cut-in and cut-out.
        if not outputs["pump_1"] or faults.compressor_failed:
            plant.compressor_running = False
        elif plant.accumulator_psi <= params.compressor_cut_in_psi:
            plant.compressor_running = True
        elif plant.accumulator_psi >= params.compressor_cut_out_psi:
            plant.compressor_running = False

        fill = params.compressor_fill_psi_s if plant.compressor_running else 0.0
        drain = params.flow_drain_psi_s_per_lpm * plant.fm_1_lpm
        if not v_1_closed:
            drain += params.vent_drain_psi_s
        plant.accumulator_psi = min(
            max(plant.accumulator_psi + (fill - drain) * dt, 0.0),
            params.compressor_cut_out_psi,
        )

        if not v_1_closed:
            header_target = 0.0
        elif faults.regulator_failed:
            header_target = plant.accumulator_psi
        else:
            header_target = min(plant.accumulator_psi, params.pr_1_setpoint_psi)
        header_target = min(header_target, params.rv_1_relief_psi)
        plant.p_1_psi = _approach(plant.p_1_psi, header_target, params.header_tau_s, dt)

        # Leg 1: v_2's full scale is in L/min, so its command is directly the target flow.
        supply = min(max(plant.p_1_psi / params.pr_1_setpoint_psi, 0.0), 1.0)
        commanded_lpm = max(
            ma_to_eng(SIGNALS_BY_REF["v_2"], float(outputs["v_2"])), 0.0
        )
        plant.fm_1_lpm = _approach(
            plant.fm_1_lpm, commanded_lpm * supply, params.flow_tau_s, dt
        )

        # Leg 2: the actuator only moves with enough header pressure behind it.
        if not faults.actuator_stuck and plant.p_1_psi >= params.actuator_min_psi:
            direction = 1.0 if outputs["v_3"] else -1.0
            plant.actuator_position = min(
                max(
                    plant.actuator_position + direction * dt / params.actuator_travel_s,
                    0.0,
                ),
                1.0,
            )

        # Leg 3: pr_2 can't deliver more than the header supplies it.
        if v_5_closed:
            leak_tau_s = faults.vessel_leak_tau_s or params.vessel_leak_tau_s
            plant.p_2_psi = _approach(plant.p_2_psi, 0.0, leak_tau_s, dt)
        else:
            setpoint = max(
                ma_to_eng(SIGNALS_BY_REF["pr_2"], float(outputs["pr_2"])), 0.0
            )
            plant.p_2_psi = _approach(
                plant.p_2_psi,
                min(setpoint, plant.p_1_psi),
                params.vessel_fill_tau_s,
                dt,
            )
