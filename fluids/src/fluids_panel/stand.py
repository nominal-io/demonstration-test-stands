"""The fluids panel as a stand: engineering-unit channels, interlocks, trips, and sequences.

The stand is tick-driven rather than blocking: each `update()` reads the backend, checks trip
conditions, advances the state machine and any running sequence by one step, then writes every
output. Commands only change setpoints; they reach the hardware on the next `update()`. This
keeps a caller's loop (e.g. the Connect script) streaming while the panel moves.

Relay channels (`pump_1`, `v_1`, `v_3`, `v_5`) hold whether the relay is energized, matching
the DAQ. For the normally-open valves `v_1` and `v_5`, energized means closed.

The stand is not thread-safe: call its commands and `update()` from one thread.
"""

import logging
import math
import time
from collections.abc import Callable, Generator
from dataclasses import dataclass
from enum import IntEnum
from functools import partial
from types import TracebackType
from typing import Self

from fluids_panel.backend import Backend, RawValue
from fluids_panel.channels import (
    Controllable,
    ControllableNumeric,
    Measurable,
    Monitorable,
)
from fluids_panel.signals import (
    SIGNALS_BY_REF,
    analog_full_scale,
    eng_to_ma,
    is_loop_fault,
    ma_to_eng,
)
from fluids_panel.states import CompressorState, FlowState, LightState, ValveState

logger = logging.getLogger(__name__)

ANALOG_INPUTS = ("p_1", "p_2", "fm_1")


class InterlockError(RuntimeError):
    """A command was refused because the stand's state does not allow it."""


class FluidsStandState(IntEnum):
    """The supply's state, which gates which commands are allowed. Streamed as its value."""

    VENTED = 0
    PRESSURIZING = 1
    READY = 2
    VENTING = 3
    TRIPPED = 4


@dataclass(frozen=True)
class FluidsStandConfig:
    """Thresholds and timeouts for the stand's state machine and trips."""

    ready_psi: float = 75.0  # p_1 at which PRESSURIZING becomes READY
    supply_lost_psi: float = 50.0  # p_1 below which READY trips (E-stop, burst line)
    vented_psi: float = 2.0  # p_1 at which VENTING becomes VENTED
    pressurize_timeout_s: float = 90.0
    actuator_timeout_s: float = 5.0
    p_1_max_psi: float = 95.0  # above pr_1's ~85 psi, below rv_1's 100 psi
    p_2_max_psi: float = 95.0
    # fm_1 below which there is no flow: a low-flow cutoff well clear of the 0-100 L/min
    # meter's noise, but under 7% of v_2's 15.6 L/min range so any real command reads as flow.
    flowing_lpm: float = 1.0


@dataclass(frozen=True)
class WaitUntil:
    """Yielded by a sequence to pause until `condition` holds. Timing out trips the stand."""

    description: str
    condition: Callable[[], bool]
    timeout_s: float | None = None


type Sequence = Generator[WaitUntil]


class FluidsStand:
    """The fluids panel behind a Backend, in engineering units."""

    def __init__(
        self,
        backend: Backend,
        config: FluidsStandConfig | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.backend = backend
        self.config = config or FluidsStandConfig()
        self._clock = clock

        self.pump_1 = Controllable(default=False)
        self.v_1 = Controllable(default=False)
        self.v_3 = Controllable(default=False)
        self.v_5 = Controllable(default=False)
        self.v_2 = ControllableNumeric(
            default=0.0,
            minimum=0.0,
            maximum=analog_full_scale(SIGNALS_BY_REF["v_2"]),
        )
        self.pr_2 = ControllableNumeric(
            default=0.0,
            minimum=0.0,
            maximum=analog_full_scale(SIGNALS_BY_REF["pr_2"]),
        )

        self.p_1 = Monitorable(minimum=-math.inf, maximum=self.config.p_1_max_psi)
        self.p_2 = Monitorable(minimum=-math.inf, maximum=self.config.p_2_max_psi)
        self.p_1.on_trip = partial(self._on_limit, "p_1")
        self.p_2.on_trip = partial(self._on_limit, "p_2")
        self.fm_1: Measurable[float] = Measurable()
        self.sw_1: Measurable[bool] = Measurable()
        self.sw_2: Measurable[bool] = Measurable()

        self.loop_faults: set[str] = set()
        self.trips: list[str] = []
        self.leak_rate_psi_min: float | None = None
        self.sequence_name: str | None = None

        self._state = FluidsStandState.VENTED
        self._state_since = 0.0
        self._sequence: Sequence | None = None
        self._wait: WaitUntil | None = None
        self._wait_started = 0.0
        self._actuator_off_target_since: float | None = None
        self._compressor_fault = False
        self._actuator_fault = False

    @property
    def state(self) -> FluidsStandState:
        """The stand's current state."""
        return self._state

    def _set_state(self, value: FluidsStandState) -> None:
        if value != self._state:
            logger.info("state %s -> %s", self._state.name, value.name)
        self._state = value
        self._state_since = self._clock()

    def now(self) -> float:
        """The stand's clock, for sequences that time their own steps."""
        return self._clock()

    @property
    def p_1_psi(self) -> float:
        """Supply pressure, or NaN before the first reading."""
        return self._reading(self.p_1)

    @property
    def p_2_psi(self) -> float:
        """Vessel pressure, or NaN before the first reading."""
        return self._reading(self.p_2)

    @property
    def fm_1_lpm(self) -> float:
        """Leg 1 flow, or NaN before the first reading."""
        return self._reading(self.fm_1)

    def open(self) -> None:
        """Open the backend with every output safe, and take a first reading."""
        self.backend.open()
        self._command_safe()
        self._set_state(FluidsStandState.VENTING)
        self.update()

    def close(self) -> None:
        """Abort any sequence, drive every output safe, and close the backend."""
        self.abort_sequence()
        self._command_safe()
        self.backend.write(self._outputs())
        self.backend.close()

    def __enter__(self) -> Self:
        self.open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    def update(self) -> None:
        """Read inputs, check trips, advance the state machine and sequence, and write outputs."""
        now = self._clock()
        self._ingest(self.backend.read())
        self._check_trips(now)
        self._advance_state()
        if self.trips and self._state != FluidsStandState.TRIPPED:
            self._enter_trip()
        if self._state != FluidsStandState.TRIPPED:
            self._advance_sequence(now)
        self.backend.write(self._outputs())

    # Commands. Energizing commands are interlocked; de-energizing (safe-direction) ones are not.

    def pressurize(self) -> None:
        """Start the compressor and close the main vent (startup steps 1.4-1.5)."""
        self._require(FluidsStandState.VENTED, FluidsStandState.VENTING)
        self.pump_1.setpoint = True
        self.v_1.setpoint = True
        self._set_state(FluidsStandState.PRESSURIZING)

    def vent(self) -> None:
        """Return every leg to default, stop the compressor and open the main vent (shutdown 5.2-5.3)."""
        self._command_safe()
        if self._state not in (FluidsStandState.VENTED, FluidsStandState.TRIPPED):
            self._set_state(FluidsStandState.VENTING)

    def abort(self) -> None:
        """Abort any running sequence and vent."""
        self.abort_sequence()
        self.vent()

    def set_flow(self, lpm: float) -> None:
        """Command v_2 to a flow in L/min, clamped to its range."""
        if lpm > 0:
            self._require(FluidsStandState.READY)
        self.v_2.setpoint = lpm
        if self.v_2.setpoint != self.v_2.requested:
            logger.warning(
                "v_2: requested %s L/min clamped to %s", lpm, self.v_2.setpoint
            )

    def extend(self) -> None:
        """Energize v_3 to extend the actuator."""
        self._require(FluidsStandState.READY)
        self.v_3.setpoint = True

    def retract(self) -> None:
        """De-energize v_3 to retract the actuator."""
        self.v_3.setpoint = False

    def set_vessel_pressure(self, psig: float) -> None:
        """Command pr_2 to a pressure in psig, clamped to its range."""
        if psig > 0:
            self._require(FluidsStandState.READY)
        self.pr_2.setpoint = psig
        if self.pr_2.setpoint != self.pr_2.requested:
            logger.warning(
                "pr_2: requested %s psig clamped to %s", psig, self.pr_2.setpoint
            )

    def isolate_vessel(self, isolated: bool) -> None:
        """Close (energize) or open v_5 between pr_2 and the vessel."""
        if isolated:
            self._require(FluidsStandState.READY)
        self.v_5.setpoint = isolated

    def reset(self) -> None:
        """Clear a trip once its causes are gone. Outputs stay safe; pressurize again to continue."""
        self._require(FluidsStandState.TRIPPED)
        if active := self.active_faults():
            raise InterlockError(f"cannot reset while faulted: {active}")
        self.trips.clear()
        self._compressor_fault = False
        self._actuator_fault = False
        self._set_state(FluidsStandState.VENTING)

    def active_faults(self) -> list[str]:
        """Faults that are present right now, as opposed to the latched `trips`."""
        faults = [f"{ref} loop fault" for ref in sorted(self.loop_faults)]
        if self.sw_1.measured and self.sw_2.measured:
            faults.append("sw_1 and sw_2 both active")
        faults += [
            f"{name} over limit"
            for name in ("p_1", "p_2")
            if getattr(self, name).tripped
        ]
        return faults

    # Sequences.

    @property
    def sequence_running(self) -> bool:
        return self._sequence is not None

    @property
    def waiting_for(self) -> str | None:
        """What the running sequence's current step is waiting on, if one is running."""
        return self._wait.description if self._wait is not None else None

    def start_sequence(self, name: str, sequence: Sequence) -> None:
        """Run `sequence`'s first step now, then one step per update(). One sequence at a time.

        If the first step's commands are refused, the InterlockError propagates here and the
        sequence never starts. Once running, a refused command or timed-out wait trips the stand.
        """
        if self._sequence is not None:
            raise InterlockError(f"sequence {self.sequence_name!r} is already running")
        if self._state == FluidsStandState.TRIPPED:
            raise InterlockError("cannot start a sequence while TRIPPED")
        try:
            wait = next(sequence)
        except StopIteration:
            logger.info("sequence %s: complete", name)
            return
        logger.info("sequence %s: started", name)
        self._sequence = sequence
        self.sequence_name = name
        self._begin_wait(wait)

    def abort_sequence(self) -> None:
        """Stop the running sequence where it is, leaving outputs as they are."""
        if self._sequence is None:
            return
        logger.warning("sequence %s: aborted", self.sequence_name)
        self._end_sequence()

    def wait(self, seconds: float) -> WaitUntil:
        """A WaitUntil that holds a sequence for `seconds`."""
        deadline = self._clock() + seconds
        return WaitUntil(f"{seconds:g} s to pass", lambda: self._clock() >= deadline)

    # Telemetry.

    def telemetry(self) -> dict[str, float]:
        """Every streamed channel, keyed by channel name. All values are numeric for the canvas."""
        values: dict[str, float] = {
            "p_1": self._reading(self.p_1),
            "p_2": self._reading(self.p_2),
            "fm_1": self._reading(self.fm_1),
            "v_2": self.v_2.setpoint,
            "pr_2": self.pr_2.setpoint,
            "pump_1.state": self._compressor_state(),
            "v_1.state": ValveState.CLOSED if self.v_1.setpoint else ValveState.OPEN,
            "v_2.state": self._v_2_state(),
            "v_5.state": ValveState.CLOSED if self.v_5.setpoint else ValveState.OPEN,
            "actuator.state": self._actuator_state(),
            "fm_1.state": self._flow_state(),
            "sw_1.state": self._switch_light(self.sw_1),
            "sw_2.state": self._switch_light(self.sw_2),
            "stand.state": self._state,
            "stand.status": self._status_light(),
        }
        if self.leak_rate_psi_min is not None:
            values["p_2.leak_rate"] = self.leak_rate_psi_min
        return {name: float(value) for name, value in values.items()}

    # Internals.

    def _require(self, *states: FluidsStandState) -> None:
        if self._state not in states:
            allowed = ", ".join(s.name for s in states)
            raise InterlockError(f"requires {allowed}, not {self._state.name}")

    def _command_safe(self) -> None:
        self.pump_1.setpoint = False
        self.v_1.setpoint = False
        self.v_3.setpoint = False
        self.v_5.setpoint = False
        self.v_2.setpoint = 0.0
        self.pr_2.setpoint = 0.0

    def _outputs(self) -> dict[str, RawValue]:
        return {
            "pump_1": self.pump_1.setpoint,
            "v_1": self.v_1.setpoint,
            "v_3": self.v_3.setpoint,
            "v_5": self.v_5.setpoint,
            "v_2": eng_to_ma(SIGNALS_BY_REF["v_2"], self.v_2.setpoint),
            "pr_2": eng_to_ma(SIGNALS_BY_REF["pr_2"], self.pr_2.setpoint),
        }

    @staticmethod
    def _reading(channel: Measurable[float]) -> float:
        return channel.measured if channel.measured is not None else math.nan

    def _ingest(self, inputs: dict[str, RawValue]) -> None:
        self.loop_faults = {
            ref for ref in ANALOG_INPUTS if is_loop_fault(float(inputs[ref]))
        }
        self.p_1.measured = ma_to_eng(SIGNALS_BY_REF["p_1"], float(inputs["p_1"]))
        self.p_2.measured = ma_to_eng(SIGNALS_BY_REF["p_2"], float(inputs["p_2"]))
        self.fm_1.measured = ma_to_eng(SIGNALS_BY_REF["fm_1"], float(inputs["fm_1"]))
        self.sw_1.measured = bool(inputs["sw_1"])
        self.sw_2.measured = bool(inputs["sw_2"])

    def _trip(self, reason: str) -> None:
        if reason not in self.trips:
            logger.error("trip: %s", reason)
            self.trips.append(reason)

    def _on_limit(self, name: str, channel: Monitorable[float]) -> None:
        # Keep the reason free of the reading so it latches once rather than once per tick.
        logger.warning("%s at %.1f psi", name, channel.measured)
        self._trip(f"{name} over the {channel.maximum:g} psi limit")

    def _check_trips(self, now: float) -> None:
        config = self.config
        for ref in sorted(self.loop_faults):
            self._trip(
                f"{ref} loop current out of range (broken wire or failed sensor)"
            )
        if self.sw_1.measured and self.sw_2.measured:
            self._actuator_fault = True
            self._trip("sw_1 and sw_2 both active")

        p_1 = self._reading(self.p_1)
        if (
            self._state == FluidsStandState.PRESSURIZING
            and now - self._state_since > config.pressurize_timeout_s
        ):
            self._compressor_fault = True
            self._trip(
                f"p_1 did not reach {config.ready_psi:g} psi within {config.pressurize_timeout_s:g} s"
            )
        if self._state == FluidsStandState.READY and p_1 < config.supply_lost_psi:
            self._trip(f"supply lost: p_1 fell to {p_1:.1f} psi (E-stop pressed?)")

        # The actuator should reach whichever switch v_3 is driving it towards.
        if self._state == FluidsStandState.READY:
            at_target = self.sw_2.measured if self.v_3.setpoint else self.sw_1.measured
            if at_target:
                self._actuator_off_target_since = None
            elif self._actuator_off_target_since is None:
                self._actuator_off_target_since = now
            elif now - self._actuator_off_target_since > config.actuator_timeout_s:
                target = "sw_2 (extended)" if self.v_3.setpoint else "sw_1 (retracted)"
                self._actuator_fault = True
                self._trip(
                    f"actuator did not reach {target} within {config.actuator_timeout_s:g} s"
                )
        else:
            self._actuator_off_target_since = None

    def _advance_state(self) -> None:
        p_1 = self._reading(self.p_1)
        if (
            self._state == FluidsStandState.PRESSURIZING
            and p_1 >= self.config.ready_psi
        ):
            self._set_state(FluidsStandState.READY)
        elif self._state == FluidsStandState.VENTING and p_1 <= self.config.vented_psi:
            self._set_state(FluidsStandState.VENTED)

    def _enter_trip(self) -> None:
        self.abort_sequence()
        self._command_safe()
        self._set_state(FluidsStandState.TRIPPED)

    def _end_sequence(self) -> None:
        if self._sequence is not None:
            self._sequence.close()
        self._sequence = None
        self.sequence_name = None
        self._wait = None

    def _advance_sequence(self, now: float) -> None:
        if self._sequence is None:
            return
        name = self.sequence_name
        if self._wait is not None:
            if self._wait.condition():
                self._wait = None
            elif (
                self._wait.timeout_s is not None
                and now - self._wait_started > self._wait.timeout_s
            ):
                self._trip(
                    f"sequence {name}: timed out waiting for {self._wait.description}"
                )
                self._enter_trip()
                return
            else:
                return
        try:
            wait = next(self._sequence)
        except StopIteration:
            logger.info("sequence %s: complete", name)
            self._end_sequence()
            return
        except InterlockError as error:
            self._trip(f"sequence {name}: {error}")
            self._enter_trip()
            return
        self._begin_wait(wait)

    def _begin_wait(self, wait: WaitUntil) -> None:
        self._wait = wait
        self._wait_started = self._clock()
        logger.info("sequence %s: waiting for %s", self.sequence_name, wait.description)

    def _compressor_state(self) -> CompressorState:
        if self._compressor_fault:
            return CompressorState.FAULT
        return CompressorState.RUNNING if self.pump_1.setpoint else CompressorState.OFF

    def _v_2_state(self) -> ValveState:
        if self.v_2.setpoint <= 0:
            return ValveState.CLOSED
        if self.v_2.setpoint >= self.v_2.maximum:
            return ValveState.OPEN
        return ValveState.PARTIAL

    def _actuator_state(self) -> ValveState:
        if self._actuator_fault or (self.sw_1.measured and self.sw_2.measured):
            return ValveState.FAULT
        if self.sw_2.measured:
            return ValveState.OPEN
        if self.sw_1.measured:
            return ValveState.CLOSED
        return ValveState.TRANSITIONING

    def _flow_state(self) -> FlowState:
        if "fm_1" in self.loop_faults:
            return FlowState.FAULT
        if self._reading(self.fm_1) >= self.config.flowing_lpm:
            return FlowState.FLOWING
        return FlowState.NO_FLOW

    def _switch_light(self, switch: Measurable[bool]) -> LightState:
        if self.sw_1.measured and self.sw_2.measured:
            return LightState.FAULT
        return LightState.OK if switch.measured else LightState.OFF

    def _status_light(self) -> LightState:
        return {
            FluidsStandState.VENTED: LightState.OFF,
            FluidsStandState.PRESSURIZING: LightState.WARNING,
            FluidsStandState.READY: LightState.OK,
            FluidsStandState.VENTING: LightState.WARNING,
            FluidsStandState.TRIPPED: LightState.FAULT,
        }[self._state]
