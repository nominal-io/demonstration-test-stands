"""Concrete Stand implementation. Instro objects are injected by main.py."""

import time
from typing import Callable, TYPE_CHECKING

if TYPE_CHECKING:
    # Deferred: never imported at runtime, so this module needs no instro
    # install to be imported or unit-tested with hand-rolled doubles.
    from instro.psu import InstroPSU
    from instro.unstable.motorcontroller import InstroMotorController


class HardwareStand:
    """Constructor-injected -- never constructs or imports instro at runtime.
    Absorbs two Instro quirks: stop() only halts telemetry (stop_motor() is
    the real motor-stop), and get_telemetry() returns a prefixed Measurement
    wrapper, not a plain dict."""

    def __init__(
        self,
        *,
        mdc: "InstroMotorController",
        dmc_l: "InstroMotorController",
        dmc_r: "InstroMotorController",
        psu: "InstroPSU",
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._controllers = {"mdc": mdc, "dmc_l": dmc_l, "dmc_r": dmc_r}
        self._psu = psu
        self._clock = clock
        self._psu_enabled = False

    @property
    def now(self) -> float:
        return self._clock()

    def open(self) -> None:
        # Order among controllers doesn't matter -- see README.md "Instro API
        # quirks" (shared CAN transport). We never construct a second CanDriver.
        for controller in self._controllers.values():
            controller.open()
        self._psu.open()

    def close(self) -> None:
        self._psu.close()
        for controller in self._controllers.values():
            controller.close()

    def get_telemetry(self) -> dict[str, dict[str, float]]:
        # {"mdc": {...}, "dmc_l": {...}, "dmc_r": {...}}. Unwraps Instro's
        # Measurement wrapper -- see README.md "Instro API quirks".
        result: dict[str, dict[str, float]] = {}
        for name, controller in self._controllers.items():
            measurement = controller.get_telemetry()
            if measurement is None:
                result[name] = {}
            elif hasattr(measurement, "channel_data"):
                prefix = f"{name}."
                result[name] = {
                    key[len(prefix):]: values[-1]
                    for key, values in measurement.channel_data.items()
                    if key.startswith(prefix)
                }
            else:
                # Hand-rolled test doubles return an already-unprefixed dict.
                result[name] = measurement
        return result

    def set_speed_erpm(self, erpm: float) -> None:
        self._controllers["mdc"].set_velocity(erpm)

    def set_brake_current_a(self, amps: float) -> None:
        # Ganged: both absorbers receive the same commanded value.
        self._controllers["dmc_l"].set_brake_current(amps)
        self._controllers["dmc_r"].set_brake_current(amps)

    def zero_all(self) -> None:
        # Immediate trip-path zeroing, bypassing session.py's ramps. Absorbers
        # before drive -- see README.md "STOP sequencing: brake before speed".
        for name in ("dmc_l", "dmc_r"):
            self._controllers[name].set_brake_current(0.0)
        self._controllers["mdc"].stop_motor()

    def zero_bus_voltage(self) -> None:
        # T-6's disable path -- 0V setpoint, output left enabled. See
        # README.md "T-6" for why this replaces output_enable(False) here.
        self._psu.set_voltage(0.0, channel=1)

    def set_psu_output_enabled(self, enabled: bool) -> None:
        # No set_output_enabled on the real InstroPSU -- see README.md
        # "Instro API quirks". hasattr also supports hand-rolled test doubles.
        if hasattr(self._psu, "output_enable"):
            self._psu.output_enable(enabled, channel=1)
        else:
            self._psu.set_output_enabled(enabled)
        self._psu_enabled = enabled

    def psu_output_enabled(self) -> bool:
        return self._psu_enabled
