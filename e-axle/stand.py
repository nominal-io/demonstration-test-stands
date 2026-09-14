import threading
from time import monotonic, sleep
from types import TracebackType
from enum import Enum, auto
from typing import Any, Callable, Literal

from instro.eload import InstroELoad, LoadMode
from instro.psu import InstroPSU
from instro.unstable.motorcontroller import InstroMotorController

from channels import Controllable, ControllableNumeric, Measurable, Monitorable
from stand_config import DutControllerConfig, EAxleStandConfig, LoadControllerConfig, SinkConfig, SourceConfig


class Motor:
    """One motor controller's channels and control logic: torque, speed, current, active mode, and temperature."""

    name: str
    controller: InstroMotorController
    torque: ControllableNumeric
    speed: ControllableNumeric
    current: ControllableNumeric
    control_mode: Controllable[Literal["torque", "speed", "current"]]
    temperature: Monitorable[float]

    def __init__(self, name: str, controller: InstroMotorController, config: DutControllerConfig | LoadControllerConfig) -> None:
        """Build this motor's channels from its config, hold a reference to its controller, and
        register (but do not start) this motor's background resend daemon function."""
        self.name = name
        self.controller = controller
        self.torque = ControllableNumeric(default=config.torque.default, minimum=config.torque.minimum, maximum=config.torque.maximum)
        self.speed = ControllableNumeric(default=config.speed.default, minimum=config.speed.minimum, maximum=config.speed.maximum)
        self.current = ControllableNumeric(default=config.current.default, minimum=config.current.minimum, maximum=config.current.maximum)
        self.control_mode = Controllable(default="torque")
        self.temperature = Monitorable(minimum=config.temperature.minimum, maximum=config.temperature.maximum)
        self.command_enabled: Callable[[], bool] | None = None
        self._effective_kt = self.torque.maximum / self.current.maximum
        controller.add_background_daemon_function(self.command)
        controller.add_background_daemon_function(self.refresh)

    def open(self) -> None:
        """Open the controller's connection and start its background daemon."""
        self.controller.open()
        self.controller.start()

    def close(self) -> None:
        """Stop the background daemon and close the controller's connection."""
        self.controller.stop()
        self.controller.close()

    def refresh(self) -> None:
        """Pull fresh telemetry from the controller and update this motor's channels."""
        measurement = self.controller.get_telemetry()
        if measurement is None:
            return
        prefix = self.controller.name
        if (key := f"{prefix}.motor_current") in measurement.channel_data:
            current = float(measurement.channel_data[key][-1])
            self.current.measured = current
            self.torque.measured = current * self._effective_kt
        if (key := f"{prefix}.velocity") in measurement.channel_data:
            self.speed.measured = float(measurement.channel_data[key][-1])
        if (key := f"{prefix}.motor_temperature") in measurement.channel_data:
            self.temperature.measured = float(measurement.channel_data[key][-1])

    def command(self) -> None:
        """Send the active control mode's setpoint to the controller, if command_enabled allows it."""
        if self.command_enabled is not None and not self.command_enabled():
            return
        mode = self.control_mode.setpoint
        if mode == "torque":
            self.controller.set_current(self.torque.setpoint / self._effective_kt)
        elif mode == "speed":
            self.controller.set_velocity(self.speed.setpoint)
        elif mode == "current":
            self.controller.set_current(self.current.setpoint)

    @property
    def active_channel(self) -> ControllableNumeric:
        """The channel currently being commanded, per this motor's control mode."""
        return {"torque": self.torque, "speed": self.speed, "current": self.current}[self.control_mode.setpoint]

    def tripped_channels(self) -> list[tuple[str, Monitorable[Any]]]:
        """Every channel on this motor currently outside its safe range, named with this motor's prefix."""
        return [
            (f"{self.name}.{attr}", channel)
            for attr, channel in vars(self).items()
            if isinstance(channel, Monitorable) and channel.tripped
        ]


class Source:
    """The bidirectional supply's source quadrant: voltage, current, enable, and protection limits."""

    name: str
    driver: InstroPSU
    voltage: ControllableNumeric
    current: ControllableNumeric
    enabled: Controllable[bool]
    ovp_limit: Controllable[float]
    ocp_limit: Controllable[float]

    def __init__(self, name: str, driver: InstroPSU, config: SourceConfig) -> None:
        """Build this source's channels from its config and hold a reference to its driver."""
        self.name = name
        self.driver = driver
        self._channel = config.psu_channel_number
        self.voltage = ControllableNumeric(default=config.voltage.default, minimum=config.voltage.minimum, maximum=config.voltage.maximum)
        self.current = ControllableNumeric(default=config.current.default, minimum=config.current.minimum, maximum=config.current.maximum)
        self.enabled = Controllable(default=config.enabled.default)
        # Commandable, not monitorable: this is a configured protection *threshold*
        # readback (did our write land?), not a measured safety quantity -- it must
        # not be wired to on_trip (its stale/default value at ARMING, before
        # command() ever runs, would otherwise fire a spurious trip).
        self.ovp_limit = Controllable(default=config.ovp_limit.default)
        self.ocp_limit = Controllable(default=config.ocp_limit.default)
        driver.add_background_daemon_function(self.refresh)

    def open(self) -> None:
        """Open the supply's connection and start its default telemetry-only background daemon."""
        self.driver.open()
        self.driver.start()

    def close(self) -> None:
        """Stop the background daemon and close the supply's connection."""
        self.driver.stop()
        self.driver.close()

    def refresh(self) -> None:
        """Pull fresh telemetry from the supply and update this source's channels."""
        prefix = f"{self.driver.name}.ch{self._channel}"

        voltage = self.driver.get_voltage(channel=self._channel)
        if voltage is not None and (key := f"{prefix}.voltage") in voltage.channel_data:
            self.voltage.measured = float(voltage.channel_data[key][-1])

        current = self.driver.get_current(channel=self._channel)
        if current is not None and (key := f"{prefix}.current") in current.channel_data:
            self.current.measured = float(current.channel_data[key][-1])

        status = self.driver.get_output_status(channel=self._channel)
        if status is not None and (key := f"{prefix}.enabled") in status.channel_data:
            self.enabled.measured = bool(status.channel_data[key][-1])

        ovp = self.driver.get_overvoltage_protection_level(channel=self._channel)
        if ovp is not None and (key := f"{prefix}.ovp") in ovp.channel_data:
            self.ovp_limit.measured = float(ovp.channel_data[key][-1])

        ocp = self.driver.get_overcurrent_protection_level(channel=self._channel)
        if ocp is not None and (key := f"{prefix}.ocp") in ocp.channel_data:
            self.ocp_limit.measured = float(ocp.channel_data[key][-1])

    def command(self) -> None:
        """Send this source's setpoints to the supply."""
        self.driver.set_voltage(self.voltage.setpoint, channel=self._channel)
        self.driver.set_current_limit(self.current.setpoint, channel=self._channel)
        self.driver.output_enable(self.enabled.setpoint, channel=self._channel)
        self.driver.set_overvoltage_protection_level(self.ovp_limit.setpoint, channel=self._channel)
        self.driver.set_overcurrent_protection_level(self.ocp_limit.setpoint, channel=self._channel)

    def tripped_channels(self) -> list[tuple[str, Monitorable[Any]]]:
        """Every channel on this source currently outside its safe range, named with this source's prefix."""
        return [
            (f"{self.name}.{attr}", channel)
            for attr, channel in vars(self).items()
            if isinstance(channel, Monitorable) and channel.tripped
        ]


class Sink:
    """The bidirectional supply's sink quadrant: voltage (CV setpoint), current (limit), and enable."""

    name: str
    driver: InstroELoad
    voltage: ControllableNumeric
    current: ControllableNumeric
    enabled: Controllable[bool]

    def __init__(self, name: str, driver: InstroELoad, config: SinkConfig) -> None:
        """Build this sink's channels from its config and hold a reference to its driver."""
        self.name = name
        self.driver = driver
        self._channel = config.psu_channel_number
        self.voltage = ControllableNumeric(default=config.voltage.default, minimum=config.voltage.minimum, maximum=config.voltage.maximum)
        self.current = ControllableNumeric(default=config.current.default, minimum=config.current.minimum, maximum=config.current.maximum)
        self.enabled = Controllable(default=config.enabled.default)
        driver.add_background_daemon_function(self.refresh)

    def open(self) -> None:
        """Open the load's connection, fix it in CV mode, and start its default telemetry-only background daemon."""
        self.driver.open()
        self.driver.set_mode(LoadMode.CV, channel=self._channel)
        self.driver.start()

    def close(self) -> None:
        """Stop the background daemon and close the load's connection."""
        self.driver.stop()
        self.driver.close()

    def refresh(self) -> None:
        """Pull fresh telemetry from the load and update this sink's channels."""
        prefix = f"{self.driver.name}.ch{self._channel}"

        voltage = self.driver.get_voltage(channel=self._channel)
        if voltage is not None and (key := f"{prefix}.voltage") in voltage.channel_data:
            self.voltage.measured = float(voltage.channel_data[key][-1])

        current = self.driver.get_current(channel=self._channel)
        if current is not None and (key := f"{prefix}.current") in current.channel_data:
            self.current.measured = float(current.channel_data[key][-1])

    def command(self) -> None:
        """Send this sink's setpoints to the load."""
        self.driver.set_level(self.voltage.setpoint, channel=self._channel, curr_limit=self.current.setpoint)
        self.driver.output_enable(self.enabled.setpoint, channel=self._channel)

    def tripped_channels(self) -> list[tuple[str, Monitorable[Any]]]:
        """Every channel on this sink currently outside its safe range, named with this sink's prefix."""
        return [
            (f"{self.name}.{attr}", channel)
            for attr, channel in vars(self).items()
            if isinstance(channel, Monitorable) and channel.tripped
        ]


class EAxleStandState(Enum):
    """The stand's states, which govern what commands are valid at any given time."""

    OFF = auto()
    ARMED = auto()
    ARMING = auto()
    DISARMING = auto()
    STOPPING = auto()
    RUNNING = auto()
    TRIP_STOPPING = auto()
    TRIPPED = auto()


class EAxleStand:

    state: EAxleStandState
    config: EAxleStandConfig
    dut: Motor
    left_load: Motor
    right_load: Motor
    source: Source
    sink: Sink
    _disarm_timeout_s: float
    _arm_timeout_s: float
    _stop_timeout_s: float
    _trip_stop_timeout_s: float
    _boot_timeout_s: float
    _trip_lock: threading.Lock

    def __init__(
        self,
        config: EAxleStandConfig,
    ) -> None:
        ...

    def open(self) -> None:
        """Connect every instrument and bring the bus to its default voltage."""
        if self.state != EAxleStandState.OFF:
            raise ValueError(f"open() requires OFF, not {self.state}")
        self.state = EAxleStandState.ARMING
        self.source.open()
        self.sink.open()
        self.source.voltage.setpoint = self.source.voltage.default
        self.source.current.setpoint = self.source.current.default
        self.source.enabled.setpoint = True
        self.source.command()
        for instrument in (self.dut, self.left_load, self.right_load):
            instrument.open()
        booted = self._wait_for_measurement(
            self.dut.temperature, self.left_load.temperature, self.right_load.temperature,
            timeout=self._boot_timeout_s,
            refresh=(self.dut.refresh, self.left_load.refresh, self.right_load.refresh),
        )
        if not booted:
            raise TimeoutError("Motor controllers did not report telemetry within the boot timeout.")
        self.state = EAxleStandState.ARMED

    def close(self) -> None:
        """Safe the stand for its current state, then disconnect every instrument."""
        if self.state == EAxleStandState.OFF:
            return
        if self.state != EAxleStandState.ARMED:
            self._trip_stop()
        self.state = EAxleStandState.DISARMING
        self.source.enabled.setpoint = False
        self.source.command()
        self._wait_for_setpoint(self.source.enabled, timeout=self._disarm_timeout_s, refresh=(self.source.refresh,))
        for instrument in (self.dut, self.left_load, self.right_load, self.source, self.sink):
            instrument.close()
        self.state = EAxleStandState.OFF

    def __enter__(self) -> "EAxleStand":
        """Open the stand and return it."""
        self.open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        """Close the stand on the way out of a with-block."""
        self.close()

    def arm(self) -> None:
        """Bring the stand from idle to armed. All setpoints are inactive."""
        if self.state != EAxleStandState.OFF:
            raise ValueError(f"arm() requires OFF, not {self.state}")
        self.open()

    def run(self) -> None:
        """Bring the stand from armed to running. Begin commanding setpoints on active channels."""
        if self.state != EAxleStandState.ARMED:
            raise ValueError(f"run() requires ARMED, not {self.state}")
        self.state = EAxleStandState.RUNNING
        self.dut.command()
        self.left_load.command()
        self.right_load.command()

    def stop(self) -> None:
        """Ramp the stand down from running to armed."""
        if self.state != EAxleStandState.RUNNING:
            raise ValueError(f"stop() requires RUNNING, not {self.state}")
        self.state = EAxleStandState.STOPPING
        for motor in (self.dut, self.left_load, self.right_load):
            motor.torque.setpoint = 0.0
            motor.speed.setpoint = 0.0
            motor.current.setpoint = 0.0
            motor.command()
        ramped_down = self._wait_for_setpoint(
            self.dut.active_channel,
            self.left_load.active_channel,
            self.right_load.active_channel,
            timeout=self._stop_timeout_s,
            refresh=(
                self.dut.refresh, 
                self.left_load.refresh, 
                self.right_load.refresh
            ),
        )
        if not ramped_down:
            self._trip_stop()
            raise TimeoutError("Motors did not reach idle within the timeout.")
        self.state = EAxleStandState.ARMED

    def _trip_stop(self) -> None:
        """Run the emergency shutdown sequence and settle into TRIPPED."""
        with self._trip_lock:
            if self.state in (EAxleStandState.TRIP_STOPPING, EAxleStandState.TRIPPED):
                return
            self.state = EAxleStandState.TRIP_STOPPING
        for motor in (self.dut, self.left_load, self.right_load):
            motor.torque.setpoint = 0.0
            motor.speed.setpoint = 0.0
            motor.current.setpoint = 0.0
            motor.command()
        self.source.enabled.setpoint = False
        self.source.command()
        self.sink.enabled.setpoint = False
        self.sink.command()
        self._wait_for_setpoint(
            self.dut.active_channel,
            self.left_load.active_channel,
            self.right_load.active_channel,
            self.source.enabled,
            timeout=self._trip_stop_timeout_s,
            refresh=(
                self.dut.refresh,
                self.left_load.refresh,
                self.right_load.refresh,
                self.source.refresh,
                self.sink.refresh,
            ),
        )
        self.state = EAxleStandState.TRIPPED

    def _on_trip(self, channel: Monitorable[Any]) -> None:
        """React to channel becoming tripped, appropriately for the current state."""
        if self.state in (EAxleStandState.ARMED, EAxleStandState.ARMING):
            with self._trip_lock:
                if self.state not in (EAxleStandState.TRIP_STOPPING, EAxleStandState.TRIPPED):
                    self.state = EAxleStandState.TRIPPED
        elif self.state in (EAxleStandState.RUNNING, EAxleStandState.STOPPING):
            self._trip_stop()

    def _wire_trip_delegates(self) -> None:
        """Register _on_trip on every Monitorable channel across all five instruments."""
        for instrument in (self.dut, self.left_load, self.right_load, self.source, self.sink):
            for channel in vars(instrument).values():
                if isinstance(channel, Monitorable):
                    channel.on_trip = self._on_trip

    def _wire_command_interlock(self) -> None:
        """Wire every motor's command_enabled to whether the stand is RUNNING, STOPPING, or TRIP_STOPPING."""
        for motor in (self.dut, self.left_load, self.right_load):
            motor.command_enabled = lambda: self.state in (
                EAxleStandState.RUNNING, EAxleStandState.STOPPING, EAxleStandState.TRIP_STOPPING
            )

    def disarm(self) -> None:
        """Bring the stand from armed to idle, output disabled."""
        if self.state != EAxleStandState.ARMED:
            raise ValueError(f"disarm() requires ARMED, not {self.state}")
        self.close()

    def reset(self) -> None:
        """Clear a confirmed trip and re-arm."""
        if self.state != EAxleStandState.TRIPPED:
            raise ValueError(f"reset() requires TRIPPED, not {self.state}")
        for instrument in (self.dut, self.left_load, self.right_load, self.source, self.sink):
            instrument.refresh()
        if self.tripped:
            raise ValueError(f"Cannot reset while still tripped: {self.tripped_channels()}")
        self.state = EAxleStandState.ARMED

    def _wait_for_setpoint(
        self,
        *channels: Controllable[Any],
        timeout: float,
        refresh: tuple[Callable[[], None], ...] = (),
        poll_interval: float = 0.05,
    ) -> bool:
        """Poll `channels` until every one reports `at_setpoint`, or `timeout` elapses."""
        deadline = monotonic() + timeout
        while True:
            for step in refresh:
                step()
            if all(channel.at_setpoint for channel in channels):
                return True
            if monotonic() >= deadline:
                return False
            sleep(poll_interval)

    def _wait_for_measurement(
        self,
        *channels: Measurable[Any],
        timeout: float,
        refresh: tuple[Callable[[], None], ...] = (),
        poll_interval: float = 0.05,
    ) -> bool:
        """Poll channels until every one has a real measured value, or timeout elapses."""
        deadline = monotonic() + timeout
        while True:
            for step in refresh:
                step()
            if all(channel.measured is not None for channel in channels):
                return True
            if monotonic() >= deadline:
                return False
            sleep(poll_interval)

    def tripped_channels(self) -> list[tuple[str, Monitorable[Any]]]:
        """Every channel currently outside its safe range, across every instrument on the stand."""
        return (
            self.dut.tripped_channels()
            + self.left_load.tripped_channels()
            + self.right_load.tripped_channels()
            + self.source.tripped_channels()
            + self.sink.tripped_channels()
        )

    @property
    def tripped(self) -> bool:
        """Whether any channel on the stand is currently outside its safe range."""
        return bool(self.tripped_channels())
