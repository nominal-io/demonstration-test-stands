import threading
from functools import partial
from typing import Any, cast

import pytest
from instro.eload import InstroELoad, LoadMode
from instro.lib import Measurement
from instro.psu import InstroPSU
from instro.unstable.motorcontroller import InstroMotorController

from e_axle.channels import Controllable, Monitorable
from e_axle.stand import EAxleStand, EAxleStandState, Motor, Sink, Source
from e_axle.stand_config import (
    ControllableConfig,
    ControllableNumericConfig,
    DutControllerConfig,
    EAxleStandConfig,
    LoadControllerConfig,
    MonitorableConfig,
    SinkConfig,
    SourceConfig,
)


def _bare_stand() -> EAxleStand:
    return EAxleStand.__new__(EAxleStand)


class _FakeSource:
    def __init__(self, confirms: bool) -> None:
        self.enabled = Controllable(default=True)
        self.enabled.measured = True
        self._confirms = confirms
        self.commanded = False

    def command(self) -> None:
        self.commanded = True

    def refresh(self) -> None:
        if self._confirms:
            self.enabled.measured = self.enabled.setpoint


class _FakeController:
    def __init__(self, name: str = "dut") -> None:
        self.name = name
        self.daemon_functions = []
        self.opened = False
        self.started = False
        self.stopped = False
        self.closed = False
        self.set_current_calls = []
        self.set_velocity_calls = []
        self.telemetry: Measurement | None = None

    def add_background_daemon_function(self, func) -> None:
        self.daemon_functions.append(func)

    def open(self) -> None:
        self.opened = True

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def close(self) -> None:
        self.closed = True

    def set_current(self, amps: float) -> None:
        self.set_current_calls.append(amps)

    def set_velocity(self, rpm: float) -> None:
        self.set_velocity_calls.append(rpm)

    def get_telemetry(self) -> Measurement | None:
        return self.telemetry


def _dut_config() -> DutControllerConfig:
    return DutControllerConfig(
        torque=ControllableNumericConfig(default=0.0, minimum=-27.5, maximum=27.5),
        speed=ControllableNumericConfig(default=0.0, minimum=-3000.0, maximum=3000.0),
        current=ControllableNumericConfig(default=0.0, minimum=-35.0, maximum=35.0),
        temperature=MonitorableConfig(minimum=0.0, maximum=100.0),
    )


def _make_motor(controller: _FakeController) -> Motor:
    return Motor(
        name="dut",
        controller=cast(InstroMotorController, controller),
        config=_dut_config(),
    )


def test_motor_init_builds_channels_from_config():
    controller = _FakeController()
    motor = _make_motor(controller)
    assert motor.name == "dut"
    assert motor.controller is cast(InstroMotorController, controller)
    assert motor.torque.maximum == 27.5
    assert motor.speed.minimum == -3000.0
    assert motor.current.maximum == 35.0
    assert motor.temperature.minimum == 0.0
    assert motor.temperature.maximum == 100.0
    assert motor.control_mode.setpoint == "torque"


def test_motor_init_registers_command_on_the_daemon():
    controller = _FakeController()
    motor = _make_motor(controller)
    assert motor.command in controller.daemon_functions


def test_motor_open_opens_and_starts_the_controller():
    controller = _FakeController()
    motor = _make_motor(controller)
    motor.open()
    assert controller.opened is True
    assert controller.started is True


def test_motor_close_stops_and_closes_the_controller():
    controller = _FakeController()
    motor = _make_motor(controller)
    motor.close()
    assert controller.stopped is True
    assert controller.closed is True


def test_motor_command_torque_mode_converts_to_current():
    controller = _FakeController()
    motor = _make_motor(controller)
    motor.torque.setpoint = 10.0
    motor.control_mode.setpoint = "torque"
    motor.command()
    assert controller.set_current_calls == [pytest.approx(10.0 / motor._effective_kt)]


def test_motor_command_speed_mode_sends_velocity():
    controller = _FakeController()
    motor = _make_motor(controller)
    motor.speed.setpoint = 500.0
    motor.control_mode.setpoint = "speed"
    motor.command()
    assert controller.set_velocity_calls == [500.0]


def test_motor_command_current_mode_sends_current_directly():
    controller = _FakeController()
    motor = _make_motor(controller)
    motor.current.setpoint = 15.0
    motor.control_mode.setpoint = "current"
    motor.command()
    assert controller.set_current_calls == [15.0]


def test_motor_command_transmits_when_no_interlock_wired():
    controller = _FakeController()
    motor = _make_motor(controller)
    motor.torque.setpoint = 10.0
    motor.command()
    assert controller.set_current_calls == [pytest.approx(10.0 / motor._effective_kt)]


def test_motor_command_transmits_nothing_when_interlock_closed():
    controller = _FakeController()
    motor = _make_motor(controller)
    motor.command_enabled = lambda: False
    motor.torque.setpoint = 10.0
    motor.command()
    assert controller.set_current_calls == []
    assert controller.set_velocity_calls == []


def test_motor_command_transmits_when_interlock_open():
    controller = _FakeController()
    motor = _make_motor(controller)
    motor.command_enabled = lambda: True
    motor.torque.setpoint = 10.0
    motor.command()
    assert controller.set_current_calls == [pytest.approx(10.0 / motor._effective_kt)]


def test_motor_refresh_does_nothing_when_no_telemetry():
    controller = _FakeController()
    motor = _make_motor(controller)
    motor.refresh()
    assert motor.current.measured is None
    assert motor.speed.measured is None
    assert motor.temperature.measured is None


def test_motor_refresh_updates_present_fields():
    controller = _FakeController()
    motor = _make_motor(controller)
    controller.telemetry = Measurement(
        channel_data={
            "dut.motor_current": [7.0],
            "dut.velocity": [1200.0],
            "dut.motor_temperature": [42.0],
        },
        timestamps=[123],
    )
    motor.refresh()
    assert motor.current.measured == 7.0
    assert motor.speed.measured == 1200.0
    assert motor.temperature.measured == 42.0
    assert motor.torque.measured == pytest.approx(7.0 * motor._effective_kt)


def test_motor_refresh_ignores_missing_fields():
    controller = _FakeController()
    motor = _make_motor(controller)
    controller.telemetry = Measurement(
        channel_data={"dut.velocity": [900.0]}, timestamps=[123]
    )
    motor.refresh()
    assert motor.speed.measured == 900.0
    assert motor.current.measured is None
    assert motor.temperature.measured is None


def test_motor_tripped_channels_prefixes_with_motor_name():
    controller = _FakeController()
    motor = _make_motor(controller)
    motor.temperature.measured = 150.0
    assert motor.tripped_channels() == [("dut.temperature", motor.temperature)]


class _FakePSUDriver:
    def __init__(self, name: str = "source") -> None:
        self.name = name
        self.opened = False
        self.started = False
        self.stopped = False
        self.closed = False
        self.set_voltage_calls = []
        self.set_current_limit_calls = []
        self.output_enable_calls = []
        self.set_ovp_calls = []
        self.set_ocp_calls = []
        self.voltage_telemetry: Measurement | None = None
        self.current_telemetry: Measurement | None = None
        self.status_telemetry: Measurement | None = None
        self.ovp_telemetry: Measurement | None = None
        self.ocp_telemetry: Measurement | None = None
        self.daemon_functions = []

    def add_background_daemon_function(self, func) -> None:
        self.daemon_functions.append(func)

    def open(self) -> None:
        self.opened = True

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def close(self) -> None:
        self.closed = True

    def set_voltage(self, voltage: float, channel: int) -> None:
        self.set_voltage_calls.append((voltage, channel))

    def set_current_limit(self, current_limit: float, channel: int) -> None:
        self.set_current_limit_calls.append((current_limit, channel))

    def output_enable(self, enable: bool, channel: int) -> None:
        self.output_enable_calls.append((enable, channel))

    def set_overvoltage_protection_level(self, voltage: float, channel: int) -> None:
        self.set_ovp_calls.append((voltage, channel))

    def set_overcurrent_protection_level(self, current: float, channel: int) -> None:
        self.set_ocp_calls.append((current, channel))

    def get_voltage(self, channel: int) -> Measurement | None:
        return self.voltage_telemetry

    def get_current(self, channel: int) -> Measurement | None:
        return self.current_telemetry

    def get_output_status(self, channel: int) -> Measurement | None:
        return self.status_telemetry

    def get_overvoltage_protection_level(self, channel: int) -> Measurement | None:
        return self.ovp_telemetry

    def get_overcurrent_protection_level(self, channel: int) -> Measurement | None:
        return self.ocp_telemetry


def _source_config() -> SourceConfig:
    return SourceConfig(
        psu_channel_number=1,
        voltage=ControllableNumericConfig(default=48.0, minimum=0.0, maximum=54.0),
        current=ControllableNumericConfig(default=0.0, minimum=0.0, maximum=20.0),
        enabled=ControllableConfig(default=False),
        ovp_limit=ControllableNumericConfig(default=60.0, minimum=54.0, maximum=65.0),
        ocp_limit=ControllableNumericConfig(default=25.0, minimum=20.0, maximum=30.0),
    )


def _make_source(driver: _FakePSUDriver) -> Source:
    return Source(
        name="source", driver=cast(InstroPSU, driver), config=_source_config()
    )


def test_source_init_builds_channels_from_config():
    driver = _FakePSUDriver()
    source = _make_source(driver)
    assert source.name == "source"
    assert source.driver is cast(InstroPSU, driver)
    assert source.voltage.default == 48.0
    assert source.voltage.maximum == 54.0
    assert source.current.maximum == 20.0
    assert source.enabled.setpoint is False
    assert source.ovp_limit.default == 60.0
    assert source.ocp_limit.default == 25.0


def test_source_open_opens_and_starts_the_driver():
    driver = _FakePSUDriver()
    source = _make_source(driver)
    source.open()
    assert driver.opened is True
    assert driver.started is True


def test_source_close_stops_and_closes_the_driver():
    driver = _FakePSUDriver()
    source = _make_source(driver)
    source.close()
    assert driver.stopped is True
    assert driver.closed is True


def test_source_command_sends_every_setpoint():
    driver = _FakePSUDriver()
    source = _make_source(driver)
    source.voltage.setpoint = 48.0
    source.current.setpoint = 10.0
    source.enabled.setpoint = True
    source.ovp_limit.setpoint = 60.0
    source.ocp_limit.setpoint = 25.0
    source.command()
    assert driver.set_voltage_calls == [(48.0, 1)]
    assert driver.set_current_limit_calls == [(10.0, 1)]
    assert driver.output_enable_calls == [(True, 1)]
    assert driver.set_ovp_calls == [(60.0, 1)]
    assert driver.set_ocp_calls == [(25.0, 1)]


def test_source_refresh_does_nothing_when_no_telemetry():
    driver = _FakePSUDriver()
    source = _make_source(driver)
    source.refresh()
    assert source.voltage.measured is None
    assert source.current.measured is None
    assert source.enabled.measured is None
    assert source.ovp_limit.measured is None
    assert source.ocp_limit.measured is None


def test_source_refresh_updates_present_fields():
    driver = _FakePSUDriver()
    source = _make_source(driver)
    driver.voltage_telemetry = Measurement(
        channel_data={"source.ch1.voltage": [48.2]}, timestamps=[1]
    )
    driver.current_telemetry = Measurement(
        channel_data={"source.ch1.current": [9.5]}, timestamps=[1]
    )
    driver.status_telemetry = Measurement(
        channel_data={"source.ch1.enabled": [1.0]}, timestamps=[1]
    )
    driver.ovp_telemetry = Measurement(
        channel_data={"source.ch1.ovp": [60.0]}, timestamps=[1]
    )
    driver.ocp_telemetry = Measurement(
        channel_data={"source.ch1.ocp": [25.0]}, timestamps=[1]
    )
    source.refresh()
    assert source.voltage.measured == 48.2
    assert source.current.measured == 9.5
    assert source.enabled.measured is True
    assert source.ovp_limit.measured == 60.0
    assert source.ocp_limit.measured == 25.0


def test_source_refresh_decodes_disabled_status_as_false():
    driver = _FakePSUDriver()
    source = _make_source(driver)
    driver.status_telemetry = Measurement(
        channel_data={"source.ch1.enabled": [0.0]}, timestamps=[1]
    )
    source.refresh()
    assert source.enabled.measured is False


def test_source_tripped_channels_prefixes_with_source_name():
    driver = _FakePSUDriver()
    source = _make_source(driver)
    source.voltage.measured = 100.0
    assert source.tripped_channels() == [("source.voltage", source.voltage)]


def test_source_ovp_and_ocp_are_commandable_not_monitorable():
    driver = _FakePSUDriver()
    source = _make_source(driver)
    assert not isinstance(source.ovp_limit, Monitorable)
    assert not isinstance(source.ocp_limit, Monitorable)
    source.ovp_limit.measured = 999.0
    source.ocp_limit.measured = 999.0
    assert source.tripped_channels() == []


class _FakeELoadDriver:
    def __init__(self, name: str = "sink") -> None:
        self.name = name
        self.opened = False
        self.started = False
        self.stopped = False
        self.closed = False
        self.set_mode_calls = []
        self.set_level_calls = []
        self.output_enable_calls = []
        self.voltage_telemetry: Measurement | None = None
        self.current_telemetry: Measurement | None = None
        self.daemon_functions = []

    def add_background_daemon_function(self, func) -> None:
        self.daemon_functions.append(func)

    def open(self) -> None:
        self.opened = True

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def close(self) -> None:
        self.closed = True

    def set_mode(self, mode: LoadMode, channel: int) -> None:
        self.set_mode_calls.append((mode, channel))

    def set_level(
        self, value: float, channel: int, curr_limit: float | None = None
    ) -> None:
        self.set_level_calls.append((value, channel, curr_limit))

    def output_enable(self, enable: bool, channel: int) -> None:
        self.output_enable_calls.append((enable, channel))

    def get_voltage(self, channel: int) -> Measurement | None:
        return self.voltage_telemetry

    def get_current(self, channel: int) -> Measurement | None:
        return self.current_telemetry


def _sink_config() -> SinkConfig:
    return SinkConfig(
        psu_channel_number=1,
        voltage=ControllableNumericConfig(default=48.0, minimum=0.0, maximum=54.0),
        current=ControllableNumericConfig(default=0.0, minimum=0.0, maximum=40.0),
        enabled=ControllableConfig(default=False),
    )


def _make_sink(driver: _FakeELoadDriver) -> Sink:
    return Sink(name="sink", driver=cast(InstroELoad, driver), config=_sink_config())


def test_sink_init_builds_channels_from_config():
    driver = _FakeELoadDriver()
    sink = _make_sink(driver)
    assert sink.name == "sink"
    assert sink.driver is cast(InstroELoad, driver)
    assert sink.voltage.default == 48.0
    assert sink.voltage.maximum == 54.0
    assert sink.current.maximum == 40.0
    assert sink.enabled.setpoint is False


def test_sink_open_opens_fixes_mode_and_starts_the_driver():
    driver = _FakeELoadDriver()
    sink = _make_sink(driver)
    sink.open()
    assert driver.opened is True
    assert driver.set_mode_calls == [(LoadMode.CV, 1)]
    assert driver.started is True


def test_sink_close_stops_and_closes_the_driver():
    driver = _FakeELoadDriver()
    sink = _make_sink(driver)
    sink.close()
    assert driver.stopped is True
    assert driver.closed is True


def test_sink_command_sends_cv_level_with_current_limit_and_enable():
    driver = _FakeELoadDriver()
    sink = _make_sink(driver)
    sink.voltage.setpoint = 48.0
    sink.current.setpoint = 12.0
    sink.enabled.setpoint = True
    sink.command()
    assert driver.set_level_calls == [(48.0, 1, 12.0)]
    assert driver.output_enable_calls == [(True, 1)]


def test_sink_refresh_does_nothing_when_no_telemetry():
    driver = _FakeELoadDriver()
    sink = _make_sink(driver)
    sink.refresh()
    assert sink.voltage.measured is None
    assert sink.current.measured is None


def test_sink_refresh_updates_present_fields():
    driver = _FakeELoadDriver()
    sink = _make_sink(driver)
    driver.voltage_telemetry = Measurement(
        channel_data={"sink.ch1.voltage": [47.9]}, timestamps=[1]
    )
    driver.current_telemetry = Measurement(
        channel_data={"sink.ch1.current": [11.0]}, timestamps=[1]
    )
    sink.refresh()
    assert sink.voltage.measured == 47.9
    assert sink.current.measured == 11.0


def test_sink_tripped_channels_prefixes_with_sink_name():
    driver = _FakeELoadDriver()
    sink = _make_sink(driver)
    sink.voltage.measured = 100.0
    assert sink.tripped_channels() == [("sink.voltage", sink.voltage)]


class _FakeInstrument:
    def __init__(self, tripped: list[tuple[str, Monitorable[Any]]]) -> None:
        self._tripped = tripped

    def tripped_channels(self) -> list[tuple[str, Monitorable[Any]]]:
        return self._tripped


def test_wait_for_setpoint_returns_true_when_already_at_setpoint():
    stand = _bare_stand()
    channel = Controllable(default=True)
    channel.measured = True
    assert stand._wait_for_setpoint(channel, timeout=1.0) is True


def test_wait_for_setpoint_returns_false_on_timeout():
    stand = _bare_stand()
    channel = Controllable(default=True)
    channel.measured = False
    assert stand._wait_for_setpoint(channel, timeout=0.05, poll_interval=0.01) is False


def test_wait_for_setpoint_calls_refresh_each_poll():
    stand = _bare_stand()
    channel = Controllable(default=True)
    channel.measured = False
    calls = []

    def refresh() -> None:
        calls.append(None)
        if len(calls) >= 3:
            channel.measured = True

    assert (
        stand._wait_for_setpoint(
            channel, timeout=1.0, refresh=(refresh,), poll_interval=0.01
        )
        is True
    )
    assert len(calls) == 3


def test_wait_for_setpoint_calls_every_refresh_function_each_poll():
    stand = _bare_stand()
    channel = Controllable(default=True)
    channel.measured = True
    calls = []

    assert (
        stand._wait_for_setpoint(
            channel,
            timeout=1.0,
            refresh=(lambda: calls.append("a"), lambda: calls.append("b")),
            poll_interval=0.01,
        )
        is True
    )
    assert calls == ["a", "b"]


def test_wait_for_setpoint_requires_all_channels_at_setpoint():
    stand = _bare_stand()
    ready = Controllable(default=True)
    ready.measured = True
    not_ready = Controllable(default=True)
    not_ready.measured = False
    assert (
        stand._wait_for_setpoint(ready, not_ready, timeout=0.05, poll_interval=0.01)
        is False
    )


def _stand_with_instruments(
    **tripped_by_instrument: list[tuple[str, Monitorable[Any]]],
) -> EAxleStand:
    stand = _bare_stand()
    for attr in ("dut", "left_load", "right_load", "source", "sink"):
        setattr(stand, attr, _FakeInstrument(tripped_by_instrument.get(attr, [])))
    return stand


def test_tripped_is_false_with_no_tripped_instruments():
    stand = _stand_with_instruments()
    assert stand.tripped is False
    assert stand.tripped_channels() == []


def test_tripped_channels_aggregates_across_every_instrument():
    channel = Monitorable(minimum=0.0, maximum=90.0)
    channel.measured = 95.0
    stand = _stand_with_instruments(dut=[("dut.temperature", channel)])
    assert stand.tripped is True
    assert stand.tripped_channels() == [("dut.temperature", channel)]


def test_tripped_channels_reports_every_tripped_instrument():
    dut_channel = Monitorable(minimum=0.0, maximum=90.0)
    dut_channel.measured = 95.0
    sink_channel = Monitorable(minimum=0.0, maximum=54.0)
    sink_channel.measured = 60.0
    stand = _stand_with_instruments(
        dut=[("dut.temperature", dut_channel)],
        sink=[("sink.voltage", sink_channel)],
    )
    assert {name for name, _ in stand.tripped_channels()} == {
        "dut.temperature",
        "sink.voltage",
    }


def test_disarm_requires_armed_state():
    stand = _bare_stand()
    stand.state = EAxleStandState.OFF
    with pytest.raises(ValueError):
        stand.disarm()


def test_disarm_transitions_to_off_and_disconnects():
    stand, dut_ctrl, _, _, psu, eload = _full_stand()
    stand.state = EAxleStandState.ARMED
    stand._disarm_timeout_s = 0.05
    stand.disarm()
    assert stand.state == EAxleStandState.OFF
    assert stand.source.enabled.setpoint is False
    assert psu.output_enable_calls[-1] == (False, 1)
    assert dut_ctrl.closed is True
    assert psu.closed is True
    assert eload.closed is True


def test_run_requires_armed_state():
    stand, *_ = _full_stand()
    stand.state = EAxleStandState.OFF
    with pytest.raises(ValueError):
        stand.run()


def test_run_commands_every_motor_and_transitions_to_running():
    stand, dut_ctrl, left_ctrl, right_ctrl, _, _ = _full_stand()
    stand.state = EAxleStandState.ARMED
    for motor in (stand.dut, stand.left_load, stand.right_load):
        motor.torque.setpoint = 1.0
    stand.run()
    assert stand.state == EAxleStandState.RUNNING
    for controller, motor in (
        (dut_ctrl, stand.dut),
        (left_ctrl, stand.left_load),
        (right_ctrl, stand.right_load),
    ):
        assert controller.set_current_calls == [
            pytest.approx(1.0 / motor._effective_kt)
        ]


def test_command_interlock_blocks_transmission_while_armed():
    stand, dut_ctrl, *_ = _full_stand()
    stand._wire_command_interlock()
    stand.state = EAxleStandState.ARMED
    stand.dut.torque.setpoint = 15.0
    stand.dut.command()
    assert dut_ctrl.set_current_calls == []


def test_command_interlock_permits_transmission_once_running():
    stand, dut_ctrl, *_ = _full_stand()
    stand._wire_command_interlock()
    stand.state = EAxleStandState.RUNNING
    stand.dut.torque.setpoint = 15.0
    stand.dut.command()
    assert dut_ctrl.set_current_calls == [pytest.approx(15.0 / stand.dut._effective_kt)]


def test_command_interlock_permits_transmission_while_stopping_and_trip_stopping():
    stand, dut_ctrl, *_ = _full_stand()
    stand._wire_command_interlock()
    for state in (EAxleStandState.STOPPING, EAxleStandState.TRIP_STOPPING):
        dut_ctrl.set_current_calls.clear()
        stand.state = state
        stand.dut.command()
        assert dut_ctrl.set_current_calls != []


def test_command_interlock_blocks_transmission_once_tripped():
    stand, dut_ctrl, *_ = _full_stand()
    stand._wire_command_interlock()
    stand.state = EAxleStandState.TRIPPED
    stand.dut.torque.setpoint = 15.0
    stand.dut.command()
    assert dut_ctrl.set_current_calls == []


def test_run_with_interlock_wired_actually_transmits():
    stand, dut_ctrl, *_ = _full_stand()
    stand._wire_command_interlock()
    stand.state = EAxleStandState.ARMED
    stand.dut.torque.setpoint = 15.0
    stand.run()
    assert dut_ctrl.set_current_calls == [pytest.approx(15.0 / stand.dut._effective_kt)]


def test_wait_for_measurement_returns_true_when_already_measured():
    stand = _bare_stand()
    channel = Monitorable(minimum=0.0, maximum=10.0)
    channel.measured = 5.0
    assert stand._wait_for_measurement(channel, timeout=1.0) is True


def test_wait_for_measurement_returns_false_on_timeout():
    stand = _bare_stand()
    channel = Monitorable(minimum=0.0, maximum=10.0)
    assert (
        stand._wait_for_measurement(channel, timeout=0.05, poll_interval=0.01) is False
    )


def test_wait_for_measurement_calls_refresh_each_poll():
    stand = _bare_stand()
    channel = Monitorable(minimum=0.0, maximum=10.0)
    calls = []

    def refresh() -> None:
        calls.append(None)
        if len(calls) >= 3:
            channel.measured = 5.0

    assert (
        stand._wait_for_measurement(
            channel, timeout=1.0, refresh=(refresh,), poll_interval=0.01
        )
        is True
    )
    assert len(calls) == 3


def _full_stand() -> tuple[
    EAxleStand,
    _FakeController,
    _FakeController,
    _FakeController,
    _FakePSUDriver,
    _FakeELoadDriver,
]:
    dut_ctrl = _FakeController("dut")
    left_ctrl = _FakeController("left_load")
    right_ctrl = _FakeController("right_load")
    psu = _FakePSUDriver()
    eload = _FakeELoadDriver()
    stand = EAxleStand(
        _stand_config(),
        dut=_make_motor(dut_ctrl),
        left_load=_make_motor(left_ctrl),
        right_load=_make_motor(right_ctrl),
        source=_make_source(psu),
        sink=_make_sink(eload),
    )
    stand._arm_timeout_s = 1.0
    stand._stop_timeout_s = 1.0
    stand._trip_stop_timeout_s = 1.0
    stand._boot_timeout_s = 1.0
    stand._disarm_timeout_s = 1.0
    stand._trip_lock = threading.Lock()
    return stand, dut_ctrl, left_ctrl, right_ctrl, psu, eload


def test_arm_requires_off_state():
    stand, *_ = _full_stand()
    stand.state = EAxleStandState.ARMED
    with pytest.raises(ValueError):
        stand.arm()


def test_arm_transitions_to_armed_when_confirmed():
    stand, dut_ctrl, left_ctrl, right_ctrl, psu, _ = _full_stand()
    stand.state = EAxleStandState.OFF
    psu.status_telemetry = Measurement(
        channel_data={"source.ch1.enabled": [1.0]}, timestamps=[1]
    )
    dut_ctrl.telemetry = Measurement(
        channel_data={"dut.motor_temperature": [25.0]}, timestamps=[1]
    )
    left_ctrl.telemetry = Measurement(
        channel_data={"left_load.motor_temperature": [25.0]}, timestamps=[1]
    )
    right_ctrl.telemetry = Measurement(
        channel_data={"right_load.motor_temperature": [25.0]}, timestamps=[1]
    )
    stand.arm()
    assert stand.state == EAxleStandState.ARMED
    assert stand.source.enabled.setpoint is True


def test_arm_raises_when_motors_never_report_telemetry():
    stand, *_ = _full_stand()
    stand.state = EAxleStandState.OFF
    stand._boot_timeout_s = 0.05
    with pytest.raises(TimeoutError):
        stand.arm()


def test_stop_requires_running_state():
    stand, *_ = _full_stand()
    stand.state = EAxleStandState.OFF
    with pytest.raises(ValueError):
        stand.stop()


def test_stop_transitions_to_armed_when_ramped_down():
    stand, dut_ctrl, left_ctrl, right_ctrl, _, _ = _full_stand()
    stand.state = EAxleStandState.RUNNING
    dut_ctrl.telemetry = Measurement(
        channel_data={"dut.motor_current": [0.0]}, timestamps=[1]
    )
    left_ctrl.telemetry = Measurement(
        channel_data={"left_load.motor_current": [0.0]}, timestamps=[1]
    )
    right_ctrl.telemetry = Measurement(
        channel_data={"right_load.motor_current": [0.0]}, timestamps=[1]
    )
    stand.stop()
    assert stand.state == EAxleStandState.ARMED


def test_stop_trips_when_ramp_never_completes():
    stand, *_ = _full_stand()
    stand.state = EAxleStandState.RUNNING
    stand._stop_timeout_s = 0.05
    stand._trip_stop_timeout_s = 0.05
    with pytest.raises(TimeoutError):
        stand.stop()
    assert stand.state == EAxleStandState.TRIPPED


def test_stop_commands_zero_even_when_motor_default_is_nonzero():
    stand, dut_ctrl, left_ctrl, right_ctrl, _, _ = _full_stand()
    stand.state = EAxleStandState.RUNNING
    nonzero_default_config = DutControllerConfig(
        torque=ControllableNumericConfig(default=5.0, minimum=-27.5, maximum=27.5),
        speed=ControllableNumericConfig(default=500.0, minimum=-3000.0, maximum=3000.0),
        current=ControllableNumericConfig(default=2.0, minimum=-35.0, maximum=35.0),
        temperature=MonitorableConfig(minimum=0.0, maximum=100.0),
    )
    stand.dut = Motor(
        name="dut",
        controller=cast(InstroMotorController, dut_ctrl),
        config=nonzero_default_config,
    )
    dut_ctrl.telemetry = Measurement(
        channel_data={"dut.motor_current": [0.0]}, timestamps=[1]
    )
    left_ctrl.telemetry = Measurement(
        channel_data={"left_load.motor_current": [0.0]}, timestamps=[1]
    )
    right_ctrl.telemetry = Measurement(
        channel_data={"right_load.motor_current": [0.0]}, timestamps=[1]
    )
    stand.stop()
    assert stand.dut.torque.setpoint == 0.0
    assert stand.dut.speed.setpoint == 0.0
    assert stand.dut.current.setpoint == 0.0


def test_trip_stop_zeros_every_motor_and_disables_source_and_sink():
    stand, *_ = _full_stand()
    stand.state = EAxleStandState.RUNNING
    stand._trip_stop_timeout_s = 0.05
    for motor in (stand.dut, stand.left_load, stand.right_load):
        motor.torque.setpoint = 10.0
        motor.speed.setpoint = 500.0
        motor.current.setpoint = 5.0
    stand.source.enabled.setpoint = True
    stand.sink.enabled.setpoint = True
    stand._trip_stop()
    for motor in (stand.dut, stand.left_load, stand.right_load):
        assert motor.torque.setpoint == motor.torque.default
        assert motor.speed.setpoint == motor.speed.default
        assert motor.current.setpoint == motor.current.default
    assert stand.source.enabled.setpoint is False
    assert stand.sink.enabled.setpoint is False


def test_trip_stop_commands_zero_even_when_motor_default_is_nonzero():
    stand, *_ = _full_stand()
    stand.state = EAxleStandState.RUNNING
    stand._trip_stop_timeout_s = 0.05
    nonzero_default_config = DutControllerConfig(
        torque=ControllableNumericConfig(default=5.0, minimum=-27.5, maximum=27.5),
        speed=ControllableNumericConfig(default=500.0, minimum=-3000.0, maximum=3000.0),
        current=ControllableNumericConfig(default=2.0, minimum=-35.0, maximum=35.0),
        temperature=MonitorableConfig(minimum=0.0, maximum=100.0),
    )
    dut_ctrl = _FakeController("dut")
    stand.dut = Motor(
        name="dut",
        controller=cast(InstroMotorController, dut_ctrl),
        config=nonzero_default_config,
    )
    stand._trip_stop()
    assert stand.dut.torque.setpoint == 0.0
    assert stand.dut.speed.setpoint == 0.0
    assert stand.dut.current.setpoint == 0.0


def test_trip_stop_settles_into_tripped_even_without_confirmation():
    stand, *_ = _full_stand()
    stand.state = EAxleStandState.RUNNING
    stand._trip_stop_timeout_s = 0.05
    stand._trip_stop()
    assert stand.state == EAxleStandState.TRIPPED


def test_trip_stop_settles_into_tripped_when_confirmed():
    stand, dut_ctrl, left_ctrl, right_ctrl, psu, _ = _full_stand()
    stand.state = EAxleStandState.RUNNING
    stand._trip_stop_timeout_s = 1.0
    dut_ctrl.telemetry = Measurement(
        channel_data={"dut.motor_current": [0.0]}, timestamps=[1]
    )
    left_ctrl.telemetry = Measurement(
        channel_data={"left_load.motor_current": [0.0]}, timestamps=[1]
    )
    right_ctrl.telemetry = Measurement(
        channel_data={"right_load.motor_current": [0.0]}, timestamps=[1]
    )
    psu.status_telemetry = Measurement(
        channel_data={"source.ch1.enabled": [0.0]}, timestamps=[1]
    )
    stand._trip_stop()
    assert stand.state == EAxleStandState.TRIPPED


def test_reset_requires_tripped_state():
    stand, *_ = _full_stand()
    stand.state = EAxleStandState.OFF
    with pytest.raises(ValueError):
        stand.reset()


def test_reset_transitions_to_armed_when_not_tripped():
    stand, *_ = _full_stand()
    stand.state = EAxleStandState.TRIPPED
    stand.reset()
    assert stand.state == EAxleStandState.ARMED


def test_reset_raises_when_still_tripped():
    stand, dut_ctrl, *_ = _full_stand()
    stand.state = EAxleStandState.TRIPPED
    dut_ctrl.telemetry = Measurement(
        channel_data={"dut.motor_temperature": [150.0]}, timestamps=[1]
    )
    with pytest.raises(ValueError):
        stand.reset()
    assert stand.state == EAxleStandState.TRIPPED


def test_open_commands_source_enabled_and_opens_every_instrument_when_controllers_boot():
    stand, dut_ctrl, left_ctrl, right_ctrl, psu, eload = _full_stand()
    stand.state = EAxleStandState.OFF
    dut_ctrl.telemetry = Measurement(
        channel_data={"dut.motor_temperature": [25.0]}, timestamps=[1]
    )
    left_ctrl.telemetry = Measurement(
        channel_data={"left_load.motor_temperature": [25.0]}, timestamps=[1]
    )
    right_ctrl.telemetry = Measurement(
        channel_data={"right_load.motor_temperature": [25.0]}, timestamps=[1]
    )
    stand.open()
    assert psu.output_enable_calls == [(True, 1)]
    assert stand.source.enabled.setpoint is True
    # The source's current limit must be commanded to its real operating
    # maximum, not its 0.0 idle default -- a 0 A limit would prevent the bus
    # from ever actually reaching voltage, so the motor controllers can't boot.
    assert stand.source.current.setpoint == stand.source.current.maximum
    assert psu.set_current_limit_calls == [(stand.source.current.maximum, 1)]
    # The sink must actually be commanded during open(), not just connected --
    # otherwise it never gets enabled or configured during normal operation.
    assert stand.sink.enabled.setpoint is True
    assert stand.sink.current.setpoint == stand.sink.current.maximum
    assert eload.set_level_calls == [
        (stand.sink.voltage.setpoint, 1, stand.sink.current.setpoint)
    ]
    assert eload.output_enable_calls == [(True, 1)]
    for controller in (dut_ctrl, left_ctrl, right_ctrl):
        assert controller.opened is True
        assert controller.started is True
    assert psu.opened is True
    assert eload.opened is True


def test_open_raises_when_controllers_never_report_telemetry():
    stand, dut_ctrl, left_ctrl, right_ctrl, psu, _ = _full_stand()
    stand.state = EAxleStandState.OFF
    stand._boot_timeout_s = 0.05
    psu.status_telemetry = Measurement(
        channel_data={"source.ch1.enabled": [1.0]}, timestamps=[1]
    )
    with pytest.raises(TimeoutError):
        stand.open()
    for controller in (dut_ctrl, left_ctrl, right_ctrl):
        assert controller.opened is True


def test_close_from_off_is_a_noop():
    stand, dut_ctrl, left_ctrl, right_ctrl, psu, eload = _full_stand()
    stand.state = EAxleStandState.OFF
    stand.close()  # must not raise, and must not touch already-idle instruments
    assert stand.state == EAxleStandState.OFF
    assert dut_ctrl.closed is False
    assert left_ctrl.closed is False
    assert right_ctrl.closed is False
    assert psu.closed is False
    assert eload.closed is False


def test_close_from_running_trip_stops_then_disconnects():
    stand, dut_ctrl, _, _, psu, eload = _full_stand()
    stand.state = EAxleStandState.RUNNING
    stand._trip_stop_timeout_s = 0.05
    stand._disarm_timeout_s = 0.05
    for motor in (stand.dut, stand.left_load, stand.right_load):
        motor.torque.setpoint = 10.0
    stand.source.enabled.setpoint = True
    stand.sink.enabled.setpoint = True
    stand.close()
    for motor in (stand.dut, stand.left_load, stand.right_load):
        assert motor.torque.setpoint == motor.torque.default
    assert stand.source.enabled.setpoint is False
    assert stand.sink.enabled.setpoint is False
    assert stand.state == EAxleStandState.OFF
    assert dut_ctrl.closed is True
    assert psu.closed is True
    assert eload.closed is True


def test_close_does_not_raise_when_called_unexpectedly_mid_run():
    stand, *_ = _full_stand()
    stand.state = EAxleStandState.RUNNING
    stand._trip_stop_timeout_s = 0.05
    stand._disarm_timeout_s = 0.05
    stand.close()  # must not raise
    assert stand.state == EAxleStandState.OFF


def test_close_disconnects_even_when_still_tripped_after_trip_stop():
    stand, dut_ctrl, *_ = _full_stand()
    stand.state = EAxleStandState.RUNNING
    stand._trip_stop_timeout_s = 0.05
    stand._disarm_timeout_s = 0.05
    dut_ctrl.telemetry = Measurement(
        channel_data={"dut.motor_temperature": [150.0]}, timestamps=[1]
    )
    stand.close()  # must not raise
    assert stand.state == EAxleStandState.OFF


def test_close_from_already_tripped_does_not_re_trip_but_still_disconnects():
    stand, dut_ctrl, _, _, psu, eload = _full_stand()
    stand.state = EAxleStandState.TRIPPED
    stand._disarm_timeout_s = 0.05
    stand.close()
    assert stand.state == EAxleStandState.OFF
    assert dut_ctrl.closed is True
    assert psu.closed is True
    assert eload.closed is True


def _load_config() -> LoadControllerConfig:
    return LoadControllerConfig(
        torque=ControllableNumericConfig(default=0.0, minimum=-3.8, maximum=3.8),
        speed=ControllableNumericConfig(default=0.0, minimum=-471.0, maximum=471.0),
        current=ControllableNumericConfig(default=0.0, minimum=-20.0, maximum=20.0),
        temperature=MonitorableConfig(minimum=0.0, maximum=100.0),
    )


def _stand_config() -> EAxleStandConfig:
    return EAxleStandConfig(
        dut_controller=_dut_config(),
        left_load_controller=_load_config(),
        right_load_controller=_load_config(),
        source=_source_config(),
        sink=_sink_config(),
        disarm_timeout_s=2.0,
        arm_timeout_s=5.0,
        stop_timeout_s=10.0,
        trip_stop_timeout_s=1.0,
        boot_timeout_s=5.0,
    )


def _init_stand() -> EAxleStand:
    return EAxleStand.from_drivers(
        _stand_config(),
        cast(InstroMotorController, _FakeController("dut")),
        cast(InstroMotorController, _FakeController("left_load")),
        cast(InstroMotorController, _FakeController("right_load")),
        cast(InstroPSU, _FakePSUDriver("source")),
        cast(InstroELoad, _FakeELoadDriver("sink")),
    )


def test_init_builds_every_instrument_and_starts_off():
    stand = _init_stand()
    assert stand.state == EAxleStandState.OFF
    assert stand.dut.controller.name == "dut"
    assert stand.left_load.controller.name == "left_load"
    assert stand.right_load.controller.name == "right_load"
    assert stand.source.driver.name == "source"
    assert stand.sink.driver.name == "sink"


def test_init_sets_every_timeout_from_config():
    stand = _init_stand()
    assert stand._disarm_timeout_s == 2.0
    assert stand._arm_timeout_s == 5.0
    assert stand._stop_timeout_s == 10.0
    assert stand._trip_stop_timeout_s == 1.0
    assert stand._boot_timeout_s == 5.0
    assert isinstance(stand._trip_lock, type(threading.Lock()))


def test_init_wires_trip_delegates_on_monitorable_channels():
    stand = _init_stand()
    for channel, name in (
        (stand.dut.temperature, "dut.temperature"),
        (stand.source.voltage, "source.voltage"),
        (stand.sink.voltage, "sink.voltage"),
    ):
        on_trip = cast(partial, channel.on_trip)
        assert on_trip.func == stand._on_trip
        assert on_trip.args == (name,)


def test_init_does_not_wire_trip_delegate_on_ovp_ocp():
    stand = _init_stand()
    assert not isinstance(stand.source.ovp_limit, Monitorable)
    assert not isinstance(stand.source.ocp_limit, Monitorable)


def test_init_wires_command_interlock_on_every_motor():
    stand = _init_stand()
    for motor in (stand.dut, stand.left_load, stand.right_load):
        assert motor.command_enabled is not None
        assert motor.command_enabled() is False
        stand.state = EAxleStandState.RUNNING
        assert motor.command_enabled() is True
        stand.state = EAxleStandState.OFF
