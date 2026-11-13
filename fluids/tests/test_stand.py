import math

import pytest
from conftest import FakeClock, tick, tick_until

from fluids_panel.backend import SAFE_OUTPUTS
from fluids_panel.mock import MockBackend
from fluids_panel.stand import FluidsStand, FluidsStandState, InterlockError
from fluids_panel.states import CompressorState, FlowState, LightState, ValveState


def make_ready(stand: FluidsStand, clock: FakeClock) -> None:
    stand.pressurize()
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.READY, 60.0)


def test_opens_vented_and_safe(stand, mock):
    assert stand.state == FluidsStandState.VENTED
    assert mock.outputs == SAFE_OUTPUTS
    telemetry = stand.telemetry()
    assert telemetry["stand.status"] == LightState.OFF
    assert telemetry["v_1.state"] == ValveState.OPEN
    assert telemetry["actuator.state"] == ValveState.CLOSED


def test_close_leaves_outputs_safe(clock, mock):
    with FluidsStand(mock, clock=clock) as stand:
        make_ready(stand, clock)
    assert mock.outputs == SAFE_OUTPUTS


@pytest.mark.parametrize(
    "command",
    [
        lambda s: s.set_flow(5.0),
        lambda s: s.extend(),
        lambda s: s.set_vessel_pressure(10.0),
        lambda s: s.isolate_vessel(True),
        lambda s: s.reset(),
    ],
)
def test_energizing_commands_need_ready(stand, command):
    with pytest.raises(InterlockError):
        command(stand)


def test_safe_direction_commands_always_allowed(stand):
    stand.set_flow(0.0)
    stand.retract()
    stand.set_vessel_pressure(0.0)
    stand.isolate_vessel(False)
    stand.vent()
    assert stand.state == FluidsStandState.VENTED


def test_pressurize_reaches_ready(stand, mock, clock):
    stand.pressurize()
    assert stand.state == FluidsStandState.PRESSURIZING
    tick(stand, clock, 0.1)
    assert mock.outputs["pump_1"] is True
    assert mock.outputs["v_1"] is True
    assert stand.telemetry()["stand.status"] == LightState.WARNING

    tick_until(stand, clock, lambda: stand.state == FluidsStandState.READY, 60.0)
    telemetry = stand.telemetry()
    assert telemetry["stand.status"] == LightState.OK
    assert telemetry["pump_1.state"] == CompressorState.RUNNING
    assert telemetry["v_1.state"] == ValveState.CLOSED


def test_pressurize_only_from_vented(stand, clock):
    make_ready(stand, clock)
    with pytest.raises(InterlockError):
        stand.pressurize()


def test_vent_returns_to_vented(stand, mock, clock):
    make_ready(stand, clock)
    stand.set_flow(5.0)
    stand.vent()
    assert stand.state == FluidsStandState.VENTING
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.VENTED, 30.0)
    assert mock.outputs == SAFE_OUTPUTS


def test_flow(stand, clock):
    make_ready(stand, clock)
    stand.set_flow(7.8)
    tick(stand, clock, 5.0)
    telemetry = stand.telemetry()
    assert math.isclose(telemetry["fm_1"], 7.8, abs_tol=0.1)
    assert telemetry["v_2"] == 7.8
    assert telemetry["v_2.state"] == ValveState.PARTIAL
    assert telemetry["fm_1.state"] == FlowState.FLOWING


def test_flow_clamps_to_valve_range(stand, clock):
    make_ready(stand, clock)
    stand.set_flow(100.0)
    assert stand.v_2.setpoint == 15.6
    assert stand.telemetry()["v_2.state"] == ValveState.OPEN


def test_actuator_states(stand, clock):
    make_ready(stand, clock)
    stand.extend()
    tick(stand, clock, 0.5)
    assert stand.telemetry()["actuator.state"] == ValveState.TRANSITIONING
    tick(stand, clock, 1.5)
    telemetry = stand.telemetry()
    assert telemetry["actuator.state"] == ValveState.OPEN
    assert telemetry["sw_1.state"] == LightState.OFF
    assert telemetry["sw_2.state"] == LightState.OK


def test_vessel(stand, clock):
    make_ready(stand, clock)
    stand.set_vessel_pressure(72.5)
    tick(stand, clock, 20.0)
    assert math.isclose(stand.p_2_psi, 72.5, abs_tol=0.1)
    stand.isolate_vessel(True)
    tick(stand, clock, 0.1)
    assert stand.telemetry()["v_5.state"] == ValveState.CLOSED


def assert_tripped(stand: FluidsStand, mock: MockBackend, reason: str) -> None:
    assert stand.state == FluidsStandState.TRIPPED
    assert any(reason in trip for trip in stand.trips), stand.trips
    assert mock.outputs == SAFE_OUTPUTS
    assert stand.telemetry()["stand.status"] == LightState.FAULT


def test_estop_trips_on_supply_loss(stand, mock, clock):
    make_ready(stand, clock)
    mock.faults.estop = True
    tick(stand, clock, 5.0)
    assert_tripped(stand, mock, "supply lost")


def test_compressor_failure_trips_after_pressurize_timeout(stand, mock, clock):
    mock.faults.compressor_failed = True
    stand.pressurize()
    tick(stand, clock, stand.config.pressurize_timeout_s + 1.0)
    assert_tripped(stand, mock, "did not reach")
    assert stand.telemetry()["pump_1.state"] == CompressorState.FAULT


def test_regulator_failure_trips_on_overpressure(stand, mock, clock):
    mock.faults.regulator_failed = True
    stand.pressurize()
    tick(stand, clock, 60.0)
    assert_tripped(stand, mock, "p_1 over")
    assert len(stand.trips) == 1


def test_stuck_actuator_trips(stand, mock, clock):
    make_ready(stand, clock)
    mock.faults.actuator_stuck = True
    stand.extend()
    tick(stand, clock, stand.config.actuator_timeout_s + 1.0)
    assert_tripped(stand, mock, "actuator did not reach sw_2")
    assert stand.telemetry()["actuator.state"] == ValveState.FAULT


def test_broken_wire_trips_and_blocks_reset(stand, mock, clock):
    make_ready(stand, clock)
    mock.faults.broken_wires.add("fm_1")
    tick(stand, clock, 0.1)
    assert_tripped(stand, mock, "fm_1 loop current")
    assert stand.telemetry()["fm_1.state"] == FlowState.FAULT

    with pytest.raises(InterlockError, match="fm_1 loop fault"):
        stand.reset()

    mock.faults.broken_wires.clear()
    tick(stand, clock, 0.1)
    stand.reset()
    assert stand.trips == []
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.VENTED, 30.0)


def test_reset_then_pressurize_again(stand, mock, clock):
    make_ready(stand, clock)
    mock.faults.estop = True
    tick(stand, clock, 5.0)
    mock.faults.estop = False
    stand.reset()
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.VENTED, 30.0)
    make_ready(stand, clock)
    assert stand.telemetry()["pump_1.state"] == CompressorState.RUNNING


def test_tripped_refuses_pressurize(stand, mock, clock):
    make_ready(stand, clock)
    mock.faults.estop = True
    tick(stand, clock, 5.0)
    with pytest.raises(InterlockError):
        stand.pressurize()


@pytest.mark.parametrize("pressurized", [False, True], ids=["vented", "ready"])
def test_idle_noise_never_reads_as_flow(noisy_stand, clock, pressurized):
    if pressurized:
        make_ready(noisy_stand, clock)
    for _ in range(600):
        clock.now += 0.1
        noisy_stand.update()
        assert noisy_stand.telemetry()["fm_1.state"] == FlowState.NO_FLOW
    assert noisy_stand.trips == []


def test_telemetry_is_numeric(stand, clock):
    make_ready(stand, clock)
    telemetry = stand.telemetry()
    assert all(isinstance(value, float) for value in telemetry.values())
    assert telemetry["stand.state"] == FluidsStandState.READY
