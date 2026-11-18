import math

import pytest
from conftest import FakeClock, tick, tick_until

from fluids_panel import sequences
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


def test_close_closes_the_backend_even_if_the_last_write_fails(
    clock, mock, monkeypatch
):
    stand = FluidsStand(mock, clock=clock)
    stand.open()
    make_ready(stand, clock)
    assert mock.outputs != SAFE_OUTPUTS

    def write(outputs):
        raise RuntimeError("bus down")

    monkeypatch.setattr(mock, "write", write)
    with pytest.raises(RuntimeError, match="bus down"):
        stand.close()
    # The write never landed, so only the backend's own close() made these safe.
    assert mock.outputs == SAFE_OUTPUTS


def test_open_closes_the_backend_if_the_first_reading_fails(clock, mock, monkeypatch):
    def read():
        raise RuntimeError("no inputs")

    monkeypatch.setattr(mock, "read", read)
    with pytest.raises(RuntimeError, match="no inputs"):
        FluidsStand(mock, clock=clock).open()
    with pytest.raises(RuntimeError, match="not open"):
        mock.write({})


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
    # p_1's loop stays healthy, so this is a real loss of supply, not a failed sensor.
    assert stand.low_loops == set()
    assert_tripped(stand, mock, "supply lost")
    assert any("E-stop pressed?" in trip for trip in stand.trips)


def test_compressor_failure_warns_and_vents_after_pressurize_timeout(
    stand, mock, clock
):
    mock.faults.compressor_failed = True
    stand.pressurize()
    tick(stand, clock, stand.config.pressurize_timeout_s + 1.0)
    assert stand.trips == []
    assert stand.state in (FluidsStandState.VENTING, FluidsStandState.VENTED)
    assert mock.outputs == SAFE_OUTPUTS
    assert stand.warnings == ["p_1 did not reach 75 psi within 90 s; venting"]
    assert stand.telemetry()["pump_1.state"] == CompressorState.FAULT

    mock.faults.compressor_failed = False
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.VENTED, 30.0)
    stand.pressurize()  # a fresh attempt clears the fault
    assert stand.telemetry()["pump_1.state"] == CompressorState.RUNNING


def test_startup_stops_when_pressurize_times_out(stand, mock, clock):
    mock.faults.compressor_failed = True
    sequences.start(stand, "startup")
    tick(stand, clock, stand.config.pressurize_timeout_s + 1.0)
    assert not stand.sequence_running
    assert stand.trips == []
    assert (
        len(stand.warnings) == 1
    )  # the stand's warning, not a second from the sequence


def test_regulator_failure_trips_on_overpressure(stand, mock, clock):
    mock.faults.regulator_failed = True
    stand.pressurize()
    tick(stand, clock, 60.0)
    assert_tripped(stand, mock, "p_1 over")
    assert len(stand.trips) == 1


def test_stuck_actuator_warns_once_and_retracts(stand, mock, clock):
    make_ready(stand, clock)
    stand.extend()
    tick(stand, clock, 0.5)  # partway out
    mock.faults.actuator_stuck = True
    tick(stand, clock, stand.config.actuator_timeout_s + 1.0)
    assert stand.trips == []
    assert stand.state == FluidsStandState.READY
    assert stand.v_3.setpoint is False
    assert stand.warnings == [
        "actuator did not reach sw_2 (extended) within 5 s; retracting"
    ]
    assert stand.telemetry()["actuator.state"] == ValveState.FAULT

    # Stuck mid-stroke it can't retract either, but that doesn't warn again every 5 s.
    tick(stand, clock, 3 * stand.config.actuator_timeout_s)
    assert len(stand.warnings) == 1
    assert stand.telemetry()["actuator.state"] == ValveState.FAULT

    mock.faults.actuator_stuck = False
    tick_until(stand, clock, lambda: bool(stand.sw_1.measured), 10.0)
    tick(stand, clock, 0.1)
    assert stand.telemetry()["actuator.state"] == ValveState.CLOSED


def test_actuate_stops_when_the_actuator_sticks(stand, mock, clock):
    make_ready(stand, clock)
    mock.faults.actuator_stuck = True
    sequences.start(stand, "actuate")
    tick(stand, clock, stand.config.actuator_timeout_s + 2.0)
    assert not stand.sequence_running
    assert stand.trips == []
    assert (
        len(stand.warnings) == 1
    )  # the stand's warning, not a second from the sequence


def test_broken_wire_warns_without_tripping(stand, mock, clock):
    make_ready(stand, clock)
    mock.faults.broken_wires.add("fm_1")
    tick(stand, clock, 0.1)
    assert stand.state == FluidsStandState.READY
    assert stand.trips == []
    assert stand.low_loops == {"fm_1"}
    assert stand.warnings == [
        (
            "fm_1 loop current low at 0.00 mA (broken wire or failed sensor?); "
            "its readings can't be trusted"
        )
    ]
    assert stand.telemetry()["fm_1.state"] == FlowState.FAULT

    mock.faults.broken_wires.clear()
    tick(stand, clock, stand.config.loop_low_clear_s - 0.5)
    assert stand.low_loops == {"fm_1"}  # not healthy for long enough yet
    tick(stand, clock, 1.0)
    assert stand.low_loops == set()
    assert stand.warnings[-1] == "fm_1 loop current back in range"


def test_noisy_low_loop_warns_once(stand, mock, clock):
    for _ in range(10):  # dips shorter than the clear time apart
        mock.faults.broken_wires.add("p_2")
        tick(stand, clock, 0.1)
        mock.faults.broken_wires.clear()
        tick(stand, clock, 1.0)
    assert stand.trips == []
    assert len(stand.warnings) == 1
    assert stand.low_loops == {"p_2"}


def test_low_p_1_loop_blocks_pressurize(stand, mock, clock):
    mock.faults.broken_wires.add("p_1")
    tick(stand, clock, 0.1)
    with pytest.raises(InterlockError, match="p_1's loop current is low"):
        stand.pressurize()
    assert mock.outputs == SAFE_OUTPUTS

    mock.faults.broken_wires.clear()
    tick(stand, clock, stand.config.loop_low_clear_s + 0.5)
    stand.pressurize()
    assert stand.state == FluidsStandState.PRESSURIZING


def assert_vented_on_low_p_1_loop(stand: FluidsStand, mock: MockBackend, clock) -> None:
    mock.faults.broken_wires.add("p_1")
    tick(stand, clock, 0.1)
    assert stand.state == FluidsStandState.VENTING
    assert stand.trips == []
    assert not stand.sequence_running
    assert mock.outputs == SAFE_OUTPUTS
    assert stand.warnings[-1] == "p_1 can't be read with the compressor on; venting"
    assert len(stand.warnings) == 2  # the low loop, then the vent; once each
    assert stand.telemetry()["pump_1.state"] == CompressorState.OFF

    tick(stand, clock, 10.0)
    assert len(stand.warnings) == 2


def test_low_p_1_loop_while_pressurizing_vents(stand, mock, clock):
    sequences.start(stand, "startup")
    tick(stand, clock, 5.0)
    assert_vented_on_low_p_1_loop(stand, mock, clock)


def test_low_p_1_loop_while_ready_vents_rather_than_trips(stand, mock, clock):
    make_ready(stand, clock)
    sequences.start(stand, "flow")
    tick(stand, clock, 1.0)
    assert_vented_on_low_p_1_loop(stand, mock, clock)


def test_low_p_1_loop_holds_venting_until_it_reads_again(stand, mock, clock):
    make_ready(stand, clock)
    stand.vent()
    tick(stand, clock, 0.1)
    mock.faults.broken_wires.add("p_1")
    tick(stand, clock, 30.0)  # long enough to have vented, but it can't be confirmed
    assert stand.state == FluidsStandState.VENTING
    assert stand.trips == []

    mock.faults.broken_wires.clear()
    tick(stand, clock, stand.config.loop_low_clear_s - 0.5)
    assert stand.state == FluidsStandState.VENTING
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.VENTED, 2.0)


def test_shorted_loop_trips_and_blocks_reset(stand, mock, clock):
    make_ready(stand, clock)
    mock.faults.shorted_loops.add("fm_1")
    tick(stand, clock, 0.1)
    assert_tripped(stand, mock, "fm_1 loop current over range")
    assert stand.telemetry()["fm_1.state"] == FlowState.FAULT

    with pytest.raises(InterlockError, match="fm_1 loop over range"):
        stand.reset()

    mock.faults.shorted_loops.clear()
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


def test_trip_history_survives_reset(stand, mock, clock):
    make_ready(stand, clock)
    mock.faults.estop = True
    tick(stand, clock, 5.0)
    mock.faults.estop = False
    stand.reset()
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.VENTED, 30.0)
    make_ready(stand, clock)
    mock.faults.estop = True
    tick(stand, clock, 5.0)

    # A second trip after a reset leaves `trips` the same length as before, so a caller
    # counting `trips` would miss it; `trip_history` keeps both.
    assert len(stand.trips) == 1
    assert len(stand.trip_history) == 2
    assert stand.trip_history[-1] == stand.trips[0]


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
