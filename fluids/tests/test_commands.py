import pytest
from conftest import FakeClock, tick_until

from fluids_panel.build import build_stand
from fluids_panel.commands import CommandError, handle_message
from fluids_panel.mock import MockBackend
from fluids_panel.stand import FluidsStand, FluidsStandState, InterlockError

# Connect adds these to every on_click_action message.
CLICK = {"widget_id": "96ccc0e2", "node_id": "2bda1a28"}


def send(stand: FluidsStand, **contents) -> str:
    return handle_message(stand, CLICK | contents)


def make_ready(stand: FluidsStand, clock: FakeClock) -> None:
    send(stand, action="pressurize")
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.READY, 60.0)


@pytest.mark.parametrize("target", ["supply", "pump_1", "v_1"])
def test_toggle_supply(stand, clock, target):
    assert send(stand, action="toggle", target=target) == "pressurizing"
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.READY, 60.0)
    assert send(stand, action="toggle", target=target) == "venting"
    assert stand.state == FluidsStandState.VENTING


def test_toggle_actuator(stand, clock):
    make_ready(stand, clock)
    assert send(stand, action="toggle", target="v_3") == "extending"
    assert stand.v_3.setpoint is True
    assert send(stand, action="toggle", target="v_3") == "retracting"
    assert stand.v_3.setpoint is False


def test_toggle_vessel(stand, clock):
    make_ready(stand, clock)
    send(stand, action="toggle", target="v_5")
    assert stand.v_5.setpoint is True
    send(stand, action="toggle", target="v_5")
    assert stand.v_5.setpoint is False


@pytest.mark.parametrize("value", [7.8, 7, "7.8"])
def test_set_flow(stand, clock, value):
    make_ready(stand, clock)
    send(stand, action="set", target="v_2", value=value)
    assert stand.v_2.setpoint == float(value)


def test_set_vessel_pressure(stand, clock):
    make_ready(stand, clock)
    send(stand, action="set", target="pr_2", value=72.5)
    assert stand.pr_2.setpoint == 72.5


def test_sequence_passes_only_its_parameters(stand, clock):
    make_ready(stand, clock)
    send(stand, action="sequence", name="flow", lpm=5.0, hold_s=1.0, unrelated=3)
    assert stand.sequence_name == "flow"
    assert stand.v_2.setpoint == 5.0


def test_interlocks_propagate(stand):
    with pytest.raises(InterlockError):
        send(stand, action="toggle", target="v_3")


def test_only_vent_abort_and_reset_while_sequence_runs(stand, clock):
    send(stand, action="sequence", name="startup")
    with pytest.raises(InterlockError, match="abort it before"):
        send(stand, action="toggle", target="v_3")
    send(stand, action="abort")
    assert not stand.sequence_running


def test_vent_aborts_a_running_sequence(stand, clock):
    send(stand, action="sequence", name="startup")
    assert send(stand, action="vent") == "sequence startup aborted; venting"
    assert not stand.sequence_running
    assert stand.state == FluidsStandState.VENTING


def test_vent_when_already_vented(stand):
    assert stand.state == FluidsStandState.VENTED
    assert send(stand, action="vent") == "already vented"
    assert stand.state == FluidsStandState.VENTED


def test_vent_while_tripped(stand, mock, clock):
    make_ready(stand, clock)
    mock.faults.estop = True
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.TRIPPED, 10.0)
    assert "press Reset trip" in send(stand, action="vent")
    assert stand.state == FluidsStandState.TRIPPED


def test_vent_while_pressurized(stand, clock):
    make_ready(stand, clock)
    assert send(stand, action="vent") == "venting"


@pytest.mark.parametrize(
    ("target", "value", "channel"),
    [("v_2", 7.8, "v_2"), ("pr_2", 72.5, "pr_2")],
)
def test_toggle_setpoint_valves(stand, clock, target, value, channel):
    make_ready(stand, clock)
    send(stand, action="toggle", target=target, value=value)
    assert getattr(stand, channel).setpoint == value
    send(stand, action="toggle", target=target, value=value)
    assert getattr(stand, channel).setpoint == 0.0


def test_toggle_setpoint_valve_needs_value_to_open(stand, clock):
    make_ready(stand, clock)
    with pytest.raises(CommandError):
        send(stand, action="toggle", target="v_2")


def test_toggle_setpoint_valve_closes_without_ready(stand, clock):
    make_ready(stand, clock)
    send(stand, action="toggle", target="v_2", value=5.0)
    stand.vent()
    stand.v_2.setpoint = 5.0  # as if a vent raced the click
    send(stand, action="toggle", target="v_2", value=5.0)
    assert stand.v_2.setpoint == 0.0


@pytest.mark.parametrize(
    "contents",
    [
        {},
        {"action": 3},
        {"action": "nope"},
        {"action": "toggle"},
        {"action": "toggle", "target": "v_2"},
        {"action": "set", "target": "v_3", "value": 1.0},
        {"action": "set", "target": "v_2"},
        {"action": "set", "target": "v_2", "value": None},
        {"action": "set", "target": "v_2", "value": True},
        {"action": "set", "target": "v_2", "value": "lots"},
        {"action": "sequence", "name": "nope"},
        {"action": "sequence", "name": "flow", "lpm": "fast"},
        # Faults are switches now, not messages.
        {"action": "fault", "name": "estop", "value": True},
    ],
)
def test_malformed_messages(stand, contents):
    with pytest.raises(CommandError):
        handle_message(stand, contents)


def test_reset_through_messages(stand, mock, clock):
    make_ready(stand, clock)
    mock.faults.estop = True
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.TRIPPED, 10.0)
    mock.faults.estop = False
    assert send(stand, action="reset") == "trip reset"


def test_build_stand_defaults_to_mock():
    assert isinstance(build_stand().backend, MockBackend)


def test_build_stand_rejects_unknown_backend():
    with pytest.raises(ValueError):
        build_stand("daqmx")
