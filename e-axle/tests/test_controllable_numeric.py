import pytest

from channels import Controllable, ControllableNumeric, Monitorable


def test_controllable_numeric_default_construction():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    assert channel.setpoint == 5.0
    assert channel.default == 5.0
    assert channel.measured is None
    assert channel.minimum == 0.0
    assert channel.maximum == 10.0


def test_controllable_numeric_raises_when_default_out_of_bounds():
    with pytest.raises(ValueError):
        ControllableNumeric(default=500.0, minimum=0.0, maximum=100.0)


def test_controllable_numeric_clamps_below_minimum():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.setpoint = -5.0
    assert channel.setpoint == 0.0
    assert channel.requested == -5.0


def test_controllable_numeric_clamps_above_maximum():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.setpoint = 50.0
    assert channel.setpoint == 10.0
    assert channel.requested == 50.0


def test_controllable_numeric_passes_through_in_range():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.setpoint = 7.0
    assert channel.setpoint == 7.0
    assert channel.requested == 7.0


def test_controllable_numeric_raises_when_minimum_greater_than_maximum():
    with pytest.raises(ValueError):
        ControllableNumeric(default=5.0, minimum=100.0, maximum=0.0)


def test_controllable_numeric_unbounded_by_default():
    channel = ControllableNumeric(default=5.0)
    channel.setpoint = 1e300
    assert channel.setpoint == 1e300


def test_controllable_numeric_repr_shows_requested_when_clamped():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.setpoint = 50.0
    assert repr(channel) == "ControllableNumeric(None: requested=50.0, setpoint=10.0, measured=None)"


def test_controllable_numeric_repr_omits_requested_when_not_clamped():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.setpoint = 7.0
    assert repr(channel) == "ControllableNumeric(None: setpoint=7.0, measured=None)"


def test_controllable_numeric_is_both_controllable_and_monitorable():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    assert isinstance(channel, Controllable)
    assert isinstance(channel, Monitorable)


def test_controllable_numeric_not_tripped_within_bounds():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.measured = 5.0
    assert not channel.tripped


def test_controllable_numeric_not_tripped_when_unmeasured():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    assert not channel.tripped


def test_controllable_numeric_tripped_when_measured_out_of_bounds():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.measured = 15.0
    assert channel.tripped


def test_controllable_numeric_on_trip_called_when_out_of_bounds():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    calls = []
    channel.on_trip = calls.append
    channel.measured = 15.0
    assert calls == [channel]


def test_controllable_numeric_raises_when_deadband_negative():
    with pytest.raises(ValueError):
        ControllableNumeric(default=5.0, deadband=-0.1)


def test_controllable_numeric_zero_deadband_requires_exact_match():
    channel = ControllableNumeric(default=5.0)
    channel.measured = 5.001
    assert not channel.at_setpoint


def test_controllable_numeric_at_setpoint_within_deadband():
    channel = ControllableNumeric(default=5.0, deadband=0.5)
    channel.measured = 5.4
    assert channel.at_setpoint


def test_controllable_numeric_not_at_setpoint_outside_deadband():
    channel = ControllableNumeric(default=5.0, deadband=0.5)
    channel.measured = 5.6
    assert not channel.at_setpoint


def test_controllable_numeric_not_at_setpoint_when_unmeasured():
    channel = ControllableNumeric(default=5.0, deadband=0.5)
    assert not channel.at_setpoint
