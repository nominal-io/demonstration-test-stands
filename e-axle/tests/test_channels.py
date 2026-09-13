import time

import pytest

from channels import Controllable, Measureable, Monitorable, ControllableNumeric

def test_measurable_default_construction():
    point = Measureable()
    assert point.measured is None
    assert point.timestamp is None


def test_measureable_timestamp_is_not_settable():
    point = Measureable()
    with pytest.raises(AttributeError):
        point.timestamp = 5.0


def test_measureable_timestamp_set_after_measurement():
    point = Measureable()
    point.measured = 3.0
    assert point.timestamp is not None


def test_measureable_timestamp_set_even_when_measurement_produces_none():
    point = Measureable()
    point.measured = None
    assert point.timestamp is not None


def test_measureable_timestamp_updates_on_each_measurement():
    point = Measureable()
    point.measured = 3.0
    first = point.timestamp
    time.sleep(0.001)
    point.measured = 4.0
    second = point.timestamp
    assert second is not None and first is not None
    assert second > first


def test_controllable_default_construction():
    point = Controllable(default=1.0)
    assert point.setpoint == 1.0
    assert point.default == 1.0
    assert point.measured is None


def test_controllable_is_not_monitorable():
    point = Controllable(default=False)
    assert not isinstance(point, Monitorable)


def test_controllable_setpoint_roundtrip():
    point = Controllable(default=1.0)
    point.setpoint = 2.0
    assert point.setpoint == 2.0
    assert point.requested == 2.0


def test_controllable_measured_roundtrip():
    point = Controllable(default=1.0)
    point.measured = 3.0
    assert point.measured == 3.0


def test_controllable_default_is_read_only():
    point = Controllable(default=1.0)
    with pytest.raises(AttributeError):
        point.default = 5.0


def test_numeric_control_channel_default_construction():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    assert channel.setpoint == 5.0
    assert channel.default == 5.0
    assert channel.measured is None
    assert channel.minimum == 0.0
    assert channel.maximum == 10.0


def test_numeric_control_channel_raises_when_default_out_of_bounds():
    with pytest.raises(ValueError):
        ControllableNumeric(default=500.0, minimum=0.0, maximum=100.0)


def test_numeric_control_channel_clamps_below_minimum():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.setpoint = -5.0
    assert channel.setpoint == 0.0
    assert channel.requested == -5.0


def test_numeric_control_channel_clamps_above_maximum():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.setpoint = 50.0
    assert channel.setpoint == 10.0
    assert channel.requested == 50.0


def test_numeric_control_channel_passes_through_in_range():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.setpoint = 7.0
    assert channel.setpoint == 7.0
    assert channel.requested == 7.0


def test_numeric_control_channel_raises_when_minimum_greater_than_maximum():
    with pytest.raises(ValueError):
        ControllableNumeric(default=5.0, minimum=100.0, maximum=0.0)


def test_numeric_control_channel_unbounded_by_default():
    channel = ControllableNumeric(default=5.0)
    channel.setpoint = 1e300
    assert channel.setpoint == 1e300


def test_numeric_control_channel_repr_shows_requested_when_clamped():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.setpoint = 50.0
    assert repr(channel) == "ControllableNumeric(requested=50.0, setpoint=10.0, measured=None)"


def test_numeric_control_channel_repr_omits_requested_when_not_clamped():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.setpoint = 7.0
    assert repr(channel) == "ControllableNumeric(setpoint=7.0, measured=None)"


def test_numeric_control_channel_is_both_controllable_and_monitorable():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    assert isinstance(channel, Controllable)
    assert isinstance(channel, Monitorable)


def test_numeric_control_channel_not_tripped_within_bounds():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.measured = 5.0
    assert not channel.tripped


def test_numeric_control_channel_not_tripped_when_unmeasured():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    assert not channel.tripped


def test_numeric_control_channel_tripped_when_measured_out_of_bounds():
    channel = ControllableNumeric(default=5.0, minimum=0.0, maximum=10.0)
    channel.measured = 15.0
    assert channel.tripped


def test_monitorable_default_construction():
    point = Monitorable(minimum=0.0, maximum=90.0)
    assert point.measured is None
    assert point.minimum == 0.0
    assert point.maximum == 90.0


def test_monitorable_not_controllable():
    point = Monitorable(minimum=0.0, maximum=90.0)
    assert not isinstance(point, Controllable)


def test_monitorable_raises_when_minimum_greater_than_maximum():
    with pytest.raises(ValueError):
        Monitorable(minimum=90.0, maximum=0.0)


def test_monitorable_not_tripped_within_bounds():
    point = Monitorable(minimum=0.0, maximum=90.0)
    point.measured = 50.0
    assert not point.tripped


def test_monitorable_tripped_when_measured_out_of_bounds():
    point = Monitorable(minimum=0.0, maximum=90.0)
    point.measured = 95.0
    assert point.tripped


def test_monitorable_not_tripped_when_at_minimum():
    point = Monitorable(minimum=0.0, maximum=90.0)
    point.measured = 0.0
    assert not point.tripped


def test_monitorable_not_tripped_when_at_maximum():
    point = Monitorable(minimum=0.0, maximum=90.0)
    point.measured = 90.0
    assert not point.tripped


def test_monitorable_repr():
    point = Monitorable(minimum=0.0, maximum=90.0)
    point.measured = 50.0
    assert repr(point) == "Monitorable(minimum=0.0, measured=50.0, maximum=90.0)"
