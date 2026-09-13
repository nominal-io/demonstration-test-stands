import pytest

from control_point import ControlPoint, ControlPointNumeric


def test_control_point_default_construction():
    point = ControlPoint(default=1.0)
    assert point.setpoint == 1.0
    assert point.default == 1.0
    assert point.measured is None


def test_control_point_setpoint_roundtrip():
    point = ControlPoint(default=1.0)
    point.setpoint = 2.0
    assert point.setpoint == 2.0
    assert point.requested == 2.0


def test_control_point_measured_roundtrip():
    point = ControlPoint(default=1.0)
    point.measured = 3.0
    assert point.measured == 3.0


def test_control_point_default_is_read_only():
    point = ControlPoint(default=1.0)
    with pytest.raises(AttributeError):
        point.default = 5.0


def test_control_point_numeric_default_construction():
    point = ControlPointNumeric(default=5.0, minimum=0.0, maximum=10.0)
    assert point.setpoint == 5.0
    assert point.default == 5.0
    assert point.measured is None
    assert point.minimum == 0.0
    assert point.maximum == 10.0


def test_control_point_numeric_raises_when_default_out_of_bounds():
    with pytest.raises(ValueError):
        ControlPointNumeric(default=500.0, minimum=0.0, maximum=100.0)


def test_control_point_numeric_clamps_below_minimum():
    point = ControlPointNumeric(default=5.0, minimum=0.0, maximum=10.0)
    point.setpoint = -5.0
    assert point.setpoint == 0.0
    assert point.requested == -5.0


def test_control_point_numeric_clamps_above_maximum():
    point = ControlPointNumeric(default=5.0, minimum=0.0, maximum=10.0)
    point.setpoint = 50.0
    assert point.setpoint == 10.0
    assert point.requested == 50.0


def test_control_point_numeric_passes_through_in_range():
    point = ControlPointNumeric(default=5.0, minimum=0.0, maximum=10.0)
    point.setpoint = 7.0
    assert point.setpoint == 7.0
    assert point.requested == 7.0


def test_control_point_numeric_raises_when_minimum_greater_than_maximum():
    with pytest.raises(ValueError):
        ControlPointNumeric(default=5.0, minimum=100.0, maximum=0.0)


def test_control_point_numeric_unbounded_by_default():
    point = ControlPointNumeric(default=5.0)
    point.setpoint = 1e300
    assert point.setpoint == 1e300


def test_control_point_repr_shows_requested_when_clamped():
    point = ControlPointNumeric(default=5.0, minimum=0.0, maximum=10.0)
    point.setpoint = 50.0
    assert repr(point) == "ControlPointNumeric(requested=50.0, setpoint=10.0, measured=None)"


def test_control_point_repr_omits_requested_when_not_clamped():
    point = ControlPointNumeric(default=5.0, minimum=0.0, maximum=10.0)
    point.setpoint = 7.0
    assert repr(point) == "ControlPointNumeric(setpoint=7.0, measured=None)"
