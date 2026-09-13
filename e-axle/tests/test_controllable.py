import pytest

from channels import Controllable, Monitorable


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
