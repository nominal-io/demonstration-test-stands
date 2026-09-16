from datetime import timedelta

import pytest

from e_axle.channels import Controllable, Measurable, Monitorable


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


def test_monitorable_on_trip_not_called_when_within_bounds():
    point = Monitorable(minimum=0.0, maximum=90.0)
    calls = []
    point.on_trip = calls.append
    point.measured = 50.0
    assert calls == []


def test_monitorable_on_trip_called_when_out_of_bounds():
    point = Monitorable(minimum=0.0, maximum=90.0)
    calls = []
    point.on_trip = calls.append
    point.measured = 95.0
    assert calls == [point]


def test_monitorable_on_trip_called_every_time_while_still_tripped():
    point = Monitorable(minimum=0.0, maximum=90.0)
    calls = []
    point.on_trip = calls.append
    point.measured = 95.0
    point.measured = 100.0
    assert calls == [point, point]


def test_monitorable_on_trip_default_is_none_and_does_not_raise():
    point = Monitorable(minimum=0.0, maximum=90.0)
    point.measured = 95.0  # no on_trip registered -- must not raise


def test_monitorable_repr(monkeypatch):
    monkeypatch.setattr("e_axle.channels.monotonic", lambda: 12345.0)
    point = Monitorable(minimum=0.0, maximum=90.0)
    point.measured = 50.0
    when = (Measurable._wall_origin + timedelta(seconds=12345.0 - Measurable._monotonic_origin)).isoformat(
        timespec="seconds"
    )
    assert repr(point) == f"Monitorable({when}: minimum=0.0, measured=50.0, maximum=90.0)"
