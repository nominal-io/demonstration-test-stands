import time
from datetime import timedelta

import pytest

from channels import Measurable


def test_measurable_default_construction():
    point = Measurable()
    assert point.measured is None
    assert point.timestamp is None


def test_measurable_timestamp_is_not_settable():
    point = Measurable()
    with pytest.raises(AttributeError):
        point.timestamp = 5.0


def test_measurable_timestamp_set_after_measurement():
    point = Measurable()
    point.measured = 3.0
    assert point.timestamp is not None


def test_measurable_timestamp_set_even_when_measurement_produces_none():
    point = Measurable()
    point.measured = None
    assert point.timestamp is not None


def test_measurable_timestamp_updates_on_each_measurement():
    point = Measurable()
    point.measured = 3.0
    first = point.timestamp
    time.sleep(0.001)
    point.measured = 4.0
    second = point.timestamp
    assert second is not None and first is not None
    assert second > first


def test_measurable_timestamp_is_the_raw_monotonic_reading(monkeypatch):
    monkeypatch.setattr("channels.monotonic", lambda: 12345.0)
    point = Measurable()
    point.measured = 3.0
    assert point.timestamp == 12345.0


def test_measurable_to_isoformat_is_none_before_measurement():
    point = Measurable()
    assert point.to_isoformat() is None


def test_measurable_to_isoformat_matches_origin_conversion():
    point = Measurable()
    point.measured = 3.0
    assert point.timestamp is not None
    expected = Measurable._wall_origin + timedelta(seconds=point.timestamp - Measurable._monotonic_origin)
    assert point.to_isoformat() == expected.isoformat(timespec="seconds")
