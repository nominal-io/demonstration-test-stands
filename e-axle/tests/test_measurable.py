import time

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
