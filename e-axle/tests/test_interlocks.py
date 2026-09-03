"""Step 1 ("Interlocks & units") tests for `interlocks.py`'s trip-check
functions.

Every threshold below is transcribed from design.md's "Limits and
interlocks" table and from `LIMITS` (interlocks.py), which is itself
transcribed from that table. Where design.md states an explicit comparison
operator (FET temperature, motor temperature, bus overvoltage: all "> X"),
this file pins that operator with a boundary test. Where design.md gives a
threshold but not an explicit operator (DUT/dyno overspeed, speed-tracking
error, consecutive send failures, spread's percentage/duration composition),
this file makes an explicit, documented choice and pins it -- see the
per-function comments below and the Step 1 report for the human-reviewable
list of those judgment calls.
"""

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from interlocks import LIMITS, Trip, check_bus_overvoltage  # noqa: E402
from interlocks import check_consecutive_send_failures  # noqa: E402
from interlocks import check_dropout  # noqa: E402
from interlocks import check_dut_overspeed  # noqa: E402
from interlocks import check_dyno_overspeed  # noqa: E402
from interlocks import check_fet_temperature  # noqa: E402
from interlocks import check_motor_temperature  # noqa: E402
from interlocks import check_spread  # noqa: E402
from interlocks import check_speed_tracking_error  # noqa: E402
from interlocks import check_staleness  # noqa: E402
from interlocks import is_fresh  # noqa: E402


# ---------------------------------------------------------------------------
# 2: DUT overspeed
# ---------------------------------------------------------------------------


def test_check_dut_overspeed_quiet_under_the_limit():
    assert check_dut_overspeed(12_000.0) is None


def test_check_dut_overspeed_trips_over_the_limit():
    trip = check_dut_overspeed(12_601.0)
    assert isinstance(trip, Trip)
    assert trip.node == "mdc"


def test_check_dut_overspeed_does_not_trip_exactly_at_the_limit():
    # Judgment call (design.md gives no explicit operator for this interlock,
    # unlike FET/motor temp and bus overvoltage): treated consistently with
    # those three, strictly-greater-than trips.
    assert check_dut_overspeed(LIMITS.dut_overspeed_erpm) is None


# ---------------------------------------------------------------------------
# 3: Dyno overspeed
# ---------------------------------------------------------------------------


def test_check_dyno_overspeed_quiet_under_the_limit():
    assert check_dyno_overspeed("dmc_l", 3_000.0) is None


def test_check_dyno_overspeed_trips_over_the_limit_and_names_its_node():
    trip = check_dyno_overspeed("dmc_r", 3_301.0)
    assert isinstance(trip, Trip)
    assert trip.node == "dmc_r"


def test_check_dyno_overspeed_does_not_trip_exactly_at_the_limit():
    assert check_dyno_overspeed("dmc_l", LIMITS.dyno_overspeed_erpm) is None


# ---------------------------------------------------------------------------
# 4: FET temperature -- design.md states "> 80 C" explicitly.
# ---------------------------------------------------------------------------


def test_check_fet_temperature_quiet_under_the_limit():
    assert check_fet_temperature("mdc", 79.9) is None


def test_check_fet_temperature_trips_over_the_limit():
    trip = check_fet_temperature("mdc", 80.1)
    assert isinstance(trip, Trip)
    assert trip.node == "mdc"


def test_check_fet_temperature_does_not_trip_exactly_at_the_limit():
    assert check_fet_temperature("mdc", 80.0) is None


# ---------------------------------------------------------------------------
# 5: Motor temperature -- design.md states "> 100 C, only when the reading
# is plausible (0 < T < 200)"; 0.0 means "no sensor fitted."
# ---------------------------------------------------------------------------


def test_check_motor_temperature_quiet_under_the_limit():
    assert check_motor_temperature("dmc_l", 99.9) is None


def test_check_motor_temperature_trips_over_the_limit():
    trip = check_motor_temperature("dmc_l", 100.1)
    assert isinstance(trip, Trip)
    assert trip.node == "dmc_l"


def test_check_motor_temperature_does_not_trip_exactly_at_the_limit():
    assert check_motor_temperature("dmc_l", 100.0) is None


def test_check_motor_temperature_zero_means_no_sensor_fitted_and_never_trips():
    assert check_motor_temperature("dmc_l", 0.0) is None


def test_check_motor_temperature_ignores_an_implausible_reading():
    # 0 < T < 200 is the plausibility window; 200.0 and above is implausible
    # and must not be treated as a real over-temperature.
    assert check_motor_temperature("dmc_l", 200.0) is None
    assert check_motor_temperature("dmc_l", 250.0) is None


# ---------------------------------------------------------------------------
# 6: Bus overvoltage -- design.md states "> 54 V on any node" explicitly.
# ---------------------------------------------------------------------------


def test_check_bus_overvoltage_quiet_under_the_limit():
    assert check_bus_overvoltage("mdc", 53.9) is None


def test_check_bus_overvoltage_trips_over_the_limit():
    trip = check_bus_overvoltage("mdc", 54.1)
    assert isinstance(trip, Trip)
    assert trip.node == "mdc"


def test_check_bus_overvoltage_does_not_trip_exactly_at_54_volts():
    assert check_bus_overvoltage("mdc", 54.0) is None


# ---------------------------------------------------------------------------
# 7: Dyno torque dropout -- design.md: "commanded >= 1 A, delivering < 30%
# for 0.3 s". This is the plan's own worked example test, reproduced as-is.
# ---------------------------------------------------------------------------


def test_check_dropout_fires_past_duration_not_before():
    assert (
        check_dropout("dmc_l", commanded_a=10.0, measured_a=1.0, below_threshold_for_s=0.29)
        is None
    )
    trip = check_dropout(
        "dmc_l", commanded_a=10.0, measured_a=1.0, below_threshold_for_s=0.30
    )
    assert trip is not None and trip.node == "dmc_l"


def test_check_dropout_ignores_commanded_current_below_the_1a_floor():
    # commanded < 1 A: even zero delivered current is not a dropout, per
    # design.md's "commanded >= 1 A" precondition.
    assert (
        check_dropout("dmc_r", commanded_a=0.5, measured_a=0.0, below_threshold_for_s=1.0)
        is None
    )


def test_check_dropout_does_not_fire_exactly_at_the_30_percent_floor():
    # measured == 30% of commanded is "delivering 30%", not "< 30%".
    trip = check_dropout(
        "dmc_r", commanded_a=10.0, measured_a=3.0, below_threshold_for_s=1.0
    )
    assert trip is None


def test_check_dropout_fires_at_exactly_the_1a_commanded_floor():
    trip = check_dropout(
        "dmc_l", commanded_a=1.0, measured_a=0.0, below_threshold_for_s=0.3
    )
    assert trip is not None and trip.node == "dmc_l"


# ---------------------------------------------------------------------------
# 8: Half-shaft spread -- design.md: "15% for 0.2s, above a 500 ERPM floor".
# Judgment call (no formula given in design.md): fraction is
# |L - R| / max(|L|, |R|); the 0.2s duration gates on `above_floor_for_s`
# (the parameter's own name), i.e. the floor condition must have held for
# 0.2s before the (instantaneous) percentage check is allowed to trip.
# ---------------------------------------------------------------------------


def test_check_spread_quiet_when_within_15_percent():
    assert check_spread(1000.0, 900.0, above_floor_for_s=1.0) is None


def test_check_spread_trips_over_15_percent_once_above_floor_long_enough():
    trip = check_spread(1000.0, 700.0, above_floor_for_s=0.2)
    assert isinstance(trip, Trip)
    assert trip.node is None  # not node-specific, per interlocks.py's Trip docstring


def test_check_spread_does_not_trip_exactly_at_the_500_erpm_floor():
    # Both sides sit exactly at the floor -- must not fire even with huge
    # percentage divergence, since the floor is "above which the trip is
    # live," not "at or above."
    assert check_spread(500.0, 500.0, above_floor_for_s=10.0) is None


def test_check_spread_does_not_trip_below_the_500_erpm_floor():
    assert check_spread(400.0, 100.0, above_floor_for_s=10.0) is None


def test_check_spread_does_not_trip_before_the_0_2s_duration_is_reached():
    assert check_spread(1000.0, 700.0, above_floor_for_s=0.19) is None


def test_check_spread_does_not_trip_at_exactly_15_percent():
    # 850 / 1000 = 0.15 exactly.
    assert check_spread(1000.0, 850.0, above_floor_for_s=1.0) is None


# ---------------------------------------------------------------------------
# 9: DUT speed-tracking error -- design.md: "1500 ERPM sustained 3.0 s".
# ---------------------------------------------------------------------------


def test_check_speed_tracking_error_quiet_within_tolerance():
    assert check_speed_tracking_error(8_000.0, 7_000.0, sustained_for_s=5.0) is None


def test_check_speed_tracking_error_trips_once_sustained():
    trip = check_speed_tracking_error(8_000.0, 6_000.0, sustained_for_s=3.0)
    assert isinstance(trip, Trip)


def test_check_speed_tracking_error_does_not_trip_before_3s():
    assert check_speed_tracking_error(8_000.0, 6_000.0, sustained_for_s=2.99) is None


def test_check_speed_tracking_error_does_not_trip_at_exactly_1500_erpm():
    assert check_speed_tracking_error(8_000.0, 6_500.0, sustained_for_s=5.0) is None


def test_check_speed_tracking_error_is_symmetric_in_sign():
    trip = check_speed_tracking_error(6_000.0, 8_000.0, sustained_for_s=3.0)
    assert isinstance(trip, Trip)


# ---------------------------------------------------------------------------
# 10: Node staleness -- design.md: "no frame for 0.5 s"; Step 2's can_arm()
# outline treats `now - seen <= LIMITS.staleness_s` as fresh, so the trip
# side is the complement: strictly greater than 0.5s.
# ---------------------------------------------------------------------------


def test_check_staleness_quiet_within_the_window():
    assert check_staleness("mdc", 0.4) is None


def test_check_staleness_trips_past_the_window():
    trip = check_staleness("mdc", 0.51)
    assert isinstance(trip, Trip)
    assert trip.node == "mdc"


def test_check_staleness_does_not_trip_exactly_at_0_5s():
    assert check_staleness("mdc", 0.5) is None


# ---------------------------------------------------------------------------
# 11: Consecutive send failures -- design.md: "25 in a row".
# ---------------------------------------------------------------------------


def test_check_consecutive_send_failures_quiet_under_25():
    assert check_consecutive_send_failures(24) is None


def test_check_consecutive_send_failures_trips_at_25():
    trip = check_consecutive_send_failures(25)
    assert isinstance(trip, Trip)


def test_check_consecutive_send_failures_trips_above_25():
    assert isinstance(check_consecutive_send_failures(26), Trip)


# ---------------------------------------------------------------------------
# 12: Freshness gate -- design.md: "counted when data is older than 0.15s",
# i.e. exactly-0.15s-old data is not yet "older than" the gate.
# ---------------------------------------------------------------------------


def test_is_fresh_true_within_the_window():
    assert is_fresh(0.1) is True


def test_is_fresh_false_past_the_window():
    assert is_fresh(0.16) is False


def test_is_fresh_true_at_exactly_0_15s():
    assert is_fresh(LIMITS.freshness_s) is True
