"""Pure interlock trip evaluation and the commissioned machine's limits.

No `instro` import, at all, ever -- see plan.md's plan-decision box. These are
pure functions of plain numbers and telemetry dictionaries; nothing about
their thirteen trip evaluations has any reason to know Instro exists.
"""

from dataclasses import dataclass
from typing import Literal

Node = Literal["mdc", "dmc_l", "dmc_r"]


@dataclass(frozen=True)
class Limits:
    """Value object -- transcribed from the commissioned machine (design.md,
    'Limits and interlocks'). Immutable: configuration, not state."""

    max_brake_a: float = 20.0
    dut_overspeed_erpm: float = 12_600.0
    dyno_overspeed_erpm: float = 3_300.0
    fet_temperature_c: float = 80.0
    motor_temperature_c: float = 100.0  # open question 2 in design.md: human said stick with 100
    bus_overvoltage_v: float = 54.0
    dropout_min_commanded_a: float = 1.0
    dropout_fraction: float = 0.30
    dropout_duration_s: float = 0.3
    spread_fraction: float = 0.15
    spread_floor_erpm: float = 500.0
    spread_duration_s: float = 0.2
    speed_tracking_error_erpm: float = 1_500.0
    speed_tracking_duration_s: float = 3.0
    staleness_s: float = 0.5
    consecutive_send_failures: int = 25
    freshness_s: float = 0.15
    brake_ramp_a_per_s: float = 2.0
    speed_ramp_erpm_per_s: float = 1_000.0


LIMITS = Limits()


@dataclass(frozen=True)
class Trip:
    """A latched trip: what tripped, and which node (None if not node-specific,
    e.g. half-shaft spread)."""

    reason: str
    node: "Node | None"


# Thirteen pure trip-evaluation functions -- signatures only, numbered to match
# design.md's "Limits and interlocks" table. NOT exercised by the one feature
# test (its scripted plant keeps every interlock quiet by design -- design-gate
# Finding 9 / rev-4 review finding R4-2); each is unit-test work in Step 1.


def check_dut_overspeed(erpm: float) -> "Trip | None":  # 2
    if erpm > LIMITS.dut_overspeed_erpm:
        return Trip(reason="dut_overspeed", node="mdc")
    return None


def check_dyno_overspeed(node: Node, rpm: float) -> "Trip | None":  # 3
    if rpm > LIMITS.dyno_overspeed_erpm:
        return Trip(reason="dyno_overspeed", node=node)
    return None


def check_fet_temperature(node: Node, temp_c: float) -> "Trip | None":  # 4
    if temp_c > LIMITS.fet_temperature_c:
        return Trip(reason="fet_temperature", node=node)
    return None


def check_motor_temperature(node: Node, temp_c: float) -> "Trip | None":  # 5
    # 0.0 means "no sensor fitted"; readings at/above 200 C are implausible.
    # Only a plausible reading (0 < T < 200) is eligible to trip at all.
    if not (0.0 < temp_c < 200.0):
        return None
    if temp_c > LIMITS.motor_temperature_c:
        return Trip(reason="motor_temperature", node=node)
    return None


def check_bus_overvoltage(node: Node, voltage_v: float) -> "Trip | None":  # 6
    if voltage_v > LIMITS.bus_overvoltage_v:
        return Trip(reason="bus_overvoltage", node=node)
    return None


def check_dropout(
    node: Node, commanded_a: float, measured_a: float, below_threshold_for_s: float
) -> "Trip | None":  # 7
    if commanded_a < LIMITS.dropout_min_commanded_a:
        return None
    if measured_a < LIMITS.dropout_fraction * commanded_a:
        if below_threshold_for_s >= LIMITS.dropout_duration_s:
            return Trip(reason="dropout", node=node)
    return None


def check_spread(
    dmc_l_rpm: float, dmc_r_rpm: float, above_floor_for_s: float
) -> "Trip | None":  # 8
    floor_basis = max(abs(dmc_l_rpm), abs(dmc_r_rpm))
    if floor_basis <= LIMITS.spread_floor_erpm:
        return None
    if above_floor_for_s < LIMITS.spread_duration_s:
        return None
    fraction = abs(dmc_l_rpm - dmc_r_rpm) / floor_basis
    if fraction > LIMITS.spread_fraction:
        return Trip(reason="spread", node=None)
    return None


def check_speed_tracking_error(
    commanded_erpm: float, measured_erpm: float, sustained_for_s: float
) -> "Trip | None":  # 9
    if sustained_for_s < LIMITS.speed_tracking_duration_s:
        return None
    if abs(commanded_erpm - measured_erpm) > LIMITS.speed_tracking_error_erpm:
        return Trip(reason="speed_tracking_error", node="mdc")
    return None


def check_staleness(node: Node, seconds_since_last_frame: float) -> "Trip | None":  # 10
    if seconds_since_last_frame > LIMITS.staleness_s:
        return Trip(reason="staleness", node=node)
    return None


def check_consecutive_send_failures(count: int) -> "Trip | None":  # 11
    if count >= LIMITS.consecutive_send_failures:
        return Trip(reason="consecutive_send_failures", node=None)
    return None


def is_fresh(seconds_since_value: float) -> bool:  # 12, a gate not a Trip
    return seconds_since_value <= LIMITS.freshness_s


# 1 (brake clamp) and 13 (regen-before-stop ordering) are enforced structurally
# by stand.py/session.py, not as standalone check_* functions -- see Steps 4/5.
