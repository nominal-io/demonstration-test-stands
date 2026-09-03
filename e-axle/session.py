"""The tick, the command/interlock state machine, and the telemetry cache.
Never imports instro; talks only to the injected HardwareStand."""

from enum import StrEnum, auto
from typing import TYPE_CHECKING

from interlocks import (
    LIMITS,
    Trip,
    check_bus_overvoltage,
    check_dropout,
    check_dut_overspeed,
    check_dyno_overspeed,
    check_fet_temperature,
    check_motor_temperature,
    check_speed_tracking_error,
    check_spread,
    check_staleness,
)
from units import dyno_rpm_to_erpm

if TYPE_CHECKING:
    from stand import HardwareStand

_ALL_NODES = ("mdc", "dmc_l", "dmc_r")
_DYNO_NODES = ("dmc_l", "dmc_r")


# The command surface's ceiling -- distinct from LIMITS.dut_overspeed_erpm
# (12_600), which is the interlock trip threshold, not the commandable range.
_MAX_SPEED_SETPOINT_ERPM = 12_000.0


def _ramp_toward(current: float, target: float, max_step: float) -> float:
    """Step from `current` toward `target` by at most `max_step`, clamping at
    `target` rather than overshooting. `max_step` must be >= 0."""
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + max_step * (1.0 if delta > 0.0 else -1.0)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class State(StrEnum):
    IDLE = auto()
    ARMED = auto()
    RUNNING = auto()
    STOPPING = auto()
    TRIPPED = auto()


class StandSession:
    """See module docstring."""

    stand: "HardwareStand"
    state: State
    commanded_erpm: float
    commanded_brake_a: float
    trip: "Trip | None"
    freshness_skips: int

    def __init__(self, stand: "HardwareStand", tick_hz: float) -> None:
        self.stand = stand
        self.tick_hz = tick_hz
        self.state = State.IDLE
        # node -> {field: (value, seen_at)}, merged per drain, never replaced.
        self._node_cache: dict[str, dict[str, tuple[float, float]]] = {}
        self._psu_enabled = False
        self.commanded_erpm = 0.0
        self.commanded_brake_a = 0.0
        self._speed_setpoint_erpm = 0.0
        self._brake_setpoint_a = 0.0
        self.trip = None
        self.freshness_skips = 0
        self._reset_accumulators()

    def can_arm(self) -> bool:
        if not self._psu_enabled:
            return False
        if len(self._node_cache) < 3:
            return False
        # A node is fresh if its MOST RECENT field is within staleness_s --
        # node-level liveness, not field-level completeness.
        fresh = all(
            self.stand.now - max(seen for _value, seen in fields.values())
            <= LIMITS.staleness_s
            for fields in self._node_cache.values()
        )
        if not fresh:
            return False
        # No active trip, and a fresh re-check of rows 1-6 only -- see
        # README.md "Recovery".
        if self.trip is not None:
            return False
        return self._evaluate_instantaneous() is None

    def arm(self) -> None:
        if self.state is State.IDLE and self.can_arm():
            self.state = State.ARMED

    def disarm(self) -> None:
        # Deliberately narrow -- see README.md "disarm() is deliberately narrow".
        if self.state is State.ARMED:
            self.state = State.IDLE

    def set_speed_setpoint_erpm(self, erpm: float) -> None:
        self._speed_setpoint_erpm = _clamp(erpm, 0.0, _MAX_SPEED_SETPOINT_ERPM)

    def set_brake_current_a(self, amps: float) -> None:
        # Clamped at tick() time, not here -- LIMITS.max_brake_a stays the
        # single point of truth the tests exercise.
        self._brake_setpoint_a = amps

    def set_psu_output_enabled(self, enabled: bool) -> None:
        if enabled:
            # Safe direction, unchanged -- see README.md "T-6".
            self.stand.set_psu_output_enabled(True)
            self._psu_enabled = True
            return
        # Disabling never disables the output -- see README.md "T-6". While
        # active, force the same trip action tick() uses; either way, the bus
        # goes to 0V via zero_bus_voltage(), never output_enable(False).
        if self.state in (State.ARMED, State.RUNNING, State.STOPPING):
            self._trip(Trip(reason="psu_disabled", node=None))
        self.stand.zero_bus_voltage()
        self._psu_enabled = False

    def stop(self) -> None:
        if self.state in (State.ARMED, State.RUNNING):
            self.state = State.STOPPING

    def acknowledge_trip(self) -> None:
        # Dismissal only -- does not re-check the fault. See README.md
        # "Recovery".
        if self.state is State.TRIPPED:
            self.trip = None
            self.state = State.IDLE

    def _reset_accumulators(self) -> None:
        # Called on trip and on ordinary STOPPING -> IDLE: a duration
        # accumulator surviving into the next arm could trip for a reason
        # unrelated to the new run.
        self._speed_tracking_accum_s = 0.0
        self._dropout_accum_s: dict[str, float] = {"dmc_l": 0.0, "dmc_r": 0.0}
        self._spread_accum_s = 0.0

    def _node_age(self, node: str) -> float:
        """Seconds since `node`'s most recently updated field. A node with no
        cached fields is treated as infinitely stale, not a separate case."""
        fields = self._node_cache.get(node)
        if not fields:
            return float("inf")
        newest = max(seen for _value, seen in fields.values())
        return self.stand.now - newest

    def _fresh_field(self, node: str, field: str) -> "float | None":
        """Interlock 12's freshness gate -- see README.md "Freshness gating".
        Returns the cached value if fresh, else increments `freshness_skips`
        and returns None so the caller skips that check only, this tick."""
        entry = self._node_cache.get(node, {}).get(field)
        if entry is None:
            self.freshness_skips += 1
            return None
        value, seen_at = entry
        if self.stand.now - seen_at > LIMITS.freshness_s:
            self.freshness_skips += 1
            return None
        return value

    def _evaluate_instantaneous(self) -> "Trip | None":
        """Rows 1-6 (§2): staleness, both overspeeds, overvoltage, both
        temperatures. No accumulator reads or writes -- see README.md
        "Recovery" for why `can_arm()` calls this directly."""

        # 1. Node staleness (interlock 10), all three nodes.
        for node in _ALL_NODES:
            trip = check_staleness(node, self._node_age(node))
            if trip is not None:
                return trip

        # 2. DUT overspeed. mdc is pole_pairs=1, so `velocity` is already ERPM.
        mdc_velocity = self._fresh_field("mdc", "velocity")
        if mdc_velocity is not None:
            trip = check_dut_overspeed(mdc_velocity)
            if trip is not None:
                return trip

        # 3. Dyno overspeed, both absorbers. `velocity` is mechanical RPM
        # (pole_pairs=7); convert to ERPM before comparing.
        for node in _DYNO_NODES:
            velocity = self._fresh_field(node, "velocity")
            if velocity is not None:
                trip = check_dyno_overspeed(node, dyno_rpm_to_erpm(velocity))
                if trip is not None:
                    return trip

        # 4. Bus overvoltage, per node.
        for node in _ALL_NODES:
            bus_voltage = self._fresh_field(node, "bus_voltage")
            if bus_voltage is not None:
                trip = check_bus_overvoltage(node, bus_voltage)
                if trip is not None:
                    return trip

        # 5. FET temperature, per node.
        for node in _ALL_NODES:
            fet_temperature = self._fresh_field(node, "fet_temperature")
            if fet_temperature is not None:
                trip = check_fet_temperature(node, fet_temperature)
                if trip is not None:
                    return trip

        # 6. Motor temperature, per node.
        for node in _ALL_NODES:
            motor_temperature = self._fresh_field(node, "motor_temperature")
            if motor_temperature is not None:
                trip = check_motor_temperature(node, motor_temperature)
                if trip is not None:
                    return trip

        return None

    def _evaluate_trip(self, dt: float) -> "Trip | None":
        """Fixed, first-wins order -- see README.md "Trip evaluation"."""

        trip = self._evaluate_instantaneous()
        if trip is not None:
            return trip

        # 7. DUT speed-tracking error: accumulate dt while diverged, else reset.
        mdc_velocity = self._fresh_field("mdc", "velocity")
        if mdc_velocity is not None:
            tracking_now = (
                abs(self.commanded_erpm - mdc_velocity)
                > LIMITS.speed_tracking_error_erpm
            )
            self._speed_tracking_accum_s = (
                self._speed_tracking_accum_s + dt if tracking_now else 0.0
            )
            trip = check_speed_tracking_error(
                self.commanded_erpm, mdc_velocity, self._speed_tracking_accum_s
            )
            if trip is not None:
                return trip

        # 8. Dyno torque dropout, per absorber. abs(motor_current) is
        # deliberate -- see README.md "Dropout interlock uses unsigned current".
        for node in _DYNO_NODES:
            motor_current = self._fresh_field(node, "motor_current")
            if motor_current is not None:
                commanded = self.commanded_brake_a
                below_now = (
                    commanded >= LIMITS.dropout_min_commanded_a
                    and abs(motor_current) < LIMITS.dropout_fraction * commanded
                )
                self._dropout_accum_s[node] = (
                    self._dropout_accum_s[node] + dt if below_now else 0.0
                )
                trip = check_dropout(
                    node, commanded, abs(motor_current), self._dropout_accum_s[node]
                )
                if trip is not None:
                    return trip

        # 9. Half-shaft spread -- accumulator workaround, see README.md
        # "Trip evaluation" (check_spread's above_floor_for_s paragraph).
        l_velocity = self._fresh_field("dmc_l", "velocity")
        r_velocity = self._fresh_field("dmc_r", "velocity")
        if l_velocity is not None and r_velocity is not None:
            l_erpm = dyno_rpm_to_erpm(l_velocity)
            r_erpm = dyno_rpm_to_erpm(r_velocity)
            floor_basis = max(abs(l_erpm), abs(r_erpm))
            diverged_now = False
            if floor_basis > LIMITS.spread_floor_erpm:
                fraction = abs(l_erpm - r_erpm) / floor_basis
                diverged_now = fraction > LIMITS.spread_fraction
            self._spread_accum_s = (
                self._spread_accum_s + dt if diverged_now else 0.0
            )
            trip = check_spread(l_erpm, r_erpm, self._spread_accum_s)
            if trip is not None:
                return trip

        return None

    def _trip(self, trip: "Trip") -> None:
        """The one trip action -- zero the machine, latch, clear setpoints.
        Shared by tick()'s interlock evaluation and set_psu_output_enabled()'s
        forced trip (README.md "T-6")."""
        self.stand.zero_all()
        self.trip = trip
        self.state = State.TRIPPED
        self.commanded_erpm = self.commanded_brake_a = 0.0
        self._speed_setpoint_erpm = self._brake_setpoint_a = 0.0
        self._reset_accumulators()

    def tick(self, dt: float) -> None:
        telemetry = self.stand.get_telemetry()
        now = self.stand.now
        for node, fields in telemetry.items():
            # Merged per field, never replaced -- a field absent this tick
            # leaves its cached value untouched.
            for field, value in fields.items():
                self._node_cache.setdefault(node, {})[field] = (value, now)

        # Trip evaluation -- see README.md "Trip evaluation". Runs
        # immediately after the drain, before any command; active states
        # only (IDLE has nothing to protect, TRIPPED is already latched).
        if self.state in (State.ARMED, State.RUNNING, State.STOPPING):
            trip = self._evaluate_trip(dt)
            if trip is not None:
                self._trip(trip)
                return

        # Speed commanding: rate-limited ramp, resent unconditionally every
        # tick -- see README.md "Watchdog / command resend".
        if self.state in (State.ARMED, State.RUNNING):
            self.commanded_erpm = _ramp_toward(
                self.commanded_erpm,
                self._speed_setpoint_erpm,
                LIMITS.speed_ramp_erpm_per_s * dt,
            )
            self.stand.set_speed_erpm(self.commanded_erpm)
            if self.commanded_erpm > 0.0:
                self.state = State.RUNNING

            # Brake commanding: clamped to LIMITS.max_brake_a, instant (no
            # ramp-up -- that rate is reserved for STOP's ramp-down below).
            self.commanded_brake_a = _clamp(
                self._brake_setpoint_a, 0.0, LIMITS.max_brake_a
            )
            self.stand.set_brake_current_a(self.commanded_brake_a)

        # STOP sequencing: brake-then-speed, by construction -- see README.md
        # "STOP sequencing: brake before speed".
        if self.state is State.STOPPING:
            self.commanded_brake_a = _ramp_toward(
                self.commanded_brake_a, 0.0, LIMITS.brake_ramp_a_per_s * dt
            )
            self.stand.set_brake_current_a(self.commanded_brake_a)

            if self.commanded_brake_a > 0.0:
                # Resend commanded_erpm UNCHANGED (not a ramp) to feed the
                # watchdog during bleed-off. ⚠️ Needs hardware confirmation
                # -- see README.md "T-5" and TASKS.md.
                self.stand.set_speed_erpm(self.commanded_erpm)
            elif self.commanded_erpm > 0.0:
                self.commanded_erpm = _ramp_toward(
                    self.commanded_erpm, 0.0, LIMITS.speed_ramp_erpm_per_s * dt
                )
                self.stand.set_speed_erpm(self.commanded_erpm)
            else:
                # Both are at rest: brake == 0.0 (checked above) and
                # commanded_erpm == 0.0 (the only way to reach this branch).
                self.state = State.IDLE
                self._reset_accumulators()
