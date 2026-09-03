"""Step 3 ("Trip evaluation and the trip action -- T-1's core") tests for
`StandSession`.

This is the most safety-critical step in the whole plan: `tick()` gains a
trip-evaluation pass, run in ARMED/RUNNING/STOPPING only, in design.md §2's
fixed, first-wins order, immediately after the telemetry drain and before any
command is computed or sent. On the first hit, the trip action runs:
`stand.zero_all()` inline, latch `session.trip`, move to TRIPPED, clear both
the commanded values and the operator's setpoints, reset every duration
accumulator, and return -- no command frame this tick.

Built and unit-tested against hand-rolled `FakeController`/`FakePsu` doubles,
passed into a real `HardwareStand` -- no `instro` import anywhere in this
file, per plan.md's plan-decision box and the pattern established in
test_session.py/test_speed_commanding.py/test_stop_sequencing.py. Unlike
those files' doubles, this one's `fields` dict is publicly mutable per node
per tick, since Step 3's checks are the first code in this project to care
about the *specific* field names telemetry carries (`velocity`,
`bus_voltage`, `fet_temperature`, `motor_temperature`, `motor_current`) --
not just an opaque "the node has some fields."
"""

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from interlocks import LIMITS, Trip  # noqa: E402
from session import State, StandSession  # noqa: E402
from stand import HardwareStand  # noqa: E402

TICK_HZ = 50.0
DT = 1.0 / TICK_HZ

RUN_ERPM = 8000.0
RUN_BRAKE_A = LIMITS.max_brake_a


class _FixedClock:
    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


class FakeController:
    """Stand-in for `InstroMotorController`. `.fields` is a plain, publicly
    mutable dict a test can rewrite between ticks -- e.g. deleting a key
    simulates that field simply not arriving this tick (design.md §4: a
    drain that omits a field must not refresh its cached freshness), which is
    exactly the tool needed to script an individual field going stale while
    its node's other fields keep reporting."""

    def __init__(self, fields: dict) -> None:
        self.fields = dict(fields)
        self.velocity_commands: list[float] = []
        self.brake_commands: list[float] = []
        self.stop_motor_called = False

    def get_telemetry(self) -> dict:
        return dict(self.fields)

    def set_velocity(self, erpm: float) -> None:
        self.velocity_commands.append(erpm)

    def set_brake_current(self, amps: float) -> None:
        self.brake_commands.append(amps)

    def stop_motor(self) -> None:
        self.stop_motor_called = True


class FakePsu:
    def __init__(self) -> None:
        self.output_enabled = False
        self.last_voltage_command: tuple[float, int] | None = None

    def set_output_enabled(self, enabled: bool) -> None:
        self.output_enabled = enabled

    def set_voltage(self, voltage: float, channel: int) -> None:
        self.last_voltage_command = (voltage, channel)


def _healthy_mdc_fields(velocity: float = RUN_ERPM) -> dict:
    # 8000 ERPM, comfortably under the 12_600 ERPM DUT overspeed limit; 48 V,
    # 40 C and 40 C are comfortably under interlocks 6/4/5's thresholds.
    return {
        "velocity": velocity,
        "bus_voltage": 48.0,
        "fet_temperature": 40.0,
        "motor_temperature": 40.0,
    }


def _healthy_dyno_fields(velocity: float = 100.0) -> dict:
    # 100 mechanical RPM -> 700 ERPM (pole_pairs=7): above the 500 ERPM spread
    # floor but comfortably under the 3300 ERPM dyno overspeed limit, and
    # identical on both absorbers so the spread check reads 0 % divergence.
    # motor_current == 20.0 matches RUN_BRAKE_A -- no dropout.
    return {
        "velocity": velocity,
        "bus_voltage": 48.0,
        "fet_temperature": 40.0,
        "motor_temperature": 40.0,
        "motor_current": RUN_BRAKE_A,
    }


def _build_armed_session(clock: _FixedClock):
    """ARMED, at rest -- nothing ramped. Mirrors the plan's own phrasing
    ("session <- ARMED, ...") for the tests that don't need a loaded run."""
    mdc = FakeController(_healthy_mdc_fields(velocity=0.0))
    dmc_l = FakeController(_healthy_dyno_fields())
    dmc_r = FakeController(_healthy_dyno_fields())
    stand = HardwareStand(mdc=mdc, dmc_l=dmc_l, dmc_r=dmc_r, psu=FakePsu(), clock=clock)
    session = StandSession(stand=stand, tick_hz=TICK_HZ)

    session.tick(DT)
    session.set_psu_output_enabled(True)
    session.arm()
    assert session.state is State.ARMED
    return session, mdc, dmc_l, dmc_r


def _build_running_session(clock: _FixedClock):
    """ARMED, then ramped to a real speed and a real (RUN_BRAKE_A) brake
    load -- the shape Act 2 of the feature test itself exercises, and what
    the accumulator-based checks (rows 7-9) need a genuine commanded value to
    diverge from.

    `mdc`'s reported velocity tracks `commanded_erpm` during the ramp itself
    (an obedient plant, same convention as the cleared feature test's
    harness) so that row 7's own new speed-tracking accumulator does not
    mistake a normal, rate-limited ramp-up for a tracking-error fault -- the
    divergence tests below introduce their own fault deliberately, on top of
    this otherwise-obedient baseline.
    """
    mdc = FakeController(_healthy_mdc_fields(velocity=0.0))
    dmc_l = FakeController(_healthy_dyno_fields())
    dmc_r = FakeController(_healthy_dyno_fields())
    stand = HardwareStand(mdc=mdc, dmc_l=dmc_l, dmc_r=dmc_r, psu=FakePsu(), clock=clock)
    session = StandSession(stand=stand, tick_hz=TICK_HZ)

    session.tick(DT)
    clock.t += DT
    session.set_psu_output_enabled(True)
    session.arm()
    assert session.state is State.ARMED

    session.set_speed_setpoint_erpm(RUN_ERPM)
    for _ in range(2000):
        session.tick(DT)
        clock.t += DT
        mdc.fields["velocity"] = session.commanded_erpm
        if session.commanded_erpm >= RUN_ERPM:
            break
    session.set_brake_current_a(RUN_BRAKE_A)
    for _ in range(2000):
        session.tick(DT)
        clock.t += DT
        if session.commanded_brake_a >= RUN_BRAKE_A:
            break
    assert session.state is State.RUNNING
    return session, mdc, dmc_l, dmc_r


# ---------------------------------------------------------------------------
# The plan's own literal main scenario.
# ---------------------------------------------------------------------------


def test_an_instantaneous_overspeed_trips_the_stand_and_zeroes_the_machine():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_armed_session(clock)

    mdc.fields["velocity"] = LIMITS.dut_overspeed_erpm + 500.0
    session.tick(DT)

    assert session.state is State.TRIPPED
    assert session.trip == Trip(reason="dut_overspeed", node="mdc")
    assert mdc.stop_motor_called
    assert dmc_l.brake_commands[-1] == 0.0
    assert dmc_r.brake_commands[-1] == 0.0
    # No ramp, no command frame this tick -- the trip action returns
    # immediately, so nothing beyond the zeroing above should have been sent.
    assert mdc.velocity_commands == []


# ---------------------------------------------------------------------------
# Edge cases from plan.md Step 3.
# ---------------------------------------------------------------------------


def test_staleness_wins_over_overspeed_on_the_same_tick():
    # design.md §2's row order: staleness (row 1) is evaluated before DUT
    # overspeed (row 2). Construct a double where both are true on the same
    # tick and confirm only staleness latches.
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_armed_session(clock)

    dmc_l.fields = {}  # dmc_l stops reporting anything at all, starting now
    clock.t = LIMITS.staleness_s + 0.3  # dmc_l's cache is now well over 0.5s old
    mdc.fields["velocity"] = LIMITS.dut_overspeed_erpm + 500.0  # also true this tick

    session.tick(DT)

    assert session.state is State.TRIPPED
    assert session.trip == Trip(reason="staleness", node="dmc_l")


def test_same_tick_collision_within_the_accumulator_rows_speed_tracking_wins_over_dropout():
    # design.md §2's corrected row order (plan.md's draft-gate Finding 2):
    # speed-tracking (row 7) is evaluated before dropout (row 8). Construct a
    # double where both accumulators cross their (different) duration
    # thresholds on the exact same tick, and confirm row 7 wins.
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_running_session(clock)

    tracking_ticks = round(LIMITS.speed_tracking_duration_s / DT)  # 150
    dropout_ticks = round(LIMITS.dropout_duration_s / DT)  # 15
    assert tracking_ticks > dropout_ticks  # the scenario below depends on this

    # A DUT telemetry reading far enough off commanded_erpm to trip row 7's
    # instantaneous test, held from the very first tick of this loop.
    mdc.fields["velocity"] = RUN_ERPM - (LIMITS.speed_tracking_error_erpm + 1.0)

    dropout_starts_at = tracking_ticks - dropout_ticks + 1  # 1-indexed within loop
    for i in range(1, tracking_ticks + 1):
        if i == dropout_starts_at:
            # A dropout condition that starts late enough to cross its own
            # (shorter) 0.3s threshold on the SAME tick speed-tracking
            # crosses its 3.0s threshold.
            dmc_l.fields["motor_current"] = 0.0
            dmc_r.fields["motor_current"] = 0.0
        if i < tracking_ticks:
            session.tick(DT)
            clock.t += DT
            assert session.state is State.RUNNING, f"tripped early, at tick {i}"
        else:
            session.tick(DT)
            clock.t += DT

    assert session.state is State.TRIPPED
    assert session.trip == Trip(reason="speed_tracking_error", node="mdc")


def test_a_trip_fires_while_stopping_and_abandons_the_graceful_ramp():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_running_session(clock)

    session.stop()
    assert session.state is State.STOPPING

    mdc.fields["velocity"] = LIMITS.dut_overspeed_erpm + 500.0
    session.tick(DT)

    assert session.state is State.TRIPPED
    assert session.trip == Trip(reason="dut_overspeed", node="mdc")
    assert mdc.stop_motor_called
    assert dmc_l.brake_commands[-1] == 0.0
    assert dmc_r.brake_commands[-1] == 0.0


def test_freshness_gate_skips_a_stale_bus_voltage_reading_and_counts_the_skip():
    clock = _FixedClock()
    mdc = FakeController({
        **_healthy_mdc_fields(velocity=0.0),
        "bus_voltage": LIMITS.bus_overvoltage_v + 10.0,  # trip-worthy, but about to go stale
    })
    dmc_l = FakeController(_healthy_dyno_fields())
    dmc_r = FakeController(_healthy_dyno_fields())
    stand = HardwareStand(mdc=mdc, dmc_l=dmc_l, dmc_r=dmc_r, psu=FakePsu(), clock=clock)
    session = StandSession(stand=stand, tick_hz=TICK_HZ)

    session.tick(DT)  # IDLE -- caches bus_voltage at t=0.0; no evaluation runs
    clock.t = LIMITS.freshness_s + 0.05  # stale for interlock 12, still node-fresh
    session.set_psu_output_enabled(True)
    session.arm()
    assert session.state is State.ARMED

    del mdc.fields["bus_voltage"]  # withheld this tick; velocity/temps still refresh
    skips_before = session.freshness_skips
    session.tick(DT)

    assert session.state is State.ARMED, (
        f"a stale bus_voltage reading of {LIMITS.bus_overvoltage_v + 10.0} V "
        "must be skipped, not trip"
    )
    assert session.freshness_skips == skips_before + 1


def test_freshness_gate_skips_a_stale_non_bus_voltage_field_too():
    # The same gate, on a field other than bus_voltage -- this is what pins
    # interlock 12's scope as general over rows 2-9 (design.md §4, plan.md's
    # draft-gate Finding 3), not narrowed to check_bus_overvoltage alone.
    clock = _FixedClock()
    mdc = FakeController({
        **_healthy_mdc_fields(velocity=0.0),
        "fet_temperature": LIMITS.fet_temperature_c + 10.0,  # trip-worthy, about to go stale
    })
    dmc_l = FakeController(_healthy_dyno_fields())
    dmc_r = FakeController(_healthy_dyno_fields())
    stand = HardwareStand(mdc=mdc, dmc_l=dmc_l, dmc_r=dmc_r, psu=FakePsu(), clock=clock)
    session = StandSession(stand=stand, tick_hz=TICK_HZ)

    session.tick(DT)
    clock.t = LIMITS.freshness_s + 0.05
    session.set_psu_output_enabled(True)
    session.arm()
    assert session.state is State.ARMED

    del mdc.fields["fet_temperature"]
    skips_before = session.freshness_skips
    session.tick(DT)

    assert session.state is State.ARMED, (
        f"a stale fet_temperature reading of {LIMITS.fet_temperature_c + 10.0} C "
        "must be skipped, not trip -- interlock 12 is general, not bus-voltage-only"
    )
    assert session.freshness_skips == skips_before + 1


def test_check_spread_workaround_requires_sustained_divergence_not_a_single_sample():
    # design.md Open question 2: check_spread's own duration parameter
    # conflates "time above the floor" with an instantaneous divergence test.
    # The session-side workaround accumulates "time both above the floor AND
    # diverged beyond 15%" -- this pins that composed behaviour directly,
    # independent of the feature test.
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_running_session(clock)

    dmc_l.fields["velocity"] = 100.0  # 700 ERPM
    dmc_r.fields["velocity"] = 70.0  # 490 ERPM -- 30% divergence, above the 500 floor

    spread_ticks = round(LIMITS.spread_duration_s / DT)  # 10, i.e. 0.2s
    for _ in range(spread_ticks - 1):
        session.tick(DT)
        clock.t += DT
        assert session.state is State.RUNNING, (
            "tripped before 0.2s of sustained divergence had elapsed"
        )

    # The accumulator crosses the threshold within a tick or two of the
    # nominal 10 (repeated float addition of DT can land fractionally under
    # 0.2 for one extra tick -- negligible, ~20ms, at a 50 Hz tick rate; the
    # loop tolerates it rather than asserting a razor-exact tick count).
    for _ in range(3):
        session.tick(DT)
        clock.t += DT
        if session.state is State.TRIPPED:
            break
    assert session.state is State.TRIPPED
    assert session.trip == Trip(reason="spread", node=None)


def test_accumulators_reset_on_the_stopping_to_idle_transition_not_only_on_trip():
    # design.md's own reasoning for this rule: the STOPPING -> IDLE
    # transition is the one place a duration-accumulator's underlying
    # condition can still be true on the very last evaluated tick (spread
    # depends only on dyno velocities, never on the ramping commanded
    # values) with no *subsequent* evaluated tick to naturally reset it --
    # IDLE does not evaluate at all. Without an explicit reset here, that
    # leftover value could combine with a fresh condition on the very next
    # arm and trip for a reason that has nothing to do with the new run.
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_armed_session(clock)

    dmc_l.fields["velocity"] = 100.0  # 700 ERPM
    dmc_r.fields["velocity"] = 70.0  # 490 ERPM -- 30% divergence, above the floor

    spread_ticks = round(LIMITS.spread_duration_s / DT)  # 10
    almost = spread_ticks - 4  # comfortably short of tripping (6 ticks -> 0.12s)
    for _ in range(almost):
        session.tick(DT)
        clock.t += DT
        assert session.state is State.ARMED

    # Divergence is still live on this very tick, which is also the tick
    # STOPPING collapses straight to IDLE on (both commanded values are
    # already at rest -- neither was ever ramped up in _build_armed_session).
    session.stop()
    assert session.state is State.STOPPING
    session.tick(DT)
    clock.t += DT
    assert session.state is State.IDLE, "expected STOPPING to collapse straight to IDLE"

    session.arm()
    assert session.state is State.ARMED

    # Divergence is STILL live. If the accumulator had survived the STOPPING
    # -> IDLE transition (instead of being explicitly reset), this many more
    # ticks would already exceed the 0.2s threshold and trip.
    for _ in range(almost - 1):
        session.tick(DT)
        clock.t += DT
        assert session.state is State.ARMED, (
            "the spread accumulator was not reset on the STOPPING -> IDLE "
            "transition -- a leftover value from the finished run tripped "
            "the stand almost immediately on the new one"
        )

    # ...and it still works: enough further ticks with the same live
    # divergence does eventually trip, proving the reset did not just freeze
    # the accumulator forever.
    for _ in range(spread_ticks):
        session.tick(DT)
        clock.t += DT
        if session.state is State.TRIPPED:
            break
    assert session.state is State.TRIPPED
    assert session.trip == Trip(reason="spread", node=None)


def test_zero_all_zeroes_both_absorbers_before_releasing_the_drive():
    # design.md §5: zero_all()'s internal order is reversed relative to its
    # current implementation -- absorbers first, then the drive -- the same
    # brake-before-speed invariant T-5 protects, applied to the trip path.
    # A direct HardwareStand-level test with hand-rolled controllers
    # recording call order, in the ordering-by-construction style
    # test_stop_sequencing.py already established.
    calls: list[tuple[str, str]] = []

    class _RecordingController:
        def __init__(self, name: str) -> None:
            self.name = name

        def set_brake_current(self, amps: float) -> None:
            calls.append((self.name, "set_brake_current"))

        def stop_motor(self) -> None:
            calls.append((self.name, "stop_motor"))

    stand = HardwareStand(
        mdc=_RecordingController("mdc"),
        dmc_l=_RecordingController("dmc_l"),
        dmc_r=_RecordingController("dmc_r"),
        psu=FakePsu(),
        clock=_FixedClock(),
    )

    stand.zero_all()

    assert calls == [
        ("dmc_l", "set_brake_current"),
        ("dmc_r", "set_brake_current"),
        ("mdc", "stop_motor"),
    ], (
        "zero_all() must release both absorbers' brake current before "
        f"stopping the drive motor; got {calls}"
    )


# ---------------------------------------------------------------------------
# Step 5 (T-6): PSU-disable-while-active forces a trip; the disable path
# never touches output_enable, only zero_bus_voltage().
# ---------------------------------------------------------------------------


def _build_psu_session(clock: _FixedClock):
    """ARMED, at rest, PSU enabled -- also returns the FakePsu double so Step
    5 tests can inspect its set_voltage()/output_enabled directly."""
    mdc = FakeController(_healthy_mdc_fields(velocity=0.0))
    dmc_l = FakeController(_healthy_dyno_fields())
    dmc_r = FakeController(_healthy_dyno_fields())
    psu = FakePsu()
    stand = HardwareStand(mdc=mdc, dmc_l=dmc_l, dmc_r=dmc_r, psu=psu, clock=clock)
    session = StandSession(stand=stand, tick_hz=TICK_HZ)

    session.tick(DT)
    session.set_psu_output_enabled(True)
    session.arm()
    assert session.state is State.ARMED
    return session, mdc, dmc_l, dmc_r, psu


def _ramp_to_running(session, mdc, clock: _FixedClock) -> None:
    """Ramps an ARMED session up to RUNNING with a genuine RUN_BRAKE_A load --
    the state Act 3 of the feature test drives set_psu_output_enabled(False)
    from, so a trip fired from rest alone wouldn't be testing the same thing."""
    session.set_speed_setpoint_erpm(RUN_ERPM)
    for _ in range(2000):
        session.tick(DT)
        clock.t += DT
        mdc.fields["velocity"] = session.commanded_erpm
        if session.commanded_erpm >= RUN_ERPM:
            break
    session.set_brake_current_a(RUN_BRAKE_A)
    for _ in range(2000):
        session.tick(DT)
        clock.t += DT
        if session.commanded_brake_a >= RUN_BRAKE_A:
            break
    assert session.state is State.RUNNING


def test_disabling_the_psu_while_running_forces_a_trip_and_never_disables_output():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r, psu = _build_psu_session(clock)
    _ramp_to_running(session, mdc, clock)
    assert session.commanded_brake_a == RUN_BRAKE_A  # genuinely loaded, not already at rest

    session.set_psu_output_enabled(False)

    assert session.state is State.TRIPPED
    assert session.trip == Trip(reason="psu_disabled", node=None)
    assert psu.output_enabled is True, "output_enable(False) must never be called"
    assert psu.last_voltage_command == (0.0, 1)
    assert dmc_l.brake_commands[-1] == 0.0
    assert dmc_r.brake_commands[-1] == 0.0
    assert mdc.stop_motor_called


def test_disabling_the_psu_while_idle_does_not_trip_but_still_zeroes_the_bus():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r, psu = _build_psu_session(clock)
    session.disarm()
    assert session.state is State.IDLE

    session.set_psu_output_enabled(False)

    assert session.state is State.IDLE
    assert session.trip is None
    assert psu.output_enabled is True
    assert psu.last_voltage_command == (0.0, 1)
    assert not session.can_arm(), "_psu_enabled must still go False from IDLE"


def test_disabling_the_psu_while_already_tripped_stays_tripped_and_still_zeroes_the_bus():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r, psu = _build_psu_session(clock)
    mdc.fields["velocity"] = LIMITS.dut_overspeed_erpm + 500.0
    session.tick(DT)
    assert session.state is State.TRIPPED
    original_trip = session.trip

    session.set_psu_output_enabled(False)

    assert session.state is State.TRIPPED
    assert session.trip == original_trip, "the trip action must not re-fire"
    assert psu.output_enabled is True
    assert psu.last_voltage_command == (0.0, 1)


def test_enabling_the_psu_after_a_forced_trip_neither_clears_it_nor_restores_voltage():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r, psu = _build_psu_session(clock)
    _ramp_to_running(session, mdc, clock)
    session.set_psu_output_enabled(False)
    assert session.state is State.TRIPPED

    session.set_psu_output_enabled(True)

    assert session.state is State.TRIPPED, "enabling the PSU must not clear a latched trip"
    assert session.trip is not None
    assert psu.output_enabled is True
    assert psu.last_voltage_command == (0.0, 1), "enabling never re-commands voltage"


def test_the_machine_is_zeroed_before_the_supply_is_commanded_to_zero_volts():
    # Step 3's zero_all() ordering, exercised through this new call site
    # rather than re-implemented -- see the feature test's Act 3.
    calls: list[str] = []

    class _OrderedController(FakeController):
        def set_brake_current(self, amps: float) -> None:
            calls.append(f"{self._name}.set_brake_current")
            super().set_brake_current(amps)

        def stop_motor(self) -> None:
            calls.append(f"{self._name}.stop_motor")
            super().stop_motor()

    class _OrderedPsu(FakePsu):
        def set_voltage(self, voltage: float, channel: int) -> None:
            calls.append("psu.set_voltage")
            super().set_voltage(voltage, channel)

    clock = _FixedClock()
    mdc = _OrderedController(_healthy_mdc_fields(velocity=0.0))
    mdc._name = "mdc"
    dmc_l = _OrderedController(_healthy_dyno_fields())
    dmc_l._name = "dmc_l"
    dmc_r = _OrderedController(_healthy_dyno_fields())
    dmc_r._name = "dmc_r"
    psu = _OrderedPsu()
    stand = HardwareStand(mdc=mdc, dmc_l=dmc_l, dmc_r=dmc_r, psu=psu, clock=clock)
    session = StandSession(stand=stand, tick_hz=TICK_HZ)

    session.tick(DT)
    session.set_psu_output_enabled(True)
    session.arm()
    _ramp_to_running(session, mdc, clock)
    calls.clear()  # only the disable call's own ordering matters here

    session.set_psu_output_enabled(False)

    assert calls == [
        "dmc_l.set_brake_current",
        "dmc_r.set_brake_current",
        "mdc.stop_motor",
        "psu.set_voltage",
    ], f"the machine must be zeroed before the supply is commanded to 0V; got {calls}"
