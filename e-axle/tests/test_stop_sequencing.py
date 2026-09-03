"""Step 5 ("STOP sequencing -- brake-then-speed, to rest") tests for
`StandSession`.

This is the most safety-critical property in the plan: on `stop()`, brake
current must ramp to zero on both dyno absorbers *before* the drive motor's
speed setpoint begins to fall, so the driveline is never asked to decelerate
against motor torque while regenerative braking is still loading it. The
ordering must hold *by construction* -- an explicit branch on the
freshly-computed brake value, not a threshold/timing comparison that could
happen to pass in a test without actually being enforced.

Built and unit-tested against a hand-rolled `FakeController`/`FakePsu`, passed
into a real `HardwareStand` -- no `instro` import anywhere in this file, per
plan.md's plan-decision box and the pattern established in
test_session.py/test_speed_commanding.py/test_brake_commanding.py.
"""

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from interlocks import LIMITS  # noqa: E402
from session import State, StandSession  # noqa: E402
from stand import HardwareStand  # noqa: E402

TICK_HZ = 50.0
DT = 1.0 / TICK_HZ


class _FixedClock:
    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


class FakeController:
    """Stand-in for `InstroMotorController`. Exposes `get_telemetry()` (for
    the arm precondition), `set_velocity()` (mdc), and `set_brake_current()`
    (dmc_l/dmc_r) -- recording every call of both so tests can assert on the
    exact frame-send order the ordering invariant depends on."""

    def __init__(self, fields: dict | None = None) -> None:
        self._fields = {"erpm": 0.0} if fields is None else fields
        self.velocity_commands: list[float] = []
        self.brake_commands: list[float] = []

    def get_telemetry(self) -> dict:
        return dict(self._fields)

    def set_velocity(self, erpm: float) -> None:
        self.velocity_commands.append(erpm)

    def set_brake_current(self, amps: float) -> None:
        self.brake_commands.append(amps)


class FakePsu:
    def __init__(self) -> None:
        self.output_enabled = False

    def set_output_enabled(self, enabled: bool) -> None:
        self.output_enabled = enabled


def _build_session(
    clock: _FixedClock,
) -> tuple[StandSession, FakeController, FakeController, FakeController]:
    mdc = FakeController()
    dmc_l = FakeController()
    dmc_r = FakeController()
    stand = HardwareStand(
        mdc=mdc, dmc_l=dmc_l, dmc_r=dmc_r, psu=FakePsu(), clock=clock,
    )
    session = StandSession(stand=stand, tick_hz=TICK_HZ)
    session.tick(DT)
    session.set_psu_output_enabled(True)
    session.arm()
    assert session.state is State.ARMED
    return session, mdc, dmc_l, dmc_r


def _run_until_erpm(session: StandSession, clock: _FixedClock, target: float) -> None:
    for _ in range(10_000):
        session.tick(DT)
        clock.t += DT
        if session.commanded_erpm >= target:
            return
    raise AssertionError("never reached target erpm")


def _run_until_brake(session: StandSession, clock: _FixedClock, target: float) -> None:
    for _ in range(10_000):
        session.tick(DT)
        clock.t += DT
        if session.commanded_brake_a >= target:
            return
    raise AssertionError("never reached target brake current")


# ---------------------------------------------------------------------------
# The plan's own literal main scenario.
# ---------------------------------------------------------------------------


def test_stop_ramps_brake_to_zero_before_speed_begins_to_fall():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_session(clock)

    session.set_speed_setpoint_erpm(8000.0)
    _run_until_erpm(session, clock, 8000.0)
    session.set_brake_current_a(20.0)
    _run_until_brake(session, clock, 20.0)

    session.stop()
    assert session.state is State.STOPPING

    speed_started_falling = False
    reached_idle = False
    for _ in range(2000):
        session.tick(DT)
        clock.t += DT
        if session.commanded_erpm < 8000.0:
            speed_started_falling = True
        if speed_started_falling:
            assert session.commanded_brake_a == 0.0, (
                "speed began falling while brake current was still nonzero -- "
                "the brake-then-speed ordering invariant was violated"
            )
        if session.state is State.IDLE:
            reached_idle = True
            break

    assert reached_idle, "session never reached IDLE"
    assert session.commanded_erpm == 0.0
    assert session.commanded_brake_a == 0.0


# ---------------------------------------------------------------------------
# The anti-coincidence test: this must fail if brake and speed were ramped
# down concurrently instead of sequentially. It does not rely on catching a
# threshold crossing -- it asserts, on every single tick, that speed is
# frozen for as long as brake current remains above zero after that tick's
# ramp step. A concurrent (non-gated) implementation would move both values
# down together from the very first STOPPING tick and fail this immediately.
# ---------------------------------------------------------------------------


def test_speed_is_frozen_on_every_tick_while_brake_remains_above_zero():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_session(clock)

    session.set_speed_setpoint_erpm(8000.0)
    _run_until_erpm(session, clock, 8000.0)
    session.set_brake_current_a(20.0)
    _run_until_brake(session, clock, 20.0)

    session.stop()

    for _ in range(2000):
        erpm_before = session.commanded_erpm
        session.tick(DT)
        clock.t += DT
        if session.commanded_brake_a > 0.0:
            assert session.commanded_erpm == erpm_before, (
                "commanded_erpm changed on a tick where brake current was "
                "still above zero after ramping -- brake and speed are being "
                "ramped concurrently, not sequentially"
            )
        else:
            break
    else:
        raise AssertionError("brake current never reached zero")


def test_brake_reaches_zero_and_speed_begins_falling_on_the_same_tick():
    # Corrected per rev-1 plan review finding P1-4: the ordering invariant is
    # carried by frame-send order *within* one tick (brake frame sent before
    # any speed frame that tick), not by gating on a value cached from the
    # previous tick. This pins that the speed ramp is not deferred an extra
    # tick past the one where brake first lands exactly on zero.
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_session(clock)

    session.set_speed_setpoint_erpm(8000.0)
    _run_until_erpm(session, clock, 8000.0)

    # Choose a brake setpoint that reaches exactly zero after precisely two
    # STOPPING ticks: brake_ramp_a_per_s * DT == 0.04 A/tick, so 0.08 A takes
    # exactly two ticks to hit 0.0 with no remainder.
    step = LIMITS.brake_ramp_a_per_s * DT
    session.set_brake_current_a(step * 2)
    _run_until_brake(session, clock, step * 2)

    session.stop()

    session.tick(DT)  # STOPPING tick 1: brake step -> still > 0.0
    clock.t += DT
    assert session.commanded_brake_a > 0.0
    assert session.commanded_erpm == 8000.0, "speed must not move before brake is at rest"

    session.tick(DT)  # STOPPING tick 2: brake reaches exactly 0.0 this tick
    clock.t += DT
    assert session.commanded_brake_a == 0.0
    assert session.commanded_erpm < 8000.0, (
        "speed must begin falling on the SAME tick brake first reaches zero, "
        "not be deferred to a later tick"
    )
    assert dmc_l.brake_commands[-1] == 0.0
    assert dmc_r.brake_commands[-1] == 0.0
    assert mdc.velocity_commands[-1] == session.commanded_erpm


# ---------------------------------------------------------------------------
# Edge cases from plan.md Step 5.
# ---------------------------------------------------------------------------


def test_stop_with_brake_already_zero_starts_speed_ramp_immediately():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_session(clock)

    session.set_speed_setpoint_erpm(8000.0)
    _run_until_erpm(session, clock, 8000.0)
    assert session.commanded_brake_a == 0.0  # never braked

    session.stop()
    session.tick(DT)
    clock.t += DT

    assert session.commanded_erpm < 8000.0, (
        "with brake already at zero, the speed ramp must not wait on a gate "
        "that is already satisfied"
    )


def test_stop_from_armed_never_reached_running_reaches_idle_cleanly():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_session(clock)
    assert session.state is State.ARMED
    assert session.commanded_erpm == 0.0
    assert session.commanded_brake_a == 0.0

    session.stop()
    assert session.state is State.STOPPING

    reached_idle = False
    for _ in range(10):
        session.tick(DT)
        clock.t += DT
        if session.state is State.IDLE:
            reached_idle = True
            break

    assert reached_idle, (
        "stop() from ARMED (speed/brake already at rest) must still reach "
        "IDLE, not get stuck waiting for a speed that was already zero to fall"
    )
    assert session.commanded_erpm == 0.0
    assert session.commanded_brake_a == 0.0
