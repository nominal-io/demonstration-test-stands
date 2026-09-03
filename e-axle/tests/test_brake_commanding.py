"""Step 4 ("Brake commanding") tests for `StandSession`'s clamped, ganged
brake setter.

This is the safety-adjacent clamp: `LIMITS.max_brake_a` (20.0 A) is a real
current limit on real hardware, and the boundary is deliberately pinned by a
test (request exactly at the limit passes through unchanged; anything above
it is clamped down to it -- the clamp's upper bound is inclusive).

Per plan.md's Step 4 ("Note carried from the map ... This step chooses
instant-clamp-and-hold as the simpler option") and Step 4's own test
("session.set_brake_current_a(25.0); session.tick(1/50); assert
session.commanded_brake_a == LIMITS.max_brake_a" -- reachable in a single
0.02s tick, which a 2 A/s ramp could not do from 0.0), brake commanding here
is applied instantly, with no ramp-up: unlike Step 5's STOP path (which does
ramp brake down at LIMITS.brake_ramp_a_per_s), this step's setter clamps and
holds in one tick. See this step's report for the tension between this
default and the human comment recorded in plan.md's "Risks and open
questions" item 2 ("we should try to put limits on deceleration ... don't
want to hammer the driveline") -- that comment is not treated as silently
overriding Step 4's own test/spec above.

Built and unit-tested against a hand-rolled `FakeController`/`FakePsu`,
passed into a real `HardwareStand` -- no `instro` import anywhere in this
file, per plan.md's plan-decision box and the pattern established in
test_session.py/test_speed_commanding.py.
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
    the arm precondition) and `set_brake_current()` (what `HardwareStand.
    set_brake_current_a()` is specified to call on each dyno absorber,
    plan.md Step 0) -- and records every call so tests can assert on both
    the clamp and the per-tick resend cadence directly."""

    def __init__(self, fields: dict | None = None) -> None:
        self._fields = {"erpm": 0.0} if fields is None else fields
        self.brake_commands: list[float] = []

    def get_telemetry(self) -> dict:
        return dict(self._fields)

    def set_brake_current(self, amps: float) -> None:
        self.brake_commands.append(amps)

    def set_velocity(self, erpm: float) -> None:
        # tick() unconditionally resends the speed setpoint too (Step 3) --
        # this double stands in as `mdc` in some tests and as `dmc_l`/`dmc_r`
        # in others, so it must tolerate both calls even though this file
        # only asserts on brake_commands.
        pass


class FakePsu:
    def __init__(self) -> None:
        self.output_enabled = False

    def set_output_enabled(self, enabled: bool) -> None:
        self.output_enabled = enabled


def _build_armed_session(
    clock: _FixedClock,
) -> tuple[StandSession, FakeController, FakeController]:
    dmc_l = FakeController()
    dmc_r = FakeController()
    stand = HardwareStand(
        mdc=FakeController(), dmc_l=dmc_l, dmc_r=dmc_r, psu=FakePsu(), clock=clock,
    )
    session = StandSession(stand=stand, tick_hz=TICK_HZ)
    session.tick(DT)
    session.set_psu_output_enabled(True)
    session.arm()
    assert session.state is State.ARMED
    return session, dmc_l, dmc_r


# ---------------------------------------------------------------------------
# The plan's own literal main scenario.
# ---------------------------------------------------------------------------


def test_requested_brake_current_above_the_limit_is_clamped_to_it_every_frame():
    clock = _FixedClock()
    session, dmc_l, dmc_r = _build_armed_session(clock)

    session.set_brake_current_a(25.0)
    session.tick(DT)

    assert session.commanded_brake_a == LIMITS.max_brake_a
    assert dmc_l.brake_commands[-1] == 20.0
    assert dmc_r.brake_commands[-1] == 20.0


# ---------------------------------------------------------------------------
# Edge cases from plan.md Step 4.
# ---------------------------------------------------------------------------


def test_negative_brake_request_clamps_to_zero_never_sent_negative():
    clock = _FixedClock()
    session, dmc_l, dmc_r = _build_armed_session(clock)

    session.set_brake_current_a(-5.0)
    session.tick(DT)

    assert session.commanded_brake_a == 0.0
    assert dmc_l.brake_commands[-1] == 0.0
    assert dmc_r.brake_commands[-1] == 0.0
    assert all(a >= 0.0 for a in dmc_l.brake_commands)
    assert all(a >= 0.0 for a in dmc_r.brake_commands)


def test_brake_request_exactly_at_the_limit_passes_through_unclamped():
    # A test that only checked the above-limit case could pass with an
    # off-by-one clamp boundary; this pins the limit itself as inclusive.
    clock = _FixedClock()
    session, dmc_l, dmc_r = _build_armed_session(clock)

    session.set_brake_current_a(20.0)
    session.tick(DT)

    assert session.commanded_brake_a == 20.0
    assert dmc_l.brake_commands[-1] == 20.0
    assert dmc_r.brake_commands[-1] == 20.0


def test_dmc_l_and_dmc_r_receive_the_same_commanded_value_every_tick():
    clock = _FixedClock()
    session, dmc_l, dmc_r = _build_armed_session(clock)

    session.set_brake_current_a(12.5)
    for _ in range(5):
        session.tick(DT)
        clock.t += DT

    assert len(dmc_l.brake_commands) == len(dmc_r.brake_commands)
    assert dmc_l.brake_commands == dmc_r.brake_commands
    assert dmc_l.brake_commands[-1] == 12.5


def test_brake_commanded_while_idle_never_reaches_the_wire():
    clock = _FixedClock()
    dmc_l = FakeController()
    dmc_r = FakeController()
    stand = HardwareStand(
        mdc=FakeController(), dmc_l=dmc_l, dmc_r=dmc_r, psu=FakePsu(), clock=clock,
    )
    session = StandSession(stand=stand, tick_hz=TICK_HZ)
    assert session.state is State.IDLE

    session.set_brake_current_a(10.0)
    for _ in range(10):
        session.tick(DT)
        clock.t += DT

    assert session.state is State.IDLE
    assert dmc_l.brake_commands == [], (
        "a brake request made before arm() must not reach the wire while the "
        "session is IDLE"
    )
    assert dmc_r.brake_commands == []
    assert session.commanded_brake_a == 0.0


def test_held_brake_current_is_resent_every_tick_never_silently_skipped():
    # Same watchdog-hold safety property Step 3 established for speed: the
    # controllers' firmware auto-releases ~1000ms after the last frame, and
    # Instro has no periodic-transmit facility of its own, so this per-tick
    # resend is the mechanism that holds the brake command against it.
    clock = _FixedClock()
    session, dmc_l, dmc_r = _build_armed_session(clock)

    session.set_brake_current_a(10.0)
    for _ in range(200):
        session.tick(DT)
        clock.t += DT

    assert len(dmc_l.brake_commands) >= 200
    assert len(dmc_r.brake_commands) >= 200
    assert session.commanded_brake_a == 10.0
    assert dmc_l.brake_commands[-1] == 10.0
    assert dmc_r.brake_commands[-1] == 10.0
