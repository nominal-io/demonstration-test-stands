"""Step 3 ("Speed commanding") tests for `StandSession`'s rate-limited speed
ramp and unconditional per-tick resend.

This is the mechanism that satisfies the watchdog-hold safety property: the
VESC6 firmware auto-releases the motor ~1000ms after the last frame it
receives (design.md, "Firmware watchdog: 1000ms since last command"), and
Instro exposes no periodic-transmit facility of its own (design.md: "Instro
offers no periodic transmit, so this property is structural rather than a
choice that could be accidentally reversed"). So `StandSession.tick()` must
resend the commanded speed -- unconditionally, every tick, whether or not the
operator changed anything -- for as long as the session is armed/running.

Built and unit-tested against a hand-rolled `FakeController`/`FakePsu`, passed
into a real `HardwareStand` -- no `instro` import anywhere in this file, per
plan.md's plan-decision box and the pattern established in test_session.py.
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
    the arm precondition) and `set_velocity()` (what `HardwareStand.
    set_speed_erpm()` is specified to call on the `mdc` controller,
    plan.md Step 0) -- and records every call to the latter so tests can
    assert on the resend cadence directly, without needing a CAN bus double."""

    def __init__(self, fields: dict | None = None) -> None:
        self._fields = {"erpm": 0.0} if fields is None else fields
        self.velocity_commands: list[float] = []

    def get_telemetry(self) -> dict:
        return dict(self._fields)

    def set_velocity(self, erpm: float) -> None:
        self.velocity_commands.append(erpm)

    def set_brake_current(self, amps: float) -> None:
        # tick() unconditionally resends the brake setpoint too (Step 4) --
        # this double stands in as `mdc` in some tests and as `dmc_l`/
        # `dmc_r` in others, so it must tolerate both calls even though this
        # file only asserts on velocity_commands.
        pass


class FakePsu:
    def __init__(self) -> None:
        self.output_enabled = False

    def set_output_enabled(self, enabled: bool) -> None:
        self.output_enabled = enabled


def _build_armed_session(clock: _FixedClock) -> tuple[StandSession, FakeController]:
    mdc = FakeController()
    stand = HardwareStand(
        mdc=mdc, dmc_l=FakeController(), dmc_r=FakeController(), psu=FakePsu(),
        clock=clock,
    )
    session = StandSession(stand=stand, tick_hz=TICK_HZ)
    session.tick(DT)
    session.set_psu_output_enabled(True)
    session.arm()
    assert session.state is State.ARMED
    return session, mdc


# ---------------------------------------------------------------------------
# The plan's own literal main scenarios.
# ---------------------------------------------------------------------------


def test_held_setpoint_is_resent_every_tick_never_silently_skipped():
    clock = _FixedClock()
    session, mdc = _build_armed_session(clock)

    session.set_speed_setpoint_erpm(1000.0)
    for _ in range(200):  # far more ticks than needed to reach target
        session.tick(DT)
        clock.t += DT

    assert len(mdc.velocity_commands) >= 200, (
        "the drive motor's speed frame must be resent unconditionally every "
        "tick -- the controllers' 1000ms watchdog auto-releases the motor "
        "otherwise, and Instro has no periodic-transmit facility of its own"
    )
    assert session.commanded_erpm == 1000.0
    assert mdc.velocity_commands[-1] == 1000.0


def test_ramp_rate_is_bounded_at_1000_erpm_per_second():
    # The feature test cannot check the instantaneous ramp rate directly (it
    # only observes frames on the bus); this pins the rate at the unit level.
    clock = _FixedClock()
    session, mdc = _build_armed_session(clock)

    session.set_speed_setpoint_erpm(12_000.0)
    session.tick(DT)  # one tick, 0.02s
    assert session.commanded_erpm <= LIMITS.speed_ramp_erpm_per_s * DT + 1e-9
    assert mdc.velocity_commands[-1] == session.commanded_erpm


# ---------------------------------------------------------------------------
# Edge cases from plan.md Step 3.
# ---------------------------------------------------------------------------


def test_ramp_moves_downward_at_the_same_bounded_rate_when_setpoint_drops():
    clock = _FixedClock()
    session, mdc = _build_armed_session(clock)

    session.set_speed_setpoint_erpm(1000.0)
    for _ in range(100):
        session.tick(DT)
        clock.t += DT
    assert session.commanded_erpm == 1000.0

    session.set_speed_setpoint_erpm(200.0)
    session.tick(DT)
    clock.t += DT
    expected_step = LIMITS.speed_ramp_erpm_per_s * DT
    assert session.commanded_erpm == 1000.0 - expected_step
    assert mdc.velocity_commands[-1] == session.commanded_erpm

    for _ in range(100):
        session.tick(DT)
        clock.t += DT
    assert session.commanded_erpm == 200.0, "ramp must reach the lower setpoint, not stall"


def test_ramp_clamps_exactly_at_target_on_the_tick_that_would_overshoot():
    clock = _FixedClock()
    session, mdc = _build_armed_session(clock)

    session.set_speed_setpoint_erpm(5.0)  # far less than one tick's max step
    session.tick(DT)
    assert session.commanded_erpm == 5.0, (
        "must clamp exactly at the target, not step past it and rely on a "
        "later correction"
    )
    assert mdc.velocity_commands[-1] == 5.0

    # Holding afterward must not oscillate back off the target.
    session.tick(DT)
    assert session.commanded_erpm == 5.0


def test_tick_while_idle_never_commands_a_speed_onto_the_wire():
    clock = _FixedClock()
    mdc = FakeController()
    stand = HardwareStand(
        mdc=mdc, dmc_l=FakeController(), dmc_r=FakeController(), psu=FakePsu(),
        clock=clock,
    )
    session = StandSession(stand=stand, tick_hz=TICK_HZ)
    assert session.state is State.IDLE

    session.set_speed_setpoint_erpm(1000.0)
    for _ in range(10):
        session.tick(DT)
        clock.t += DT

    assert session.state is State.IDLE
    assert mdc.velocity_commands == [], (
        "a setpoint requested before arm() must not reach the wire while the "
        "session is IDLE, armed or not"
    )
    assert session.commanded_erpm == 0.0


def test_setpoint_above_the_command_surface_ceiling_is_clamped_not_passed_through():
    clock = _FixedClock()
    session, mdc = _build_armed_session(clock)

    session.set_speed_setpoint_erpm(50_000.0)
    for _ in range(700):  # 12_000 ERPM / (1000 ERPM/s * DT) ~= 600 ticks
        session.tick(DT)
        clock.t += DT

    assert session.commanded_erpm == 12_000.0
    assert mdc.velocity_commands[-1] == 12_000.0


def test_setpoint_exactly_at_the_ceiling_passes_through_unclamped():
    clock = _FixedClock()
    session, mdc = _build_armed_session(clock)

    session.set_speed_setpoint_erpm(12_000.0)
    for _ in range(700):  # 12_000 ERPM / (1000 ERPM/s * DT) ~= 600 ticks
        session.tick(DT)
        clock.t += DT

    assert session.commanded_erpm == 12_000.0


def test_state_transitions_to_running_once_commanded_speed_is_above_zero():
    clock = _FixedClock()
    session, mdc = _build_armed_session(clock)
    assert session.state is State.ARMED

    session.set_speed_setpoint_erpm(1000.0)
    session.tick(DT)
    clock.t += DT

    assert session.commanded_erpm > 0.0
    assert session.state is State.RUNNING
