"""Step 4 ("Recovery -- acknowledge_trip() and can_arm()'s no-active-trip
gate") tests for `StandSession`.

Built on the same FakeController/FakePsu + HardwareStand doubles as
test_trip_wiring.py (no `instro` import) -- kept local to this file, per the
project's existing convention of not sharing test doubles across files.
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
    def __init__(self, fields: dict) -> None:
        self.fields = dict(fields)

    def get_telemetry(self) -> dict:
        return dict(self.fields)

    def set_velocity(self, erpm: float) -> None:
        pass

    def set_brake_current(self, amps: float) -> None:
        pass

    def stop_motor(self) -> None:
        pass


class FakePsu:
    def __init__(self) -> None:
        self.output_enabled = False

    def set_output_enabled(self, enabled: bool) -> None:
        self.output_enabled = enabled


def _healthy_mdc_fields(velocity: float = 0.0) -> dict:
    return {
        "velocity": velocity,
        "bus_voltage": 48.0,
        "fet_temperature": 40.0,
        "motor_temperature": 40.0,
    }


def _healthy_dyno_fields(velocity: float = 100.0) -> dict:
    return {
        "velocity": velocity,
        "bus_voltage": 48.0,
        "fet_temperature": 40.0,
        "motor_temperature": 40.0,
        "motor_current": 0.0,
    }


def _build_idle_session(clock: _FixedClock) -> StandSession:
    mdc = FakeController(_healthy_mdc_fields())
    dmc_l = FakeController(_healthy_dyno_fields())
    dmc_r = FakeController(_healthy_dyno_fields())
    stand = HardwareStand(mdc=mdc, dmc_l=dmc_l, dmc_r=dmc_r, psu=FakePsu(), clock=clock)
    return StandSession(stand=stand, tick_hz=TICK_HZ)


def _build_tripped_session(clock: _FixedClock):
    """ARMED at rest, then an instantaneous DUT overspeed trips it on one
    tick -- the same shape as test_trip_wiring.py's own main-scenario test."""
    mdc = FakeController(_healthy_mdc_fields(velocity=0.0))
    dmc_l = FakeController(_healthy_dyno_fields())
    dmc_r = FakeController(_healthy_dyno_fields())
    stand = HardwareStand(mdc=mdc, dmc_l=dmc_l, dmc_r=dmc_r, psu=FakePsu(), clock=clock)
    session = StandSession(stand=stand, tick_hz=TICK_HZ)

    session.tick(DT)
    session.set_psu_output_enabled(True)
    session.arm()
    assert session.state is State.ARMED

    mdc.fields["velocity"] = LIMITS.dut_overspeed_erpm + 500.0
    session.tick(DT)
    assert session.state is State.TRIPPED
    return session, mdc, dmc_l, dmc_r


# ---------------------------------------------------------------------------
# The plan's own literal main scenario.
# ---------------------------------------------------------------------------


def test_acknowledge_trip_clears_the_latch_and_returns_to_idle():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_tripped_session(clock)

    session.acknowledge_trip()

    assert session.state is State.IDLE
    assert session.trip is None


# ---------------------------------------------------------------------------
# Edge cases from plan.md Step 4.
# ---------------------------------------------------------------------------


def test_acknowledge_trip_is_a_no_op_when_not_tripped():
    # Mirrors the existing no-op-guard style of arm()/disarm()/stop().
    clock = _FixedClock()
    session = _build_idle_session(clock)

    session.acknowledge_trip()
    assert session.state is State.IDLE

    session.tick(DT)
    session.set_psu_output_enabled(True)
    session.arm()
    assert session.state is State.ARMED

    session.acknowledge_trip()
    assert session.state is State.ARMED, "acknowledge_trip() only fires from TRIPPED"


def test_can_arm_stays_false_immediately_after_acknowledging_while_the_fault_persists():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_tripped_session(clock)

    session.acknowledge_trip()
    assert session.state is State.IDLE

    assert not session.can_arm(), (
        "mdc is still reporting an overspeed velocity in the cache -- "
        "acknowledging dismisses the banner, not the fault"
    )


def test_can_arm_recovers_once_a_later_ticks_drain_shows_the_fault_has_cleared():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_tripped_session(clock)
    session.acknowledge_trip()
    assert not session.can_arm()

    mdc.fields["velocity"] = 0.0
    session.tick(DT)  # IDLE -- drains telemetry only, no trip evaluation runs

    assert session.can_arm(), (
        "can_arm() must recover on its own once the cache shows the fault "
        "gone, with no arm() call needed to observe it"
    )


def test_can_arm_re_check_does_not_mutate_trip_state_or_accumulators():
    clock = _FixedClock()
    session, mdc, dmc_l, dmc_r = _build_tripped_session(clock)
    session.acknowledge_trip()

    mdc.fields["velocity"] = 0.0
    session.tick(DT)
    assert session.can_arm()

    # Calling can_arm() repeatedly must not itself latch a trip or nudge any
    # duration accumulator -- it only evaluates rows 1-6, which read no
    # accumulator. If a future change routed this through the full
    # accumulator-bearing evaluation instead, repeated calls here would
    # eventually flip state or latch a trip on their own.
    for _ in range(50):
        assert session.can_arm()
    assert session.trip is None
    assert session.state is State.IDLE

    session.arm()
    assert session.state is State.ARMED

    session.tick(DT)
    assert session.state is State.ARMED, (
        "re-arming immediately re-tripped -- can_arm()'s repeated calls must "
        "have silently advanced a duration accumulator"
    )
