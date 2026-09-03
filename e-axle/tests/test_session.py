"""Step 2 ("Telemetry drain and the arm precondition") tests for
`session.py`'s `StandSession`.

Built and unit-tested against hand-rolled test doubles -- no `instro` import
anywhere in this file, per plan.md's plan-decision box and Step 2's own note.
Two flavors of double are used, for two different things under test:

  * `FakeController`/`FakePsu`, passed into a real `HardwareStand` (via
    `clock=`), for the plan's own literal main scenario -- this exercises
    `stand.py`'s Step-2-era `get_telemetry()`/`set_psu_output_enabled()`/
    `psu_output_enabled()`/`now` passthrough bodies together with
    `session.py`'s drain/arm logic, end to end.
  * `_ScriptedStand`, a smaller double implementing only the three methods
    `StandSession` itself calls on its stand (`get_telemetry`, `now`,
    `set_psu_output_enabled`), used where a test needs to script an exact
    telemetry shape per tick -- a missing node key, or a node key present but
    mapped to an empty dict -- that `HardwareStand` + `FakeController` would
    not organically produce. `StandSession` never constructs or
    isinstance-checks its `stand`; both doubles are equally valid stand-ins.
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
    """A mutable, manually-advanced clock -- `HardwareStand.clock` and
    `_ScriptedStand.now` both read `.t` through this, so tests can jump the
    clock arbitrarily far without a real sleep."""

    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t


class FakeController:
    """Stand-in for `InstroMotorController`, exposing exactly the one method
    `HardwareStand`'s Step-2-era `get_telemetry()` calls."""

    def __init__(self, fields: dict | None = None) -> None:
        self._fields = {"erpm": 0.0} if fields is None else fields

    def get_telemetry(self) -> dict:
        return dict(self._fields)


class FakePsu:
    """Stand-in for `InstroPSU`, exposing exactly the one method
    `HardwareStand.set_psu_output_enabled()` calls."""

    def __init__(self) -> None:
        self.output_enabled = False

    def set_output_enabled(self, enabled: bool) -> None:
        self.output_enabled = enabled


def _build_session(clock: _FixedClock) -> StandSession:
    stand = HardwareStand(
        mdc=FakeController(),
        dmc_l=FakeController(),
        dmc_r=FakeController(),
        psu=FakePsu(),
        clock=clock,
    )
    return StandSession(stand=stand, tick_hz=TICK_HZ)


class _ScriptedStand:
    """A hand-rolled double exposing exactly the three methods `StandSession`
    calls on its stand -- used instead of `HardwareStand` + `FakeController`
    where a test needs to script precisely what one tick's telemetry drain
    returns (see module docstring)."""

    def __init__(self, clock: _FixedClock) -> None:
        self._clock = clock
        self._script: list[dict] = []

    def queue(self, telemetry: dict) -> None:
        self._script.append(telemetry)

    def get_telemetry(self) -> dict:
        return self._script.pop(0) if self._script else {}

    @property
    def now(self) -> float:
        return self._clock.t

    def set_psu_output_enabled(self, enabled: bool) -> None:
        pass  # no real PSU behind this double; StandSession tracks it itself


# ---------------------------------------------------------------------------
# The plan's own literal main scenario.
# ---------------------------------------------------------------------------


def test_can_arm_is_false_until_every_node_has_produced_at_least_one_telemetry_drain():
    clock = _FixedClock()
    session = _build_session(clock)
    assert not session.can_arm()

    session.tick(DT)  # drains once; FakeController.get_telemetry() returns fields
    session.set_psu_output_enabled(True)
    assert session.can_arm()

    session.arm()
    assert session.state is State.ARMED


# ---------------------------------------------------------------------------
# Edge cases from plan.md Step 2.
# ---------------------------------------------------------------------------


def test_arm_refuses_when_psu_is_still_disabled():
    clock = _FixedClock()
    session = _build_session(clock)

    session.tick(DT)
    assert not session.can_arm()  # telemetry is live but PSU was never enabled

    session.arm()
    assert session.state is State.IDLE


def test_arm_refuses_before_any_tick_even_with_psu_enabled():
    clock = _FixedClock()
    session = _build_session(clock)

    session.set_psu_output_enabled(True)
    assert not session.can_arm()  # no telemetry at all yet -- a different
    # refusal reason than the disabled-PSU case above, same observable result.

    session.arm()
    assert session.state is State.IDLE


def test_disarm_from_armed_returns_to_idle_without_touching_commanded_values():
    clock = _FixedClock()
    session = _build_session(clock)

    session.tick(DT)
    session.set_psu_output_enabled(True)
    session.arm()
    assert session.state is State.ARMED

    session.disarm()
    assert session.state is State.IDLE
    # Nothing has been commanded yet at this point in the test, but a general
    # disarm() should not assume that -- it must not zero or otherwise touch
    # these, only the state.
    assert session.commanded_erpm == 0.0
    assert session.commanded_brake_a == 0.0


def test_missing_node_key_this_tick_does_not_refresh_that_nodes_cached_freshness():
    clock = _FixedClock()
    stand = _ScriptedStand(clock)
    session = StandSession(stand=stand, tick_hz=TICK_HZ)

    stand.queue({"mdc": {"erpm": 0.0}, "dmc_l": {"erpm": 0.0}, "dmc_r": {"erpm": 0.0}})
    session.tick(DT)  # all three nodes cached at t=0.0

    clock.t = LIMITS.staleness_s + 0.1  # far enough that a stale node trips can_arm()
    stand.queue({"mdc": {"erpm": 0.0}})  # dmc_l, dmc_r absent from this tick's mapping
    session.tick(DT)

    session.set_psu_output_enabled(True)
    assert not session.can_arm(), (
        "dmc_l/dmc_r were not keys in this tick's telemetry mapping at all, "
        "so their cached freshness must not have been refreshed by mdc's entry"
    )


def test_empty_node_dict_this_tick_does_not_refresh_that_nodes_cached_freshness():
    clock = _FixedClock()
    stand = _ScriptedStand(clock)
    session = StandSession(stand=stand, tick_hz=TICK_HZ)

    stand.queue({"mdc": {"erpm": 0.0}, "dmc_l": {"erpm": 0.0}, "dmc_r": {"erpm": 0.0}})
    session.tick(DT)

    clock.t = LIMITS.staleness_s + 0.1
    stand.queue({"mdc": {"erpm": 0.0}, "dmc_l": {}, "dmc_r": {"erpm": 0.0}})
    session.tick(DT)

    session.set_psu_output_enabled(True)
    assert not session.can_arm(), (
        "dmc_l's drain returned an empty dict -- design.md: a drain that "
        "returns no fields must not refresh a stale cached value, and must "
        "read identically to that node not having been in the mapping at all"
    )


def test_can_arm_liveness_for_a_node_depends_on_its_freshest_field_not_on_a_field_it_never_produced():
    # Step 2's per-field cache restructuring (design.md §4): a node counts as
    # present-and-fresh if it has at least one cached field, and its MOST
    # RECENT field's timestamp is within LIMITS.staleness_s. mdc here reports
    # "erpm" every tick but never "bus_voltage" -- can_arm()'s liveness for
    # mdc must track erpm's freshness, not treat the never-seen bus_voltage
    # field as missing data that blocks arming.
    clock = _FixedClock()
    stand = _ScriptedStand(clock)
    session = StandSession(stand=stand, tick_hz=TICK_HZ)

    stand.queue({"mdc": {"erpm": 0.0}, "dmc_l": {"erpm": 0.0}, "dmc_r": {"erpm": 0.0}})
    session.tick(DT)

    clock.t = LIMITS.staleness_s / 2.0
    stand.queue({"mdc": {"erpm": 1.0}, "dmc_l": {"erpm": 0.0}, "dmc_r": {"erpm": 0.0}})
    session.tick(DT)  # mdc's "erpm" refreshed again; "bus_voltage" never seen

    session.set_psu_output_enabled(True)
    assert session.can_arm(), (
        "mdc has never produced a bus_voltage field at all, but its erpm "
        "field is fresh -- that alone must be enough for node-level liveness"
    )


def test_can_arm_recovers_once_every_node_is_fresh_again():
    # Companion to the two tests above: confirms _ScriptedStand and can_arm()
    # aren't just permanently False after a partial drain -- a subsequent
    # tick that does refresh every node clears the staleness.
    clock = _FixedClock()
    stand = _ScriptedStand(clock)
    session = StandSession(stand=stand, tick_hz=TICK_HZ)

    stand.queue({"mdc": {"erpm": 0.0}, "dmc_l": {"erpm": 0.0}, "dmc_r": {"erpm": 0.0}})
    session.tick(DT)

    clock.t = LIMITS.staleness_s + 0.1
    stand.queue({"mdc": {"erpm": 0.0}})
    session.tick(DT)
    session.set_psu_output_enabled(True)
    assert not session.can_arm()

    stand.queue({"mdc": {"erpm": 0.0}, "dmc_l": {"erpm": 0.0}, "dmc_r": {"erpm": 0.0}})
    session.tick(DT)
    assert session.can_arm()
