"""Step 0 ("API surface area") tests.

Step 0 builds stub types/protocols/skeletons only -- no plan-step behavior is
implemented yet (that starts at Step 1). What Step 0 *does* commit to, for
real, and what this file exercises:

  * `interlocks.py`'s `Limits`/`LIMITS`/`Trip` are real (frozen, non-stub)
    value objects transcribed from the commissioned machine -- not stubs.
  * `session.py`'s `State` enum is a real (non-stub) discriminated union of
    lifecycle phases.
  * The architectural bet the whole plan leans on (plan.md's plan-decision
    box, confirmed by the human 2026-08-27): `session.py` and `stand.py` hold
    their only Instro-naming references under `TYPE_CHECKING` and are
    importable with `instro` entirely unavailable. This is verified
    behaviorally here (an import-blocking meta path finder), not by a
    grep/substring check -- see plan.md's Step 1 note (rev-1 plan review
    finding P1-2) on why a source-text search is a weak, gameable proxy for
    this property.

Everything else declared in interlocks.py/units.py/session.py/stand.py at this
step is an intentional stub (`...` body) with no behavior yet; this file does
not assert anything about what those stub bodies return. Steps 1-6 each add
their own tests as those stubs are replaced with real implementations.
"""

import ast
import importlib
import inspect
import sys
from contextlib import contextmanager
from dataclasses import FrozenInstanceError
from pathlib import Path
from typing import Iterator

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import interlocks  # noqa: E402
import session as _session_module  # noqa: E402,F401
import stand  # noqa: E402,F401
import units  # noqa: E402
from interlocks import LIMITS, Limits, Trip  # noqa: E402
from session import State, StandSession  # noqa: E402
from stand import HardwareStand  # noqa: E402


class _BlockInstroFinder:
    """A meta path finder installed ahead of the real one that raises
    ImportError for "instro" and everything under it -- the same technique
    plan.md's Step 1 specifies for interlocks.py/units.py, applied here to
    session.py/stand.py, since keeping *those* two importable without instro
    is this step's own architectural bet (plan.md's plan-decision box)."""

    def find_spec(self, fullname, path, target=None):
        if fullname == "instro" or fullname.startswith("instro."):
            raise ImportError(f"blocked for test: {fullname}")
        return None


@contextmanager
def _blocked_instro(*module_names: str) -> Iterator[None]:
    """Remove `module_names` from sys.modules, block any import of `instro`
    or a submodule of it, and restore both on exit -- so each use gets a
    genuinely fresh re-execution of the module's top-level code, and other
    tests are not left with a module object imported under the block."""
    saved = {name: sys.modules.get(name) for name in module_names}
    finder = _BlockInstroFinder()
    sys.meta_path.insert(0, finder)
    for name in module_names:
        sys.modules.pop(name, None)
    try:
        yield
    finally:
        sys.meta_path.remove(finder)
        for name in module_names:
            sys.modules.pop(name, None)
            if saved[name] is not None:
                # Restore exactly the module object callers had before.
                sys.modules[name] = saved[name]
            else:
                # Nothing to restore -- leave a normally-imported copy behind
                # for anything that runs after this test.
                importlib.import_module(name)


def test_stand_and_session_import_successfully_with_instro_unavailable():
    with _blocked_instro("stand", "session"):
        fresh_stand = importlib.import_module("stand")
        fresh_session = importlib.import_module("session")
        assert callable(fresh_stand.HardwareStand)
        assert callable(fresh_session.StandSession)
        assert fresh_session.State.IDLE.name == "IDLE"


class _UnguardedImportCollector(ast.NodeVisitor):
    """Collects Import/ImportFrom module names *outside* any `if
    TYPE_CHECKING:` block. Unlike a flat `ast.walk`, this does not descend
    into a TYPE_CHECKING-guarded `If` node's body at all (that code is never
    executed at runtime, by construction), while still descending into its
    `else` branch and into any other, non-guarded `If`."""

    def __init__(self) -> None:
        self.names: list[str] = []

    def visit_If(self, node: ast.If) -> None:
        if not _is_type_checking_guard(node):
            for child in node.body:
                self.visit(child)
        for child in node.orelse:
            self.visit(child)

    def visit_Import(self, node: ast.Import) -> None:
        self.names.extend(alias.name for alias in node.names)

    def visit_ImportFrom(self, node: ast.ImportFrom) -> None:
        self.names.append(node.module or "")


def _is_type_checking_guard(node: ast.If) -> bool:
    test = node.test
    if isinstance(test, ast.Name):
        return test.id == "TYPE_CHECKING"
    if isinstance(test, ast.Attribute):
        return test.attr == "TYPE_CHECKING"
    return False


def test_stand_and_session_source_has_no_module_level_instro_import():
    # A cheap secondary guard alongside the behavioral test above (plan.md's
    # Step 1 note): parse each file and confirm no Import/ImportFrom node
    # outside a `TYPE_CHECKING` guard names "instro"/"instro.*". The guarded
    # imports are expected and exempted -- they are never executed at
    # runtime, which is exactly what the behavioral test above checks for.
    for path in (PROJECT_ROOT / "stand.py", PROJECT_ROOT / "session.py"):
        collector = _UnguardedImportCollector()
        collector.visit(ast.parse(path.read_text()))
        for name in collector.names:
            assert not (name == "instro" or name.startswith("instro.")), (
                f"{path}: unguarded import of {name!r}"
            )


def test_limits_defaults_match_the_commissioned_machine():
    assert LIMITS.max_brake_a == 20.0
    assert LIMITS.dut_overspeed_erpm == 12_600.0
    assert LIMITS.dyno_overspeed_erpm == 3_300.0
    assert LIMITS.brake_ramp_a_per_s == 2.0
    assert LIMITS.speed_ramp_erpm_per_s == 1_000.0
    assert LIMITS.staleness_s == 0.5
    assert isinstance(LIMITS, Limits)


def test_limits_is_frozen():
    with pytest.raises(FrozenInstanceError):
        LIMITS.max_brake_a = 0.0


def test_trip_is_a_frozen_value_object_with_optional_node():
    trip = Trip(reason="dropout", node="dmc_l")
    assert trip.reason == "dropout"
    assert trip.node == "dmc_l"
    assert trip == Trip(reason="dropout", node="dmc_l")
    with pytest.raises(FrozenInstanceError):
        trip.reason = "other"

    untargeted = Trip(reason="half-shaft spread", node=None)
    assert untargeted.node is None


def test_node_literal_has_exactly_the_three_stand_nodes():
    assert set(interlocks.Node.__args__) == {"mdc", "dmc_l", "dmc_r"}


def test_state_enum_has_the_five_lifecycle_phases():
    assert {member.name for member in State} == {
        "IDLE",
        "ARMED",
        "RUNNING",
        "STOPPING",
        "TRIPPED",
    }


def test_units_module_exposes_the_four_conversions_and_stays_instro_free():
    for name in (
        "dut_erpm_to_stub_rpm",
        "stub_rpm_to_dut_erpm",
        "amps_to_brake_torque_nm",
        "dyno_rpm_to_erpm",
    ):
        assert callable(getattr(units, name))
    tree = ast.parse((PROJECT_ROOT / "units.py").read_text())
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = (
                [alias.name for alias in node.names]
                if isinstance(node, ast.Import)
                else [node.module or ""]
            )
            assert not any(n == "instro" or n.startswith("instro.") for n in names)


def test_hardwarestand_constructor_takes_three_controllers_a_psu_and_a_clock():
    params = inspect.signature(HardwareStand.__init__).parameters
    assert set(params) >= {"self", "mdc", "dmc_l", "dmc_r", "psu", "clock"}
    for name in ("mdc", "dmc_l", "dmc_r", "psu", "clock"):
        assert params[name].kind is inspect.Parameter.KEYWORD_ONLY
    assert params["clock"].default is not inspect._empty


def test_hardwarestand_exposes_the_command_and_lifecycle_surface():
    for name in (
        "open",
        "close",
        "get_telemetry",
        "set_speed_erpm",
        "set_brake_current_a",
        "zero_all",
        "zero_bus_voltage",
        "set_psu_output_enabled",
        "psu_output_enabled",
    ):
        assert callable(getattr(HardwareStand, name))
    assert isinstance(HardwareStand.now, property)


def test_standsession_constructor_takes_a_stand_and_a_tick_rate():
    params = inspect.signature(StandSession.__init__).parameters
    assert set(params) >= {"self", "stand", "tick_hz"}


def test_standsession_exposes_the_operator_command_surface():
    for name in (
        "can_arm",
        "arm",
        "disarm",
        "set_speed_setpoint_erpm",
        "set_brake_current_a",
        "set_psu_output_enabled",
        "stop",
        "tick",
        "acknowledge_trip",
    ):
        assert callable(getattr(StandSession, name))


def test_standsession_exposes_trip_state_and_a_freshness_counter():
    # Construct-then-assert-defaults, no state transitions -- the same
    # convention this file's other constructor tests use. StandSession's
    # __init__ never calls anything on `stand`, so a bare `None` is enough.
    session = StandSession(stand=None, tick_hz=50.0)
    assert session.trip is None
    assert session.freshness_skips == 0
