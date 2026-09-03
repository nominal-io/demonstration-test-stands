"""Step 1's own instance of the behavioral instro-import-block guard.

plan.md's plan-decision box: `interlocks.py` and `units.py` contain no
`instro` import, at all, ever. This is checked the same way
`tests/test_api_surface.py` checks it for `stand.py`/`session.py` (Step 0) --
a behavioral test using a meta path finder that raises ImportError for
"instro" and everything under it, not a source-text/substring grep (a
substring search is gameable by a string-built import and false-positives on
a `TYPE_CHECKING` guard -- rev-1 plan review finding P1-2, plan.md's Step 1
note). The ast-based check below is kept alongside, not instead of, the
behavioral one, as a cheap secondary guard with a clearer failure message.
"""

import ast
import importlib
import sys
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import interlocks  # noqa: E402,F401
import units  # noqa: E402,F401


class _BlockInstroFinder:
    def find_spec(self, fullname, path, target=None):
        if fullname == "instro" or fullname.startswith("instro."):
            raise ImportError(f"blocked for test: {fullname}")
        return None


@contextmanager
def _blocked_instro(*module_names: str) -> Iterator[None]:
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
                sys.modules[name] = saved[name]
            else:
                importlib.import_module(name)


def test_interlocks_and_units_import_successfully_with_instro_unavailable():
    with _blocked_instro("interlocks", "units"):
        fresh_interlocks = importlib.import_module("interlocks")
        fresh_units = importlib.import_module("units")
        assert fresh_interlocks.LIMITS.max_brake_a == 20.0
        assert fresh_units.amps_to_brake_torque_nm(1.0) > 0.0


def test_interlocks_and_units_source_has_no_instro_import_at_all():
    # Unlike stand.py/session.py (Step 0), these two modules are not even
    # allowed a TYPE_CHECKING-guarded reference -- plan.md's decision is "no
    # instro import, at all, ever" -- so this is a flat ast.walk with no
    # TYPE_CHECKING exemption.
    for path in (PROJECT_ROOT / "interlocks.py", PROJECT_ROOT / "units.py"):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            else:
                continue
            for name in names:
                assert not (name == "instro" or name.startswith("instro.")), (
                    f"{path}: import of {name!r}"
                )
