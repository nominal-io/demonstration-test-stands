import math

import pytest

from fluids_panel.signals import (
    SIGNALS,
    SIGNALS_BY_REF,
    IoKind,
    Signal,
    eng_to_ma,
    is_loop_fault,
    ma_to_eng,
)


def test_refs_are_unique():
    assert len(SIGNALS_BY_REF) == len(SIGNALS)


def test_module_lines_are_unique():
    assert len({(s.module, s.line) for s in SIGNALS}) == len(SIGNALS)


def test_io_counts_match_interface_sheet():
    counts = {kind: sum(s.kind == kind for s in SIGNALS) for kind in IoKind}
    assert counts == {IoKind.DO: 4, IoKind.DI: 2, IoKind.AI: 3, IoKind.AO: 2}


def test_only_vent_and_iso_valves_are_normally_open():
    assert {s.ref for s in SIGNALS if s.normally_open} == {"v_1", "v_5"}


@pytest.mark.parametrize(("ma", "expected"), [(4.0, 0.0), (12.0, 50.0), (20.0, 100.0)])
def test_ma_to_eng(ma, expected):
    assert ma_to_eng(SIGNALS_BY_REF["p_1"], ma) == expected


def test_ma_to_eng_does_not_clamp():
    assert ma_to_eng(SIGNALS_BY_REF["p_1"], 2.0) < 0.0


def test_eng_to_ma_pr_2_half_scale():
    # Proof & leak step 4.2: 50% on pr_2 is ~72 psi.
    pr_2 = SIGNALS_BY_REF["pr_2"]
    assert eng_to_ma(pr_2, 72.5) == 12.0


@pytest.mark.parametrize(("value", "expected"), [(-5.0, 4.0), (1000.0, 20.0)])
def test_eng_to_ma_clamps(value, expected):
    assert eng_to_ma(SIGNALS_BY_REF["v_2"], value) == expected


@pytest.mark.parametrize("value", [0.0, 3.9, 7.8, 15.6])
def test_round_trip(value):
    v_2 = SIGNALS_BY_REF["v_2"]
    assert math.isclose(ma_to_eng(v_2, eng_to_ma(v_2, value)), value)


def test_scaling_rejects_digital_signals():
    with pytest.raises(ValueError):
        ma_to_eng(SIGNALS_BY_REF["sw_1"], 12.0)


@pytest.mark.parametrize(
    ("ma", "fault"),
    [
        (0.0, True),
        (3.59, True),
        (3.6, False),
        (4.0, False),
        (20.0, False),
        (21.0, False),
        (21.01, True),
    ],
)
def test_is_loop_fault(ma, fault):
    assert is_loop_fault(ma) is fault


@pytest.mark.parametrize(
    "kwargs",
    [
        {"ref": "x", "kind": IoKind.AI, "module": "m", "line": 0},
        {"ref": "x", "kind": IoKind.AI, "module": "m", "line": 0, "full_scale": 0.0},
        {"ref": "x", "kind": IoKind.DI, "module": "m", "line": 0, "full_scale": 1.0},
        {
            "ref": "x",
            "kind": IoKind.DI,
            "module": "m",
            "line": 0,
            "normally_open": True,
        },
    ],
)
def test_signal_validation(kwargs):
    with pytest.raises(ValueError):
        Signal(**kwargs)
