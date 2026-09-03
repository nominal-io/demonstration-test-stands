"""Step 1 ("Interlocks & units") tests for `units.py`.

Pure conversions, no `instro` import -- see plan.md's plan-decision box and
`tests/test_interlocks_units_import_guard.py` for the behavioral guard on that
property. This file only exercises the arithmetic.
"""

import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import units  # noqa: E402


def test_amps_to_brake_torque_nm_matches_the_commissioned_kt():
    # design.md "Two conversion facts are load-bearing": Kt = 0.1906 N*m/A.
    # 20 A is the stand's brake-current clamp.
    assert math.isclose(units.amps_to_brake_torque_nm(20.0), 3.812, rel_tol=1e-9)


def test_amps_to_brake_torque_nm_scales_linearly_and_handles_zero():
    assert math.isclose(units.amps_to_brake_torque_nm(10.0), 1.906, rel_tol=1e-9)
    assert units.amps_to_brake_torque_nm(0.0) == 0.0


def test_amps_to_brake_torque_nm_accepts_a_non_default_kt():
    assert math.isclose(units.amps_to_brake_torque_nm(10.0, kt=0.2), 2.0, rel_tol=1e-9)


def test_dut_erpm_to_stub_rpm_uses_the_commissioned_36_38_chain_ratio():
    # design.md: "The DUT chain ratio is 36.38 (DUT ERPM per dyno mechanical
    # RPM)" -- stub RPM = DUT ERPM / chain_ratio.
    assert math.isclose(units.dut_erpm_to_stub_rpm(3638.0), 100.0, rel_tol=1e-9)
    assert math.isclose(units.dut_erpm_to_stub_rpm(12_600.0), 346.34414513469765, rel_tol=1e-9)


def test_stub_rpm_to_dut_erpm_is_the_inverse_conversion():
    assert math.isclose(units.stub_rpm_to_dut_erpm(100.0), 3638.0, rel_tol=1e-9)


def test_erpm_stub_rpm_conversions_round_trip():
    for original_erpm in (0.0, 100.0, 3638.0, 12_600.0):
        stub_rpm = units.dut_erpm_to_stub_rpm(original_erpm)
        assert math.isclose(
            units.stub_rpm_to_dut_erpm(stub_rpm), original_erpm, rel_tol=1e-9
        )


def test_chain_ratio_is_a_configurable_parameter_not_hardcoded():
    # Both earlier-scripts splits (3 x 9.5 = 28.5) and the design's own
    # measured 36.38 must be usable without editing the function body.
    assert units.dut_erpm_to_stub_rpm(285.0, chain_ratio=28.5) == 10.0
    assert units.stub_rpm_to_dut_erpm(10.0, chain_ratio=28.5) == 285.0


def test_dyno_rpm_to_erpm_pins_the_two_safety_thresholds_design_md_names():
    # design.md §3: LIMITS.dyno_overspeed_erpm (3_300.0) and
    # LIMITS.spread_floor_erpm (500.0) are both transcribed in ERPM, but the
    # absorbers' telemetry `velocity` field is mechanical RPM
    # (pole_pairs=7). These are the two thresholds' mechanical-RPM
    # round-trips -- get this arithmetic wrong and interlocks 3 and 8 are
    # silently dead.
    assert math.isclose(units.dyno_rpm_to_erpm(471.428571428571), 3300.0, rel_tol=1e-9)
    assert math.isclose(units.dyno_rpm_to_erpm(71.42857142857143), 500.0, rel_tol=1e-9)


def test_dyno_rpm_to_erpm_of_zero_is_zero():
    assert units.dyno_rpm_to_erpm(0.0) == 0.0


def test_dyno_rpm_to_erpm_pole_pairs_is_a_configurable_parameter_not_hardcoded():
    # mdc's velocity needs no conversion because pole_pairs=1 makes it
    # already ERPM -- design.md §3. That only works if pole_pairs is a real
    # parameter, not a hardcoded 7.
    assert units.dyno_rpm_to_erpm(100.0, pole_pairs=1) == 100.0


def test_dyno_rpm_to_erpm_scales_linearly_with_sign_preserved():
    # This design does not special-case direction anywhere (plan.md Step 2).
    assert units.dyno_rpm_to_erpm(-100.0, pole_pairs=7) == -700.0
