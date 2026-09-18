import math

import units
from e_axle.constants import (
    DUT_CHAIN,
    DUT_ERPM_PER_DYNO_ERPM,
    DUT_GEAR,
    DUT_KT,
    DUT_POLE_PAIRS,
    DYNO_GEAR,
    DYNO_KT,
    DYNO_POLE_PAIRS,
    KT_BY_ID,
    NODES,
)


def test_dut_split_multiplies_back_to_the_measured_chain():
    """Only the product is measurable from CAN; the split must preserve it."""
    assert math.isclose(DUT_POLE_PAIRS * DUT_GEAR, DUT_CHAIN, rel_tol=1e-9)


def test_dut_gear_is_near_the_nominal_final_drive():
    """4 pole pairs is chosen because it puts the ratio near the nominal 9.5.

    3 pole pairs would imply 12.13, which matches nothing (manual 5.2).
    """
    assert math.isclose(DUT_GEAR, 9.095, rel_tol=1e-3)


def test_dut_to_dyno_erpm_ratio_matches_commissioning():
    """MEASURED at 5.1973 (sd 0.0021) across all load levels."""
    assert math.isclose(DUT_ERPM_PER_DYNO_ERPM, 5.1973, rel_tol=1e-4)


def test_carrier_speed_at_the_operating_ceiling():
    """12,000 ERPM must give the ~330 RPM half shaft speed in the manual."""
    assert math.isclose(
        units.erpm_to_geared_rpm(12_000.0, ratio=DUT_CHAIN), 330.0, abs_tol=0.5
    )


def test_dyno_shaft_speed_at_the_operating_ceiling():
    """The dynos are 1:1 to the half shafts, so both routes must agree."""
    dyno_erpm = 12_000.0 / DUT_ERPM_PER_DYNO_ERPM
    dyno_rpm = units.erpm_to_mechanical_rpm(dyno_erpm, pole_pairs=DYNO_POLE_PAIRS)
    carrier_rpm = units.erpm_to_geared_rpm(12_000.0, ratio=DUT_CHAIN)
    assert math.isclose(dyno_rpm, carrier_rpm, rel_tol=1e-9)


def test_torque_constants_follow_the_pmsm_relation():
    """Kt = 1.5 * pole_pairs * lambda, exact for a sinusoidally wound PMSM."""
    assert math.isclose(DUT_KT, 1.5 * DUT_POLE_PAIRS * 0.014423, rel_tol=1e-9)
    assert math.isclose(DYNO_KT, 1.5 * DYNO_POLE_PAIRS * 0.018148, rel_tol=1e-9)
    assert math.isclose(DYNO_KT, 0.1906, abs_tol=5e-5)


def test_node_map_agrees_with_the_scalar_constants():
    assert NODES[0] == ("DUT", DUT_POLE_PAIRS, DUT_GEAR)
    for vid in (1, 2):
        name, pole_pairs, gear = NODES[vid]
        assert (pole_pairs, gear) == (DYNO_POLE_PAIRS, DYNO_GEAR)
        assert name.startswith("DMC-")


def test_kt_lookup_covers_every_node():
    assert KT_BY_ID.keys() == NODES.keys()
    assert KT_BY_ID[0] == DUT_KT
    assert KT_BY_ID[1] == KT_BY_ID[2] == DYNO_KT
