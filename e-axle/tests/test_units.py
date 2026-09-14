import math
import units


def test_mechanical_rpm_to_erpm():
    assert units.mechanical_rpm_to_erpm(100.0, pole_pairs=4) == 400.0
    assert units.mechanical_rpm_to_erpm(100, pole_pairs=1) == 100.0


def test_erpm_to_mechanical_rpm():
    assert units.erpm_to_mechanical_rpm(400.0, pole_pairs=4) == 100.0
    assert units.erpm_to_mechanical_rpm(100, pole_pairs=1) == 100.0

def test_erpm_to_geared_rpm():
    assert units.erpm_to_geared_rpm(3638.0, ratio=36.38) == 100.0
    assert units.erpm_to_geared_rpm(100, ratio=1) == 100.0


def test_geared_rpm_to_erpm():
    assert math.isclose(units.geared_rpm_to_erpm(100.0, ratio=36.38), 3638.0, rel_tol=1e-9)
    assert units.geared_rpm_to_erpm(100, ratio=1) == 100.0


def test_current_to_torque_nm():
    assert units.current_to_torque_nm(20.0, kt=0.1906) == 3.812
