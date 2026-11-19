import pytest

from fluids_panel.simulation import (
    ACTUATOR_JAMMED,
    BIG_LEAK,
    BIG_LEAK_TAU_S,
    ESTOP,
    NOISE,
    P_1_WIRE,
    SIMULATION_INPUT_IDS,
    apply_simulation_settings,
)


def test_all_off_by_default(mock):
    apply_simulation_settings(mock, {})
    assert mock.noise is False
    assert mock.faults.estop is False
    assert mock.faults.actuator_stuck is False
    assert mock.faults.vessel_leak_tau_s is None
    assert mock.faults.broken_wires == set()


def test_all_on(mock):
    apply_simulation_settings(mock, dict.fromkeys(SIMULATION_INPUT_IDS, True))
    assert mock.noise is True
    assert mock.faults.estop is True
    assert mock.faults.actuator_stuck is True
    assert mock.faults.vessel_leak_tau_s == BIG_LEAK_TAU_S
    assert mock.faults.broken_wires == {"p_1"}


@pytest.mark.parametrize(
    ("switch", "read"),
    [
        (NOISE, lambda m: m.noise),
        (ESTOP, lambda m: m.faults.estop),
        (ACTUATOR_JAMMED, lambda m: m.faults.actuator_stuck),
        (BIG_LEAK, lambda m: m.faults.vessel_leak_tau_s is not None),
        (P_1_WIRE, lambda m: "p_1" in m.faults.broken_wires),
    ],
)
def test_each_switch_turns_off_again(mock, switch, read):
    apply_simulation_settings(mock, {switch: True})
    assert read(mock)
    apply_simulation_settings(mock, {switch: False})
    assert not read(mock)


def test_p_1_switch_leaves_other_broken_wires(mock):
    mock.faults.broken_wires.add("p_2")
    apply_simulation_settings(mock, {P_1_WIRE: True})
    apply_simulation_settings(mock, {P_1_WIRE: False})
    assert mock.faults.broken_wires == {"p_2"}


@pytest.mark.parametrize("value", [None, "true", 1])
def test_only_a_real_true_counts_as_on(mock, value):
    apply_simulation_settings(mock, {ESTOP: value})
    assert mock.faults.estop is False
