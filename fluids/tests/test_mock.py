import math

import pytest
from conftest import FakeClock

from fluids_panel.mock import MockBackend, MockParams
from fluids_panel.signals import SIGNALS_BY_REF, eng_to_ma, is_loop_fault, ma_to_eng


@pytest.fixture
def mock(clock: FakeClock):
    """Overrides conftest's fixture: these tests drive the backend directly, already open."""
    backend = MockBackend(MockParams(noise_fraction=0.0), clock=clock)
    with backend:
        yield backend


def run(mock: MockBackend, clock: FakeClock, seconds: float) -> dict:
    clock.now += seconds
    return mock.read()


def eng(inputs: dict, ref: str) -> float:
    return ma_to_eng(SIGNALS_BY_REF[ref], inputs[ref])


def pressurize(mock: MockBackend, clock: FakeClock) -> dict:
    """Startup sequence steps 1.4-1.6: compressor on, main vent closed, wait for supply."""
    mock.write({"pump_1": True, "v_1": True})
    return run(mock, clock, 60.0)


def test_read_before_open_raises(clock):
    with pytest.raises(RuntimeError):
        MockBackend(clock=clock).read()


def test_read_after_close_raises(clock):
    backend = MockBackend(clock=clock)
    with backend:
        pass
    with pytest.raises(RuntimeError):
        backend.read()


def test_write_validates(mock):
    with pytest.raises(TypeError):
        mock.write({"v_1": 1})


def test_initial_state_is_vented_and_retracted(mock, clock):
    inputs = run(mock, clock, 1.0)
    assert eng(inputs, "p_1") == 0.0
    assert eng(inputs, "p_2") == 0.0
    assert eng(inputs, "fm_1") == 0.0
    assert inputs["sw_1"] is True
    assert inputs["sw_2"] is False


def test_startup_builds_supply_to_regulator_setpoint(mock, clock):
    inputs = pressurize(mock, clock)
    assert math.isclose(eng(inputs, "p_1"), 85.0, abs_tol=0.5)
    assert not is_loop_fault(inputs["p_1"])


def test_compressor_stops_at_cut_out(mock, clock):
    pressurize(mock, clock)
    assert mock.plant.accumulator_psi == mock.params.compressor_cut_out_psi
    assert mock.plant.compressor_running is False


def test_supply_stays_vented_while_v_1_is_open(mock, clock):
    mock.write({"pump_1": True})
    inputs = run(mock, clock, 60.0)
    assert eng(inputs, "p_1") < 0.5


def test_opening_v_1_vents_supply(mock, clock):
    pressurize(mock, clock)
    mock.write({"pump_1": False, "v_1": False})
    inputs = run(mock, clock, 10.0)
    assert eng(inputs, "p_1") < 0.5


def test_flow_follows_v_2(mock, clock):
    pressurize(mock, clock)
    mock.write({"v_2": eng_to_ma(SIGNALS_BY_REF["v_2"], 7.8)})
    inputs = run(mock, clock, 5.0)
    assert math.isclose(eng(inputs, "fm_1"), 7.8, abs_tol=0.1)


def test_no_flow_without_supply(mock, clock):
    mock.write({"v_2": 20.0})
    inputs = run(mock, clock, 5.0)
    assert eng(inputs, "fm_1") == 0.0


def test_flow_draws_down_accumulator_and_cycles_compressor(mock, clock):
    pressurize(mock, clock)
    # Full flow drains ~1.56 psi/s, reaching the 95 psi cut-in after ~20 s.
    mock.write({"v_2": 20.0})
    run(mock, clock, 25.0)
    assert mock.plant.accumulator_psi < mock.params.compressor_cut_out_psi
    assert mock.plant.compressor_running is True


def test_actuator_extends_and_retracts(mock, clock):
    pressurize(mock, clock)

    mock.write({"v_3": True})
    inputs = run(mock, clock, 0.5)
    assert (inputs["sw_1"], inputs["sw_2"]) == (False, False)
    inputs = run(mock, clock, 1.5)
    assert (inputs["sw_1"], inputs["sw_2"]) == (False, True)

    mock.write({"v_3": False})
    inputs = run(mock, clock, 2.0)
    assert (inputs["sw_1"], inputs["sw_2"]) == (True, False)


def test_actuator_needs_supply_pressure(mock, clock):
    mock.write({"v_3": True})
    inputs = run(mock, clock, 5.0)
    assert (inputs["sw_1"], inputs["sw_2"]) == (True, False)


def test_proof_and_leak(mock, clock):
    pressurize(mock, clock)
    mock.write({"pr_2": eng_to_ma(SIGNALS_BY_REF["pr_2"], 72.5)})
    inputs = run(mock, clock, 20.0)
    assert math.isclose(eng(inputs, "p_2"), 72.5, abs_tol=0.1)

    mock.write({"v_5": True})
    inputs = run(mock, clock, 60.0)
    expected = 72.5 * math.exp(-60.0 / mock.params.vessel_leak_tau_s)
    assert math.isclose(eng(inputs, "p_2"), expected, abs_tol=0.1)


def test_vessel_limited_by_supply(mock, clock):
    pressurize(mock, clock)
    mock.write({"pr_2": 20.0})
    inputs = run(mock, clock, 30.0)
    assert math.isclose(eng(inputs, "p_2"), 85.0, abs_tol=0.5)


def test_estop_vents_while_outputs_stay_commanded(mock, clock):
    pressurize(mock, clock)
    mock.write({"v_3": True, "pr_2": 12.0})
    run(mock, clock, 20.0)

    mock.faults.estop = True
    inputs = run(mock, clock, 30.0)
    assert eng(inputs, "p_1") < 0.5
    assert eng(inputs, "p_2") < 0.5
    assert mock.outputs["pump_1"] is True
    assert mock.outputs["v_1"] is True


def test_compressor_failure(mock, clock):
    mock.faults.compressor_failed = True
    inputs = pressurize(mock, clock)
    assert eng(inputs, "p_1") < 0.5


def test_regulator_failure_relieves_at_rv_1(mock, clock):
    mock.faults.regulator_failed = True
    inputs = pressurize(mock, clock)
    assert math.isclose(eng(inputs, "p_1"), 100.0, abs_tol=0.5)


def test_stuck_actuator(mock, clock):
    pressurize(mock, clock)
    mock.faults.actuator_stuck = True
    mock.write({"v_3": True})
    inputs = run(mock, clock, 5.0)
    assert (inputs["sw_1"], inputs["sw_2"]) == (True, False)


def test_big_leak(mock, clock):
    pressurize(mock, clock)
    mock.write({"pr_2": eng_to_ma(SIGNALS_BY_REF["pr_2"], 72.5)})
    run(mock, clock, 20.0)
    mock.faults.vessel_leak_tau_s = 10.0
    mock.write({"v_5": True})
    inputs = run(mock, clock, 30.0)
    assert eng(inputs, "p_2") < 5.0


def test_broken_wire_reads_open_loop(mock, clock):
    pressurize(mock, clock)
    mock.faults.broken_wires.add("p_1")
    mock.faults.broken_wires.add("sw_1")
    inputs = run(mock, clock, 0.1)
    assert inputs["p_1"] == 0.0
    assert is_loop_fault(inputs["p_1"])
    assert inputs["sw_1"] is False


def test_broken_wire_must_be_an_input(mock, clock):
    mock.faults.broken_wires.add("v_1")
    with pytest.raises(ValueError):
        run(mock, clock, 0.1)


def test_noise_is_off_by_default(clock):
    with MockBackend(clock=clock, seed=7) as backend:
        readings = {backend.read()["fm_1"] for _ in range(20)}
    assert readings == {4.0}


def test_noise_can_be_switched_while_running(clock):
    with MockBackend(clock=clock, seed=7) as backend:
        backend.noise = True
        assert len({backend.read()["fm_1"] for _ in range(20)}) > 1
        backend.noise = False
        assert {backend.read()["fm_1"] for _ in range(20)} == {4.0}


def test_noise_is_seeded(clock):
    readings = []
    for _ in range(2):
        clock.now = 0.0
        with MockBackend(clock=clock, seed=7, noise=True) as backend:
            readings.append(pressurize(backend, clock)["p_1"])
    assert readings[0] == readings[1]
    assert readings[0] != eng_to_ma(SIGNALS_BY_REF["p_1"], 85.0)


def test_noise_at_zero_is_centred_and_healthy(clock):
    with MockBackend(clock=clock, seed=3, noise=True) as backend:
        currents = []
        for _ in range(2000):
            clock.now += 0.1
            currents.append(backend.read()["fm_1"])
    readings = [ma_to_eng(SIGNALS_BY_REF["fm_1"], ma) for ma in currents]
    assert abs(sum(readings) / len(readings)) < 0.05
    assert min(readings) < 0.0 < max(readings)
    assert not any(is_loop_fault(ma) for ma in currents)


def test_one_large_step_matches_many_small_ones(clock):
    def settle(steps: int) -> float:
        clock.now = 0.0
        with MockBackend(MockParams(noise_fraction=0.0), clock=clock) as backend:
            backend.write({"pump_1": True, "v_1": True})
            for _ in range(steps):
                clock.now += 60.0 / steps
                inputs = backend.read()
        return inputs["p_1"]

    assert math.isclose(settle(1), settle(600), abs_tol=1e-6)
