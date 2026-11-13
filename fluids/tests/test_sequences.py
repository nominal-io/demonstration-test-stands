import math

import pytest
from conftest import FakeClock, tick_until

from fluids_panel import sequences
from fluids_panel.backend import SAFE_OUTPUTS
from fluids_panel.stand import FluidsStand, FluidsStandState, InterlockError, WaitUntil


def run_sequence(
    stand: FluidsStand, clock: FakeClock, name: str, timeout_s: float = 300.0, **kwargs
) -> None:
    sequences.start(stand, name, **kwargs)
    tick_until(stand, clock, lambda: not stand.sequence_running, timeout_s)


def test_startup(stand, clock):
    run_sequence(stand, clock, "startup")
    assert stand.state == FluidsStandState.READY


def test_flow(stand, mock, clock):
    run_sequence(stand, clock, "startup")
    sequences.start(stand, "flow", lpm=7.8, hold_s=5.0)
    tick_until(stand, clock, lambda: stand.fm_1_lpm > 7.0, 10.0)
    tick_until(stand, clock, lambda: not stand.sequence_running, 30.0)
    assert stand.state == FluidsStandState.READY
    assert stand.v_2.setpoint == 0.0
    assert stand.fm_1_lpm < stand.config.flowing_lpm


def test_actuate(stand, mock, clock):
    run_sequence(stand, clock, "startup")
    extensions = 0
    was_extended = False
    sequences.start(stand, "actuate", cycles=2)
    deadline = clock.now + 30.0
    while stand.sequence_running:
        assert clock.now < deadline, "actuate did not finish"
        clock.now += 0.1
        stand.update()
        extended = bool(stand.sw_2.measured)
        extensions += extended and not was_extended
        was_extended = extended
    assert extensions == 2
    assert stand.state == FluidsStandState.READY
    assert stand.sw_1.measured is True


def test_proof_leak_measures_leak_rate(stand, mock, clock):
    run_sequence(stand, clock, "startup")
    run_sequence(stand, clock, "proof_leak", psig=72.5, hold_s=60.0)
    # The mock's vessel decays as exp(-t / 600 s), so 60 s from 72.5 psi loses ~6.9 psi.
    expected = 72.5 * (1 - math.exp(-60.0 / mock.params.vessel_leak_tau_s))
    assert stand.leak_rate_psi_min is not None
    assert math.isclose(stand.leak_rate_psi_min, expected, abs_tol=0.2)
    assert stand.telemetry()["p_2.leak_rate"] == stand.leak_rate_psi_min
    assert stand.p_2_psi <= stand.config.vented_psi
    assert stand.v_5.setpoint is False


def test_shutdown(stand, mock, clock):
    run_sequence(stand, clock, "startup")
    stand.extend()
    stand.set_flow(5.0)
    run_sequence(stand, clock, "shutdown")
    assert stand.state == FluidsStandState.VENTED
    assert mock.outputs == SAFE_OUTPUTS


@pytest.mark.parametrize("fixture", ["stand", "noisy_stand"])
def test_full_operations_sheet(fixture, request, clock):
    stand = request.getfixturevalue(fixture)
    for name in ("startup", "flow", "actuate", "proof_leak", "shutdown"):
        run_sequence(stand, clock, name)
        assert stand.trips == [], name
    assert stand.state == FluidsStandState.VENTED


def test_refused_first_step_raises_without_tripping(stand, clock):
    with pytest.raises(InterlockError):
        sequences.start(stand, "flow")
    assert not stand.sequence_running
    assert stand.state == FluidsStandState.VENTED


def test_unknown_sequence(stand):
    with pytest.raises(ValueError):
        sequences.start(stand, "nope")


def test_one_sequence_at_a_time(stand):
    sequences.start(stand, "startup")
    with pytest.raises(InterlockError):
        sequences.start(stand, "shutdown")


def test_wait_timeout_trips(stand, mock, clock):
    def never(s: FluidsStand):
        yield WaitUntil("never", lambda: False, timeout_s=1.0)

    stand.start_sequence("never", never(stand))
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.TRIPPED, 5.0)
    assert any("timed out waiting for never" in trip for trip in stand.trips)
    assert not stand.sequence_running


def test_refused_later_step_trips(stand, mock, clock):
    def extend_while_vented(s: FluidsStand):
        yield s.wait(0.5)
        s.extend()
        yield s.wait(0.5)

    stand.start_sequence("bad", extend_while_vented(stand))
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.TRIPPED, 5.0)
    assert any("sequence bad" in trip for trip in stand.trips)
    assert mock.outputs == SAFE_OUTPUTS


def test_trip_aborts_sequence(stand, mock, clock):
    run_sequence(stand, clock, "startup")
    sequences.start(stand, "flow", hold_s=60.0)
    mock.faults.estop = True
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.TRIPPED, 10.0)
    assert not stand.sequence_running


def test_abort_stops_and_vents(stand, clock):
    run_sequence(stand, clock, "startup")
    sequences.start(stand, "flow", hold_s=60.0)
    stand.abort()
    assert not stand.sequence_running
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.VENTED, 30.0)
