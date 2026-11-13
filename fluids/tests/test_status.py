from conftest import tick, tick_until

from fluids_panel import sequences
from fluids_panel.stand import FluidsStandState
from fluids_panel.status import status_message


def test_vented(stand):
    assert status_message(stand).startswith("Vented. Press Pressurize")


def test_pressurizing_reports_progress(stand, clock):
    stand.pressurize()
    tick(stand, clock, 5.0)
    message = status_message(stand)
    assert message.startswith("Pressurizing: p_1 at ")
    assert message.endswith(" of 75 psi.")


def test_ready(stand, clock):
    stand.pressurize()
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.READY, 60.0)
    assert status_message(stand).startswith("Ready.")


def test_venting(stand, clock):
    stand.pressurize()
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.READY, 60.0)
    stand.vent()
    tick(stand, clock, 0.1)
    assert status_message(stand).startswith("Venting: p_1 at ")


def test_sequence_names_what_it_waits_for(stand, clock):
    sequences.start(stand, "startup")
    assert status_message(stand) == (
        "Running startup: waiting for p_1 to reach 75 psi. "
        "Press Abort or Vent to stop it."
    )


def test_tripped_gives_reason_and_next_step(stand, mock, clock):
    stand.pressurize()
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.READY, 60.0)
    mock.faults.estop = True
    tick_until(stand, clock, lambda: stand.state == FluidsStandState.TRIPPED, 10.0)
    message = status_message(stand)
    assert message.startswith("TRIPPED: supply lost")
    assert message.endswith("Fix the cause, then press Reset trip.")


def test_tripped_counts_extra_reasons(stand, mock, clock):
    mock.faults.broken_wires.update({"p_2", "fm_1"})
    tick(stand, clock, 0.1)
    assert "(+1 more in the logs)" in status_message(stand)
