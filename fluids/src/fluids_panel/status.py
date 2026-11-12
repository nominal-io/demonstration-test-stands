"""A one-line, operator-facing description of what the stand is doing and what to do next.

The Connect script publishes it to the app value STATUS_VALUE_ID, which a callout at the top of
the controls displays, so the panel always says what state it is in and which controls apply.
"""

from fluids_panel.stand import FluidsStand, FluidsStandState

STATUS_VALUE_ID = "stand-status"

NOT_RUNNING = "Stand Control is not running. Start it under Script Controls."


def status_message(stand: FluidsStand) -> str:
    """Describe the stand's state and the next useful action."""
    if stand.state == FluidsStandState.TRIPPED:
        reason = stand.trips[0] if stand.trips else "unknown cause"
        more = (
            f" (+{len(stand.trips) - 1} more in the logs)"
            if len(stand.trips) > 1
            else ""
        )
        return f"TRIPPED: {reason}{more}. Fix the cause, then press Reset trip."
    if stand.sequence_running:
        return (
            f"Running {stand.sequence_name}: waiting for {stand.waiting_for}. "
            "Press Abort or Vent to stop it."
        )
    p_1 = stand.p_1_psi
    match stand.state:
        case FluidsStandState.VENTED:
            return "Vented. Press Pressurize (or click the compressor) to start."
        case FluidsStandState.PRESSURIZING:
            return f"Pressurizing: p_1 at {p_1:.0f} of {stand.config.ready_psi:g} psi."
        case FluidsStandState.READY:
            return "Ready. Click a valve on the diagram or run a sequence."
        case FluidsStandState.VENTING:
            return f"Venting: p_1 at {p_1:.0f} psi."
    return stand.state.name
