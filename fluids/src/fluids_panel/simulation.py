"""The SIMULATION pane's switches, applied to the mock backend.

Switches are settings rather than one-shot actions, so the Connect script reads their current
values every tick (`client.get_values()`) and applies them here, rather than reacting to clicks.
Each switch shows whether its condition is currently on.
"""

from collections.abc import Mapping
from typing import Any

from fluids_panel.mock import MockBackend

NOISE = "sim-noise"
ESTOP = "sim-estop"
P_1_WIRE = "sim-p1-wire"
ACTUATOR_JAMMED = "sim-actuator-jammed"
BIG_LEAK = "sim-big-leak"

SIMULATION_INPUT_IDS = (NOISE, ESTOP, P_1_WIRE, ACTUATOR_JAMMED, BIG_LEAK)

BIG_LEAK_TAU_S = 10.0  # vessel leak-down time constant while the big-leak switch is on


def apply_simulation_settings(mock: MockBackend, values: Mapping[str, Any]) -> None:
    """Set the mock's noise and faults from the switches' values. A missing switch reads as off."""

    def on(input_id: str) -> bool:
        return values.get(input_id) is True

    mock.noise = on(NOISE)
    mock.faults.estop = on(ESTOP)
    mock.faults.actuator_stuck = on(ACTUATOR_JAMMED)
    mock.faults.vessel_leak_tau_s = BIG_LEAK_TAU_S if on(BIG_LEAK) else None
    if on(P_1_WIRE):
        mock.faults.broken_wires.add("p_1")
    else:
        mock.faults.broken_wires.discard("p_1")
