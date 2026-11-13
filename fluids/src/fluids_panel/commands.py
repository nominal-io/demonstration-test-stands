"""Turn Connect message-bus messages into stand commands.

Canvas nodes and buttons send `on_click_action` messages on COMMAND_TOPIC. Connect adds
`widget_id` and `node_id`, which are ignored; every message names what it wants explicitly:

    {action: toggle, target: supply | pump_1 | v_1 | v_3 | v_5}
    {action: toggle, target: v_2 | pr_2, value: <L/min | psig>}  # on at value, or off
    {action: set, target: v_2 | pr_2, value: <L/min | psig>}     # value is often `$<input-id>`
    {action: sequence, name: <sequence>, <param>: <number>, ...}
    {action: pressurize} | {action: vent} | {action: abort} | {action: reset}

Simulated faults are settings, not actions: see `simulation.py`.

Every canvas click is a toggle: a second click undoes the first. While a sequence runs, only
`vent`, `abort` and `reset` are accepted, since anything else would fight it; `vent` aborts the
sequence first, so it always makes the panel safe.
"""

import inspect
import logging
from collections.abc import Mapping
from typing import Any

from fluids_panel import sequences
from fluids_panel.stand import FluidsStand, FluidsStandState, InterlockError

logger = logging.getLogger(__name__)

COMMAND_TOPIC = "script/fluids/cmd"

_ALWAYS_ALLOWED = ("vent", "abort", "reset")
_SUPPLY_TARGETS = ("supply", "pump_1", "v_1")
_PRESSURIZED_STATES = (FluidsStandState.PRESSURIZING, FluidsStandState.READY)


class CommandError(ValueError):
    """A message was malformed or asked for something that doesn't exist."""


def handle_message(stand: FluidsStand, contents: Mapping[str, Any]) -> str:
    """Apply one message to the stand and return a short description of what was done.

    Raises CommandError for a malformed message and InterlockError for a refused command.
    """
    action = contents.get("action")
    if not isinstance(action, str):
        raise CommandError(f"message has no action: {dict(contents)}")
    if stand.sequence_running and action not in _ALWAYS_ALLOWED:
        raise InterlockError(
            f"sequence {stand.sequence_name!r} is running; abort it before {action!r}"
        )

    match action:
        case "toggle":
            return _toggle(stand, _str_field(contents, "target"), contents)
        case "set":
            return _set(
                stand, _str_field(contents, "target"), _number(contents, "value")
            )
        case "sequence":
            return _sequence(stand, contents)
        case "pressurize":
            stand.pressurize()
            return "pressurizing"
        case "vent":
            return _vent(stand)
        case "abort":
            stand.abort()
            return "aborted and venting"
        case "reset":
            stand.reset()
            return "trip reset"
        case _:
            raise CommandError(f"unknown action {action!r}")


def _vent(stand: FluidsStand) -> str:
    """Make the panel safe from any state. Idempotent, and never refused."""
    if stand.sequence_running:
        name = stand.sequence_name
        stand.abort()
        return f"sequence {name} aborted; venting"
    if stand.state == FluidsStandState.TRIPPED:
        stand.vent()
        return "tripped: outputs are already safe; press Reset trip"
    if stand.state == FluidsStandState.VENTED:
        stand.vent()
        return "already vented"
    stand.vent()
    return "venting"


def _toggle(stand: FluidsStand, target: str, contents: Mapping[str, Any]) -> str:
    if target in _SUPPLY_TARGETS:
        if stand.state in _PRESSURIZED_STATES:
            stand.vent()
            return "venting"
        stand.pressurize()
        return "pressurizing"
    if target == "v_2":
        if stand.v_2.setpoint > 0:
            stand.set_flow(0.0)
            return "v_2 closed"
        stand.set_flow(_number(contents, "value"))
        return f"v_2 open at {stand.v_2.setpoint:g} L/min"
    if target == "pr_2":
        if stand.pr_2.setpoint > 0:
            stand.set_vessel_pressure(0.0)
            return "pr_2 set to 0 psig"
        stand.set_vessel_pressure(_number(contents, "value"))
        return f"pr_2 set to {stand.pr_2.setpoint:g} psig"
    if target == "v_3":
        if stand.v_3.setpoint:
            stand.retract()
            return "retracting"
        stand.extend()
        return "extending"
    if target == "v_5":
        stand.isolate_vessel(not stand.v_5.setpoint)
        return "vessel isolated" if stand.v_5.setpoint else "vessel open to pr_2"
    raise CommandError(f"cannot toggle {target!r}")


def _set(stand: FluidsStand, target: str, value: float) -> str:
    if target == "v_2":
        stand.set_flow(value)
        return f"v_2 set to {stand.v_2.setpoint:g} L/min"
    if target == "pr_2":
        stand.set_vessel_pressure(value)
        return f"pr_2 set to {stand.pr_2.setpoint:g} psig"
    raise CommandError(f"cannot set {target!r}")


def _sequence(stand: FluidsStand, contents: Mapping[str, Any]) -> str:
    name = _str_field(contents, "name")
    factory = sequences.SEQUENCES.get(name)
    if factory is None:
        raise CommandError(
            f"unknown sequence {name!r}; expected one of {list(sequences.SEQUENCES)}"
        )
    # Pass through only the parameters the sequence takes, e.g. `lpm` for flow.
    parameters = list(inspect.signature(factory).parameters)[1:]
    kwargs = {key: _number(contents, key) for key in parameters if key in contents}
    sequences.start(stand, name, **kwargs)
    return f"sequence {name} started"


def _str_field(contents: Mapping[str, Any], key: str) -> str:
    value = contents.get(key)
    if not isinstance(value, str) or not value:
        raise CommandError(f"{key!r} must be a non-empty string, got {value!r}")
    return value


def _number(contents: Mapping[str, Any], key: str) -> float:
    value = contents.get(key)
    # Inputs referenced as `$input-id` arrive as numbers, or as strings from text inputs.
    if isinstance(value, bool) or not isinstance(value, int | float | str):
        raise CommandError(f"{key!r} must be a number, got {value!r}")
    try:
        return float(value)
    except ValueError:
        raise CommandError(f"{key!r} must be a number, got {value!r}") from None
