"""Numeric state codes streamed to the Connect canvas.

Connect's HMI canvas nodes (`!gate_valve`, `!compressor`, `!flow_meter`, `!status_light`) pick
their visual state by matching a channel's last numeric value against their `states:` list,
so every enum here is the contract between what the stand streams and what the canvas draws.
Each member's name maps to the Connect variant of the same name in PascalCase
(`NO_FLOW` -> `NoFlow`), which `canvas_states` relies on.
"""

from enum import IntEnum


class ValveState(IntEnum):
    """States of a `!gate_valve` node. Also used for the actuator: CLOSED is retracted, OPEN is extended."""

    CLOSED = 0
    OPEN = 1
    PARTIAL = 2
    TRANSITIONING = 3
    FAULT = 4


class CompressorState(IntEnum):
    """States of a `!compressor` node."""

    OFF = 0
    RUNNING = 1
    FAULT = 2


class FlowState(IntEnum):
    """States of a `!flow_meter` node."""

    NO_FLOW = 0
    FLOWING = 1
    REVERSE_FLOW = 2
    FAULT = 3


class LightState(IntEnum):
    """States of a `!status_light` node."""

    OFF = 0
    OK = 1
    WARNING = 2
    FAULT = 3


def connect_variant(member: IntEnum) -> str:
    """Get the Connect state variant name for a member, e.g. `FlowState.NO_FLOW` -> `NoFlow`."""
    return "".join(word.capitalize() for word in member.name.split("_"))


def canvas_states(enum: type[IntEnum]) -> list[dict]:
    """Build the `states:` list for a canvas node from an enum, ready to dump into the canvas yaml."""
    return [
        {
            "condition": {"kind": "equal", "value": float(member.value)},
            "value": connect_variant(member),
        }
        for member in enum
    ]
