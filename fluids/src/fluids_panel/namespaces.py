"""Where each backend's stand script meets its Connect tab: stream, command topic, status value.

Every backend gets its own names, so each backend's tab can run side by side with the others and
a click on one tab never reaches another tab's stand. The canvases and `app.connect` spell these
names out literally; `tests/test_namespaces.py` checks that each canvas stays on its own.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Namespace:
    """The Connect names one stand script uses.

    stream_id: Stream for telemetry; the canvas reads its channels as `<stream_id>.<name>`.
    command_topic: Message-bus topic the tab's canvas and buttons send commands on.
    status_value_id: App value the status banner displays.
    """

    stream_id: str
    command_topic: str
    status_value_id: str


NAMESPACES = {
    "mock": Namespace(
        stream_id="fluids",
        command_topic="script/fluids/cmd",
        status_value_id="stand-status",
    ),
}
