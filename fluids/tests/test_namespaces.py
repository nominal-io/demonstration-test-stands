import re
from pathlib import Path

import pytest

from fluids_panel.build import BACKENDS
from fluids_panel.namespaces import NAMESPACES

APP = Path(__file__).parents[1] / "app"

# Each tab's P&ID canvas, by the backend its tab runs.
CANVASES = {
    "mock": APP / "96ccc0e2-42d7-458a-a38d-2185ede3d4c2.yaml",
    "daq": APP / "0fbfc996-4451-45c5-ba3c-efbbf67cc42d.yaml",
}


def test_every_backend_has_its_own_names():
    assert set(NAMESPACES) == set(BACKENDS)
    for field in ("stream_id", "command_topic", "status_value_id"):
        values = [getattr(names, field) for names in NAMESPACES.values()]
        assert len(set(values)) == len(values), field


@pytest.mark.parametrize("backend", BACKENDS)
def test_canvas_only_talks_to_its_own_stand(backend):
    names = NAMESPACES[backend]
    text = CANVASES[backend].read_text()

    topics = re.findall(r"^\s*topic: (\S+)$", text, re.MULTILINE)
    streams = re.findall(r"^\s*stream: (\S+)$", text, re.MULTILINE)
    channels = re.findall(r"^\s*channel: (\S+)$", text, re.MULTILINE)
    assert topics and set(topics) == {names.command_topic}
    assert streams and set(streams) == {names.stream_id}
    # State channels are absolute (`<stream>.<name>`); values beside a `stream:` are relative.
    others = [n.stream_id for b, n in NAMESPACES.items() if b != backend]
    for channel in channels:
        assert not any(channel.startswith(f"{other}.") for other in others), channel
    assert any(channel.startswith(f"{names.stream_id}.") for channel in channels)


@pytest.mark.parametrize("backend", BACKENDS)
def test_every_canvas_click_toggles(backend):
    # The guide (controls.md) promises every click on the diagram toggles, so a second click
    # undoes the first; `set` would only ever open.
    text = CANVASES[backend].read_text()
    actions = re.findall(r"^\s*action: (\S+)$", text, re.MULTILINE)
    assert actions and set(actions) == {"toggle"}
