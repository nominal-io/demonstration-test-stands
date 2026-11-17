"""The Connect device config in `app/` must wire the cDAQ exactly as SIGNALS (the sheet) does.

The config may be re-saved from Connect's Devices panel, so these check what the stand relies on
rather than the file's exact text.
"""

import json
from pathlib import Path

import pytest

from fluids_panel.backend import SAFE_OUTPUTS
from fluids_panel.connect_device import DEVICE_NAME, PRODUCT_TYPES, relative_channel
from fluids_panel.signals import SIGNALS, IoKind

APP = Path(__file__).parents[1] / "app"
CONFIG_FILE = "fluids_daq.ni-daqmx.json"
CONFIG = json.loads((APP / CONFIG_FILE).read_text())

CHANNEL_TYPES = {
    IoKind.DO: "DigitalOutput",
    IoKind.DI: "DigitalInput",
    IoKind.AI: "AnalogInput",
    IoKind.AO: "AnalogOutput",
}


def channels_by_module() -> dict[str, dict[str, dict]]:
    return {module["product_type"]: module["channels"] for module in CONFIG["modules"]}


def channel_type(channel: dict) -> str:
    kind = channel["channel_type"]
    return kind if isinstance(kind, str) else next(iter(kind))


def test_device_name_is_the_stream_the_backend_reads():
    assert CONFIG["device_name"] == DEVICE_NAME
    assert CONFIG["stream_id"] == DEVICE_NAME


def test_config_names_every_module_as_daqmx_does():
    assert set(PRODUCT_TYPES) == {s.module for s in SIGNALS}
    assert sorted(channels_by_module()) == sorted(PRODUCT_TYPES.values())


def test_app_applies_the_config_to_the_cdaq():
    text = (APP / "app.connect").read_text()
    assert f"NI-DAQmx:\n    cDAQ1: {CONFIG_FILE}" in text


@pytest.mark.parametrize("signal", SIGNALS, ids=lambda s: s.ref)
def test_every_signal_is_wired_per_the_sheet(signal):
    module = channels_by_module()[PRODUCT_TYPES[signal.module]]
    channel = module[relative_channel(signal)]
    assert channel["alias"] == signal.ref
    assert channel["enabled"] is True
    assert channel_type(channel) == CHANNEL_TYPES[signal.kind]
    # The backend converts amps itself; a scale here would apply twice.
    assert channel["scale"] is None
    if signal.kind.is_analog:
        params = channel["channel_type"][CHANNEL_TYPES[signal.kind]]
        assert "current" in params


def test_no_other_channels_are_enabled():
    enabled = [
        channel["alias"]
        for channels in channels_by_module().values()
        for channel in channels.values()
        if channel["enabled"]
    ]
    assert sorted(enabled) == sorted(s.ref for s in SIGNALS)


@pytest.mark.parametrize(
    "signal",
    [s for s in SIGNALS if s.kind in (IoKind.DO, IoKind.AO)],
    ids=lambda s: s.ref,
)
def test_device_starts_with_every_output_safe(signal):
    channel = channels_by_module()[PRODUCT_TYPES[signal.module]][
        relative_channel(signal)
    ]
    value = channel["channel_type"][CHANNEL_TYPES[signal.kind]]["value"]
    if signal.kind == IoKind.DO:
        assert value == {"binary": int(SAFE_OUTPUTS[signal.ref])}
    else:
        assert value == {"constant": pytest.approx(SAFE_OUTPUTS[signal.ref] / 1000)}
