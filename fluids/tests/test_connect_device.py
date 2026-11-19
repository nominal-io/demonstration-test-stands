"""ConnectDeviceBackend against a fake Connect: no app, device or cDAQ needed."""

import pytest
from conftest import FakeClock

from fluids_panel.backend import SAFE_OUTPUTS, BackendFault
from fluids_panel.connect_device import (
    DEVICE_NAME,
    RESEND_INTERVAL_S,
    STALE_AFTER_S,
    START_TIMEOUT_S,
    ConnectDeviceBackend,
    device_topic,
)
from fluids_panel.signals import SIGNALS_BY_REF
from fluids_panel.stand import FluidsStand, FluidsStandState, InterlockError

TOPIC = "devices/fluids_daq/command"


class FakeConnect:
    """Connect's NI-DAQmx device as a script's client sees it.

    While streaming, every poll of the stream sees a new sample; `values` are in amps and 0/1,
    as the device streams them. Commands are recorded and applied to `outputs`.
    """

    def __init__(self) -> None:
        self.streaming = False
        self.can_start = True  # False: the device isn't there to start
        self.samples = 0
        self.values: dict[str, float] = {
            "sw_1": 0.0,
            "sw_2": 0.0,
            "p_1": 0.004,
            "p_2": 0.004,
            "fm_1": 0.004,
        }
        self.messages: list[dict] = []
        self.outputs: dict[str, float] = {}

    def get_channel_values(self, stream_id, channels=None):
        assert stream_id == DEVICE_NAME
        if self.streaming:
            self.samples += 1
        if not self.samples:
            return {}
        return {
            ref: {"timestamp": self.samples * 10_000_000, "value": self.values[ref]}
            for ref in channels or self.values
            if ref in self.values
        }

    def send_message(self, topic, contents):
        assert topic == TOPIC
        self.messages.append(contents)
        if "set_streaming" in contents:
            self.streaming = contents["set_streaming"] and self.can_start
        if command := contents.get("set_output"):
            ((_, value),) = command["value"].items()
            self.outputs[command["channel_alias"]] = value

    def set_outputs_sent(self) -> list[str]:
        return [
            m["set_output"]["channel_alias"] for m in self.messages if "set_output" in m
        ]


SAFE_COMMANDS = {
    "pump_1": 0,
    "v_1": 0,
    "v_3": 0,
    "v_5": 0,
    "v_2": pytest.approx(0.004),
    "pr_2": pytest.approx(0.004),
}


@pytest.fixture
def connect() -> FakeConnect:
    return FakeConnect()


@pytest.fixture
def backend(connect: FakeConnect, clock: FakeClock) -> ConnectDeviceBackend:
    def sleep(seconds: float) -> None:
        clock.now += seconds

    return ConnectDeviceBackend(connect, clock=clock, sleep=sleep)


def test_open_starts_streaming_and_commands_outputs_safe(backend, connect):
    with backend:
        assert connect.messages[0] == {"set_streaming": True}
        assert connect.outputs == SAFE_COMMANDS
        assert backend.outputs == SAFE_OUTPUTS


def test_open_waits_for_a_sample_newer_than_the_buffered_one(backend, connect, clock):
    connect.samples = 5  # left over from an earlier run
    connect.values["p_1"] = 0.012
    with backend:
        assert connect.samples > 5
        assert backend.read()["p_1"] == pytest.approx(12.0)


def test_open_times_out_without_data_but_leaves_outputs_safe(backend, connect, clock):
    connect.can_start = False
    with pytest.raises(RuntimeError, match=DEVICE_NAME):
        backend.open()
    assert clock.now >= START_TIMEOUT_S
    assert connect.outputs == SAFE_COMMANDS
    with pytest.raises(RuntimeError, match="not open"):
        backend.read()


def test_read_converts_amps_to_ma_and_lines_to_bools(backend, connect):
    connect.values.update(p_1=0.012, fm_1=0.0, sw_2=1.0)
    with backend:
        inputs = backend.read()
    assert inputs["p_1"] == pytest.approx(12.0)
    assert inputs["p_2"] == pytest.approx(4.0)
    assert inputs["fm_1"] == 0.0  # a broken loop, which the stand must still see
    assert inputs["sw_1"] is False
    assert inputs["sw_2"] is True
    assert backend.inputs == inputs


def test_read_faults_once_the_stream_stalls_and_recovers_when_it_resumes(
    backend, connect, clock
):
    with backend:
        connect.streaming = False
        clock.now += STALE_AFTER_S / 2
        backend.read()  # still within the grace period
        clock.now += STALE_AFTER_S
        with pytest.raises(BackendFault, match=DEVICE_NAME):
            backend.read()
        connect.streaming = True
        backend.read()


def test_write_sends_only_changes_until_the_periodic_resend(backend, connect, clock):
    with backend:
        connect.messages.clear()
        backend.write({"pump_1": True, "v_1": True, "v_2": 12.0})
        backend.write({**SAFE_OUTPUTS, "pump_1": True, "v_1": True, "v_2": 12.0})
        assert connect.set_outputs_sent() == ["pump_1", "v_1", "v_2"]
        assert connect.outputs["pump_1"] == 1
        assert connect.outputs["v_2"] == pytest.approx(0.012)

        connect.messages.clear()
        clock.now += RESEND_INTERVAL_S
        backend.write({"v_3": True})
        assert connect.set_outputs_sent() == [
            "pump_1",
            "v_1",
            "v_2",
            "v_3",
            "pr_2",
            "v_5",
        ]


def test_write_rejects_bad_outputs(backend):
    with backend, pytest.raises(ValueError):
        backend.write({"p_1": 12.0})


def test_close_commands_outputs_safe(backend, connect):
    backend.open()
    backend.write({"pump_1": True, "v_1": True, "v_2": 20.0, "pr_2": 12.0})
    backend.close()
    assert connect.outputs == SAFE_COMMANDS
    backend.close()  # idempotent
    with pytest.raises(RuntimeError, match="not open"):
        backend.write({"pump_1": True})


def test_channel_names_are_module_channels(backend):
    assert backend.channel_name(SIGNALS_BY_REF["fm_1"]) == "NI 9203 ai2"
    assert backend.channel_name(SIGNALS_BY_REF["v_5"]) == "NI 9476 (Spring) port0/line3"


def test_device_topic_sanitizes_like_connect():
    assert device_topic(DEVICE_NAME) == TOPIC
    assert device_topic("Fluids DAQ-1") == "devices/fluids_daq_1/command"


def test_stand_trips_while_the_device_is_silent(connect, clock):
    backend = ConnectDeviceBackend(connect, clock=clock)
    with FluidsStand(backend, clock=clock) as stand:
        stand.pressurize()
        stand.update()
        assert connect.outputs["pump_1"] == 1

        connect.streaming = False
        clock.now += STALE_AFTER_S + 0.1
        stand.update()
        assert stand.state == FluidsStandState.TRIPPED
        assert stand.trips == [
            f"I/O fault: no new data from the cDAQ device '{DEVICE_NAME}'"
        ]
        assert connect.outputs == SAFE_COMMANDS
        with pytest.raises(InterlockError, match="I/O fault"):
            stand.reset()

        stand.update()  # still silent: the trip latches once
        assert len(stand.trips) == 1

        connect.streaming = True
        stand.update()
        assert stand.io_fault is None
        stand.reset()
        assert stand.state == FluidsStandState.VENTING
