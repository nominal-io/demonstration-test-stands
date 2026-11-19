"""The real panel through Connect's own NI-DAQmx device, behind the Backend interface.

Connect's Devices panel owns the cDAQ. `app/fluids_daq.ni-daqmx.json`, loaded through `app.connect`'s
`default_device_configs`, names every line by its P&ID ref, and the device streams them all to
the DEVICE_NAME stream. This backend reads the latest values from that stream and commands
outputs over the message bus, with the same payloads as `client.send_device_command`.

The device works in amps (its config sets no unit or scale) and the Backend interface in mA;
the conversion happens here and nowhere else.

The message bus promises less than a direct driver, so:

- Inputs are copies and can go stale. If any input's timestamp stops advancing for
  STALE_AFTER_S, `read()` raises BackendFault and the stand trips.
- Output commands are fire-and-forget. Each is sent when it changes and all of them are re-sent
  every RESEND_INTERVAL_S, so a lost message gets corrected. It also means the script, not the
  Devices panel, owns the outputs while it runs.

The device keeps running after the script stops. `close()` commands every output safe, and the
config starts the device with them safe; past that, the hardware E-stop is what makes the panel
safe.
"""

import logging
import math
import time
from collections.abc import Callable, Mapping
from typing import Any, Protocol

from fluids_panel.backend import (
    INPUT_REFS,
    OUTPUT_REFS,
    SAFE_OUTPUTS,
    Backend,
    BackendFault,
    RawValue,
    check_outputs,
)
from fluids_panel.signals import SIGNALS_BY_REF, IoKind, Signal

logger = logging.getLogger(__name__)

# The device's name in Connect, which is also its stream id. `app/fluids_daq.ni-daqmx.json` sets it.
DEVICE_NAME = "fluids_daq"

MA_PER_A = 1000.0

STALE_AFTER_S = 2.0
RESEND_INTERVAL_S = 1.0
START_TIMEOUT_S = 15.0
START_POLL_S = 0.1


class DeviceClient(Protocol):
    """The parts of `connect_python.Client` this backend uses."""

    def get_channel_values(
        self, stream_id: str, channels: str | list[str] | None = None
    ) -> dict[str, Any] | None: ...

    def send_message(self, topic: str, contents: dict) -> None: ...


# What DAQmx calls each module the sheet names. The device config matches modules by exactly
# this name, so a mismatch silently leaves that module's channels unconfigured. The installed
# NI-9476 is the spring-terminal variant.
PRODUCT_TYPES = {
    "NI-9476": "NI 9476 (Spring)",
    "NI-9421": "NI 9421",
    "NI-9203": "NI 9203",
    "NI-9265": "NI 9265",
}


def relative_channel(signal: Signal) -> str:
    """A signal's channel on its module, as DAQmx and the device config name it, e.g. `ai2`."""
    match signal.kind:
        case IoKind.DO | IoKind.DI:
            return f"port0/line{signal.line}"
        case IoKind.AI:
            return f"ai{signal.line}"
        case IoKind.AO:
            return f"ao{signal.line}"


def device_topic(device_name: str) -> str:
    """The message-bus topic a Connect device takes commands on, derived from its name."""
    sanitized = "".join(
        c.lower() if c.isascii() and (c.isalnum() or c == "_") else "_"
        for c in device_name
    )
    return f"devices/{sanitized}/command"


class ConnectDeviceBackend(Backend):
    """The panel's cDAQ, as Connect's NI-DAQmx device streams and commands it."""

    def __init__(
        self,
        client: DeviceClient,
        device_name: str = DEVICE_NAME,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._client = client
        self.device_name = device_name
        self._topic = device_topic(device_name)
        self._clock = clock
        self._sleep = sleep
        self._open = False
        self._outputs: dict[str, RawValue] = dict(SAFE_OUTPUTS)
        self._inputs: dict[str, RawValue] = {}
        # Per input: the stream's latest timestamp and value, and when (on our clock) the
        # timestamp last moved.
        self._timestamps: dict[str, Any] = {}
        self._latest: dict[str, Any] = {}
        self._fresh_at: dict[str, float] = {}
        self._sent_at = -math.inf

    def open(self) -> None:
        """Start the device streaming, command every output safe, and wait for fresh inputs."""
        if self._open:
            raise RuntimeError("ConnectDeviceBackend is already open")
        # Whatever the stream already holds predates this run; only newer samples count.
        self._timestamps = {ref: ts for ref, (ts, _) in self._fetch().items()}
        self._latest, self._fresh_at, self._inputs = {}, {}, {}
        self._client.send_message(self._topic, {"set_streaming": True})
        self._open = True
        self._outputs = dict(SAFE_OUTPUTS)
        self._send(self._outputs)
        self._sent_at = self._clock()

        deadline = self._clock() + START_TIMEOUT_S
        while missing := self._refresh():
            if self._clock() >= deadline:
                self._open = False
                raise RuntimeError(
                    f"no data from the Connect device {self.device_name!r} for {missing} "
                    f"within {START_TIMEOUT_S:g} s. Is the cDAQ streaming in the Devices "
                    "panel, with app/fluids_daq.ni-daqmx.json applied?"
                )
            self._sleep(START_POLL_S)
        logger.info("Connect device %s is streaming", self.device_name)

    def close(self) -> None:
        if not self._open:
            return
        try:
            self._outputs = dict(SAFE_OUTPUTS)
            self._send(self._outputs)
        finally:
            self._open = False

    def read(self) -> dict[str, RawValue]:
        self._require_open()
        if self._refresh():
            raise BackendFault(f"no new data from the cDAQ device {self.device_name!r}")
        inputs: dict[str, RawValue] = {}
        for ref in INPUT_REFS:
            value = self._latest[ref]
            if SIGNALS_BY_REF[ref].kind == IoKind.DI:
                inputs[ref] = bool(value)
            else:
                inputs[ref] = float(value) * MA_PER_A
        self._inputs = inputs
        return dict(inputs)

    def write(self, outputs: Mapping[str, RawValue]) -> None:
        check_outputs(outputs)
        self._require_open()
        changed = {ref: v for ref, v in outputs.items() if self._outputs[ref] != v}
        self._outputs.update(outputs)
        now = self._clock()
        if now - self._sent_at >= RESEND_INTERVAL_S:
            self._send(self._outputs)
            self._sent_at = now
        elif changed:
            self._send(changed)

    @property
    def inputs(self) -> dict[str, RawValue]:
        return dict(self._inputs)

    @property
    def outputs(self) -> dict[str, RawValue]:
        return dict(self._outputs)

    def channel_name(self, signal: Signal) -> str:
        """The module and channel the device config wires a signal to, e.g. `NI 9203 ai2`."""
        return f"{PRODUCT_TYPES[signal.module]} {relative_channel(signal)}"

    def _require_open(self) -> None:
        if not self._open:
            raise RuntimeError("ConnectDeviceBackend is not open")

    def _fetch(self) -> dict[str, tuple[Any, Any]]:
        """The stream's latest (timestamp, value) for each input it has."""
        values = self._client.get_channel_values(self.device_name, list(INPUT_REFS))
        return {
            ref: (entry["timestamp"], entry["value"])
            for ref, entry in (values or {}).items()
            if ref in INPUT_REFS
        }

    def _refresh(self) -> list[str]:
        """Take the stream's latest values, returning the inputs with no new sample lately."""
        now = self._clock()
        for ref, (timestamp, value) in self._fetch().items():
            if timestamp != self._timestamps.get(ref):
                self._timestamps[ref] = timestamp
                self._latest[ref] = value
                self._fresh_at[ref] = now
        return [
            ref
            for ref in INPUT_REFS
            if now - self._fresh_at.get(ref, -math.inf) > STALE_AFTER_S
        ]

    def _send(self, outputs: Mapping[str, RawValue]) -> None:
        """Command each output, in sheet order: relays as 0/1, analog outputs in amps."""
        for ref in OUTPUT_REFS:
            if ref not in outputs:
                continue
            value = outputs[ref]
            if SIGNALS_BY_REF[ref].kind == IoKind.DO:
                command = {"binary": int(bool(value))}
            else:
                command = {"constant": float(value) / MA_PER_A}
            self._client.send_message(
                self._topic,
                {"set_output": {"channel_alias": ref, "value": command}},
            )
