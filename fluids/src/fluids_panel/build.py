"""Composition root: builds a FluidsStand on the chosen backend."""

from fluids_panel.connect_device import ConnectDeviceBackend, DeviceClient
from fluids_panel.mock import MockBackend
from fluids_panel.stand import FluidsStand, FluidsStandConfig

BACKENDS = ("mock", "daq")


def build_stand(
    backend: str = "mock",
    config: FluidsStandConfig | None = None,
    *,
    client: DeviceClient | None = None,
) -> FluidsStand:
    """Build a stand on the named backend.

    `mock` is the simulated panel. `daq` is the cDAQ through Connect's NI-DAQmx device, which
    needs the script's Connect `client` to reach it.
    """
    if backend == "mock":
        return FluidsStand(MockBackend(), config)
    if backend == "daq":
        if client is None:
            raise ValueError(
                "the daq backend needs a Connect client to reach the device"
            )
        return FluidsStand(ConnectDeviceBackend(client), config)
    raise ValueError(f"unknown backend {backend!r}; expected one of {BACKENDS}")
