"""Composition root: builds a FluidsStand on the chosen backend.

Only the mock exists so far; the hardware backend (NI-DAQmx on the cDAQ) is added here once
the stand PC and its DAQ library are settled.
"""

from fluids_panel.mock import MockBackend
from fluids_panel.stand import FluidsStand, FluidsStandConfig

BACKENDS = ("mock",)


def build_stand(
    backend: str = "mock", config: FluidsStandConfig | None = None
) -> FluidsStand:
    """Build a stand on the named backend, defaulting to the simulated panel."""
    if backend == "mock":
        return FluidsStand(MockBackend(), config)
    raise ValueError(f"unknown backend {backend!r}; expected one of {BACKENDS}")
