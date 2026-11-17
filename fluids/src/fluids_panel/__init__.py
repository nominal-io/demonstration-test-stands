"""Control software for the fluids panel test stand."""

from fluids_panel import sequences
from fluids_panel.backend import SAFE_OUTPUTS, Backend, BackendFault, RawValue
from fluids_panel.build import BACKENDS, build_stand
from fluids_panel.commands import CommandError, handle_message
from fluids_panel.connect_device import DEVICE_NAME, ConnectDeviceBackend
from fluids_panel.mock import MockBackend, MockFaults, MockParams
from fluids_panel.namespaces import NAMESPACES, Namespace
from fluids_panel.signals import SIGNALS, SIGNALS_BY_REF, IoKind, Signal
from fluids_panel.simulation import SIMULATION_INPUT_IDS, apply_simulation_settings
from fluids_panel.stand import (
    FluidsStand,
    FluidsStandConfig,
    FluidsStandState,
    InterlockError,
    WaitUntil,
)
from fluids_panel.states import (
    CompressorState,
    FlowState,
    LightState,
    ValveState,
    canvas_states,
)
from fluids_panel.status import NOT_RUNNING, status_message

__all__ = [
    "BACKENDS",
    "DEVICE_NAME",
    "NAMESPACES",
    "NOT_RUNNING",
    "SAFE_OUTPUTS",
    "SIGNALS",
    "SIGNALS_BY_REF",
    "SIMULATION_INPUT_IDS",
    "Backend",
    "BackendFault",
    "CommandError",
    "CompressorState",
    "ConnectDeviceBackend",
    "FlowState",
    "FluidsStand",
    "FluidsStandConfig",
    "FluidsStandState",
    "InterlockError",
    "IoKind",
    "LightState",
    "MockBackend",
    "MockFaults",
    "MockParams",
    "Namespace",
    "RawValue",
    "Signal",
    "ValveState",
    "WaitUntil",
    "apply_simulation_settings",
    "build_stand",
    "canvas_states",
    "handle_message",
    "sequences",
    "status_message",
]
