import pytest

from fluids_panel.states import (
    CompressorState,
    FlowState,
    LightState,
    ValveState,
    canvas_states,
    connect_variant,
)

# Variant names of Connect's GateValveState, CompressorState, FlowMeterState and
# StatusLightState (connect-config node_config), in declaration order.
CONNECT_VARIANTS = {
    ValveState: ["Closed", "Open", "Partial", "Transitioning", "Fault"],
    CompressorState: ["Off", "Running", "Fault"],
    FlowState: ["NoFlow", "Flowing", "ReverseFlow", "Fault"],
    LightState: ["Off", "Ok", "Warning", "Fault"],
}


@pytest.mark.parametrize("enum", CONNECT_VARIANTS)
def test_members_map_to_connect_variants(enum):
    assert [connect_variant(member) for member in enum] == CONNECT_VARIANTS[enum]


@pytest.mark.parametrize("enum", CONNECT_VARIANTS)
def test_codes_are_unique(enum):
    assert len({member.value for member in enum}) == len(enum)


def test_canvas_states_shape():
    assert canvas_states(FlowState)[:2] == [
        {"condition": {"kind": "equal", "value": 0.0}, "value": "NoFlow"},
        {"condition": {"kind": "equal", "value": 1.0}, "value": "Flowing"},
    ]


def test_canvas_states_covers_every_member():
    assert len(canvas_states(ValveState)) == len(ValveState)
