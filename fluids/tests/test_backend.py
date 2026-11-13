import pytest

from fluids_panel.backend import INPUT_REFS, OUTPUT_REFS, SAFE_OUTPUTS, check_outputs


def test_inputs_and_outputs_partition_the_signals():
    assert set(INPUT_REFS) == {"p_1", "p_2", "fm_1", "sw_1", "sw_2"}
    assert set(OUTPUT_REFS) == {"pump_1", "v_1", "v_2", "v_3", "v_5", "pr_2"}


def test_safe_outputs_de_energize_everything():
    assert SAFE_OUTPUTS == {
        "pump_1": False,
        "v_1": False,
        "v_3": False,
        "v_5": False,
        "v_2": 4.0,
        "pr_2": 4.0,
    }


def test_check_outputs_accepts_valid_values():
    check_outputs({"pump_1": True, "v_2": 12.0, "pr_2": 4})


@pytest.mark.parametrize(
    ("outputs", "error"),
    [
        ({"nope": True}, ValueError),
        ({"p_1": 12.0}, ValueError),
        ({"sw_1": True}, ValueError),
        ({"pump_1": 1}, TypeError),
        ({"v_2": True}, TypeError),
        ({"v_2": "12"}, TypeError),
    ],
)
def test_check_outputs_rejects(outputs, error):
    with pytest.raises(error):
        check_outputs(outputs)
