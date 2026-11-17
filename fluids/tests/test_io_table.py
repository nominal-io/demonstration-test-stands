from fluids_panel.io_table import COLUMNS, NO_SAMPLE, io_table
from fluids_panel.mock import MockBackend
from fluids_panel.signals import SIGNALS


def rows(backend: MockBackend) -> dict[str, dict[str, str]]:
    """The table as Connect lays it out: `data` holds one list per column."""
    table = io_table(backend)
    assert table["columns"] == COLUMNS
    assert table["error"] is None
    assert table["data"] is not None
    assert len(table["data"]) == len(COLUMNS)
    return {
        row[0]: dict(zip(COLUMNS, row, strict=True))
        for row in zip(*table["data"], strict=True)
    }


def test_one_row_per_signal_in_sheet_order(mock):
    mock.open()
    table = io_table(mock)
    assert table["data"] is not None
    signal_column = table["data"][COLUMNS.index("Signal")]
    assert signal_column == [s.ref for s in SIGNALS]


def test_inputs_show_no_sample_before_the_first_read(mock):
    mock.open()
    p_1 = rows(mock)["p_1"]
    assert (p_1["Raw"], p_1["Value"], p_1["Status"]) == (
        NO_SAMPLE,
        NO_SAMPLE,
        "no sample yet",
    )


def test_outputs_show_what_was_commanded(mock):
    mock.open()
    mock.write({"v_1": True, "pump_1": True, "v_2": 12.0})
    table = rows(mock)
    assert table["v_1"] == {
        "Signal": "v_1",
        "Channel": "NI-9476 line 1",
        "I/O": "DO",
        "Raw": "ON",
        "Value": "valve closed",
        "Status": "commanded",
    }
    assert table["v_5"]["Value"] == "valve open"
    assert table["pump_1"]["Value"] == "energized"
    assert table["v_2"]["Raw"] == "12.00 mA"
    assert table["v_2"]["Value"] == "7.8 L/min"
    assert table["pr_2"]["Value"] == "0.0 psig"


def test_inputs_show_the_last_sample(mock):
    mock.open()
    mock.plant.p_1_psi = 50.0
    mock.plant.actuator_position = 1.0
    mock.read()
    table = rows(mock)
    assert table["p_1"]["Raw"] == "12.00 mA"
    assert table["p_1"]["Value"] == "50.0 psig"
    assert table["p_1"]["Status"] == "OK"
    assert table["sw_2"]["Raw"] == "HIGH"
    assert table["sw_1"]["Raw"] == "LOW"


def test_broken_loop_shows_a_fault_not_a_value(mock):
    mock.open()
    mock.faults.broken_wires.add("fm_1")
    mock.read()
    fm_1 = rows(mock)["fm_1"]
    assert (fm_1["Raw"], fm_1["Value"], fm_1["Status"]) == (
        "0.00 mA",
        NO_SAMPLE,
        "LOOP FAULT",
    )
