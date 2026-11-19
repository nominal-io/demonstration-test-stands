"""A live table of every DAQ line, for checking the panel's wiring and signals at a glance.

One row per signal, in the order of the DAQ interface sheet: where it is wired, its raw value on
the line, that value in engineering units, and whether it looks healthy. The Connect script
passes it to `client.set_output`, which a `display: table` element renders.

Inputs are the last sample read. Outputs are what was last commanded: the NI-9476 and NI-9265
can't report their lines, so a relay the E-stop has de-powered still reads ON here.
"""

from typing import TypedDict

from fluids_panel.backend import Backend, RawValue
from fluids_panel.signals import SIGNALS, IoKind, Signal, is_loop_fault, ma_to_eng

COLUMNS = ["Signal", "Channel", "I/O", "Raw", "Value", "Status"]

NO_SAMPLE = "—"


class IoTable(TypedDict):
    """connect_python's TableData, which `display: table` renders, without importing Connect.

    `data` is column-major: `data[i]` holds every cell under `columns[i]`. Connect zips the two
    (`connect-user-value`'s `TableData`), whatever TableData's docstring suggests; rows passed
    instead get paired off with the column names and the rest dropped.
    """

    columns: list[str] | None
    data: list[list[str]] | None
    error: str | None


def io_table(backend: Backend) -> IoTable:
    """Tabulate every signal's raw and engineering value on `backend`."""
    inputs, outputs = backend.inputs, backend.outputs
    rows = []
    for signal in SIGNALS:
        is_output = signal.kind in (IoKind.DO, IoKind.AO)
        raw = (outputs if is_output else inputs).get(signal.ref)
        rows.append(
            [
                signal.ref,
                backend.channel_name(signal),
                signal.kind.name,
                *_describe(signal, raw),
            ]
        )
    columns = [list(column) for column in zip(*rows, strict=True)]
    return {"columns": list(COLUMNS), "data": columns, "error": None}


def _describe(signal: Signal, raw: RawValue | None) -> tuple[str, str, str]:
    """A signal's raw value, engineering value and status, as table cells."""
    if raw is None:
        return NO_SAMPLE, NO_SAMPLE, "no sample yet"
    match signal.kind:
        case IoKind.DO:
            if signal.normally_open:
                value = "valve closed" if raw else "valve open"
            else:
                value = "energized" if raw else "off"
            return ("ON" if raw else "OFF"), value, "commanded"
        case IoKind.DI:
            return ("HIGH" if raw else "LOW"), ("active" if raw else "inactive"), "OK"
        case IoKind.AO:
            return _ma(raw), _eng(signal, raw), "commanded"
        case IoKind.AI:
            if is_loop_fault(float(raw)):
                return _ma(raw), NO_SAMPLE, "LOOP FAULT"
            return _ma(raw), _eng(signal, raw), "OK"


def _ma(raw: RawValue) -> str:
    return f"{float(raw):.2f} mA"


def _eng(signal: Signal, raw: RawValue) -> str:
    return f"{ma_to_eng(signal, float(raw)):.1f} {signal.units}"
