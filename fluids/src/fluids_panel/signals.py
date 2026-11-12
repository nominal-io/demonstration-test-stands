"""The fluids panel's DAQ signals, as listed in the panel's DAQ interface sheet.

Every analog signal is a 4-20 mA current loop scaled linearly over its full scale, so the
helpers here are shared by every backend that turns raw loop current into engineering units.
"""

from dataclasses import dataclass
from enum import Enum

LOOP_MIN_MA = 4.0
LOOP_MAX_MA = 20.0
LOOP_SPAN_MA = LOOP_MAX_MA - LOOP_MIN_MA

# NAMUR NE 43: readings outside this band mean a broken loop or failed transmitter, not a real value.
LOOP_FAULT_LOW_MA = 3.6
LOOP_FAULT_HIGH_MA = 21.0

# NAMUR NE 43: a healthy transmitter keeps measuring slightly past 4-20 mA before it saturates,
# so readings near zero or full scale wobble either side of the nominal range.
LOOP_SATURATION_LOW_MA = 3.8
LOOP_SATURATION_HIGH_MA = 20.5


class IoKind(Enum):
    """The direction and type of a DAQ channel."""

    DO = "do"  # NI-9476, 24 V sourcing output driving a relay
    DI = "di"  # NI-9421, 24 V sinking input
    AI = "ai"  # NI-9203, 4-20 mA current input
    AO = "ao"  # NI-9265, 4-20 mA current output

    @property
    def is_analog(self) -> bool:
        return self in (IoKind.AI, IoKind.AO)


@dataclass(frozen=True)
class Signal:
    """One DAQ channel on the panel.

    ref: The P&ID reference designator, also used as the channel name.
    kind: The channel's direction and type.
    module: The NI C-series module the channel is wired to.
    line: The channel's index on that module.
    full_scale: Engineering value at 20 mA; analog signals only.
    units: Engineering units of `full_scale`; analog signals only.
    normally_open: Whether a relay-driven valve is open while its relay is off.
    """

    ref: str
    kind: IoKind
    module: str
    line: int
    full_scale: float | None = None
    units: str | None = None
    normally_open: bool = False

    def __post_init__(self) -> None:
        if self.kind.is_analog:
            if self.full_scale is None or self.full_scale <= 0:
                raise ValueError(
                    f"{self.ref}: analog signals need a positive full_scale"
                )
        elif self.full_scale is not None or self.units is not None:
            raise ValueError(f"{self.ref}: digital signals have no full_scale or units")
        if self.normally_open and self.kind != IoKind.DO:
            raise ValueError(f"{self.ref}: only relay outputs can be normally open")


SIGNALS = (
    # Supply
    Signal("pump_1", IoKind.DO, "NI-9476", 0),
    Signal("v_1", IoKind.DO, "NI-9476", 1, normally_open=True),
    Signal("p_1", IoKind.AI, "NI-9203", 0, full_scale=100.0, units="psig"),
    # Leg 1, flow control
    Signal("v_2", IoKind.AO, "NI-9265", 0, full_scale=15.6, units="L/min"),
    Signal("fm_1", IoKind.AI, "NI-9203", 2, full_scale=100.0, units="L/min"),
    # Leg 2, linear actuation
    Signal("v_3", IoKind.DO, "NI-9476", 2),
    Signal("sw_1", IoKind.DI, "NI-9421", 0),
    Signal("sw_2", IoKind.DI, "NI-9421", 1),
    # Leg 3, proof & leak
    Signal("pr_2", IoKind.AO, "NI-9265", 1, full_scale=145.0, units="psig"),
    Signal("v_5", IoKind.DO, "NI-9476", 3, normally_open=True),
    Signal("p_2", IoKind.AI, "NI-9203", 1, full_scale=100.0, units="psig"),
)

SIGNALS_BY_REF = {signal.ref: signal for signal in SIGNALS}


def analog_full_scale(signal: Signal) -> float:
    """Get an analog signal's full scale, raising for digital signals."""
    if signal.full_scale is None:
        raise ValueError(f"{signal.ref} is not an analog signal")
    return signal.full_scale


def ma_to_eng(signal: Signal, ma: float) -> float:
    """Convert a loop current to the signal's engineering units. Not clamped, so faults stay visible."""
    return (ma - LOOP_MIN_MA) / LOOP_SPAN_MA * analog_full_scale(signal)


def eng_to_ma(signal: Signal, value: float) -> float:
    """Convert an engineering value to the loop current that commands it, clamped to 4-20 mA."""
    ma = LOOP_MIN_MA + value / analog_full_scale(signal) * LOOP_SPAN_MA
    return min(max(ma, LOOP_MIN_MA), LOOP_MAX_MA)


def is_loop_fault(ma: float) -> bool:
    """Whether a loop current is outside the NAMUR NE 43 valid band (broken wire, failed sensor)."""
    return ma < LOOP_FAULT_LOW_MA or ma > LOOP_FAULT_HIGH_MA
