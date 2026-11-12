"""The raw I/O boundary between the stand and whatever is behind the DAQ channels.

A backend only moves raw signal values: relay outputs and digital inputs as bools, analog
outputs and inputs as loop current in mA. Scaling, valve polarity and state derivation all
live above this layer, so a mock and the real hardware exercise the same stand code.
"""

from abc import ABC, abstractmethod
from collections.abc import Mapping
from types import TracebackType
from typing import Self

from fluids_panel.signals import LOOP_MIN_MA, SIGNALS, SIGNALS_BY_REF, IoKind

type RawValue = bool | float

INPUT_REFS = tuple(s.ref for s in SIGNALS if s.kind in (IoKind.DI, IoKind.AI))
OUTPUT_REFS = tuple(s.ref for s in SIGNALS if s.kind in (IoKind.DO, IoKind.AO))

# Every relay de-energized and every analog output at its 0% current. Normally-open valves
# are therefore open, venting the panel, which is also what the E-stop does in hardware.
SAFE_OUTPUTS: Mapping[str, RawValue] = {
    s.ref: (False if s.kind == IoKind.DO else LOOP_MIN_MA)
    for s in SIGNALS
    if s.kind in (IoKind.DO, IoKind.AO)
}


def check_outputs(outputs: Mapping[str, RawValue]) -> None:
    """Raise if any entry is not an output signal or has the wrong raw type for its kind."""
    for ref, value in outputs.items():
        signal = SIGNALS_BY_REF.get(ref)
        if signal is None or signal.kind not in (IoKind.DO, IoKind.AO):
            raise ValueError(f"{ref!r} is not an output signal")
        if signal.kind == IoKind.DO and not isinstance(value, bool):
            raise TypeError(f"{ref}: relay outputs take a bool, got {value!r}")
        if signal.kind == IoKind.AO and (
            isinstance(value, bool) or not isinstance(value, int | float)
        ):
            raise TypeError(
                f"{ref}: analog outputs take a current in mA, got {value!r}"
            )


class Backend(ABC):
    """Raw access to the panel's DAQ channels."""

    @abstractmethod
    def open(self) -> None:
        """Acquire the channels and drive every output to SAFE_OUTPUTS."""

    @abstractmethod
    def close(self) -> None:
        """Drive every output to SAFE_OUTPUTS and release the channels."""

    @abstractmethod
    def read(self) -> dict[str, RawValue]:
        """Sample every input: digital inputs as bools, analog inputs as mA."""

    @abstractmethod
    def write(self, outputs: Mapping[str, RawValue]) -> None:
        """Update some or all outputs: relays as bools, analog outputs as mA. Others hold their value."""

    def __enter__(self) -> Self:
        self.open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()
