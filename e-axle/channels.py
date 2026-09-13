from datetime import datetime, timedelta
from math import inf
from time import monotonic
from typing import ClassVar, Generic, Protocol, TypeVar


T = TypeVar("T")


class Comparable(Protocol):
    def __lt__(self, other, /) -> bool: ...
    def __gt__(self, other, /) -> bool: ...


TOrdered = TypeVar("TOrdered", bound=Comparable)


class Measurable(Generic[T]):
    """A quantity whose actual measured value is tracked."""

    _monotonic_origin: ClassVar[float]
    _wall_origin: ClassVar[datetime]

    def __init__(self) -> None:
        super().__init__()
        self._measured = None
        self._timestamp = None

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self.to_isoformat()}: measured={self.measured})"

    @property
    def measured(self) -> T | None:
        """Get the measured value."""
        return self._measured

    @measured.setter
    def measured(self, value: T | None) -> None:
        """Record a new measured value."""
        self._measured = value
        self._timestamp = monotonic()

    @property
    def timestamp(self) -> float | None:
        """Get the time.monotonic() reading of the last measurement attempt, or None if none has occurred."""
        return self._timestamp

    def to_isoformat(self) -> str | None:
        """Convert this channel's timestamp into an ISO 8601 wall-clock string, for display (e.g. logs)."""
        if self._timestamp is None:
            return None
        return (self._wall_origin + timedelta(seconds=self._timestamp - self._monotonic_origin)).isoformat(timespec="seconds")


Measurable._monotonic_origin = monotonic()
Measurable._wall_origin = datetime.now()


class Controllable(Measurable[T]):
    """A controllable quantity tracked as a commanded value and its actual measured value."""

    def __init__(self, default: T, **kwargs) -> None:
        super().__init__(**kwargs)
        self._default = default
        self.setpoint = default

    def __repr__(self) -> str:
        if self.requested != self.setpoint:
            return f"{self.__class__.__name__}({self.to_isoformat()}: requested={self.requested}, setpoint={self.setpoint}, measured={self.measured})"
        return f"{self.__class__.__name__}({self.to_isoformat()}: setpoint={self.setpoint}, measured={self.measured})"

    def _validate(self, value: T) -> T:
        """Hook for subclasses to transform or validate an incoming setpoint. No-op by default."""
        return value

    @property
    def setpoint(self) -> T:
        """Get the commanded value of the control point."""
        return self._setpoint

    @setpoint.setter
    def setpoint(self, value: T) -> None:
        """Set the commanded value of the control point."""
        self._requested = value
        self._setpoint = self._validate(value)

    @property
    def requested(self) -> T:
        """Get the requested value of the control point."""
        return self._requested

    @property
    def default(self) -> T:
        """Get the control point's default value."""
        return self._default

    @property
    def at_setpoint(self) -> bool:
        """Whether the measured value currently matches the commanded setpoint."""
        return self.measured == self.setpoint


class Monitorable(Measurable[TOrdered]):
    """A measured-only quantity watched against a safe minimum and maximum."""

    def __init__(self, minimum: TOrdered, maximum: TOrdered, **kwargs) -> None:
        super().__init__(**kwargs)
        if minimum > maximum:
            raise ValueError(f"minimum {minimum} is greater than maximum {maximum}")
        self._minimum = minimum
        self._maximum = maximum

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self.to_isoformat()}: minimum={self.minimum}, measured={self.measured}, maximum={self.maximum})"

    @property
    def minimum(self) -> TOrdered:
        """Get the safe minimum for the measured value."""
        return self._minimum

    @property
    def maximum(self) -> TOrdered:
        """Get the safe maximum for the measured value."""
        return self._maximum

    @property
    def tripped(self) -> bool:
        """Whether the measured value has left the safe range."""
        return self.measured is not None and (self.measured < self._minimum or self.measured > self._maximum)


class ControllableNumeric(Controllable[float], Monitorable[float]):
    """A numeric channel that is both commandable, clamped to a range, and monitored for trips."""

    def __init__(self, default: float, minimum: float = -inf, maximum: float = inf, deadband: float = 0.0) -> None:
        if minimum > maximum:
            raise ValueError(f"minimum {minimum} is greater than maximum {maximum}")
        if default < minimum or default > maximum:
            raise ValueError(f"Default value {default} is outside of bounds [{minimum}, {maximum}]")
        if deadband < 0:
            raise ValueError(f"deadband {deadband} is negative")
        super().__init__(default=default, minimum=minimum, maximum=maximum)
        self._deadband = deadband

    def _validate(self, value: float) -> float:
        """Clamp the incoming setpoint to the channel's range."""
        if value < self._minimum:
            return self._minimum
        if value > self._maximum:
            return self._maximum
        return value

    @property
    def deadband(self) -> float:
        """Get how far the measured value may sit from the setpoint and still count as at-setpoint."""
        return self._deadband

    @property
    def at_setpoint(self) -> bool:
        """Whether the measured value is within the deadband of the commanded setpoint."""
        return self.measured is not None and abs(self.measured - self.setpoint) <= self._deadband
