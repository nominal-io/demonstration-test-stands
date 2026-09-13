from math import inf
from typing import Generic, Protocol, TypeVar


T = TypeVar("T")


class Comparable(Protocol):
    def __lt__(self, other, /) -> bool: ...
    def __gt__(self, other, /) -> bool: ...


TOrdered = TypeVar("TOrdered", bound=Comparable)


class Measurable(Generic[T]):
    """A quantity whose actual measured value is tracked."""

    def __init__(self) -> None:
        super().__init__()
        self._measured: T | None = None

    @property
    def measured(self) -> T | None:
        """Get the measured value."""
        return self._measured

    @measured.setter
    def measured(self, value: T | None) -> None:
        """Record a new measured value."""
        self._measured = value


class Controllable(Measurable[T]):
    """A controllable quantity tracked as a commanded value and its actual measured value."""

    def __init__(self, default: T, **kwargs) -> None:
        super().__init__(**kwargs)
        self._default = default
        self.setpoint = default

    def __repr__(self) -> str:
        if self.requested != self.setpoint:
            return f"{self.__class__.__name__}(requested={self.requested}, setpoint={self.setpoint}, measured={self.measured})"
        return f"{self.__class__.__name__}(setpoint={self.setpoint}, measured={self.measured})"

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


class Monitorable(Measurable[TOrdered]):
    """A measured-only quantity watched against a safe minimum and maximum."""

    def __init__(self, minimum: TOrdered, maximum: TOrdered, **kwargs) -> None:
        super().__init__(**kwargs)
        if minimum > maximum:
            raise ValueError(f"minimum {minimum} is greater than maximum {maximum}")
        self._minimum = minimum
        self._maximum = maximum

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(minimum={self.minimum}, measured={self.measured}, maximum={self.maximum})"

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


class NumericControlChannel(Controllable[float], Monitorable[float]):
    """A numeric channel that is both commandable, clamped to a range, and monitored for trips."""

    def __init__(self, default: float, minimum: float = -inf, maximum: float = inf) -> None:
        if minimum > maximum:
            raise ValueError(f"minimum {minimum} is greater than maximum {maximum}")
        if default < minimum or default > maximum:
            raise ValueError(f"Default value {default} is outside of bounds [{minimum}, {maximum}]")
        super().__init__(default=default, minimum=minimum, maximum=maximum)

    def _validate(self, value: float) -> float:
        """Clamp the incoming setpoint to the channel's range."""
        if value < self._minimum:
            return self._minimum
        if value > self._maximum:
            return self._maximum
        return value
