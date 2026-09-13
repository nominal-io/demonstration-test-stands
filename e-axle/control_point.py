from math import inf
from typing import Generic, TypeVar


T = TypeVar("T")


class ControlPoint(Generic[T]):
    """A controllable quantity tracked as a commanded value and its actual measured value."""

    def __init__(self, default: T) -> None:
        self._default = default
        self._measured: T | None = None
        self.setpoint = default

    def __repr__(self) -> str:
        if self._requested != self._setpoint:
            return f"{self.__class__.__name__}(requested={self._requested}, setpoint={self.setpoint}, measured={self.measured})"
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

    @property
    def measured(self) -> T | None:
        """Get the measured value of the control point."""
        return self._measured

    @measured.setter
    def measured(self, value: T | None) -> None:
        """Record a new measured value."""
        self._measured = value


class ControlPointNumeric(ControlPoint[float]):
    """A control point whose commanded value is clamped to a numeric range."""

    def __init__(self, default: float, minimum: float = -inf, maximum: float = inf) -> None:
        if minimum > maximum:
            raise ValueError(f"minimum {minimum} is greater than maximum {maximum}")
        self._minimum = minimum
        self._maximum = maximum
        if default < self._minimum or default > self._maximum:
            raise ValueError(f"Default value {default} is outside of bounds [{self._minimum}, {self._maximum}]")
        super().__init__(default)

    @property
    def minimum(self) -> float:
        """Get the control point's minimum allowed value."""
        return self._minimum

    @property
    def maximum(self) -> float:
        """Get the control point's maximum allowed value."""
        return self._maximum

    def _validate(self, value: float) -> float:
        """Clamp the incoming setpoint to the control point's range."""
        if value < self._minimum:
            return self._minimum
        if value > self._maximum:
            return self._maximum
        return value
