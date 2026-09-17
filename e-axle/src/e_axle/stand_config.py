from dataclasses import dataclass
from importlib.resources import files
from pathlib import Path
from typing import TypeVar

import yaml

T = TypeVar("T")

DEFAULT_CONFIG_RESOURCE = "nominal_config.yaml"
"""Name of the config shipped inside the package -- see `EAxleStandConfig.default`."""


@dataclass(frozen=True)
class ControllableConfig[T]:
    default: T


@dataclass(frozen=True)
class ControllableNumericConfig(ControllableConfig[float]):
    minimum: float
    maximum: float


@dataclass(frozen=True)
class MonitorableConfig:
    """Safe range for a monitor-only channel -- no default, since nothing is commanded."""

    minimum: float
    maximum: float


@dataclass(frozen=True)
class DutControllerConfig:
    torque: ControllableNumericConfig
    speed: ControllableNumericConfig
    current: ControllableNumericConfig
    temperature: MonitorableConfig


@dataclass(frozen=True)
class LoadControllerConfig:
    torque: ControllableNumericConfig
    speed: ControllableNumericConfig
    current: ControllableNumericConfig
    temperature: MonitorableConfig


@dataclass(frozen=True)
class SourceConfig:
    psu_channel_number: int
    voltage: ControllableNumericConfig
    current: ControllableNumericConfig
    enabled: ControllableConfig[bool]
    ovp_limit: ControllableNumericConfig
    ocp_limit: ControllableNumericConfig


@dataclass(frozen=True)
class SinkConfig:
    psu_channel_number: int
    voltage: ControllableNumericConfig  # CV setpoint; instro's sink-side CV rejection is being fixed separately
    current: ControllableNumericConfig  # current limit, passed alongside voltage as set_level()'s curr_limit
    enabled: ControllableConfig[bool]


def _numeric_from_dict(data: dict) -> ControllableNumericConfig:
    return ControllableNumericConfig(
        default=data["default"], minimum=data["minimum"], maximum=data["maximum"]
    )


def _numeric_to_dict(channel: ControllableNumericConfig) -> dict:
    return {
        "default": channel.default,
        "minimum": channel.minimum,
        "maximum": channel.maximum,
    }


def _bounded_from_dict(data: dict) -> MonitorableConfig:
    return MonitorableConfig(minimum=data["minimum"], maximum=data["maximum"])


def _bounded_to_dict(channel: MonitorableConfig) -> dict:
    return {"minimum": channel.minimum, "maximum": channel.maximum}


@dataclass(frozen=True)
class EAxleStandConfig:
    dut_controller: DutControllerConfig
    left_load_controller: LoadControllerConfig
    right_load_controller: LoadControllerConfig
    source: SourceConfig
    sink: SinkConfig
    disarm_timeout_s: float
    arm_timeout_s: float
    stop_timeout_s: float
    trip_stop_timeout_s: float
    boot_timeout_s: float

    @classmethod
    def default(cls) -> "EAxleStandConfig":
        """Load the config shipped inside the package.

        Read through `importlib.resources` rather than a filesystem path so this works
        from an installed (zipped) wheel, where no `nominal_config.yaml` sits on disk.
        These are defaults -- several bounds are still PLACEHOLDERs -- so a stand that
        needs different limits should override them via `from_yaml`.
        """
        with (files(__package__) / DEFAULT_CONFIG_RESOURCE).open() as f:
            return cls._from_dict(yaml.safe_load(f))

    @classmethod
    def from_yaml(cls, path: str | Path) -> "EAxleStandConfig":
        with open(path) as f:
            return cls._from_dict(yaml.safe_load(f))

    @classmethod
    def _from_dict(cls, data: dict) -> "EAxleStandConfig":
        return cls(
            disarm_timeout_s=data["disarm_timeout_s"],
            arm_timeout_s=data["arm_timeout_s"],
            stop_timeout_s=data["stop_timeout_s"],
            trip_stop_timeout_s=data["trip_stop_timeout_s"],
            boot_timeout_s=data["boot_timeout_s"],
            dut_controller=DutControllerConfig(
                torque=_numeric_from_dict(data["dut_controller"]["torque"]),
                speed=_numeric_from_dict(data["dut_controller"]["speed"]),
                current=_numeric_from_dict(data["dut_controller"]["current"]),
                temperature=_bounded_from_dict(data["dut_controller"]["temperature"]),
            ),
            left_load_controller=LoadControllerConfig(
                torque=_numeric_from_dict(data["left_load_controller"]["torque"]),
                speed=_numeric_from_dict(data["left_load_controller"]["speed"]),
                current=_numeric_from_dict(data["left_load_controller"]["current"]),
                temperature=_bounded_from_dict(
                    data["left_load_controller"]["temperature"]
                ),
            ),
            right_load_controller=LoadControllerConfig(
                torque=_numeric_from_dict(data["right_load_controller"]["torque"]),
                speed=_numeric_from_dict(data["right_load_controller"]["speed"]),
                current=_numeric_from_dict(data["right_load_controller"]["current"]),
                temperature=_bounded_from_dict(
                    data["right_load_controller"]["temperature"]
                ),
            ),
            source=SourceConfig(
                psu_channel_number=data["source"]["psu_channel_number"],
                voltage=_numeric_from_dict(data["source"]["voltage"]),
                current=_numeric_from_dict(data["source"]["current"]),
                enabled=ControllableConfig(default=data["source"]["enabled"]),
                ovp_limit=_numeric_from_dict(data["source"]["ovp_limit"]),
                ocp_limit=_numeric_from_dict(data["source"]["ocp_limit"]),
            ),
            sink=SinkConfig(
                psu_channel_number=data["sink"]["psu_channel_number"],
                voltage=_numeric_from_dict(data["sink"]["voltage"]),
                current=_numeric_from_dict(data["sink"]["current"]),
                enabled=ControllableConfig(default=data["sink"]["enabled"]),
            ),
        )

    def to_dict(self) -> dict:
        return {
            "disarm_timeout_s": self.disarm_timeout_s,
            "arm_timeout_s": self.arm_timeout_s,
            "stop_timeout_s": self.stop_timeout_s,
            "trip_stop_timeout_s": self.trip_stop_timeout_s,
            "boot_timeout_s": self.boot_timeout_s,
            "dut_controller": {
                "torque": _numeric_to_dict(self.dut_controller.torque),
                "speed": _numeric_to_dict(self.dut_controller.speed),
                "current": _numeric_to_dict(self.dut_controller.current),
                "temperature": _bounded_to_dict(self.dut_controller.temperature),
            },
            "left_load_controller": {
                "torque": _numeric_to_dict(self.left_load_controller.torque),
                "speed": _numeric_to_dict(self.left_load_controller.speed),
                "current": _numeric_to_dict(self.left_load_controller.current),
                "temperature": _bounded_to_dict(self.left_load_controller.temperature),
            },
            "right_load_controller": {
                "torque": _numeric_to_dict(self.right_load_controller.torque),
                "speed": _numeric_to_dict(self.right_load_controller.speed),
                "current": _numeric_to_dict(self.right_load_controller.current),
                "temperature": _bounded_to_dict(self.right_load_controller.temperature),
            },
            "source": {
                "psu_channel_number": self.source.psu_channel_number,
                "voltage": _numeric_to_dict(self.source.voltage),
                "current": _numeric_to_dict(self.source.current),
                "enabled": self.source.enabled.default,
                "ovp_limit": _numeric_to_dict(self.source.ovp_limit),
                "ocp_limit": _numeric_to_dict(self.source.ocp_limit),
            },
            "sink": {
                "psu_channel_number": self.sink.psu_channel_number,
                "voltage": _numeric_to_dict(self.sink.voltage),
                "current": _numeric_to_dict(self.sink.current),
                "enabled": self.sink.enabled.default,
            },
        }

    def dump_to_yaml(self, path: str | Path) -> None:
        with open(path, "w") as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False)
