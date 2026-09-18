from pathlib import Path

from e_axle.channels import Controllable, ControllableNumeric, Monitorable
from e_axle.stand_config import EAxleStandConfig

_CONFIG_PATH = Path(__file__).parent.parent / "nominal_config.yaml"


def _nominal_config() -> EAxleStandConfig:
    return EAxleStandConfig.from_yaml(_CONFIG_PATH)


def test_nominal_config_initializes_every_stand_channel():
    config = _nominal_config()

    # DUT Controller - verify all three channel types
    dut_torque = ControllableNumeric(**config.dut_controller.torque.__dict__)
    dut_speed = ControllableNumeric(**config.dut_controller.speed.__dict__)
    dut_current = ControllableNumeric(**config.dut_controller.current.__dict__)

    assert dut_speed.maximum == 3_000.0
    assert dut_speed.minimum == -3_000.0
    assert dut_current.maximum == 35.0
    assert dut_current.minimum == -35.0
    assert round(dut_torque.maximum, 3) == 27.503
    dut_temperature = Monitorable(**config.dut_controller.temperature.__dict__)
    assert dut_temperature.minimum == 0.0
    assert dut_temperature.maximum == 100.0

    # Left Load Controller - verify torque bound (asymmetric)
    left_load_torque = ControllableNumeric(
        **config.left_load_controller.torque.__dict__
    )
    left_load_speed = ControllableNumeric(**config.left_load_controller.speed.__dict__)
    left_load_current = ControllableNumeric(
        **config.left_load_controller.current.__dict__
    )

    assert left_load_current.maximum == 20.0
    assert round(left_load_torque.maximum, 3) == 3.812
    assert left_load_speed.maximum == 471.429
    assert left_load_speed.minimum == -471.429
    left_load_temperature = Monitorable(
        **config.left_load_controller.temperature.__dict__
    )
    assert left_load_temperature.minimum == 0.0
    assert left_load_temperature.maximum == 100.0

    # Right Load Controller - verify torque bound (slightly different from left)
    right_load_torque = ControllableNumeric(
        **config.right_load_controller.torque.__dict__
    )
    right_load_speed = ControllableNumeric(
        **config.right_load_controller.speed.__dict__
    )
    right_load_current = ControllableNumeric(
        **config.right_load_controller.current.__dict__
    )

    assert right_load_current.maximum == 20.0
    assert round(right_load_torque.maximum, 3) == 3.80
    assert right_load_speed.maximum == 471.429
    assert right_load_speed.minimum == -471.429
    right_load_temperature = Monitorable(
        **config.right_load_controller.temperature.__dict__
    )
    assert right_load_temperature.minimum == 0.0
    assert right_load_temperature.maximum == 100.0

    # Source PSU - verify addressing and both numeric and boolean channels
    source_voltage = ControllableNumeric(**config.source.voltage.__dict__)
    source_current = ControllableNumeric(**config.source.current.__dict__)
    source_enabled = Controllable(default=config.source.enabled.default)

    assert config.source.psu_channel_number == 1
    assert source_voltage.default == 48.0
    assert source_voltage.maximum == 54.0
    assert source_current.maximum == 20.0
    assert source_enabled.setpoint is False

    # Sink PSU - verify addressing; voltage is a CV setpoint, current is its CV-mode limit
    sink_voltage = ControllableNumeric(**config.sink.voltage.__dict__)
    sink_current = ControllableNumeric(**config.sink.current.__dict__)
    sink_enabled = Controllable(default=config.sink.enabled.default)

    assert config.sink.psu_channel_number == 1
    assert sink_voltage.default == 48.0
    assert sink_voltage.maximum == 54.0
    assert sink_current.maximum == 20.0
    assert sink_enabled.setpoint is False


def test_dump_to_yaml_round_trips(tmp_path):
    config = _nominal_config()
    dumped_path = tmp_path / "dumped_config.yaml"

    config.dump_to_yaml(dumped_path)
    reloaded = EAxleStandConfig.from_yaml(dumped_path)

    assert reloaded == config
