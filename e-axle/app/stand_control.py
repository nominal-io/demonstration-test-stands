"""Drive the EAxleStand from the Connect UI.

`e_axle.build` is the composition root -- it owns building the shared CAN transport and the
shared EAPSB10000Visa device, so this script reuses its `build_stand()` rather than repeating
the wiring. The main loop here mirrors the UI's inputs onto the stand's setpoints and streams
every channel's measured value back to Connect.
"""

import time
from typing import Literal

import connect_python  # ty: ignore[unresolved-import]

from e_axle import EAxleStand, EAxleStandState, Motor, build_stand

logger = connect_python.get_logger(__name__)


POLL_INTERVAL_S = 0.1

STREAM_ID = "stand"
STATE_STREAM_ID = "stand_state"

# UI input prefix -> EAxleStand attribute
MOTORS = {"dut": "dut", "left": "left_load", "right": "right_load"}
CONTROL_MODES: dict[str, Literal["torque", "velocity", "current"]] = {
    "torque": "torque",
    "velocity": "velocity",
    "current": "current",
}


def apply_motor_inputs(
    client: connect_python.Client, motor: Motor, prefix: str
) -> None:
    """Mirror one motor's UI inputs onto its control mode and velocity/torque/current setpoints."""
    requested_mode = str(
        client.get_value(f"{prefix}-mode", motor.control_mode.setpoint)
    ).lower()
    mode = CONTROL_MODES.get(requested_mode)
    if mode is None:
        logger.warning(f"{prefix}: ignoring unknown control mode {requested_mode!r}")
    else:
        motor.control_mode.setpoint = mode

    for suffix, channel_name in CONTROL_MODES.items():
        value = client.get_value(f"{prefix}-{suffix}")
        if value is None:
            continue
        channel = getattr(motor, channel_name)
        channel.setpoint = float(value)
        if channel.setpoint != channel.requested:
            logger.warning(
                f"{prefix}.{channel_name}: requested {channel.requested} clamped to {channel.setpoint}"
            )


def apply_requested_state(
    client: connect_python.Client, stand: EAxleStand, requested: str
) -> None:
    """Step the stand one transition towards the state the UI is asking for."""
    if stand.state == EAxleStandState.TRIPPED:
        logger.error(
            f"Stand tripped on {[name for name, _ in stand.tripped_channels()]}"
        )
        if requested == "Disarmed":
            stand.reset()
        return

    if requested == "Running" and stand.state == EAxleStandState.ARMED:
        stand.run()
    elif requested != "Running" and stand.state == EAxleStandState.RUNNING:
        stand.stop()

    if requested == "Disarmed" and stand.state == EAxleStandState.ARMED:
        stand.disarm()
    elif requested in ("Armed", "Running") and stand.state == EAxleStandState.OFF:
        stand.arm()


def apply_supply_inputs(client: connect_python.Client, stand: EAxleStand) -> None:
    """Push the source and sink enable toggles to the supply, only when they change."""
    for instrument, input_id in (
        (stand.source, "source-enable"),
        (stand.sink, "sink-enable"),
    ):
        enabled = client.get_value(input_id)
        if enabled is None or bool(enabled) == instrument.enabled.setpoint:
            continue
        instrument.enabled.setpoint = bool(enabled)
        instrument.command()


def stream_telemetry(client: connect_python.Client, stand: EAxleStand) -> None:
    """Stream every measured channel on the stand, plus its state, back to Connect."""
    channels: dict[str, float | int | str] = {}
    for prefix, attr in MOTORS.items():
        motor = getattr(stand, attr)
        for channel_name in (*CONTROL_MODES.values(), "temperature"):
            measured = getattr(motor, channel_name).measured
            if measured is not None:
                channels[f"{prefix}.{channel_name}"] = measured

    for prefix, channel_names in (
        ("source", ("voltage", "current", "ovp_limit", "ocp_limit")),
        ("sink", ("voltage", "current")),
    ):
        instrument = getattr(stand, prefix)
        for channel_name in channel_names:
            measured = getattr(instrument, channel_name).measured
            if measured is not None:
                channels[f"{prefix}.{channel_name}"] = measured

    timestamp = time.time()
    if channels:
        client.stream_from_dict(STREAM_ID, timestamp, channels)
    client.stream(STATE_STREAM_ID, timestamp, stand.state.name, name="state")


@connect_python.main
def main(client: connect_python.Client) -> None:
    stand = build_stand()
    logger.info("Stand built, waiting on UI inputs")
    try:
        while True:
            requested = str(client.get_value("stand-state", "Disarmed"))
            apply_requested_state(client, stand, requested)
            if stand.state != EAxleStandState.OFF:
                for prefix, attr in MOTORS.items():
                    apply_motor_inputs(client, getattr(stand, attr), prefix)
                apply_supply_inputs(client, stand)
            stream_telemetry(client, stand)
            time.sleep(POLL_INTERVAL_S)
    finally:
        stand.close()
        logger.info("Stand closed")


if __name__ == "__main__":
    main()
