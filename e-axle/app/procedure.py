"""Run the AUTO tab's warm-up -> speed sweep -> regen -> cool-down procedure on the EAxleStand.

This script owns the stand for as long as it runs, so it must not run alongside stand_control.py.
The DUT holds speed in velocity mode while both loads apply torque relative to the DUT's direction:
negative opposes it (warm-up and the sweep's load steps), positive pushes with it (regen).
"""

import time
from collections.abc import Callable
from dataclasses import dataclass, fields

import connect_python  # ty: ignore[unresolved-import]
from stand_control import install_library_logging, stream_telemetry

from e_axle import EAxleStand, EAxleStandState, Motor, build_stand

logger = connect_python.get_logger(__name__)


LOOP_INTERVAL_S = 0.1
SETTLE_TIMEOUT_S = 15.0
SPEED_TOLERANCE_RPM = 50.0
LOAD_OVERSPEED_RPM = 471.429
MOTOR_TEMPERATURE_MAX_C = 100.0

PROCEDURE_STREAM_ID = "procedure"
START_TOPIC = "procedure/start"
ABORT_TOPIC = "procedure/abort"


@dataclass(frozen=True)
class Config:
    """The AUTO tab's inputs; each field reads from the `proc-<field-with-dashes>` input."""

    warmup_speed: float
    warmup_load: float
    warmup_temperature: float
    warmup_timeout: float
    sweep_speed_start: float
    sweep_speed_stop: float
    sweep_speed_step: float
    sweep_dwell: float
    load_1: float
    load_2: float
    load_3: float
    regen_speed: float
    regen_torque: float
    regen_duration: float
    cooldown_speed: float
    cooldown_temperature: float
    cooldown_timeout: float
    speed_ramp: float
    torque_ramp: float
    overspeed_margin: float

    @property
    def overspeed_limit(self) -> float:
        return LOAD_OVERSPEED_RPM - self.overspeed_margin

    def problems(self) -> list[str]:
        """Every reason this config can't be run; empty when it can."""
        checks = [
            (
                self.warmup_speed > 0,
                "warm-up speed must be above 0 to measure the DUT to load speed ratio",
            ),
            (
                0 < self.warmup_temperature < MOTOR_TEMPERATURE_MAX_C,
                "warm-up target temperature must be between 0 and 100 °C",
            ),
            (
                0 < self.cooldown_temperature < MOTOR_TEMPERATURE_MAX_C,
                "cool-down target temperature must be between 0 and 100 °C",
            ),
            (
                self.warmup_timeout > 0 and self.cooldown_timeout > 0,
                "warm-up and cool-down timeouts must be above 0",
            ),
            (self.sweep_speed_step > 0, "sweep speed step must be above 0"),
            (
                self.sweep_speed_start <= self.sweep_speed_stop,
                "sweep start speed must not exceed the stop speed",
            ),
            (
                self.sweep_dwell > 0 and self.regen_duration > 0,
                "sweep dwell and regen duration must be above 0",
            ),
            (
                self.speed_ramp > 0 and self.torque_ramp > 0,
                "ramp rates must be above 0",
            ),
            (self.overspeed_margin > 0, "load overspeed margin must be above 0"),
        ]
        return [message for ok, message in checks if not ok]


@dataclass(frozen=True)
class Step:
    """One setpoint to ramp to and hold, until `hold_s` elapses or `done_when` is met."""

    phase: str
    speed: float
    load_torque: float
    hold_s: float
    done_when: Callable[[EAxleStand], bool] | None = None
    check: Callable[[EAxleStand], str | None] | None = None


def loads(stand: EAxleStand) -> list[Motor]:
    return [stand.left_load, stand.right_load]


def motors(stand: EAxleStand) -> list[Motor]:
    return [stand.dut, *loads(stand)]


def read_config(client: connect_python.Client) -> Config | None:
    """Snapshot the AUTO tab's inputs, so edits mid-run don't change the run in progress."""
    values: dict[str, float] = {}
    for field in fields(Config):
        input_id = f"proc-{field.name.replace('_', '-')}"
        raw = client.get_value(input_id)
        try:
            values[field.name] = float(raw)
        except (TypeError, ValueError):
            logger.error(f"Can't start: {input_id} is {raw!r}, expected a number")
            return None

    config = Config(**values)
    problems = config.problems()
    for problem in problems:
        logger.error(f"Can't start: {problem}")
    return None if problems else config


def sweep_speeds(config: Config) -> list[float]:
    speeds = []
    speed = config.sweep_speed_start
    while speed <= config.sweep_speed_stop:
        speeds.append(speed)
        speed += config.sweep_speed_step
    return speeds


def warmed_up(stand: EAxleStand, target: float) -> bool:
    temperature = stand.dut.temperature.measured
    return temperature is not None and temperature >= target


def cooled_down(stand: EAxleStand, target: float) -> bool:
    temperatures = [motor.temperature.measured for motor in motors(stand)]
    return all(t is not None and t <= target for t in temperatures)


def load_headroom_problem(
    stand: EAxleStand, top_speed: float, limit: float
) -> str | None:
    """Project the loads' speed at the plan's top DUT speed from the measured DUT to load ratio."""
    dut_speed = stand.dut.velocity.measured
    load_speeds = [
        abs(speed)
        for speed in (load.velocity.measured for load in loads(stand))
        if speed is not None
    ]
    if not dut_speed or len(load_speeds) != len(loads(stand)):
        return (
            f"can't measure the DUT to load speed ratio: DUT at {dut_speed} RPM,"
            f" loads at {[load.velocity.measured for load in loads(stand)]} RPM"
        )

    if not max(load_speeds):
        return f"loads aren't turning with the DUT at {dut_speed:.0f} RPM"

    projected = top_speed * max(load_speeds) / abs(dut_speed)
    logger.info(
        f"DUT to load speed ratio {abs(dut_speed) / max(load_speeds):.3f}:"
        f" top DUT speed {top_speed:.0f} RPM projects to {projected:.0f} RPM at the loads"
    )
    if projected > limit:
        return (
            f"top DUT speed {top_speed:.0f} RPM projects to {projected:.0f} RPM at the loads,"
            f" above the {limit:.0f} RPM overspeed guard"
        )
    return None


def plan(config: Config) -> list[Step]:
    """Expand the config into the flat list of steps the runner walks through."""
    speeds = sweep_speeds(config)
    top_speed = max(config.regen_speed, config.cooldown_speed, *speeds)
    return [
        Step(
            "warmup",
            config.warmup_speed,
            -config.warmup_load,
            config.warmup_timeout,
            done_when=lambda stand: warmed_up(stand, config.warmup_temperature),
            check=lambda stand: load_headroom_problem(
                stand, top_speed, config.overspeed_limit
            ),
        ),
        *(
            Step("sweep", speed, -load, config.sweep_dwell)
            for load in (config.load_1, config.load_2, config.load_3)
            for speed in speeds
        ),
        Step("regen", config.regen_speed, config.regen_torque, config.regen_duration),
        Step(
            "cooldown",
            config.cooldown_speed,
            0.0,
            config.cooldown_timeout,
            done_when=lambda stand: cooled_down(stand, config.cooldown_temperature),
        ),
    ]


def received(bus: connect_python.MessageBus, topic: str) -> bool:
    """Drain the bus, reporting whether any message on `topic` arrived."""
    seen = False
    while (message := bus.try_receive_message()) is not None:
        seen = seen or message.topic == topic
    return seen


def dut_at_speed(stand: EAxleStand, target: float) -> bool:
    speed = stand.dut.velocity.measured
    return speed is not None and abs(speed - target) <= SPEED_TOLERANCE_RPM


def approach(current: float, target: float, max_delta: float) -> float:
    return min(max(target, current - max_delta), current + max_delta)


def overspeed_problem(stand: EAxleStand, limit: float) -> str | None:
    for name, load in zip(("left", "right"), loads(stand)):
        speed = load.velocity.measured
        if speed is not None and abs(speed) > limit:
            return f"{name} load at {speed:.0f} RPM exceeded the {limit:.0f} RPM overspeed guard"
    return None


def command(stand: EAxleStand, speed: float, load_torque: float) -> None:
    stand.dut.control_mode.setpoint = "velocity"
    stand.dut.velocity.setpoint = speed
    for load in loads(stand):
        load.control_mode.setpoint = "torque"
        load.torque.setpoint = load_torque


def stream_progress(
    client: connect_python.Client, phase: str, step: int, speed: float, torque: float
) -> None:
    client.stream_from_dict(
        PROCEDURE_STREAM_ID,
        time.time(),
        {
            "phase": phase,
            "step": step,
            "target_speed": speed,
            "target_torque": torque,
        },
    )


def start_stand(stand: EAxleStand) -> bool:
    """Arm, zero the DUT speed and load torque, then run."""
    stand.arm()
    if stand.state != EAxleStandState.ARMED:
        logger.error(f"Stand failed to arm, it is {stand.state.name}")
        return False

    command(stand, 0.0, 0.0)

    stand.run()
    if stand.state != EAxleStandState.RUNNING:
        logger.error(f"Stand failed to run, it is {stand.state.name}")
        return False
    return True


def shut_down(stand: EAxleStand) -> None:
    """Drop the load torque, then let the stand's own stop ramp the DUT to idle and disarm."""
    for load in loads(stand):
        load.torque.setpoint = 0.0
    if stand.state == EAxleStandState.RUNNING:
        stand.stop()
    if stand.state == EAxleStandState.ARMED:
        stand.disarm()


def stand_fault(stand: EAxleStand, limit: float) -> str | None:
    if stand.state == EAxleStandState.TRIPPED:
        return f"Stand tripped on {[name for name, _ in stand.tripped_channels()]}"
    if stand.state != EAxleStandState.RUNNING:
        return f"Stand stopped running, it is {stand.state.name}"
    return overspeed_problem(stand, limit)


def run_steps(
    client: connect_python.Client,
    bus: connect_python.MessageBus,
    stand: EAxleStand,
    config: Config,
    steps: list[Step],
) -> bool:
    """Ramp into each step and hold it until its time is up or its condition is met; False on abort."""
    speed, torque = 0.0, 0.0
    last_tick = time.monotonic()
    for index, step in enumerate(steps, start=1):
        logger.info(
            f"Step {index}/{len(steps)}: {step.phase} at {step.speed:.0f} RPM,"
            f" {step.load_torque:+.2f} Nm"
        )
        ramped_at: float | None = None
        settled_at: float | None = None
        while True:
            if received(bus, ABORT_TOPIC):
                logger.warning("Aborted from the UI")
                return False
            if fault := stand_fault(stand, config.overspeed_limit):
                logger.error(f"Aborting: {fault}")
                return False

            now = time.monotonic()
            dt, last_tick = now - last_tick, now
            speed = approach(speed, step.speed, config.speed_ramp * dt)
            torque = approach(torque, step.load_torque, config.torque_ramp * dt)
            command(stand, speed, torque)
            stream_telemetry(client, stand)
            stream_progress(client, step.phase, index, speed, torque)

            if speed == step.speed and torque == step.load_torque:
                if ramped_at is None:
                    ramped_at = now
                if settled_at is None and dut_at_speed(stand, step.speed):
                    settled_at = now
                if settled_at is None:
                    if now - ramped_at > SETTLE_TIMEOUT_S:
                        logger.error(
                            f"Aborting: DUT at {stand.dut.velocity.measured} RPM didn't reach"
                            f" {step.speed:.0f} RPM within {SETTLE_TIMEOUT_S:.0f} s"
                        )
                        return False
                elif step.done_when is not None and step.done_when(stand):
                    break
                elif now - settled_at >= step.hold_s:
                    if step.done_when is not None:
                        logger.warning(
                            f"{step.phase} timed out before reaching its target temperature"
                        )
                    break
            time.sleep(LOOP_INTERVAL_S)

        if step.check is not None and (problem := step.check(stand)):
            logger.error(f"Aborting: {problem}")
            return False
    return True


def run_procedure(
    client: connect_python.Client, bus: connect_python.MessageBus, stand: EAxleStand
) -> None:
    """Run the procedure once from the current inputs, always leaving the stand stopped."""
    config = read_config(client)
    if config is None:
        return
    if stand.state != EAxleStandState.OFF:
        logger.error(f"Can't start: stand is {stand.state.name}, expected OFF")
        return

    steps = plan(config)
    logger.info(f"Starting procedure, {len(steps)} steps")
    try:
        if start_stand(stand) and run_steps(client, bus, stand, config, steps):
            logger.info("Procedure complete")
    finally:
        shut_down(stand)
        stream_progress(client, "idle", 0, 0.0, 0.0)


@connect_python.main
def main(client: connect_python.Client) -> None:
    install_library_logging()
    stand = build_stand()
    logger.info("Stand built, press Start to run the procedure")
    try:
        with connect_python.MessageBus(client) as bus:
            bus.subscribe_to_topic(START_TOPIC)
            bus.subscribe_to_topic(ABORT_TOPIC)
            while stand.state != EAxleStandState.TRIPPED:
                if received(bus, START_TOPIC):
                    run_procedure(client, bus, stand)
                stream_telemetry(client, stand)
                time.sleep(LOOP_INTERVAL_S)
            logger.error("Stand tripped, restart the procedure script to reset it")
    finally:
        stand.close()
        logger.info("Stand closed")


if __name__ == "__main__":
    main()
