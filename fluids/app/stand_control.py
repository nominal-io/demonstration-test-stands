"""Drive the FluidsStand from the Connect UI.

Canvas clicks arrive as message-bus messages on `fluids_panel.COMMAND_TOPIC`. A MessageBus is not
thread-safe, so one listener thread owns it and hands each message to the main loop through a
queue; the stand is only ever touched from the main loop. Each tick applies queued commands,
advances the stand, and streams its telemetry, whose numeric state codes drive the canvas.

Pass `--backend` to choose what the stand runs on; only the simulated panel (`mock`) exists yet.
"""

import argparse
import logging
import queue
import threading
import time
from typing import Any

import connect_python  # ty: ignore[import-not-found]
from connect_python.stub_client import StubClient  # ty: ignore[import-not-found]

from fluids_panel import (
    BACKENDS,
    COMMAND_TOPIC,
    NOT_RUNNING,
    SIGNALS_BY_REF,
    STATUS_VALUE_ID,
    CommandError,
    FluidsStand,
    InterlockError,
    MockBackend,
    apply_simulation_settings,
    build_stand,
    handle_message,
    status_message,
)

logger = connect_python.get_logger(__name__)

POLL_INTERVAL_S = 0.1
LISTEN_INTERVAL_S = 0.05

STREAM_ID = "fluids"
UNITS = {
    ref: unit
    for ref in ("p_1", "p_2", "fm_1", "v_2", "pr_2")
    if (unit := SIGNALS_BY_REF[ref].units) is not None
} | {"p_2.leak_rate": "psi/min"}


def install_library_logging(level: int = logging.INFO) -> None:
    """Route the fluids package's stdlib logs to stderr, where Connect picks them up."""
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(name)s: %(message)s"))
    package_logger = logging.getLogger("fluids_panel")
    package_logger.setLevel(level)
    package_logger.handlers = [handler]
    package_logger.propagate = False


def listen(
    client: connect_python.Client,
    commands: "queue.Queue[dict[str, Any]]",
    stop: threading.Event,
) -> None:
    """Forward every message on COMMAND_TOPIC into `commands` until `stop` is set.

    Polls rather than blocking so the bus's sockets are closed before the client's context is
    terminated on the way out.
    """
    with connect_python.MessageBus(client) as bus:
        bus.subscribe_to_topic(COMMAND_TOPIC)
        logger.info(f"Listening for commands on {COMMAND_TOPIC}")
        while not stop.is_set():
            message = bus.try_receive_message()
            if message is None:
                stop.wait(LISTEN_INTERVAL_S)
            else:
                commands.put(message.contents)


def apply_commands(
    client: connect_python.Client,
    stand: FluidsStand,
    commands: "queue.Queue[dict[str, Any]]",
) -> None:
    """Apply every queued command, logging and toasting any that are refused."""
    while True:
        try:
            contents = commands.get_nowait()
        except queue.Empty:
            return
        try:
            logger.info(handle_message(stand, contents))
        except (CommandError, InterlockError) as error:
            logger.warning(f"Refused {contents}: {error}")
            client.send_notification(
                f"Refused: {error}", level="warn", duration_seconds=5
            )


def report_new_trips(
    client: connect_python.Client, stand: FluidsStand, reported: int
) -> int:
    """Toast any trips since the last call, returning how many have now been reported."""
    if len(stand.trips) < reported:
        reported = 0  # cleared by a reset
    for reason in stand.trips[reported:]:
        client.send_notification(f"Stand tripped: {reason}", level="error")
    return len(stand.trips)


@connect_python.main
def main(client: connect_python.Client) -> None:
    install_library_logging()
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=BACKENDS, default="mock")
    args, _ = parser.parse_known_args()

    stand = build_stand(args.backend)
    commands: queue.Queue[dict[str, Any]] = queue.Queue()
    stop = threading.Event()
    # Outside Connect there is no message bus to listen on.
    listener = threading.Thread(
        target=listen, args=(client, commands, stop), daemon=True
    )
    client.clear_stream(STREAM_ID)

    with stand:
        logger.info(f"Stand open on the {args.backend} backend")
        if not isinstance(client, StubClient):
            listener.start()
        reported = 0
        status = None
        try:
            while True:
                apply_commands(client, stand, commands)
                if isinstance(stand.backend, MockBackend):
                    apply_simulation_settings(stand.backend, client.get_values())
                stand.update()
                reported = report_new_trips(client, stand, reported)
                if (new_status := status_message(stand)) != status:
                    status = new_status
                    client.set_value(STATUS_VALUE_ID, status)
                telemetry = stand.telemetry()
                units = {name: UNITS[name] for name in telemetry if name in UNITS}
                client.stream_from_dict(STREAM_ID, time.time(), telemetry, units=units)
                time.sleep(POLL_INTERVAL_S)
        finally:
            client.set_value(STATUS_VALUE_ID, NOT_RUNNING)
            stop.set()
            if listener.is_alive():
                listener.join(timeout=1.0)
    logger.info("Stand closed")


if __name__ == "__main__":
    main()
