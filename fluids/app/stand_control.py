"""Drive the FluidsStand from the Connect UI.

Canvas clicks arrive as message-bus messages on the backend's command topic. A MessageBus is not
thread-safe, so one listener thread owns it and hands each message to the main loop through a
queue; the stand is only ever touched from the main loop. Each tick applies queued commands,
advances the stand, and streams its telemetry, whose numeric state codes drive the canvas.

Pass `--backend` to choose what the stand runs on: the simulated panel (`mock`, the SIMULATOR
tab) or the cDAQ (`daq`, the HARDWARE tab). Each backend streams, listens and reports status
under its own names (`fluids_panel.NAMESPACES`), so both tabs can run at once without crosstalk.

The cDAQ is Connect's own NI-DAQmx device (`app/fluids_daq.ni-daqmx.json`): the stand reads its stream and
commands its outputs through this script's client, from the main loop only. The script's output
is also a table of every DAQ line (`fluids_panel.io_table`), which the HARDWARE tab's I/O pane
displays.
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
    NAMESPACES,
    NOT_RUNNING,
    SIGNALS_BY_REF,
    CommandError,
    ConnectDeviceBackend,
    FluidsStand,
    InterlockError,
    MockBackend,
    apply_simulation_settings,
    build_stand,
    handle_message,
    io_table,
    status_message,
)

logger = connect_python.get_logger(__name__)

POLL_INTERVAL_S = 0.1
LISTEN_INTERVAL_S = 0.05
IO_TABLE_INTERVAL_S = 0.5

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
    topic: str,
    commands: "queue.Queue[dict[str, Any]]",
    stop: threading.Event,
) -> None:
    """Forward every message on `topic` into `commands` until `stop` is set.

    Polls rather than blocking so the bus's sockets are closed before the client's context is
    terminated on the way out.
    """
    with connect_python.MessageBus(client) as bus:
        bus.subscribe_to_topic(topic)
        logger.info(f"Listening for commands on {topic}")
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
    for reason in stand.trip_history[reported:]:
        client.send_notification(f"Stand tripped: {reason}", level="error")
    return len(stand.trip_history)


@connect_python.main
def main(client: connect_python.Client) -> None:
    install_library_logging()
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=BACKENDS, default="mock")
    args, _ = parser.parse_known_args()

    names = NAMESPACES[args.backend]
    stand = build_stand(args.backend, client=client)
    commands: queue.Queue[dict[str, Any]] = queue.Queue()
    stop = threading.Event()
    # Outside Connect there is no message bus to listen on.
    listener = threading.Thread(
        target=listen, args=(client, names.command_topic, commands, stop), daemon=True
    )
    client.clear_stream(names.stream_id)

    with stand:
        logger.info(f"Stand open on the {args.backend} backend")
        if not isinstance(client, StubClient):
            listener.start()
        reported = 0
        status = None
        show_io = isinstance(stand.backend, ConnectDeviceBackend)
        io_shown_at = -IO_TABLE_INTERVAL_S
        try:
            while True:
                apply_commands(client, stand, commands)
                if isinstance(stand.backend, MockBackend):
                    apply_simulation_settings(stand.backend, client.get_values())
                stand.update()
                reported = report_new_trips(client, stand, reported)
                if (new_status := status_message(stand)) != status:
                    status = new_status
                    client.set_value(names.status_value_id, status)
                telemetry = stand.telemetry()
                units = {name: UNITS[name] for name in telemetry if name in UNITS}
                client.stream_from_dict(
                    names.stream_id, time.time(), telemetry, units=units
                )
                if show_io and time.monotonic() - io_shown_at >= IO_TABLE_INTERVAL_S:
                    io_shown_at = time.monotonic()
                    client.set_output(io_table(stand.backend))
                time.sleep(POLL_INTERVAL_S)
        finally:
            client.set_value(names.status_value_id, NOT_RUNNING)
            stop.set()
            if listener.is_alive():
                listener.join(timeout=1.0)
    logger.info("Stand closed")


if __name__ == "__main__":
    main()
