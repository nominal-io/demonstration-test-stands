"""Feature test: the operator's manual-control journey, end to end.

GIVEN an operator at the E-Axle stand with all three VESC nodes reporting live
      telemetry and the power supply enabled,
WHEN  they arm the stand, command the drive motor to a speed setpoint, apply
      regenerative braking current to both dyno absorbers, and then press STOP,
THEN  the drive motor holds its commanded speed for as long as the setpoint is
      held (against the controllers' 1000 ms watchdog), both absorbers receive
      the commanded braking current clamped to the stand's 20 A limit, and on
      STOP the brake current ramps to exactly zero on both sides BEFORE the
      speed setpoint begins to fall -- with the stand ending at rest, disarmed.

This is the acceptance test for the manual-control MVP.

WHAT IT RUNS AGAINST (design.md rev 4). There is no fake stand and no Stand
protocol. This test builds the same object graph main.py builds -- one instro
CanTransport, three VESC6 drivers sharing it, three InstroMotorControllers, an
InstroPSU, and the one HardwareStand receiving them -- and substitutes only the
CAN bus itself, at instro's own I/O boundary:

  * instro.unstable.transports.can.can.Bus          (autospec MagicMock)
  * instro.unstable.transports.can._prime_gs_usb_backend
  * instro.psu.drivers.simulated.VisaDriver         (autospec MagicMock)

Everything above those three symbols is real code: TransportBase's holder
accounting, CanTransport's send and receive demultiplexing, VESC6's frame encode
and decode, InstroMotorController's resource lock and publish wrappers,
HardwareStand, session.py and interlocks.py. The patch targets and the fixture
shape are instro's own, from tests/unstable/motorcontroller/test_vesc_6.py and
tests/psu/simulated/test_simulated_psu_software.py -- this repo invents no
protocol and no fake of its own.

WHAT THAT COSTS. This test cannot be COLLECTED without the real instro package
importable, including instro.unstable.motorcontroller, which today exists only
on origin/issue-362-vesc6-motor-controller-driver and is NOT in the released
instro-unstable 1.7.0. Local dev install only -- never committed to app.connect:

    git -C <instro-checkout> checkout issue-362-vesc6-motor-controller-driver
    uv pip install -e <instro-checkout> \
                   -e <instro-checkout>/packages/instro-unstable

Do NOT install via `pip install git+...#subdirectory=packages/instro-unstable`:
plain pip ignores instro's uv workspace source, resolves core `instro` from
PyPI, and pairs branch-era instro-unstable code with a newer instro that nobody
tests together. See design.md, "The Feature Test".

THE NUMBERS BELOW CAME FROM THE MACHINE, NOT FROM THE DRIVER. Every scaling
constant in _Plant is transcribed from design.md's CAN contract table, which was
derived from Tyler Rowan's commissioning scripts independently of instro. They
agree with instro's VESC6 decoder, and that independent agreement is the point:
a plant that re-derives its encoding FROM the driver could agree with the driver
while both are wrong about the machine. Do not "fix" these against vesc_6.py.

SCOPE. The plant keeps every interlock quiet, so this proves the stand commands
correctly and stops in the right order. It does not prove the stand reacts
correctly to a fault. Each trip path is a pure function of a telemetry dict and
belongs in its own unit test driven directly against interlocks.py -- which, per
design.md open question 12(c), should stay importable without instro so those
unit tests remain runnable in CI.

A human can perform the same steps in the UI: open the app, confirm three live
node cards, enable the PSU, arm, drag the speed slider to its configured target,
drag the brake slider past the stand's current limit, press STOP, and watch the
brake gauges reach zero before the speed plot begins to fall.
"""

import struct
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import can

from instro.psu import InstroPSU
from instro.psu.drivers import SimulatedPSU
from instro.unstable.motorcontroller import InstroMotorController
from instro.unstable.motorcontroller.drivers import VESC6
from instro.unstable.transports import CanConfig, CanTransport

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from interlocks import LIMITS  # noqa: E402
from session import StandSession, State  # noqa: E402
from stand import HardwareStand  # noqa: E402

# --- The CAN contract (design.md, "The CAN contract"). The specification this
# --- test holds the implementation to, transcribed from the commissioned stand.
MDC_ID, DMC_L_ID, DMC_R_ID = 0, 1, 2
MDC_POLE_PAIRS, DMC_POLE_PAIRS = 1, 7

SET_CURRENT, SET_CURRENT_BRAKE, SET_RPM = 1, 2, 3
STATUS_1, STATUS_4, STATUS_5 = 9, 16, 27

CAN_INTERFACE, CAN_CHANNEL, CAN_BITRATE = "gs_usb", 0, 500_000
PSU_RESOURCE = "TCPIP0::127.0.0.1::5025::SOCKET"

# Measured on the commissioned machine: DUT ERPM per dyno mechanical RPM.
DUT_CHAIN_RATIO = 36.38

TICK_HZ = 50.0
DT = 1.0 / TICK_HZ

TARGET_ERPM = 12000.0
REQUESTED_BRAKE_A = 25.0  # deliberately above the stand's limit
HEALTHY_BUS_V = 48.0


class _Clock:
    """The injected clock. HardwareStand.now defaults to time.monotonic in
    production; the test drives it so the watchdog-margin assertion is
    deterministic instead of a five-second wall-clock run."""

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class _Plant:
    """The machine, as seen from the far side of the CAN adapter.

    Drives the mock bus's recv(). Reads the most recent commanded values back
    off bus.send.call_args_list and answers with the status frames three
    healthy controllers would broadcast: the DUT turning at the commanded ERPM,
    both absorbers delivering the current they were asked for (so the dropout
    interlock stays quiet) at matched half-shaft speeds (so the spread interlock
    stays quiet), everything in limits.

    One mechanic that is easy to get wrong: the three VESC6 drivers share one
    CanTransport, and _drain_for() routes every frame it pulls to EVERY matching
    subscription. So on each tick the first get_telemetry() drains the whole
    batch and fans it out; the second and third find the bus empty and need only
    an immediate None. Hence: refill once per clock value, then None.
    """

    def __init__(self, bus: MagicMock, clock: _Clock) -> None:
        self._bus = bus
        self._clock = clock
        self._pending: list[can.Message] = []
        self._filled_at: float | None = None

    def recv(self, timeout: float = 0.0) -> can.Message | None:
        if self._pending:
            return self._pending.pop(0)
        now = self._clock()
        if now != self._filled_at:
            self._filled_at = now
            self._pending = self._status_frames()
            if self._pending:
                return self._pending.pop(0)
        return None

    def latest_commands(self) -> tuple[float, dict[int, float]]:
        """(commanded DUT ERPM, {node: commanded brake amps}) from the wire."""
        erpm = 0.0
        brake = {DMC_L_ID: 0.0, DMC_R_ID: 0.0}
        for sent, _node, kind, value in decode_sent(self._bus):
            del sent
            if kind == SET_RPM and _node == MDC_ID:
                erpm = value
            elif kind == SET_CURRENT_BRAKE and _node in brake:
                brake[_node] = value
            elif kind == SET_CURRENT and _node in brake and value == 0.0:
                brake[_node] = 0.0  # stop_motor() zeroes via SET_CURRENT
        return erpm, brake

    def _status_frames(self) -> list[can.Message]:
        erpm, brake = self.latest_commands()
        stub_rpm = erpm / DUT_CHAIN_RATIO
        frames: list[can.Message] = []
        frames += self._node_frames(MDC_ID, MDC_POLE_PAIRS, erpm, 10.0)
        for node in (DMC_L_ID, DMC_R_ID):
            frames += self._node_frames(node, DMC_POLE_PAIRS, stub_rpm, brake[node])
        return frames

    def _node_frames(
        self, node: int, pole_pairs: int, mech_rpm: float, motor_a: float
    ) -> list[can.Message]:
        # VESC6 decodes STATUS_1's raw field as ERPM and divides by pole_pairs,
        # so the wire carries mechanical RPM x pole_pairs.
        erpm_field = round(mech_rpm * pole_pairs)
        return [
            _frame(STATUS_1, node, struct.pack(">ihh", erpm_field, round(motor_a * 10), 500)),
            _frame(STATUS_4, node, struct.pack(">hhhh", 400, 450, round(motor_a * 10), 0)),
            _frame(STATUS_5, node, struct.pack(">ihh", 0, round(HEALTHY_BUS_V * 10), 0)),
        ]


def _frame(packet_id: int, node: int, payload: bytes) -> can.Message:
    return can.Message(
        arbitration_id=(packet_id << 8) | node, data=payload, is_extended_id=True
    )


def decode_sent(bus: MagicMock) -> list[tuple[int, int, int, float]]:
    """Every command frame the app put on the bus, in order.

    Returns (sequence, node, packet_id, engineering value). Scalings are the CAN
    contract's, not instro's -- see the module docstring.
    """
    decoded = []
    for seq, call in enumerate(bus.send.call_args_list):
        message = call.args[0]
        packet_id = message.arbitration_id >> 8
        node = message.arbitration_id & 0xFF
        (raw,) = struct.unpack(">i", bytes(message.data))
        if packet_id == SET_RPM:
            value = float(raw)  # raw ERPM, unscaled
        elif packet_id in (SET_CURRENT, SET_CURRENT_BRAKE):
            value = raw / 1000.0  # amps x 1000
        else:
            continue
        decoded.append((seq, node, packet_id, value))
    return decoded


def commands_to(bus: MagicMock, node: int, packet_id: int) -> list[tuple[int, float]]:
    """(sequence, value) for every command of one kind issued to one node."""
    return [
        (seq, value)
        for seq, target, kind, value in decode_sent(bus)
        if target == node and kind == packet_id
    ]


def build_stand(bus: MagicMock, clock: _Clock) -> HardwareStand:
    """Exactly what main.py's composition root builds, with the same values."""
    can_driver = CanTransport(
        CanConfig(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=CAN_BITRATE)
    )
    controllers = {
        "mdc": InstroMotorController(
            "mdc",
            driver=VESC6(
                channel=can_driver, pole_pairs=MDC_POLE_PAIRS, controller_id=MDC_ID
            ),
        ),
        "dmc_l": InstroMotorController(
            "dmc_l",
            driver=VESC6(
                channel=can_driver, pole_pairs=DMC_POLE_PAIRS, controller_id=DMC_L_ID
            ),
        ),
        "dmc_r": InstroMotorController(
            "dmc_r",
            driver=VESC6(
                channel=can_driver, pole_pairs=DMC_POLE_PAIRS, controller_id=DMC_R_ID
            ),
        ),
    }
    psu = InstroPSU(name="psu", driver=SimulatedPSU(PSU_RESOURCE), num_channels=1)
    return HardwareStand(psu=psu, clock=clock, **controllers)


def run_until(session, clock, predicate, max_seconds=60.0):
    """Tick the session until predicate. The session drains telemetry itself."""
    deadline = clock.t + max_seconds
    while not predicate(session) and clock.t < deadline:
        session.tick(DT)
        clock.advance(DT)
    assert clock.t < deadline, "session never reached the expected condition"
    return clock.t


def test_operator_drives_the_stand_and_stops_it_safely():
    clock = _Clock()

    with (
        patch("instro.unstable.transports.can._prime_gs_usb_backend", autospec=True),
        patch("instro.unstable.transports.can.can.Bus", autospec=True) as bus_cls,
        patch("instro.psu.drivers.simulated.VisaDriver", autospec=True) as visa_cls,
    ):
        bus = bus_cls.return_value
        visa_cls.return_value.query.return_value = '0,"No error"'
        plant = _Plant(bus, clock)
        bus.recv.side_effect = plant.recv

        stand = build_stand(bus, clock)
        session = StandSession(stand=stand, tick_hz=TICK_HZ)
        stand.open()

        # Three drivers, one adapter: the first controller to open constructs
        # the bus and the rest hold the same one. A second Bus here would mean
        # three adapters the stand does not have.
        bus_cls.assert_called_once_with(
            interface=CAN_INTERFACE, channel=CAN_CHANNEL, bitrate=CAN_BITRATE
        )

        # The "live telemetry" precondition is real, not decorative: a stand
        # that is not talking cannot be armed. Nothing has been drained yet.
        assert not session.can_arm()

        session.tick(DT)
        clock.advance(DT)
        session.set_psu_output_enabled(True)

        session.arm()
        assert session.state is State.ARMED

        session.set_speed_setpoint_erpm(TARGET_ERPM)
        run_until(session, clock, lambda s: s.commanded_erpm >= TARGET_ERPM)
        assert session.state is State.RUNNING

        # Hold the setpoint untouched for two seconds. Nothing changes it during
        # this window, so every frame issued is the session feeding the
        # controllers' 1000 ms watchdog rather than responding to the operator.
        # Instro offers no periodic transmit, so this re-send is the only thing
        # keeping the machine alive -- and it stops the moment this process does.
        hold_start = clock.t
        run_until(session, clock, lambda s: clock.t >= hold_start + 2.0)

        speed_cmds = commands_to(bus, MDC_ID, SET_RPM)
        assert speed_cmds[-1][1] == TARGET_ERPM
        assert len(speed_cmds) > 1, (
            "only one speed frame was ever sent; the controllers stop 1.0s "
            "after the last frame, so a held setpoint must be re-sent"
        )
        # Frames are stamped by position, not by clock, so convert the largest
        # run of ticks without a speed frame into seconds.
        gaps = [(b[0] - a[0]) for a, b in zip(speed_cmds, speed_cmds[1:])]
        assert max(gaps) * DT <= 0.2, (
            f"largest gap between speed frames was {max(gaps) * DT:.3f}s; the "
            "controllers' watchdog expires at 1.0s and needs comfortable margin"
        )

        session.set_brake_current_a(REQUESTED_BRAKE_A)
        run_until(session, clock, lambda s: s.commanded_brake_a >= LIMITS.max_brake_a)

        for node in (DMC_L_ID, DMC_R_ID):
            brake_cmds = commands_to(bus, node, SET_CURRENT_BRAKE)
            assert brake_cmds, f"absorber {node} was never commanded"
            assert brake_cmds[-1][1] == LIMITS.max_brake_a
            assert max(v for _, v in brake_cmds) <= LIMITS.max_brake_a, (
                "a brake frame exceeded the stand's configured current limit"
            )

        session.stop()
        run_until(session, clock, lambda s: s.state is State.IDLE)

        assert session.commanded_erpm == 0.0
        assert session.commanded_brake_a == 0.0
        assert commands_to(bus, MDC_ID, SET_RPM)[-1][1] == 0.0
        for node in (DMC_L_ID, DMC_R_ID):
            assert commands_to(bus, node, SET_CURRENT_BRAKE)[-1][1] == 0.0

        # The invariant this machine cannot violate: unload before decelerate.
        # Replay every frame in the order it went out. Decelerating while the
        # absorbers still brake drives regenerated energy into a bus that is no
        # longer prepared to take it.
        speed = 0.0
        brake = {DMC_L_ID: 0.0, DMC_R_ID: 0.0}
        for _seq, node, kind, value in decode_sent(bus):
            if kind == SET_RPM and node == MDC_ID:
                if value < speed:
                    assert brake[DMC_L_ID] == 0.0 and brake[DMC_R_ID] == 0.0, (
                        f"speed was commanded down to {value} ERPM while the "
                        f"absorbers were still braking at "
                        f"L={brake[DMC_L_ID]}A R={brake[DMC_R_ID]}A"
                    )
                speed = value
            elif kind == SET_CURRENT_BRAKE and node in brake:
                brake[node] = value
            elif kind == SET_CURRENT and node in brake and value == 0.0:
                brake[node] = 0.0

        # Last controller out tears the adapter down. A partial close leaves the
        # bus held and the next run cannot acquire it.
        stand.close()
        bus.shutdown.assert_called_once_with()
