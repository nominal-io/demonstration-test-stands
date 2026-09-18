#!/usr/bin/env python3
"""
comms_check.py - CAN telemetry monitor for the E-Axle dynamometer stand.

MOSTLY PASSIVE. This script listens to the STATUS frames the VESCs broadcast
and does not command motion. The one exception is the SPACEBAR stop, which
transmits zero-torque commands to all three nodes.

Keys:
    SPACE   engage STOP - latches on and continuously commands zero torque
            to every node until released
    R       release the stop latch and return to passive monitoring
    Q       quit (zeroes all nodes on the way out)

The spacebar stop is a convenience, NOT a safety device. It depends on this
script running, the USB adapter working, and the CAN bus being intact. The
physical E-stop is the safety device. Use it.

A note on power: the VESC does not measure DC bus current directly. It derives
current_in from phase current and duty cycle, so the raw value is noisy - worst
at low duty, where a small phase-current wobble scales into a large input-current
swing. The table therefore shows an exponentially filtered value. Treat the
power supply's own reading as truth for real power accounting; these numbers are
for control and protection.

Usage:
    python comms_check.py                          # gs_usb (candlelight), default
    python comms_check.py --raw                    # dump raw frames instead
    python comms_check.py --tau 0.8                # slower/faster display filter
    python comms_check.py --interface slcan --channel COM5   # if reflashed
    python comms_check.py --listen-only            # disable TX entirely

Requires: pip install python-can rich gs_usb pyusb
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass, field

import can
from rich.console import Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

try:
    import msvcrt  # Windows only
    _PLATFORM = "windows"
except ImportError:
    import select
    import termios
    import tty
    _PLATFORM = "posix"

HAVE_KEYBOARD = sys.stdin.isatty()
_posix_saved_settings = None


def enable_raw_mode() -> None:
    """Put stdin into cbreak mode so single keys are readable without Enter. No-op on Windows."""
    global _posix_saved_settings
    if _PLATFORM != "posix" or not HAVE_KEYBOARD:
        return
    _posix_saved_settings = termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(sys.stdin.fileno())


def restore_terminal() -> None:
    """Restore stdin's settings saved by enable_raw_mode(). No-op on Windows or if never enabled."""
    if _PLATFORM != "posix" or _posix_saved_settings is None:
        return
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, _posix_saved_settings)

# --------------------------------------------------------------------------
# Stand configuration
# --------------------------------------------------------------------------

# vesc_id -> (display name, pole pairs, gear ratio from motor to output shaft)
NODES: dict[int, tuple[str, int, float]] = {
    0: ("DUT", 3, 9.5),      # E-Axle traction motor, 6 poles, 9.5:1 final drive
    1: ("DMC-L", 7, 1.0),    # Dyno left,  MP 8055, 14 poles, direct to half shaft
    2: ("DMC-R", 7, 1.0),    # Dyno right, MP 8055, 14 poles, direct to half shaft
}

STALE_AFTER_S = 0.5    # node flagged STALE if silent this long
STOP_TX_HZ = 50.0      # rate at which zero commands are repeated while latched
DEFAULT_TAU_S = 0.4    # display filter time constant for derived currents

# --------------------------------------------------------------------------
# VESC CAN protocol
#
# Extended (29-bit) IDs, encoded as:  EID = (command_id << 8) | vesc_id
# All multi-byte fields are big-endian (network byte order).
# --------------------------------------------------------------------------

CAN_PACKET_SET_CURRENT = 1        # int32, amps * 1000
CAN_PACKET_SET_CURRENT_BRAKE = 2  # int32, amps * 1000

CAN_PACKET_STATUS = 9     # ERPM, motor current, duty
CAN_PACKET_STATUS_2 = 14  # amp hours consumed / charged
CAN_PACKET_STATUS_3 = 15  # watt hours consumed / charged
CAN_PACKET_STATUS_4 = 16  # FET temp, motor temp, input current, PID position
CAN_PACKET_STATUS_5 = 27  # tachometer, input voltage
CAN_PACKET_STATUS_6 = 28  # ADC1/2/3, PPM

STATUS_IDS = (CAN_PACKET_STATUS, CAN_PACKET_STATUS_2, CAN_PACKET_STATUS_3,
              CAN_PACKET_STATUS_4, CAN_PACKET_STATUS_5, CAN_PACKET_STATUS_6)

STATUS_NAMES = {
    CAN_PACKET_STATUS: "S1", CAN_PACKET_STATUS_2: "S2",
    CAN_PACKET_STATUS_3: "S3", CAN_PACKET_STATUS_4: "S4",
    CAN_PACKET_STATUS_5: "S5", CAN_PACKET_STATUS_6: "S6",
}

# VESC counts six commutation steps per electrical revolution
STEPS_PER_ELEC_REV = 6

# Set from --tau at startup. Time constant in seconds for the display filter.
FILTER_TAU_S = DEFAULT_TAU_S


def split_eid(eid: int) -> tuple[int, int]:
    """Return (command_id, vesc_id) from a 29-bit VESC extended ID."""
    return (eid >> 8) & 0xFF, eid & 0xFF


def send_zero_all(bus: can.BusABC) -> None:
    """Command zero torque to every node.

    Sends both SET_CURRENT and SET_CURRENT_BRAKE at zero so the node releases
    regardless of which mode it was last commanded in. This is a release, not
    a hold: the driveline coasts down on its own inertia and friction.
    """
    payload = struct.pack(">i", 0)
    for vid in NODES:
        for cmd in (CAN_PACKET_SET_CURRENT, CAN_PACKET_SET_CURRENT_BRAKE):
            bus.send(can.Message(arbitration_id=(cmd << 8) | vid,
                                 data=payload, is_extended_id=True))


def poll_key() -> str | None:
    """Non-blocking single-key read. Returns None if nothing is waiting."""
    if not HAVE_KEYBOARD:
        return None
    if _PLATFORM == "windows":
        if not msvcrt.kbhit():
            return None
        ch = msvcrt.getch()
        if ch in (b"\x00", b"\xe0"):   # arrow / function keys send a second byte
            msvcrt.getch()
            return None
        return ch.decode("utf-8", "ignore").lower()

    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    ch = sys.stdin.read(1)
    if ch == "\x1b":   # ESC prefixes an ANSI arrow-key escape sequence; drain and ignore it
        if select.select([sys.stdin], [], [], 0)[0] and sys.stdin.read(1) == "[":
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.read(1)
        return None
    return ch.lower()


# --------------------------------------------------------------------------
# Per-node state
# --------------------------------------------------------------------------


@dataclass
class NodeState:
    name: str
    pole_pairs: int
    gear_ratio: float

    # STATUS 1 - raw
    erpm: float = 0.0
    current_motor: float = 0.0
    duty: float = 0.0

    # STATUS 2 / 3
    amp_hours: float = 0.0
    amp_hours_chg: float = 0.0
    watt_hours: float = 0.0
    watt_hours_chg: float = 0.0

    # STATUS 4 - raw
    temp_fet: float = 0.0
    temp_motor: float = 0.0
    current_in: float = 0.0
    pid_pos: float = 0.0

    # STATUS 5
    tachometer: int = 0
    voltage_in: float = 0.0

    # Filtered copies for display only. The raw fields above are what should
    # be logged; filtering is reversible only in one direction.
    current_in_filt: float = 0.0
    current_motor_filt: float = 0.0
    _filt_primed: bool = False
    _last_s4: float = 0.0

    last_seen: float = 0.0
    counts: dict[int, int] = field(default_factory=dict)
    first_seen: dict[int, float] = field(default_factory=dict)

    # ---- filtering -------------------------------------------------------

    def update_filters(self, now: float) -> None:
        """Exponential moving average with a time constant independent of rate.

        Using the measured interval rather than an assumed 50 Hz means the
        smoothing stays consistent if frames are dropped or the status rate
        is reconfigured.
        """
        if not self._filt_primed:
            self.current_in_filt = self.current_in
            self.current_motor_filt = self.current_motor
            self._filt_primed = True
            self._last_s4 = now
            return

        dt = now - self._last_s4
        self._last_s4 = now
        if dt <= 0.0:
            return

        alpha = 1.0 - pow(2.718281828, -dt / FILTER_TAU_S)
        alpha = min(max(alpha, 0.0), 1.0)
        self.current_in_filt += alpha * (self.current_in - self.current_in_filt)
        self.current_motor_filt += alpha * (self.current_motor - self.current_motor_filt)

    # ---- derived ---------------------------------------------------------

    @property
    def seen(self) -> bool:
        return self.last_seen > 0.0

    @property
    def stale(self) -> bool:
        return self.seen and (time.monotonic() - self.last_seen) > STALE_AFTER_S

    @property
    def motor_rpm(self) -> float:
        """Mechanical RPM at the motor shaft."""
        return self.erpm / self.pole_pairs

    @property
    def shaft_rpm(self) -> float:
        """RPM at the output shaft (diff carrier for the DUT, half shaft for dynos)."""
        return self.motor_rpm / self.gear_ratio

    @property
    def motor_revs(self) -> float:
        """Cumulative mechanical revolutions of the motor shaft, from the tachometer.

        The raw tachometer counts commutation steps: six per electrical
        revolution, and pole_pairs electrical revolutions per mechanical one.
        """
        return self.tachometer / (STEPS_PER_ELEC_REV * self.pole_pairs)

    @property
    def shaft_revs(self) -> float:
        """Cumulative revolutions at the output shaft."""
        return self.motor_revs / self.gear_ratio

    @property
    def power_in(self) -> float:
        """Filtered electrical power at the DC bus. Negative means regenerating."""
        return self.voltage_in * self.current_in_filt

    @property
    def power_in_raw(self) -> float:
        return self.voltage_in * self.current_in

    def rate(self, cmd: int) -> float:
        """Observed frame rate in Hz for one STATUS type."""
        n = self.counts.get(cmd, 0)
        if n < 2:
            return 0.0
        elapsed = time.monotonic() - self.first_seen[cmd]
        return (n - 1) / elapsed if elapsed > 0 else 0.0


def decode(node: NodeState, cmd: int, data: bytes) -> bool:
    """Decode one STATUS frame into node state. Returns True if understood."""
    now = time.monotonic()
    try:
        if cmd == CAN_PACKET_STATUS and len(data) >= 8:
            erpm, cur, duty = struct.unpack(">ihh", data[:8])
            node.erpm = float(erpm)
            node.current_motor = cur / 10.0
            node.duty = duty / 1000.0

        elif cmd == CAN_PACKET_STATUS_2 and len(data) >= 8:
            ah, ah_chg = struct.unpack(">ii", data[:8])
            node.amp_hours = ah / 10000.0
            node.amp_hours_chg = ah_chg / 10000.0

        elif cmd == CAN_PACKET_STATUS_3 and len(data) >= 8:
            wh, wh_chg = struct.unpack(">ii", data[:8])
            node.watt_hours = wh / 10000.0
            node.watt_hours_chg = wh_chg / 10000.0

        elif cmd == CAN_PACKET_STATUS_4 and len(data) >= 8:
            t_fet, t_mot, cur_in, pid = struct.unpack(">hhhh", data[:8])
            node.temp_fet = t_fet / 10.0
            node.temp_motor = t_mot / 10.0
            node.current_in = cur_in / 10.0
            node.pid_pos = pid / 50.0
            node.update_filters(now)

        elif cmd == CAN_PACKET_STATUS_5 and len(data) >= 6:
            tacho, v_in = struct.unpack(">ih", data[:6])
            node.tachometer = tacho
            node.voltage_in = v_in / 10.0

        elif cmd == CAN_PACKET_STATUS_6:
            pass  # enabled but unused on this stand

        else:
            return False

    except struct.error:
        return False

    node.last_seen = now
    node.counts[cmd] = node.counts.get(cmd, 0) + 1
    node.first_seen.setdefault(cmd, now)
    return True


# --------------------------------------------------------------------------
# Display
# --------------------------------------------------------------------------


def build_banner(stopped: bool, listen_only: bool) -> Panel:
    if listen_only:
        return Panel(Text("LISTEN ONLY - transmit disabled", style="bold cyan"),
                     border_style="cyan")
    if stopped:
        body = Text("STOP LATCHED - commanding zero torque to all nodes\n"
                    "[R] release    [Q] quit", style="bold white on red")
        return Panel(body, border_style="red")
    body = Text("MONITORING - passive, no commands being sent\n"
                "[SPACE] stop all    [Q] quit", style="bold green")
    return Panel(body, border_style="green")


def build_motion_table(nodes: dict[int, NodeState]) -> Table:
    t = Table(title=f"Motion and Power  (currents filtered, tau = {FILTER_TAU_S:.1f} s)",
              title_style="bold", expand=True)
    for col, just in (("ID", "right"), ("Node", "left"), ("State", "left"),
                      ("ERPM", "right"), ("Motor RPM", "right"),
                      ("Shaft RPM", "right"), ("Duty %", "right"),
                      ("I_mot A", "right"), ("I_in A", "right"),
                      ("P_in W", "right")):
        t.add_column(col, justify=just)

    for vid, n in sorted(nodes.items()):
        if not n.seen:
            state = Text("MISSING", style="bold red")
        elif n.stale:
            state = Text("STALE", style="bold yellow")
        else:
            state = Text("OK", style="bold green")

        t.add_row(str(vid), n.name, state,
                  f"{n.erpm:,.0f}", f"{n.motor_rpm:,.0f}", f"{n.shaft_rpm:,.1f}",
                  f"{n.duty * 100:.1f}", f"{n.current_motor_filt:+.1f}",
                  f"{n.current_in_filt:+.2f}", f"{n.power_in:+,.0f}")
    return t


def build_health_table(nodes: dict[int, NodeState]) -> Table:
    t = Table(title="Health and Bus", title_style="bold", expand=True)
    for col in ("Node", "FET C", "Motor C", "V_in", "Ah", "Wh",
                "Motor revs", "Shaft revs"):
        t.add_column(col, justify="left" if col == "Node" else "right")

    for _, n in sorted(nodes.items()):
        fet = Text(f"{n.temp_fet:.1f}", style="yellow" if n.temp_fet > 60 else "")
        # A disabled or unfitted sensor reads 0.0 or a large negative number.
        # Show it as missing rather than as a plausible temperature.
        no_sensor = n.temp_motor < -50 or n.temp_motor == 0.0
        motor_t = Text("--", style="dim") if no_sensor else Text(f"{n.temp_motor:.1f}")
        t.add_row(n.name, fet, motor_t,
                  f"{n.voltage_in:.1f}", f"{n.amp_hours:.4f}", f"{n.watt_hours:.3f}",
                  f"{n.motor_revs:,.1f}", f"{n.shaft_revs:,.1f}")
    return t


def build_rate_table(nodes: dict[int, NodeState]) -> Table:
    t = Table(title="Frame Rates (Hz)", title_style="bold", expand=True)
    t.add_column("Node")
    for cmd in STATUS_IDS:
        t.add_column(STATUS_NAMES[cmd], justify="right")
    t.add_column("Total", justify="right")

    for _, n in sorted(nodes.items()):
        row = [n.name]
        for cmd in STATUS_IDS:
            r = n.rate(cmd)
            row.append("-" if r == 0 else f"{r:.1f}")
        row.append(f"{sum(n.counts.values()):,}")
        t.add_row(*row)
    return t


def build_footer(nodes: dict[int, NodeState], unknown: dict[int, int],
                 total: int, started: float) -> Panel:
    lines: list[str] = []

    dut, dml, dmr = nodes.get(0), nodes.get(1), nodes.get(2)
    if dut and dml and dmr and all(n.seen for n in (dut, dml, dmr)):
        dyno_avg = (dml.shaft_rpm + dmr.shaft_rpm) / 2.0
        lines.append(f"Speed chain: DUT carrier {dut.shaft_rpm:,.1f} RPM   "
                     f"dyno average {dyno_avg:,.1f} RPM")
        if abs(dml.shaft_rpm) > 1 or abs(dmr.shaft_rpm) > 1:
            spread = abs(dml.shaft_rpm - dmr.shaft_rpm)
            base = max(abs(dml.shaft_rpm), abs(dmr.shaft_rpm), 1.0)
            lines.append(f"Half shaft spread: {spread:,.1f} RPM "
                         f"({spread / base * 100:.1f}%)")

    bus_total = sum(n.power_in for n in nodes.values() if n.seen)
    lines.append(f"Sum of node P_in: {bus_total:+,.0f} W   "
                 f"(compare against the supply's own reading - it has a real "
                 f"shunt, these are derived)")

    if unknown:
        ids = ", ".join(f"{v} ({c} frames)" for v, c in sorted(unknown.items()))
        lines.append(f"[bold red]Unexpected VESC IDs on bus: {ids}[/bold red]")

    elapsed = time.monotonic() - started
    lines.append(f"Elapsed {elapsed:,.1f} s   frames {total:,}   "
                 f"{total / elapsed if elapsed else 0:,.0f} fps")
    return Panel("\n".join(lines), border_style="dim")


# --------------------------------------------------------------------------
# Run modes
# --------------------------------------------------------------------------


def run_raw(bus: can.BusABC) -> None:
    """Dump every frame as it arrives. For debugging bus problems."""
    print("Raw frame dump. Ctrl-C to stop.\n")
    print(f"{'time':>10}  {'EID':>10}  {'cmd':>4}  {'id':>3}  {'len':>3}  data")
    started = time.monotonic()
    while True:
        msg = bus.recv(timeout=1.0)
        if msg is None:
            continue
        cmd, vid = split_eid(msg.arbitration_id)
        label = STATUS_NAMES.get(cmd, f"0x{cmd:02X}")
        print(f"{time.monotonic() - started:10.3f}  0x{msg.arbitration_id:08X}  "
              f"{label:>4}  {vid:3d}  {msg.dlc:3d}  {msg.data.hex(' ')}")


def run_live(bus: can.BusABC, listen_only: bool) -> None:
    nodes = {vid: NodeState(name, pp, gr) for vid, (name, pp, gr) in NODES.items()}
    unknown: dict[int, int] = {}
    total = 0
    stopped = False
    started = time.monotonic()
    last_stop_tx = 0.0
    last_draw = 0.0

    if not HAVE_KEYBOARD:
        print("stdin is not an interactive terminal - "
              "spacebar stop is disabled.")

    def render():
        return Group(
            build_banner(stopped, listen_only),
            build_motion_table(nodes),
            build_health_table(nodes),
            build_rate_table(nodes),
            build_footer(nodes, unknown, total, started),
        )

    with Live(render(), refresh_per_second=4, screen=False) as live:
        while True:
            msg = bus.recv(timeout=0.01)
            if msg is not None:
                total += 1
                if msg.is_extended_id:
                    cmd, vid = split_eid(msg.arbitration_id)
                    if vid in nodes:
                        decode(nodes[vid], cmd, bytes(msg.data))
                    elif cmd in STATUS_NAMES:
                        unknown[vid] = unknown.get(vid, 0) + 1

            key = poll_key()
            if key == " " and not listen_only:
                stopped = True
                send_zero_all(bus)          # immediate, do not wait for the tick
                last_stop_tx = time.monotonic()
            elif key == "r":
                stopped = False
            elif key == "q":
                break

            now = time.monotonic()

            # While latched, keep commanding zero so the stop holds against
            # anything else on the bus and survives a dropped frame.
            if stopped and not listen_only and now - last_stop_tx >= 1.0 / STOP_TX_HZ:
                send_zero_all(bus)
                last_stop_tx = now

            if now - last_draw >= 0.25:
                live.update(render())
                last_draw = now


def open_bus(args) -> can.BusABC:
    """Build the bus, trying a couple of argument shapes for gs_usb."""
    kwargs = {"interface": args.interface, "bitrate": args.bitrate}

    if args.interface == "gs_usb":
        attempts = [
            {**kwargs, "channel": args.channel, "index": args.index},
            {**kwargs, "channel": args.channel},
            {**kwargs, "channel": "can0", "index": 0},
        ]
    else:
        attempts = [{**kwargs, "channel": args.channel}]

    last_error: Exception | None = None
    for attempt in attempts:
        try:
            return can.Bus(**attempt)
        except Exception as e:  # noqa: BLE001 - backend raises a variety of types
            last_error = e
    raise RuntimeError(f"Could not open the CAN bus. Last error: {last_error}")


def main() -> None:
    global FILTER_TAU_S

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--interface", default="gs_usb")
    p.add_argument("--channel", default="0")
    p.add_argument("--index", type=int, default=0)
    p.add_argument("--bitrate", type=int, default=500_000)
    p.add_argument("--tau", type=float, default=DEFAULT_TAU_S,
                   help="display filter time constant in seconds. Larger is "
                        "smoother and slower to respond. 0 disables filtering.")
    p.add_argument("--raw", action="store_true",
                   help="dump raw frames instead of the live table")
    p.add_argument("--listen-only", action="store_true",
                   help="disable all transmit, including the spacebar stop")
    args = p.parse_args()

    FILTER_TAU_S = max(args.tau, 1e-6)

    print(f"Opening {args.interface} on channel {args.channel} "
          f"at {args.bitrate} bps ...")

    bus = None
    enable_raw_mode()
    try:
        bus = open_bus(args)
        run_raw(bus) if args.raw else run_live(bus, args.listen_only)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        print(f"\n{e}")
        print("\nChecks: adapter out of DFU (Boot switch OFF); WinUSB driver "
              "bound via Zadig; libusb-1.0.dll present; nothing else holding "
              "the device; CANH, CANL and GND all connected.")
    finally:
        restore_terminal()
        if bus is not None:
            # Always leave the stand with zero torque commanded, whatever
            # path we took out of the loop.
            if not args.listen_only:
                for _ in range(5):
                    try:
                        send_zero_all(bus)
                    except Exception:  # noqa: BLE001
                        break
                    time.sleep(0.01)
            bus.shutdown()
    print("\nStopped.")


if __name__ == "__main__":
    main()