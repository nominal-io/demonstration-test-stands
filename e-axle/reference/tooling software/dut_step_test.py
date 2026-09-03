#!/usr/bin/env python3
"""
dut_step_test.py - automated stepped speed sequence for the E-Axle DUT.

Profile: step from 0 to 12,000 ERPM (4,000 mechanical RPM) in 1,000 ERPM
increments held 2 s each, then hold 12,000 ERPM for 30 s, then release.
Total run time about 52 s.

THIS SCRIPT COMMANDS MOTION. The DUT will spin. Confirm the driveline is clear
and the physical E-stop is within reach before arming.

Keys:
    UP      arm and start the sequence
    SPACE   ABORT - latches on, commands zero torque to all nodes
    R       release the abort latch and return to idle
    Q       quit (zeroes all nodes on the way out)

The spacebar abort is a convenience, NOT a safety device. It depends on this
script running, the USB adapter working, and the CAN bus being intact. The
physical E-stop is the safety device.

Every run writes a CSV to ./logs/ with raw (unfiltered) telemetry at the full
status rate, including aborted runs.

Usage:
    python dut_step_test.py
    python dut_step_test.py --dry-run            # walk the profile, transmit nothing
    python dut_step_test.py --top-erpm 6000      # shorter profile for a first look
    python dut_step_test.py --interface slcan --channel COM5

Requires: pip install python-can rich gs_usb pyusb
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import can
from rich.console import Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

try:
    import msvcrt  # Windows only
    HAVE_KEYBOARD = True
except ImportError:
    HAVE_KEYBOARD = False

# --------------------------------------------------------------------------
# Stand configuration
# --------------------------------------------------------------------------

# vesc_id -> (display name, pole pairs, gear ratio from motor to output shaft)
NODES: dict[int, tuple[str, int, float]] = {
    0: ("DUT", 3, 9.5),
    1: ("DMC-L", 7, 1.0),
    2: ("DMC-R", 7, 1.0),
}

DUT_ID = 0

STALE_AFTER_S = 0.5
TX_HZ = 50.0               # command transmit rate; also the watchdog feed
DEFAULT_TAU_S = 0.4        # display filter time constant

# Profile defaults
STEP_ERPM = 1000
TOP_ERPM = 12000           # 4,000 mechanical RPM at 3 pole pairs
STEP_HOLD_S = 2.0
TOP_HOLD_S = 30.0

# Abort thresholds
MAX_ERPM_TRIP = 12600      # matches the controller's blend window floor
MAX_FET_C = 80.0
MAX_MOTOR_C = 100.0
TRACK_ERR_ERPM = 1500      # sustained speed error that trips an abort
TRACK_ERR_S = 3.0

# --------------------------------------------------------------------------
# VESC CAN protocol
#   EID = (command_id << 8) | vesc_id, big-endian payloads
# --------------------------------------------------------------------------

CAN_PACKET_SET_CURRENT = 1        # int32, amps * 1000
CAN_PACKET_SET_CURRENT_BRAKE = 2  # int32, amps * 1000
CAN_PACKET_SET_RPM = 3            # int32, ERPM (no scaling)

CAN_PACKET_STATUS = 9
CAN_PACKET_STATUS_2 = 14
CAN_PACKET_STATUS_3 = 15
CAN_PACKET_STATUS_4 = 16
CAN_PACKET_STATUS_5 = 27
CAN_PACKET_STATUS_6 = 28

STATUS_IDS = (CAN_PACKET_STATUS, CAN_PACKET_STATUS_2, CAN_PACKET_STATUS_3,
              CAN_PACKET_STATUS_4, CAN_PACKET_STATUS_5, CAN_PACKET_STATUS_6)

STATUS_NAMES = {
    CAN_PACKET_STATUS: "S1", CAN_PACKET_STATUS_2: "S2",
    CAN_PACKET_STATUS_3: "S3", CAN_PACKET_STATUS_4: "S4",
    CAN_PACKET_STATUS_5: "S5", CAN_PACKET_STATUS_6: "S6",
}

STEPS_PER_ELEC_REV = 6
FILTER_TAU_S = DEFAULT_TAU_S


def split_eid(eid: int) -> tuple[int, int]:
    return (eid >> 8) & 0xFF, eid & 0xFF


# --------------------------------------------------------------------------
# Transmit
# --------------------------------------------------------------------------


def send_rpm(bus: can.BusABC, vesc_id: int, erpm: float) -> None:
    bus.send(can.Message(
        arbitration_id=(CAN_PACKET_SET_RPM << 8) | vesc_id,
        data=struct.pack(">i", int(erpm)), is_extended_id=True))


def send_zero_all(bus: can.BusABC) -> None:
    """Command zero torque to every node.

    Sends both SET_CURRENT and SET_CURRENT_BRAKE at zero so a node releases
    regardless of which mode it was last commanded in. This is a release, not
    a hold: the driveline coasts down on its own inertia and friction.
    """
    payload = struct.pack(">i", 0)
    for vid in NODES:
        for cmd in (CAN_PACKET_SET_CURRENT, CAN_PACKET_SET_CURRENT_BRAKE):
            bus.send(can.Message(arbitration_id=(cmd << 8) | vid,
                                 data=payload, is_extended_id=True))


def poll_key() -> str | None:
    """Non-blocking single-key read. Arrow keys return 'UP', 'DOWN', etc."""
    if not HAVE_KEYBOARD or not msvcrt.kbhit():
        return None
    ch = msvcrt.getch()
    if ch in (b"\x00", b"\xe0"):
        return {b"H": "UP", b"P": "DOWN",
                b"K": "LEFT", b"M": "RIGHT"}.get(msvcrt.getch())
    return ch.decode("utf-8", "ignore").lower()


# --------------------------------------------------------------------------
# Profile
# --------------------------------------------------------------------------


def build_profile(step: int, top: int, step_hold: float,
                  top_hold: float) -> list[tuple[int, float]]:
    """Return a list of (erpm setpoint, hold seconds)."""
    profile: list[tuple[int, float]] = []
    erpm = step
    while erpm < top:
        profile.append((erpm, step_hold))
        erpm += step
    profile.append((top, top_hold))
    return profile


# --------------------------------------------------------------------------
# Per-node state
# --------------------------------------------------------------------------


@dataclass
class NodeState:
    name: str
    pole_pairs: int
    gear_ratio: float

    erpm: float = 0.0
    current_motor: float = 0.0
    duty: float = 0.0

    amp_hours: float = 0.0
    watt_hours: float = 0.0

    temp_fet: float = 0.0
    temp_motor: float = 0.0
    current_in: float = 0.0

    tachometer: int = 0
    voltage_in: float = 0.0

    current_in_filt: float = 0.0
    current_motor_filt: float = 0.0
    _primed: bool = False
    _last_s4: float = 0.0

    last_seen: float = 0.0
    counts: dict[int, int] = field(default_factory=dict)
    first_seen: dict[int, float] = field(default_factory=dict)

    def update_filters(self, now: float) -> None:
        """EMA with a time constant independent of the arrival rate."""
        if not self._primed:
            self.current_in_filt = self.current_in
            self.current_motor_filt = self.current_motor
            self._primed = True
            self._last_s4 = now
            return
        dt = now - self._last_s4
        self._last_s4 = now
        if dt <= 0.0:
            return
        a = min(max(1.0 - math.exp(-dt / FILTER_TAU_S), 0.0), 1.0)
        self.current_in_filt += a * (self.current_in - self.current_in_filt)
        self.current_motor_filt += a * (self.current_motor - self.current_motor_filt)

    @property
    def seen(self) -> bool:
        return self.last_seen > 0.0

    @property
    def stale(self) -> bool:
        return self.seen and (time.monotonic() - self.last_seen) > STALE_AFTER_S

    @property
    def motor_rpm(self) -> float:
        return self.erpm / self.pole_pairs

    @property
    def shaft_rpm(self) -> float:
        return self.motor_rpm / self.gear_ratio

    @property
    def motor_revs(self) -> float:
        return self.tachometer / (STEPS_PER_ELEC_REV * self.pole_pairs)

    @property
    def shaft_revs(self) -> float:
        return self.motor_revs / self.gear_ratio

    @property
    def power_in(self) -> float:
        return self.voltage_in * self.current_in_filt

    def rate(self, cmd: int) -> float:
        n = self.counts.get(cmd, 0)
        if n < 2:
            return 0.0
        elapsed = time.monotonic() - self.first_seen[cmd]
        return (n - 1) / elapsed if elapsed > 0 else 0.0


def decode(node: NodeState, cmd: int, data: bytes) -> bool:
    now = time.monotonic()
    try:
        if cmd == CAN_PACKET_STATUS and len(data) >= 8:
            erpm, cur, duty = struct.unpack(">ihh", data[:8])
            node.erpm = float(erpm)
            node.current_motor = cur / 10.0
            node.duty = duty / 1000.0
        elif cmd == CAN_PACKET_STATUS_2 and len(data) >= 8:
            ah, _ = struct.unpack(">ii", data[:8])
            node.amp_hours = ah / 10000.0
        elif cmd == CAN_PACKET_STATUS_3 and len(data) >= 8:
            wh, _ = struct.unpack(">ii", data[:8])
            node.watt_hours = wh / 10000.0
        elif cmd == CAN_PACKET_STATUS_4 and len(data) >= 8:
            t_fet, t_mot, cur_in, _ = struct.unpack(">hhhh", data[:8])
            node.temp_fet = t_fet / 10.0
            node.temp_motor = t_mot / 10.0
            node.current_in = cur_in / 10.0
            node.update_filters(now)
        elif cmd == CAN_PACKET_STATUS_5 and len(data) >= 6:
            tacho, v_in = struct.unpack(">ih", data[:6])
            node.tachometer = tacho
            node.voltage_in = v_in / 10.0
        elif cmd == CAN_PACKET_STATUS_6:
            pass
        else:
            return False
    except struct.error:
        return False

    node.last_seen = now
    node.counts[cmd] = node.counts.get(cmd, 0) + 1
    node.first_seen.setdefault(cmd, now)
    return True


# --------------------------------------------------------------------------
# Sequence state machine
# --------------------------------------------------------------------------


class Sequence:
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    DONE = "DONE"
    ABORTED = "ABORTED"

    def __init__(self, profile: list[tuple[int, float]]):
        self.profile = profile
        self.state = self.IDLE
        self.step_index = 0
        self.step_started = 0.0
        self.run_started = 0.0
        self.setpoint = 0
        self.abort_reason = ""
        self._err_since: float | None = None

    @property
    def total_duration(self) -> float:
        return sum(d for _, d in self.profile)

    @property
    def elapsed(self) -> float:
        if self.state == self.IDLE:
            return 0.0
        return time.monotonic() - self.run_started

    @property
    def step_elapsed(self) -> float:
        if self.state != self.RUNNING:
            return 0.0
        return time.monotonic() - self.step_started

    def start(self) -> None:
        self.state = self.RUNNING
        self.step_index = 0
        self.run_started = time.monotonic()
        self.step_started = self.run_started
        self.setpoint = self.profile[0][0]
        self.abort_reason = ""
        self._err_since = None

    def abort(self, reason: str) -> None:
        self.state = self.ABORTED
        self.setpoint = 0
        self.abort_reason = reason

    def reset(self) -> None:
        self.state = self.IDLE
        self.setpoint = 0
        self.step_index = 0
        self.abort_reason = ""
        self._err_since = None

    def tick(self, now: float) -> None:
        """Advance through the profile. Call every loop iteration."""
        if self.state != self.RUNNING:
            return
        _, hold = self.profile[self.step_index]
        if now - self.step_started >= hold:
            self.step_index += 1
            if self.step_index >= len(self.profile):
                self.state = self.DONE
                self.setpoint = 0
                return
            self.step_started = now
            self.setpoint = self.profile[self.step_index][0]

    def check_limits(self, nodes: dict[int, NodeState], now: float) -> None:
        """Trip the sequence on anything that looks wrong."""
        if self.state != self.RUNNING:
            return

        dut = nodes[DUT_ID]

        if not dut.seen:
            self.abort("DUT never reported on the bus")
            return
        if dut.stale:
            self.abort("DUT telemetry went stale")
            return
        if abs(dut.erpm) > MAX_ERPM_TRIP:
            self.abort(f"overspeed: {dut.erpm:,.0f} ERPM")
            return

        for n in nodes.values():
            if n.seen and n.temp_fet > MAX_FET_C:
                self.abort(f"{n.name} FET at {n.temp_fet:.0f} C")
                return
            # A disabled sensor reads 0.0; only trip on a plausible reading
            if n.seen and 0.0 < n.temp_motor < 200.0 and n.temp_motor > MAX_MOTOR_C:
                self.abort(f"{n.name} motor at {n.temp_motor:.0f} C")
                return

        # Sustained inability to reach setpoint means something is wrong -
        # a stall, a fault, or a limit binding that we did not expect.
        if abs(dut.erpm - self.setpoint) > TRACK_ERR_ERPM:
            if self._err_since is None:
                self._err_since = now
            elif now - self._err_since > TRACK_ERR_S:
                self.abort(f"speed error {dut.erpm - self.setpoint:+,.0f} ERPM "
                           f"held for {TRACK_ERR_S:.0f} s")
                return
        else:
            self._err_since = None


# --------------------------------------------------------------------------
# Logging
# --------------------------------------------------------------------------


class RunLogger:
    """Writes raw telemetry to CSV. Raw, not filtered - filtering is one-way."""

    FIELDS_PER_NODE = ("erpm", "motor_rpm", "shaft_rpm", "duty", "i_motor",
                       "i_in", "v_in", "temp_fet", "temp_motor", "revs")

    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self.fh = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.fh)
        header = ["t_s", "state", "step", "setpoint_erpm"]
        for _, (name, _, _) in sorted(NODES.items()):
            header += [f"{name}_{f}" for f in self.FIELDS_PER_NODE]
        self.writer.writerow(header)

    def row(self, t: float, seq: Sequence, nodes: dict[int, NodeState]) -> None:
        row: list = [f"{t:.3f}", seq.state, seq.step_index, seq.setpoint]
        for vid, n in sorted(nodes.items()):
            row += [f"{n.erpm:.0f}", f"{n.motor_rpm:.1f}", f"{n.shaft_rpm:.2f}",
                    f"{n.duty:.4f}", f"{n.current_motor:.1f}",
                    f"{n.current_in:.1f}", f"{n.voltage_in:.1f}",
                    f"{n.temp_fet:.1f}", f"{n.temp_motor:.1f}",
                    f"{n.motor_revs:.2f}"]
        self.writer.writerow(row)

    def close(self) -> None:
        self.fh.flush()
        self.fh.close()


# --------------------------------------------------------------------------
# Display
# --------------------------------------------------------------------------


def build_banner(seq: Sequence, dry_run: bool) -> Panel:
    if seq.state == Sequence.ABORTED:
        body = Text(f"ABORTED - {seq.abort_reason}\n"
                    f"commanding zero torque to all nodes\n"
                    f"[R] reset    [Q] quit", style="bold white on red")
        return Panel(body, border_style="red")

    if seq.state == Sequence.RUNNING:
        sp, hold = seq.profile[seq.step_index]
        bar_w = 40
        frac = min(seq.elapsed / seq.total_duration, 1.0)
        bar = "#" * int(bar_w * frac) + "." * (bar_w - int(bar_w * frac))
        body = Text(
            f"RUNNING   step {seq.step_index + 1}/{len(seq.profile)}   "
            f"setpoint {sp:,} ERPM ({sp / 3:,.0f} RPM)\n"
            f"step {seq.step_elapsed:4.1f}/{hold:.0f} s      "
            f"total {seq.elapsed:5.1f}/{seq.total_duration:.0f} s\n"
            f"[{bar}]\n"
            f"[SPACE] ABORT", style="bold black on yellow")
        return Panel(body, border_style="yellow")

    if seq.state == Sequence.DONE:
        body = Text(f"COMPLETE - {seq.elapsed:.1f} s, torque released\n"
                    f"[UP] run again    [Q] quit", style="bold green")
        return Panel(body, border_style="green")

    prefix = "DRY RUN - no commands will be sent\n" if dry_run else ""
    body = Text(f"{prefix}IDLE - ready to arm\n"
                f"profile: {len(seq.profile)} steps, "
                f"{seq.total_duration:.0f} s total\n"
                f"[UP] start    [Q] quit", style="bold cyan")
    return Panel(body, border_style="cyan")


def build_motion_table(nodes: dict[int, NodeState], seq: Sequence) -> Table:
    t = Table(title=f"Motion and Power  (currents filtered, tau = {FILTER_TAU_S:.1f} s)",
              title_style="bold", expand=True)
    for col, just in (("ID", "right"), ("Node", "left"), ("State", "left"),
                      ("ERPM", "right"), ("Target", "right"), ("Err", "right"),
                      ("Motor RPM", "right"), ("Shaft RPM", "right"),
                      ("Duty %", "right"), ("I_mot A", "right"),
                      ("I_in A", "right"), ("P_in W", "right")):
        t.add_column(col, justify=just)

    for vid, n in sorted(nodes.items()):
        if not n.seen:
            state = Text("MISSING", style="bold red")
        elif n.stale:
            state = Text("STALE", style="bold yellow")
        else:
            state = Text("OK", style="bold green")

        if vid == DUT_ID and seq.state == Sequence.RUNNING:
            target = f"{seq.setpoint:,}"
            err = n.erpm - seq.setpoint
            err_txt = Text(f"{err:+,.0f}",
                           style="yellow" if abs(err) > 500 else "")
        else:
            target, err_txt = "-", Text("-")

        t.add_row(str(vid), n.name, state, f"{n.erpm:,.0f}", target, err_txt,
                  f"{n.motor_rpm:,.0f}", f"{n.shaft_rpm:,.1f}",
                  f"{n.duty * 100:.1f}", f"{n.current_motor_filt:+.1f}",
                  f"{n.current_in_filt:+.2f}", f"{n.power_in:+,.0f}")
    return t


def build_health_table(nodes: dict[int, NodeState]) -> Table:
    t = Table(title="Health and Bus", title_style="bold", expand=True)
    for col in ("Node", "FET C", "Motor C", "V_in", "Ah", "Wh",
                "Motor revs", "Shaft revs"):
        t.add_column(col, justify="left" if col == "Node" else "right")

    for _, n in sorted(nodes.items()):
        fet = Text(f"{n.temp_fet:.1f}",
                   style="red" if n.temp_fet > 70 else
                   ("yellow" if n.temp_fet > 55 else ""))
        no_sensor = n.temp_motor < -50 or n.temp_motor == 0.0
        motor_t = Text("--", style="dim") if no_sensor else Text(f"{n.temp_motor:.1f}")
        t.add_row(n.name, fet, motor_t, f"{n.voltage_in:.1f}",
                  f"{n.amp_hours:.4f}", f"{n.watt_hours:.3f}",
                  f"{n.motor_revs:,.1f}", f"{n.shaft_revs:,.1f}")
    return t


def build_footer(nodes: dict[int, NodeState], log_path: Path,
                 total: int, started: float) -> Panel:
    lines: list[str] = []
    dut, dml, dmr = nodes.get(0), nodes.get(1), nodes.get(2)
    if dut and dml and dmr and all(n.seen for n in (dut, dml, dmr)):
        dyno_avg = (dml.shaft_rpm + dmr.shaft_rpm) / 2.0
        lines.append(f"Speed chain: DUT carrier {dut.shaft_rpm:,.1f} RPM   "
                     f"dyno average {dyno_avg:,.1f} RPM")

    bus_total = sum(n.power_in for n in nodes.values() if n.seen)
    lines.append(f"Sum of node P_in: {bus_total:+,.0f} W   "
                 f"(the supply's own reading is the calibrated one)")

    elapsed = time.monotonic() - started
    lines.append(f"Log: {log_path}")
    lines.append(f"Elapsed {elapsed:,.1f} s   frames {total:,}   "
                 f"{total / elapsed if elapsed else 0:,.0f} fps")
    return Panel("\n".join(lines), border_style="dim")


# --------------------------------------------------------------------------
# Main loop
# --------------------------------------------------------------------------


def run(bus: can.BusABC, seq: Sequence, dry_run: bool, log_path: Path) -> None:
    nodes = {vid: NodeState(name, pp, gr) for vid, (name, pp, gr) in NODES.items()}
    logger = RunLogger(log_path)
    total = 0
    started = time.monotonic()
    last_tx = 0.0
    last_draw = 0.0
    last_log = 0.0

    if not HAVE_KEYBOARD:
        print("Keyboard unavailable on this platform - cannot arm or abort.")
        return

    def render():
        return Group(
            build_banner(seq, dry_run),
            build_motion_table(nodes, seq),
            build_health_table(nodes),
            build_footer(nodes, log_path, total, started),
        )

    try:
        with Live(render(), refresh_per_second=4, screen=False) as live:
            while True:
                msg = bus.recv(timeout=0.005)
                if msg is not None:
                    total += 1
                    if msg.is_extended_id:
                        cmd, vid = split_eid(msg.arbitration_id)
                        if vid in nodes:
                            decode(nodes[vid], cmd, bytes(msg.data))

                now = time.monotonic()

                key = poll_key()
                if key == " ":
                    if seq.state == Sequence.RUNNING:
                        seq.abort("operator pressed SPACE")
                    else:
                        seq.abort("operator pressed SPACE")
                    if not dry_run:
                        send_zero_all(bus)
                elif key == "r" and seq.state in (Sequence.ABORTED, Sequence.DONE):
                    seq.reset()
                elif key == "UP" and seq.state in (Sequence.IDLE, Sequence.DONE):
                    dut = nodes[DUT_ID]
                    if not dut.seen or dut.stale:
                        seq.abort("DUT not reporting - check the bus before arming")
                    else:
                        seq.start()

                seq.tick(now)
                seq.check_limits(nodes, now)

                # Transmit at a fixed rate. This tick is also the controllers'
                # watchdog feed, so it must never be blocked by rendering.
                if now - last_tx >= 1.0 / TX_HZ:
                    last_tx = now
                    if not dry_run:
                        if seq.state == Sequence.RUNNING:
                            send_rpm(bus, DUT_ID, seq.setpoint)
                        elif seq.state == Sequence.ABORTED:
                            send_zero_all(bus)
                        elif seq.state == Sequence.DONE:
                            send_zero_all(bus)

                # Log while the sequence is doing anything, at 50 Hz
                if seq.state != Sequence.IDLE and now - last_log >= 0.02:
                    last_log = now
                    logger.row(now - started, seq, nodes)

                if now - last_draw >= 0.25:
                    live.update(render())
                    last_draw = now

                if key == "q":
                    break
    finally:
        logger.close()


def open_bus(args) -> can.BusABC:
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
        except Exception as e:  # noqa: BLE001
            last_error = e
    raise RuntimeError(f"Could not open the CAN bus. Last error: {last_error}")


def main() -> None:
    global FILTER_TAU_S

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--interface", default="gs_usb")
    p.add_argument("--channel", default="0")
    p.add_argument("--index", type=int, default=0)
    p.add_argument("--bitrate", type=int, default=500_000)
    p.add_argument("--tau", type=float, default=DEFAULT_TAU_S)
    p.add_argument("--step-erpm", type=int, default=STEP_ERPM)
    p.add_argument("--top-erpm", type=int, default=TOP_ERPM)
    p.add_argument("--step-hold", type=float, default=STEP_HOLD_S)
    p.add_argument("--top-hold", type=float, default=TOP_HOLD_S)
    p.add_argument("--dry-run", action="store_true",
                   help="walk the profile and log, but transmit nothing")
    p.add_argument("--log-dir", default="logs")
    args = p.parse_args()

    FILTER_TAU_S = max(args.tau, 1e-6)

    if args.top_erpm > MAX_ERPM_TRIP:
        raise SystemExit(f"--top-erpm {args.top_erpm} exceeds the "
                         f"{MAX_ERPM_TRIP} ERPM trip threshold")

    profile = build_profile(args.step_erpm, args.top_erpm,
                            args.step_hold, args.top_hold)
    seq = Sequence(profile)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = Path(args.log_dir) / f"dut_step_{stamp}.csv"

    print(f"Profile: {len(profile)} steps to {args.top_erpm:,} ERPM "
          f"({args.top_erpm / 3:,.0f} mechanical RPM), "
          f"{seq.total_duration:.0f} s total")
    print(f"Opening {args.interface} on channel {args.channel} "
          f"at {args.bitrate} bps ...")

    bus = None
    try:
        bus = open_bus(args)
        run(bus, seq, args.dry_run, log_path)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        print(f"\n{e}")
    finally:
        if bus is not None:
            if not args.dry_run:
                for _ in range(5):
                    try:
                        send_zero_all(bus)
                    except Exception:  # noqa: BLE001
                        break
                    time.sleep(0.01)
            bus.shutdown()
    print(f"\nStopped. Log written to {log_path}")


if __name__ == "__main__":
    main()