#!/usr/bin/env python3
"""
dyno_test.py - coupled dynamometer run for the E-Axle stand.

Sequence:
    1. Smooth ramp of DUT speed from 0 to the target ERPM at a fixed ramp rate
    2. Hold at target with no load for a settling period
    3. MANUAL mode - operator trims brake current on both dyno motors together
    4. Graceful end - brake ramped off first, then speed, then release

THIS SCRIPT COMMANDS MOTION AND APPLIES LOAD. Confirm the driveline is clear,
the couplings are torqued, and the physical E-stop is within reach. The
spacebar abort is a convenience, NOT a safety device.

ARCHITECTURE
    CAN receive runs on a background thread (can.Notifier). The main loop only
    commands, renders, and logs. This matters: when receive shared the main
    loop, anything that stalled the process - a console redraw, a Windows
    screenshot overlay - starved the receive buffer, and the script then made
    decisions on stale telemetry.

    Trips are additionally gated on data freshness. A node whose last frame is
    older than FRESH_WINDOW_S is not evaluated for dropout, divergence or
    tracking error; if it stays quiet past STALE_AFTER_S the staleness trip
    catches it. Missing data must never look like a measurement of zero.

BUS HEALTH
    TX fail / error frames climbing    -> physical layer or adapter
    Bus state PASSIVE or BUS OFF       -> adapter seeing transmit errors
    RX rate collapses on ONE node      -> that controller stopped talking
    RX rates low on ALL nodes          -> the PC is not keeping up
    RX and TX clean but a dyno shows
    DROPOUT                            -> genuinely controller-side

Usage:
    python dyno_test.py --dry-run
    python dyno_test.py --tx-hz 20 --max-brake 6
    python dyno_test.py --target-erpm 6000

Requires: pip install python-can rich gs_usb pyusb
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
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
# Stand constants
# --------------------------------------------------------------------------

# MEASURED: DUT_erpm / dyno_erpm = 5.1973 (sd 0.0021) across all load levels.
# With the dynos at 7 pole pairs that fixes the DUT chain at 36.38. Only this
# PRODUCT is measurable from CAN data; the 4 / 9.095 split is provisional and
# affects only motor-shaft-level figures.
DUT_CHAIN = 36.38
DUT_POLE_PAIRS = 4
DUT_GEAR = DUT_CHAIN / DUT_POLE_PAIRS
DUT_LAMBDA = 0.014423
DUT_KT = 1.5 * DUT_POLE_PAIRS * DUT_LAMBDA

DYNO_POLE_PAIRS = 7                             # physically counted, 12N14P
DYNO_LAMBDA = 0.018148
DYNO_KT = 1.5 * DYNO_POLE_PAIRS * DYNO_LAMBDA   # 0.1906 Nm/A

NODES: dict[int, tuple[str, int, float, float]] = {
    0: ("DUT", DUT_POLE_PAIRS, DUT_GEAR, DUT_KT),
    1: ("DMC-L", DYNO_POLE_PAIRS, 1.0, DYNO_KT),
    2: ("DMC-R", DYNO_POLE_PAIRS, 1.0, DYNO_KT),
}

DUT_ID = 0
DYNO_IDS = (1, 2)

# Frames older than this are not trusted for trip decisions. Long enough to
# ride out a render hiccup, far shorter than the staleness trip.
FRESH_WINDOW_S = 0.15
STALE_AFTER_S = 0.5

DEFAULT_TX_HZ = 50.0
DEFAULT_TAU_S = 0.4

# S1 and S4 at 50 Hz plus S5 at 5 Hz, with S2/S3 disabled as unused
EXPECTED_RX_HZ = 105.0

TARGET_ERPM = 12000
RAMP_ERPM_PER_S = 1000.0
SETTLE_S = 10.0
BRAKE_STEP_A = 1.0
MAX_BRAKE_A = 20.0
BRAKE_RAMP_A_PER_S = 2.0

# --------------------------------------------------------------------------
# ABORT THRESHOLDS - every automatic trip in one place
# --------------------------------------------------------------------------

# Half-shaft divergence as |L - R| over the faster side. Healthy stand ran
# 0.68% mean / 3.17% peak; a dyno dropout exceeds 100% within 400 ms.
SPREAD_TRIP_PCT = 15.0
SPREAD_TRIP_S = 0.20
SPREAD_FLOOR_ERPM = 500.0

# Dyno torque dropout - the direct signature, fires before the shafts diverge
DROPOUT_FRAC = 0.30
DROPOUT_TRIP_S = 0.30
DROPOUT_MIN_CMD_A = 1.0

# Consecutive failed sends; at 50 Hz, 25 is half a second, well inside the
# controllers' 1000 ms command timeout.
TX_FAIL_STREAK_TRIP = 25

MAX_ERPM_TRIP = 12600
DYNO_MAX_ERPM_TRIP = 3300
MAX_FET_C = 80.0
MAX_MOTOR_C = 100.0
TRACK_ERR_ERPM = 1500
TRACK_ERR_S = 3.0

# --------------------------------------------------------------------------
# VESC CAN protocol - EID = (command_id << 8) | vesc_id, big-endian payloads
# --------------------------------------------------------------------------

CAN_PACKET_SET_CURRENT = 1
CAN_PACKET_SET_CURRENT_BRAKE = 2
CAN_PACKET_SET_RPM = 3

CAN_PACKET_STATUS = 9
CAN_PACKET_STATUS_2 = 14
CAN_PACKET_STATUS_3 = 15
CAN_PACKET_STATUS_4 = 16
CAN_PACKET_STATUS_5 = 27
CAN_PACKET_STATUS_6 = 28

STEPS_PER_ELEC_REV = 6
FILTER_TAU_S = DEFAULT_TAU_S


def split_eid(eid: int) -> tuple[int, int]:
    return (eid >> 8) & 0xFF, eid & 0xFF


# --------------------------------------------------------------------------
# Per-node state
# --------------------------------------------------------------------------


@dataclass
class NodeState:
    """Written only by the receive thread, read only by the main loop.

    No lock: each field is written by exactly one thread and attribute
    assignment is atomic under the GIL. Reading ERPM from one frame and
    current from the next is harmless here.
    """

    name: str
    pole_pairs: int
    gear_ratio: float
    kt: float

    erpm: float = 0.0
    current_motor: float = 0.0
    duty: float = 0.0
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
    rx_count: int = 0
    counts: dict[int, int] = field(default_factory=dict)

    def update_filters(self, now: float) -> None:
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
    def age(self) -> float:
        return time.monotonic() - self.last_seen if self.seen else 1e9

    @property
    def fresh(self) -> bool:
        """Recent enough to make a trip decision on."""
        return self.age < FRESH_WINDOW_S

    @property
    def stale(self) -> bool:
        return self.seen and self.age > STALE_AFTER_S

    @property
    def motor_rpm(self) -> float:
        return self.erpm / self.pole_pairs

    @property
    def shaft_rpm(self) -> float:
        return self.motor_rpm / self.gear_ratio

    @property
    def shaft_rad_s(self) -> float:
        return self.shaft_rpm * 2.0 * math.pi / 60.0

    @property
    def torque(self) -> float:
        return abs(self.current_motor_filt) * self.kt * self.gear_ratio

    @property
    def mech_power(self) -> float:
        return self.torque * abs(self.shaft_rad_s)

    @property
    def power_in(self) -> float:
        return self.voltage_in * self.current_in_filt


def decode(node: NodeState, cmd: int, data: bytes, now: float) -> bool:
    try:
        if cmd == CAN_PACKET_STATUS and len(data) >= 8:
            erpm, cur, duty = struct.unpack(">ihh", data[:8])
            node.erpm = float(erpm)
            node.current_motor = cur / 10.0
            node.duty = duty / 1000.0
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
        elif cmd in (CAN_PACKET_STATUS_2, CAN_PACKET_STATUS_3, CAN_PACKET_STATUS_6):
            pass  # amp/watt hours and ADC - not used by this stand
        else:
            return False
    except struct.error:
        return False

    node.last_seen = now
    node.rx_count += 1
    node.counts[cmd] = node.counts.get(cmd, 0) + 1
    return True


# --------------------------------------------------------------------------
# Bus health and the receive listener
# --------------------------------------------------------------------------


class BusHealth:
    """Counters that separate 'our commands are not landing' from
    'that controller stopped talking' from 'the PC is not keeping up'.

    tx_* are touched only by the main thread; rx_* only by the receive
    thread. No field is written by both, so no lock is needed.
    """

    def __init__(self) -> None:
        self.tx_ok = 0
        self.tx_fail = 0
        self.tx_fail_streak = 0
        self.tx_fail_peak_streak = 0
        self.last_tx_error = ""
        self.err_frames = 0
        self.rx_total = 0
        self.state = "n/a"
        self.state_worst = "n/a"
        # Rates are sampled from counters rather than kept as timestamp
        # queues, so the receive thread never mutates anything the main
        # thread is iterating over.
        self._rate: dict[int, float] = {v: 0.0 for v in NODES}
        self._last_count: dict[int, int] = {v: 0 for v in NODES}
        self._last_sample = 0.0
        self._last_state_poll = 0.0

    def send(self, bus: can.BusABC, msg: can.Message) -> bool:
        try:
            # An explicit timeout makes a full TX queue raise instead of
            # silently buffering, which is the point of this counter.
            bus.send(msg, timeout=0.02)
        except Exception as e:  # noqa: BLE001 - backends raise varied types
            self.tx_fail += 1
            self.tx_fail_streak += 1
            self.tx_fail_peak_streak = max(self.tx_fail_peak_streak,
                                           self.tx_fail_streak)
            self.last_tx_error = str(e)[:70]
            return False
        self.tx_ok += 1
        self.tx_fail_streak = 0
        return True

    def sample_rates(self, nodes: dict[int, NodeState], now: float) -> None:
        dt = now - self._last_sample
        if dt < 0.5:
            return
        self._last_sample = now
        for vid, n in nodes.items():
            c = n.rx_count
            self._rate[vid] = (c - self._last_count[vid]) / dt
            self._last_count[vid] = c

    def rx_rate(self, vid: int) -> float:
        return self._rate.get(vid, 0.0)

    def poll_state(self, bus: can.BusABC, now: float) -> None:
        if now - self._last_state_poll < 0.25:
            return
        self._last_state_poll = now
        try:
            s = bus.state
            self.state = getattr(s, "name", str(s))
            if self.state != "ACTIVE":
                self.state_worst = self.state
        except Exception:  # noqa: BLE001 - many backends do not implement it
            self.state = "n/a"


class RxListener(can.Listener):
    """Runs on the Notifier's thread. Decodes frames into node state.

    Deliberately does no rendering, logging or control - anything slow here
    would put us back to starving the receive buffer.
    """

    def __init__(self, nodes: dict[int, NodeState], health: BusHealth):
        self.nodes = nodes
        self.health = health

    def on_message_received(self, msg: can.Message) -> None:
        now = time.monotonic()
        if getattr(msg, "is_error_frame", False):
            self.health.err_frames += 1
            return
        if not msg.is_extended_id:
            return
        cmd, vid = split_eid(msg.arbitration_id)
        node = self.nodes.get(vid)
        if node is not None and decode(node, cmd, bytes(msg.data), now):
            self.health.rx_total += 1


# --------------------------------------------------------------------------
# Transmit helpers
# --------------------------------------------------------------------------


def send_rpm(bus: can.BusABC, h: BusHealth, vesc_id: int, erpm: float) -> None:
    h.send(bus, can.Message(arbitration_id=(CAN_PACKET_SET_RPM << 8) | vesc_id,
                            data=struct.pack(">i", int(erpm)), is_extended_id=True))


def send_brake(bus: can.BusABC, h: BusHealth, vesc_id: int, amps: float) -> None:
    """Brake current always opposes rotation regardless of sign - magnitude only."""
    h.send(bus, can.Message(
        arbitration_id=(CAN_PACKET_SET_CURRENT_BRAKE << 8) | vesc_id,
        data=struct.pack(">i", int(abs(amps) * 1000)), is_extended_id=True))


def send_zero_all(bus: can.BusABC, h: BusHealth) -> None:
    payload = struct.pack(">i", 0)
    for vid in NODES:
        for cmd in (CAN_PACKET_SET_CURRENT, CAN_PACKET_SET_CURRENT_BRAKE):
            h.send(bus, can.Message(arbitration_id=(cmd << 8) | vid,
                                    data=payload, is_extended_id=True))


def poll_key() -> str | None:
    if not HAVE_KEYBOARD:
        return None
    if _PLATFORM == "windows":
        if not msvcrt.kbhit():
            return None
        ch = msvcrt.getch()
        if ch in (b"\x00", b"\xe0"):
            return {b"H": "UP", b"P": "DOWN",
                    b"K": "LEFT", b"M": "RIGHT"}.get(msvcrt.getch())
        return ch.decode("utf-8", "ignore").lower()

    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    ch = sys.stdin.read(1)
    if ch != "\x1b":  # ESC prefixes an ANSI arrow-key escape sequence
        return ch.lower()
    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    ch2 = sys.stdin.read(1)
    if ch2 != "[" or not select.select([sys.stdin], [], [], 0)[0]:
        return None
    return {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}.get(sys.stdin.read(1))


# --------------------------------------------------------------------------
# Sequence
# --------------------------------------------------------------------------


class Sequence:
    IDLE = "IDLE"
    RAMP_UP = "RAMP UP"
    SETTLE = "SETTLE"
    MANUAL = "MANUAL"
    RAMP_DOWN = "RAMP DOWN"
    DONE = "DONE"
    ABORTED = "ABORTED"

    ACTIVE_STATES = (RAMP_UP, SETTLE, MANUAL, RAMP_DOWN)

    def __init__(self, target_erpm: int, ramp_rate: float, settle_s: float,
                 max_brake: float, brake_step: float):
        self.target_erpm = target_erpm
        self.ramp_rate = ramp_rate
        self.settle_s = settle_s
        self.max_brake = max_brake
        self.brake_step = brake_step

        self.state = self.IDLE
        self.speed_sp = 0.0
        self.brake_sp = 0.0
        self.brake_target = 0.0
        self.phase_started = 0.0
        self.run_started = 0.0
        self.abort_reason = ""
        self.skipped_checks = 0     # trips not evaluated because data was stale
        self._err_since: float | None = None
        self._spread_since: float | None = None
        self._dropout_since: dict[int, float | None] = {v: None for v in DYNO_IDS}
        self._last_tick = 0.0

    @property
    def elapsed(self) -> float:
        return 0.0 if self.state == self.IDLE else time.monotonic() - self.run_started

    @property
    def phase_elapsed(self) -> float:
        return time.monotonic() - self.phase_started

    @property
    def load_allowed(self) -> bool:
        return self.state == self.MANUAL

    def start(self, now: float) -> None:
        self.state = self.RAMP_UP
        self.speed_sp = self.brake_sp = self.brake_target = 0.0
        self.run_started = self.phase_started = self._last_tick = now
        self.abort_reason = ""
        self.skipped_checks = 0
        self._err_since = self._spread_since = None
        self._dropout_since = {v: None for v in DYNO_IDS}

    def abort(self, reason: str) -> None:
        self.state = self.ABORTED
        self.speed_sp = self.brake_sp = self.brake_target = 0.0
        self.abort_reason = reason

    def reset(self) -> None:
        self.state = self.IDLE
        self.speed_sp = self.brake_sp = self.brake_target = 0.0
        self.abort_reason = ""
        self._err_since = self._spread_since = None
        self._dropout_since = {v: None for v in DYNO_IDS}

    def bump_brake(self, delta: float) -> None:
        if self.load_allowed:
            self.brake_target = min(max(self.brake_target + delta, 0.0),
                                    self.max_brake)

    def drop_load(self) -> None:
        self.brake_target = self.brake_sp = 0.0

    def end_run(self, now: float) -> None:
        if self.state in (self.RAMP_UP, self.SETTLE, self.MANUAL):
            self.state = self.RAMP_DOWN
            self.phase_started = now
            self.brake_target = 0.0

    def tick(self, now: float) -> None:
        dt = max(now - self._last_tick, 0.0)
        self._last_tick = now

        if self.state == self.RAMP_UP:
            self.speed_sp = min(self.speed_sp + self.ramp_rate * dt, self.target_erpm)
            if self.speed_sp >= self.target_erpm:
                self.state = self.SETTLE
                self.phase_started = now
        elif self.state == self.SETTLE:
            self.speed_sp = self.target_erpm
            if self.phase_elapsed >= self.settle_s:
                self.state = self.MANUAL
                self.phase_started = now
        elif self.state == self.MANUAL:
            self.speed_sp = self.target_erpm
            self.brake_sp = self.brake_target
        elif self.state == self.RAMP_DOWN:
            # Load off first, then speed. Never the other way round.
            if self.brake_sp > 0.0:
                self.brake_sp = max(self.brake_sp - BRAKE_RAMP_A_PER_S * dt, 0.0)
            else:
                self.speed_sp = max(self.speed_sp - self.ramp_rate * dt, 0.0)
                if self.speed_sp <= 0.0:
                    self.state = self.DONE
                    self.phase_started = now

    @staticmethod
    def _timed(since: float | None, tripped: bool, now: float,
               dwell: float) -> tuple[float | None, bool]:
        if not tripped:
            return None, False
        if since is None:
            return now, False
        return since, (now - since) >= dwell

    def check_limits(self, nodes: dict[int, NodeState], health: BusHealth,
                     now: float) -> None:
        if self.state not in self.ACTIVE_STATES:
            return

        dut = nodes[DUT_ID]
        dml, dmr = nodes[DYNO_IDS[0]], nodes[DYNO_IDS[1]]

        # --- our commands are not landing ---
        if health.tx_fail_streak >= TX_FAIL_STREAK_TRIP:
            self.abort(f"CAN transmit failing: {health.tx_fail_streak} consecutive "
                       f"send errors ({health.last_tx_error})")
            return

        # --- telemetry present at all ---
        if not dut.seen:
            self.abort("DUT never reported on the bus")
            return
        for n in nodes.values():
            if n.stale:
                self.abort(f"{n.name} telemetry went stale "
                           f"({n.age * 1000:.0f} ms since last frame)")
                return

        # Everything below is a measurement-based trip. Missing data must not
        # look like a measurement of zero, so anything not fresh is skipped
        # and its dwell timer reset. The staleness trip above is what catches
        # a node that stays quiet.
        stale_any = not (dut.fresh and dml.fresh and dmr.fresh)
        if stale_any:
            self.skipped_checks += 1

        # --- overspeed ---
        if dut.fresh and abs(dut.erpm) > MAX_ERPM_TRIP:
            self.abort(f"DUT overspeed: {dut.erpm:,.0f} ERPM")
            return
        for vid in DYNO_IDS:
            n = nodes[vid]
            if n.fresh and abs(n.erpm) > DYNO_MAX_ERPM_TRIP:
                self.abort(f"{n.name} overspeed: {n.erpm:,.0f} ERPM")
                return

        # --- thermal (slow signals, staleness is harmless) ---
        for n in nodes.values():
            if n.seen and n.temp_fet > MAX_FET_C:
                self.abort(f"{n.name} FET at {n.temp_fet:.0f} C")
                return
            if n.seen and 0.0 < n.temp_motor < 200.0 and n.temp_motor > MAX_MOTOR_C:
                self.abort(f"{n.name} motor at {n.temp_motor:.0f} C")
                return

        # --- dyno torque dropout ---
        if self.brake_sp >= DROPOUT_MIN_CMD_A:
            for vid in DYNO_IDS:
                n = nodes[vid]
                if not n.fresh:
                    self._dropout_since[vid] = None
                    continue
                lost = abs(n.current_motor) < DROPOUT_FRAC * self.brake_sp
                self._dropout_since[vid], tripped = self._timed(
                    self._dropout_since[vid], lost, now, DROPOUT_TRIP_S)
                if tripped:
                    self.abort(f"{n.name} torque dropout: "
                               f"{abs(n.current_motor):.1f} A measured against "
                               f"{self.brake_sp:.1f} A commanded")
                    return
        else:
            self._dropout_since = {v: None for v in DYNO_IDS}

        # --- half-shaft divergence ---
        if dml.fresh and dmr.fresh:
            faster = max(abs(dml.erpm), abs(dmr.erpm))
            if faster > SPREAD_FLOOR_ERPM:
                pct = abs(dml.erpm - dmr.erpm) / faster * 100.0
                self._spread_since, tripped = self._timed(
                    self._spread_since, pct > SPREAD_TRIP_PCT, now, SPREAD_TRIP_S)
                if tripped:
                    self.abort(f"half-shaft divergence {pct:.0f}% "
                               f"(L {dml.erpm:,.0f} / R {dmr.erpm:,.0f} ERPM)")
                    return
            else:
                self._spread_since = None
        else:
            self._spread_since = None

        # --- DUT cannot hold speed ---
        if dut.fresh:
            self._err_since, tripped = self._timed(
                self._err_since, abs(dut.erpm - self.speed_sp) > TRACK_ERR_ERPM,
                now, TRACK_ERR_S)
            if tripped:
                self.abort(f"DUT cannot hold speed: "
                           f"{dut.erpm - self.speed_sp:+,.0f} ERPM error")
        else:
            self._err_since = None


# --------------------------------------------------------------------------
# Logging
# --------------------------------------------------------------------------


class RunLogger:
    FIELDS = ("erpm", "shaft_rpm", "duty", "i_motor", "i_in", "v_in",
              "temp_fet", "temp_motor", "torque_nm", "mech_w", "rx_hz", "age_ms")

    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self.fh = path.open("w", newline="", encoding="utf-8")
        self.w = csv.writer(self.fh)
        header = ["t_s", "state", "speed_sp_erpm", "brake_sp_a"]
        for _, (name, _, _, _) in sorted(NODES.items()):
            header += [f"{name}_{f}" for f in self.FIELDS]
        header += ["absorbed_w", "dut_in_w", "efficiency", "spread_pct",
                   "chain_ratio", "tx_ok", "tx_fail", "tx_fail_streak",
                   "err_frames", "bus_state", "skipped_checks"]
        self.w.writerow(header)

    def row(self, t: float, seq: Sequence, nodes: dict[int, NodeState],
            health: BusHealth) -> None:
        row: list = [f"{t:.3f}", seq.state, f"{seq.speed_sp:.0f}",
                     f"{seq.brake_sp:.2f}"]
        for vid, n in sorted(nodes.items()):
            row += [f"{n.erpm:.0f}", f"{n.shaft_rpm:.2f}", f"{n.duty:.4f}",
                    f"{n.current_motor:.1f}", f"{n.current_in:.1f}",
                    f"{n.voltage_in:.1f}", f"{n.temp_fet:.1f}",
                    f"{n.temp_motor:.1f}", f"{n.torque:.3f}",
                    f"{n.mech_power:.1f}", f"{health.rx_rate(vid):.0f}",
                    f"{min(n.age, 9.999) * 1000:.0f}"]
        dml, dmr = nodes[DYNO_IDS[0]], nodes[DYNO_IDS[1]]
        absorbed = dml.mech_power + dmr.mech_power
        dut_in = nodes[DUT_ID].power_in
        eff = absorbed / dut_in if dut_in > 1.0 else 0.0
        faster = max(abs(dml.erpm), abs(dmr.erpm), 1.0)
        spread = abs(dml.erpm - dmr.erpm) / faster * 100.0
        dyno_avg = (abs(dml.erpm) + abs(dmr.erpm)) / 2.0
        chain = abs(nodes[DUT_ID].erpm) / dyno_avg if dyno_avg > 1.0 else 0.0
        row += [f"{absorbed:.1f}", f"{dut_in:.1f}", f"{eff:.4f}", f"{spread:.2f}",
                f"{chain:.4f}", health.tx_ok, health.tx_fail,
                health.tx_fail_streak, health.err_frames, health.state,
                seq.skipped_checks]
        self.w.writerow(row)

    def close(self) -> None:
        self.fh.flush()
        self.fh.close()


# --------------------------------------------------------------------------
# Display
# --------------------------------------------------------------------------


def build_banner(seq: Sequence, dry_run: bool) -> Panel:
    if seq.state == Sequence.ABORTED:
        return Panel(Text(f"ABORTED - {seq.abort_reason}\n"
                          f"zero torque commanded to all nodes",
                          style="bold white on red"), border_style="red")
    if seq.state == Sequence.DONE:
        return Panel(Text(f"COMPLETE - {seq.elapsed:.1f} s, torque released",
                          style="bold green"), border_style="green")
    if seq.state == Sequence.IDLE:
        prefix = "DRY RUN - nothing will be transmitted\n" if dry_run else ""
        return Panel(Text(f"{prefix}IDLE - ready to arm\n"
                          f"target {seq.target_erpm:,} ERPM at "
                          f"{seq.ramp_rate:,.0f} ERPM/s "
                          f"({seq.target_erpm / seq.ramp_rate:.0f} s ramp), "
                          f"settle {seq.settle_s:.0f} s",
                          style="bold cyan"), border_style="cyan")
    if seq.state == Sequence.MANUAL:
        bar_w = 30
        frac = seq.brake_sp / seq.max_brake if seq.max_brake else 0.0
        bar = "#" * int(bar_w * frac) + "." * (bar_w - int(bar_w * frac))
        return Panel(Text(
            f"MANUAL LOAD   speed {seq.speed_sp:,.0f} ERPM   total {seq.elapsed:.1f} s\n"
            f"brake {seq.brake_sp:.1f} A per dyno  [{bar}]  max {seq.max_brake:.0f} A",
            style="bold black on yellow"), border_style="yellow")

    detail = ""
    if seq.state == Sequence.RAMP_UP:
        detail = f"{seq.speed_sp:,.0f} / {seq.target_erpm:,} ERPM"
    elif seq.state == Sequence.SETTLE:
        detail = f"at speed, unloaded - {seq.phase_elapsed:.1f} / {seq.settle_s:.0f} s"
    elif seq.state == Sequence.RAMP_DOWN:
        detail = f"brake {seq.brake_sp:.1f} A, speed {seq.speed_sp:,.0f} ERPM"
    return Panel(Text(f"{seq.state}   {detail}\ntotal {seq.elapsed:.1f} s",
                      style="bold black on yellow"), border_style="yellow")


def build_motion_table(nodes: dict[int, NodeState], seq: Sequence) -> Table:
    t = Table(title="Motion and Power", title_style="bold", expand=True)
    for col, just in (("Node", "left"), ("State", "left"), ("Age ms", "right"),
                      ("ERPM", "right"), ("Target", "right"), ("Err", "right"),
                      ("Shaft RPM", "right"), ("Duty %", "right"),
                      ("I_mot A", "right"), ("I_in A", "right"),
                      ("P_in W", "right"), ("FET C", "right"), ("Mot C", "right")):
        t.add_column(col, justify=just)

    for vid, n in sorted(nodes.items()):
        if not n.seen:
            state = Text("MISSING", style="bold red")
        elif n.stale:
            state = Text("STALE", style="bold yellow")
        elif not n.fresh:
            state = Text("LAGGING", style="yellow")
        else:
            state = Text("OK", style="bold green")

        age = Text(f"{min(n.age, 9.999) * 1000:.0f}",
                   style="yellow" if not n.fresh else "dim")

        if vid == DUT_ID and seq.state != Sequence.IDLE:
            target = f"{seq.speed_sp:,.0f}"
            err = n.erpm - seq.speed_sp
            err_txt = Text(f"{err:+,.0f}", style="yellow" if abs(err) > 500 else "")
        elif vid in DYNO_IDS and seq.brake_sp > 0:
            target = f"{seq.brake_sp:.1f}A"
            short = n.fresh and abs(n.current_motor) < DROPOUT_FRAC * seq.brake_sp
            err_txt = Text("DROPOUT" if short else "ok",
                           style="bold white on red" if short else "green")
        else:
            target, err_txt = "-", Text("-")

        fet = Text(f"{n.temp_fet:.0f}", style="red" if n.temp_fet > 70 else
                   ("yellow" if n.temp_fet > 55 else ""))
        no_sensor = n.temp_motor < -50 or n.temp_motor == 0.0
        mot = Text("--", style="dim") if no_sensor else Text(f"{n.temp_motor:.0f}")

        t.add_row(n.name, state, age, f"{n.erpm:,.0f}", target, err_txt,
                  f"{n.shaft_rpm:,.1f}", f"{n.duty * 100:.1f}",
                  f"{n.current_motor_filt:+.1f}", f"{n.current_in_filt:+.2f}",
                  f"{n.power_in:+,.0f}", fet, mot)
    return t


def build_bus_panel(health: BusHealth, nodes: dict[int, NodeState],
                    seq: Sequence) -> Panel:
    grid = Table.grid(padding=(0, 2))
    for _ in range(3):
        grid.add_column()

    tx_style = "bold red" if health.tx_fail else "green"
    grid.add_row(Text("TX (PC -> bus)"),
                 Text(f"{health.tx_ok:,} ok / {health.tx_fail:,} fail", style=tx_style),
                 Text(f"streak {health.tx_fail_streak} "
                      f"(peak {health.tx_fail_peak_streak}, trip "
                      f"{TX_FAIL_STREAK_TRIP})",
                      style="red" if health.tx_fail_streak else "dim"))

    err_style = "bold red" if health.err_frames else "green"
    grid.add_row(Text("Error frames"), Text(f"{health.err_frames:,}", style=err_style),
                 Text("bus-level bit/form/CRC errors", style="dim"))

    st_style = "green" if health.state == "ACTIVE" else (
        "dim" if health.state == "n/a" else "bold red")
    grid.add_row(Text("Adapter CAN state"), Text(health.state, style=st_style),
                 Text(f"worst seen: {health.state_worst}", style="dim"))

    for vid, n in sorted(nodes.items()):
        r = health.rx_rate(vid)
        ok = r > EXPECTED_RX_HZ * 0.8
        grid.add_row(Text(f"RX from {n.name}"),
                     Text(f"{r:5.0f} Hz", style="green" if ok else "bold red"),
                     Text(f"expected ~{EXPECTED_RX_HZ:.0f} Hz", style="dim"))

    grid.add_row(Text("Checks skipped"),
                 Text(f"{seq.skipped_checks:,}",
                      style="yellow" if seq.skipped_checks else "green"),
                 Text("trips not evaluated on stale data", style="dim"))

    if health.last_tx_error:
        grid.add_row(Text("Last TX error"), Text(""),
                     Text(health.last_tx_error, style="red"))

    bad = health.tx_fail or health.err_frames or health.state not in ("ACTIVE", "n/a")
    return Panel(grid, title="Bus health", title_align="left",
                 border_style="red" if bad else "green")


def build_results_panel(nodes: dict[int, NodeState]) -> Panel:
    dml, dmr = nodes[DYNO_IDS[0]], nodes[DYNO_IDS[1]]
    dut = nodes[DUT_ID]
    t_l, t_r = dml.torque, dmr.torque
    p_l, p_r = dml.mech_power, dmr.mech_power
    absorbed, axle_torque = p_l + p_r, t_l + t_r
    dut_in = dut.power_in
    eff = (absorbed / dut_in * 100.0) if dut_in > 1.0 else 0.0
    faster = max(abs(dml.shaft_rpm), abs(dmr.shaft_rpm), 1.0)
    spread = abs(dml.shaft_rpm - dmr.shaft_rpm)
    spread_pct = spread / faster * 100.0
    dyno_avg = (abs(dml.erpm) + abs(dmr.erpm)) / 2.0
    chain = abs(dut.erpm) / dyno_avg if dyno_avg > 1.0 else 0.0

    lines = [
        f"Torque      L {t_l:6.2f} Nm    R {t_r:6.2f} Nm    axle {axle_torque:6.2f} Nm"
        f"    reflected to DUT {axle_torque / DUT_GEAR:5.2f} Nm",
        f"Mech power  L {p_l:6.0f} W     R {p_r:6.0f} W     absorbed {absorbed:6.0f} W",
        f"DUT input   {dut_in:6.0f} W    apparent efficiency {eff:5.1f}%"
        f"   (includes the ~198 W fixed driveline tare)",
        f"Half shafts L {dml.shaft_rpm:7.1f} RPM  R {dmr.shaft_rpm:7.1f} RPM  "
        f"spread {spread:5.1f} RPM ({spread_pct:5.1f}%, trip {SPREAD_TRIP_PCT:.0f}%)",
        f"Chain check DUT/dyno ERPM ratio {chain:6.3f}  (expected 5.197)",
    ]
    style = "red" if spread_pct > SPREAD_TRIP_PCT * 0.6 else "dim"
    return Panel("\n".join(lines), title="Measured", border_style=style)


def build_limits_panel() -> Panel:
    t = Table.grid(padding=(0, 2))
    for _ in range(3):
        t.add_column()
    rows = [
        ("CAN transmit failing", f"{TX_FAIL_STREAK_TRIP} consecutive",
         "our commands are not landing"),
        ("Dyno torque dropout", f"< {DROPOUT_FRAC * 100:.0f}% of commanded",
         f"held {DROPOUT_TRIP_S:.1f} s, above {DROPOUT_MIN_CMD_A:.0f} A cmd"),
        ("Half-shaft divergence", f"> {SPREAD_TRIP_PCT:.0f}%",
         f"held {SPREAD_TRIP_S:.1f} s, above {SPREAD_FLOOR_ERPM:.0f} ERPM"),
        ("DUT overspeed", f"> {MAX_ERPM_TRIP:,} ERPM", "immediate"),
        ("Dyno overspeed", f"> {DYNO_MAX_ERPM_TRIP:,} ERPM", "immediate"),
        ("DUT speed error", f"> {TRACK_ERR_ERPM:,} ERPM", f"held {TRACK_ERR_S:.0f} s"),
        ("FET temperature", f"> {MAX_FET_C:.0f} C", "any node"),
        ("Motor temperature", f"> {MAX_MOTOR_C:.0f} C", "nodes with a real sensor"),
        ("Telemetry stale", f"> {STALE_AFTER_S:.1f} s", "any node"),
        ("Data freshness gate", f"< {FRESH_WINDOW_S * 1000:.0f} ms",
         "measurement trips skipped on older data"),
    ]
    for a, b, c in rows:
        t.add_row(Text(a), Text(b, style="bold"), Text(c, style="dim"))
    return Panel(t, title="Automatic aborts", title_align="left", border_style="red")


def build_controls_panel(seq: Sequence) -> Panel:
    idle = seq.state == Sequence.IDLE
    active = seq.state in Sequence.ACTIVE_STATES
    manual = seq.state == Sequence.MANUAL
    finished = seq.state in (Sequence.ABORTED, Sequence.DONE)

    rows = [
        ("UP", "arm and start the ramp", idle),
        ("RIGHT", f"brake +{seq.brake_step:.0f} A on both dynos", manual),
        ("LEFT", f"brake -{seq.brake_step:.0f} A on both dynos", manual),
        ("DOWN", "drop load to zero, stay at speed", manual),
        ("E", "end run: brake off, then speed down, then release", active),
        ("SPACE", "ABORT - zero torque to all nodes immediately", True),
        ("R", "reset back to idle", finished),
        ("Q", "quit (zeroes all nodes on the way out)", True),
    ]
    grid = Table.grid(padding=(0, 2))
    grid.add_column(justify="right", no_wrap=True)
    grid.add_column(overflow="fold")
    for key, desc, enabled in rows:
        if key == "SPACE":
            ks, ds = "bold white on red", "bold red"
        elif enabled:
            ks, ds = "bold black on white", ""
        else:
            ks, ds = "dim", "dim"
        grid.add_row(Text(f" {key} ", style=ks), Text(desc, style=ds))
    return Panel(grid, title="Controls", title_align="left", border_style="blue")


def build_footer(log_path: Path, health: BusHealth, started: float) -> Panel:
    elapsed = time.monotonic() - started
    return Panel(f"Log: {log_path}\nElapsed {elapsed:,.1f} s   "
                 f"RX {health.rx_total:,} frames   TX {health.tx_ok:,}",
                 border_style="dim")


# --------------------------------------------------------------------------
# Main loop
# --------------------------------------------------------------------------


def run(bus: can.BusABC, seq: Sequence, dry_run: bool, log_path: Path,
        tx_hz: float) -> None:
    nodes = {vid: NodeState(name, pp, gr, kt)
             for vid, (name, pp, gr, kt) in NODES.items()}
    health = BusHealth()
    logger = RunLogger(log_path)
    started = time.monotonic()
    last_tx = last_draw = last_log = 0.0

    if not HAVE_KEYBOARD:
        print("stdin is not an interactive terminal - cannot arm or abort.")
        return

    # Receive on its own thread. Nothing in the main loop can starve it.
    notifier = can.Notifier(bus, [RxListener(nodes, health)])

    def render():
        return Group(
            build_banner(seq, dry_run),
            build_motion_table(nodes, seq),
            build_results_panel(nodes),
            build_bus_panel(health, nodes, seq),
            build_footer(log_path, health, started),
            build_limits_panel(),
            build_controls_panel(seq),
        )

    try:
        with Live(render(), refresh_per_second=4, screen=False) as live:
            while True:
                now = time.monotonic()
                health.sample_rates(nodes, now)
                health.poll_state(bus, now)

                key = poll_key()
                if key == " ":
                    seq.abort("operator pressed SPACE")
                    if not dry_run:
                        send_zero_all(bus, health)
                elif key == "q":
                    break
                elif key == "r" and seq.state in (Sequence.ABORTED, Sequence.DONE):
                    seq.reset()
                elif key == "UP" and seq.state == Sequence.IDLE:
                    if any(not n.seen or n.stale for n in nodes.values()):
                        seq.abort("a node is not reporting - check the bus")
                    else:
                        seq.start(now)
                elif key == "RIGHT":
                    seq.bump_brake(+seq.brake_step)
                elif key == "LEFT":
                    seq.bump_brake(-seq.brake_step)
                elif key == "DOWN":
                    seq.drop_load()
                elif key == "e":
                    seq.end_run(now)

                seq.tick(now)
                was_active = seq.state in Sequence.ACTIVE_STATES
                seq.check_limits(nodes, health, now)
                # A trip must reach the bus immediately, not on the next tick
                if was_active and seq.state == Sequence.ABORTED and not dry_run:
                    send_zero_all(bus, health)

                if now - last_tx >= 1.0 / tx_hz:
                    last_tx = now
                    if not dry_run:
                        if seq.state in Sequence.ACTIVE_STATES:
                            send_rpm(bus, health, DUT_ID, seq.speed_sp)
                            for vid in DYNO_IDS:
                                send_brake(bus, health, vid, seq.brake_sp)
                        elif seq.state in (Sequence.ABORTED, Sequence.DONE):
                            send_zero_all(bus, health)

                if seq.state != Sequence.IDLE and now - last_log >= 0.02:
                    last_log = now
                    logger.row(now - started, seq, nodes, health)

                if now - last_draw >= 0.25:
                    live.update(render())
                    last_draw = now

                # The loop no longer blocks on recv, so yield explicitly
                # rather than spinning a core. Still ~500 Hz key polling.
                time.sleep(0.002)
    finally:
        notifier.stop()
        logger.close()
        print(f"\nBus summary: TX {health.tx_ok:,} ok / {health.tx_fail:,} failed "
              f"(peak streak {health.tx_fail_peak_streak}), "
              f"error frames {health.err_frames:,}, "
              f"RX {health.rx_total:,} frames, "
              f"adapter state {health.state} (worst {health.state_worst}), "
              f"checks skipped on stale data {seq.skipped_checks:,}")


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
    raise RuntimeError(
        f"Could not open the CAN bus. Last error: {last_error}\n"
        f"If this says 'Devices found: 0', unplug the VESC USB, plug the CAN "
        f"adapter in on its own so it enumerates, then reconnect the VESC.")


def main() -> None:
    global FILTER_TAU_S

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--interface", default="gs_usb")
    p.add_argument("--channel", default="0")
    p.add_argument("--index", type=int, default=0)
    p.add_argument("--bitrate", type=int, default=500_000)
    p.add_argument("--tau", type=float, default=DEFAULT_TAU_S)
    p.add_argument("--tx-hz", type=float, default=DEFAULT_TX_HZ,
                   help="command transmit rate. The controllers time out after "
                        "1000 ms, so 20 Hz is ample.")
    p.add_argument("--target-erpm", type=int, default=TARGET_ERPM)
    p.add_argument("--ramp-erpm-s", type=float, default=RAMP_ERPM_PER_S)
    p.add_argument("--settle", type=float, default=SETTLE_S)
    p.add_argument("--brake-step", type=float, default=BRAKE_STEP_A)
    p.add_argument("--max-brake", type=float, default=MAX_BRAKE_A)
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--log-dir", default="logs")
    args = p.parse_args()

    FILTER_TAU_S = max(args.tau, 1e-6)

    if args.target_erpm > MAX_ERPM_TRIP:
        raise SystemExit(f"--target-erpm exceeds the {MAX_ERPM_TRIP} ERPM trip")
    if args.max_brake > MAX_BRAKE_A:
        raise SystemExit(f"--max-brake exceeds the {MAX_BRAKE_A} A controller limit")

    seq = Sequence(args.target_erpm, args.ramp_erpm_s, args.settle,
                   args.max_brake, args.brake_step)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = Path(args.log_dir) / f"dyno_{stamp}.csv"

    print(f"Chain {DUT_CHAIN} (pp {DUT_POLE_PAIRS} x gear {DUT_GEAR:.3f}), "
          f"DUT Kt {DUT_KT:.4f}, dyno Kt {DYNO_KT:.4f}")
    print(f"Target {args.target_erpm:,} ERPM, ramp {args.ramp_erpm_s:,.0f} ERPM/s, "
          f"settle {args.settle:.0f} s, brake step {args.brake_step:.0f} A, "
          f"max {args.max_brake:.0f} A, TX {args.tx_hz:.0f} Hz")
    print(f"Opening {args.interface} on channel {args.channel} ...")

    bus = None
    health_fallback = BusHealth()
    enable_raw_mode()
    try:
        bus = open_bus(args)
        run(bus, seq, args.dry_run, log_path, args.tx_hz)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        print(f"\n{e}")
    finally:
        restore_terminal()
        if bus is not None:
            if not args.dry_run:
                for _ in range(5):
                    try:
                        send_zero_all(bus, health_fallback)
                    except Exception:  # noqa: BLE001
                        break
                    time.sleep(0.01)
            bus.shutdown()
    print(f"\nStopped. Log written to {log_path}")


if __name__ == "__main__":
    main()