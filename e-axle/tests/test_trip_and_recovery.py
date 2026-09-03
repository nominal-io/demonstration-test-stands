"""Feature test: the stand trips itself, and the operator recovers it.

GIVEN an operator running the E-Axle stand under load, with all three VESC nodes
      reporting live telemetry and the supply enabled,
WHEN  they press STOP and let the brake bleed off; and then re-arm and the DUT
      runs away past its overspeed limit; and then, having acknowledged and
      recovered, kill the power supply mid-run,
THEN  the drive keeps receiving its UNCHANGED speed setpoint every tick
      throughout the bleed-off; the runaway latches TRIPPED naming dut_overspeed
      on mdc, with both absorbers released before the drive and no further frame
      sent afterwards; disarm() will not clear it and re-arming is refused while
      the fault persists; acknowledge_trip() returns the stand to IDLE, from
      which it re-arms AT REST once the fault has cleared; and cutting the supply
      zeroes every node BEFORE the supply's voltage setpoint is commanded to 0 V,
      never disables its output, and latches psu_disabled.

This is the acceptance test for
`.ailly/developer/2026-09-02-A-interlock-safety/design.md`, which closes
TASKS.md T-1 (interlocks are implemented and unit-tested but never wired into
production code), T-5 (no speed frame is sent during the STOPPING brake
bleed-off) and T-6 (disabling the PSU mid-run neither stops nor disarms).

ONE TEST, THREE ACTS. These are not three user stories -- they are three ways
one run ends, all converging on the same latch-and-recover path, at the same
stand, in one continuous session. T-5's assertion lives inside the STOP the
operator was going to press anyway; T-6's is a second trigger for the machinery
T-1 builds. See design.md, "One test, not three -- and why". The acts are
labelled and split cleanly if a future reviewer prefers three.

WHAT IT RUNS AGAINST. The same harness as the cleared feature test -- the real
instro object graph (one CanTransport, three VESC6 drivers sharing it, three
InstroMotorControllers, an InstroPSU, one HardwareStand) with only can.Bus,
_prime_gs_usb_backend and the simulated PSU's VisaDriver patched. That harness is
IMPORTED from tests/test_manual_control_session.py rather than duplicated: it
owns ~150 lines of struct frame arithmetic, and design.md already names a second
copy of that arithmetic as this test tree's main drift risk. See design.md's Open
Artifact Decisions for the alternatives, and the recommended follow-up to extract
tests/_can_plant.py at cleanup.

Consequently this test carries the same prerequisite as the one it borrows from:
it cannot be COLLECTED without instro.unstable.motorcontroller, which today
exists only on the branch install recorded in TASKS.md T-7.

THE PLANT IS DISOBEDIENT HERE, FOR THE FIRST TIME IN THIS PROJECT. _FaultPlant
below overrides the MDC's reported velocity so the test can drive the machine
into an overspeed the app never commanded -- exactly what design-gate Finding 9
and rev-4 review finding R4-2 said the obedient plant could never do. It does not
close them: the other eight trip paths remain TASKS.md T-8's work.
"""

import sys
from pathlib import Path
from unittest.mock import patch

TESTS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = TESTS_DIR.parent
sys.path.insert(0, str(TESTS_DIR))
sys.path.insert(0, str(PROJECT_ROOT))

import test_manual_control_session as harness  # noqa: E402

from interlocks import LIMITS  # noqa: E402
from session import StandSession, State  # noqa: E402

MDC_ID = harness.MDC_ID
DMC_L_ID = harness.DMC_L_ID
DMC_R_ID = harness.DMC_R_ID
SET_CURRENT = harness.SET_CURRENT
SET_CURRENT_BRAKE = harness.SET_CURRENT_BRAKE
SET_RPM = harness.SET_RPM

TICK_HZ = harness.TICK_HZ
DT = harness.DT

# Comfortably inside every limit: 8000 ERPM puts the half-shafts at ~220
# mechanical RPM (1540 ERPM at 7 pole pairs), well under the 3300 ERPM dyno
# overspeed trip, and 20 A is the stand's configured clamp exactly.
RUN_ERPM = 8000.0
RUN_BRAKE_A = LIMITS.max_brake_a

# The fault: a DUT that runs away past its commissioned overspeed limit. The app
# never commands this -- the plant reports it, which is the whole point.
OVERSPEED_ERPM = LIMITS.dut_overspeed_erpm + 500.0

# The 1000 ms firmware watchdog wants comfortable margin; design.md's metric is
# "no gap > 200 ms between command frames in normal operation".
MAX_WATCHDOG_GAP_S = 0.2


class _FaultPlant(harness._Plant):
    """The obedient plant, plus one lever.

    `mdc_velocity_override` replaces what the MDC *reports* without changing what
    it was *commanded*, so the test can produce a genuine divergence between the
    two. None means "obedient again": the plant echoes the wire as before.
    """

    def __init__(self, bus, clock) -> None:
        super().__init__(bus, clock)
        self.mdc_velocity_override: float | None = None

    def _node_frames(self, node, pole_pairs, mech_rpm, motor_a):
        if node == MDC_ID and self.mdc_velocity_override is not None:
            mech_rpm = self.mdc_velocity_override
        return super()._node_frames(node, pole_pairs, mech_rpm, motor_a)


def frames_from(bus, index):
    """Every command frame sent at or after `index` in bus.send's call list."""
    return [
        (seq, node, kind, value)
        for seq, node, kind, value in harness.decode_sent(bus)
        if seq >= index
    ]


def speed_frames(frames):
    return [(seq, value) for seq, node, kind, value in frames
            if node == MDC_ID and kind == SET_RPM]


def is_output_off_write(command):
    """A write that disables the supply's output.

    On the PSB this is one `OUTP OFF` to the single shared DC terminal (both the
    source and the sink quadrant call the same EAPSB10000Visa._output_enable);
    the simulated PSU spells it `:OUTP1:STAT OFF`. Act 3 asserts this NEVER
    happens on the trip path -- see design.md 10a.
    """
    text = command.upper()
    return "OUTP" in text and "OFF" in text


def is_zero_volt_write(command):
    """A voltage SETPOINT write of zero -- not a protection level, not a query.

    `:SOUR1:VOLT 0.000` on the simulated PSU, `VOLT 0.000` on the real PSB.
    """
    text = command.upper()
    if "VOLT" not in text or "PROT" in text or text.endswith("?"):
        return False
    try:
        return float(text.rsplit(" ", 1)[-1].rstrip("V")) == 0.0
    except ValueError:
        return False


def test_the_stand_trips_itself_and_the_operator_recovers_it():
    clock = harness._Clock()

    with (
        patch("instro.unstable.transports.can._prime_gs_usb_backend", autospec=True),
        patch("instro.unstable.transports.can.can.Bus", autospec=True) as bus_cls,
        patch("instro.psu.drivers.simulated.VisaDriver", autospec=True) as visa_cls,
    ):
        bus = bus_cls.return_value
        visa = visa_cls.return_value
        visa.query.return_value = '0,"No error"'
        plant = _FaultPlant(bus, clock)
        bus.recv.side_effect = plant.recv

        stand = harness.build_stand(bus, clock)
        session = StandSession(stand=stand, tick_hz=TICK_HZ)
        stand.open()

        # -------------------------------------------------------------------
        # Bring-up. Unchanged from the cleared feature test, except that a
        # freshly-built session must start with no trip latched.
        # -------------------------------------------------------------------
        session.tick(DT)
        clock.advance(DT)
        session.set_psu_output_enabled(True)
        session.arm()
        assert session.state is State.ARMED
        assert session.trip is None

        # ===================================================================
        # ACT 1 (T-5) -- a normal STOP must keep feeding the drive's watchdog
        # while the brake bleeds off.
        #
        # At LIMITS.brake_ramp_a_per_s (2.0 A/s) a full 20 A bleed-off takes
        # ~10 s. The controllers' firmware releases the motor 1000 ms after the
        # last frame, and instro has no periodic transmit, so the session's own
        # per-tick re-send is the only thing holding the drive. Sending nothing
        # for nine seconds would release the drive while the absorbers are still
        # regen-braking -- the inverse of the ordering interlock 13 guarantees.
        #
        # The fix under test holds the setpoint rather than ramping it: the
        # UNCHANGED commanded_erpm goes out every tick, after that tick's brake
        # frame. An unchanged value is not a decrease, so the ordering invariant
        # is preserved structurally, not by argument.
        # ===================================================================
        session.set_speed_setpoint_erpm(RUN_ERPM)
        harness.run_until(session, clock, lambda s: s.commanded_erpm >= RUN_ERPM)
        session.set_brake_current_a(RUN_BRAKE_A)
        harness.run_until(session, clock, lambda s: s.commanded_brake_a >= RUN_BRAKE_A)
        assert session.state is State.RUNNING

        stop_index = len(bus.send.call_args_list)
        session.stop()
        assert session.state is State.STOPPING

        bleed_ticks = 0
        while True:
            before = len(bus.send.call_args_list)
            session.tick(DT)
            clock.advance(DT)
            bleed_ticks += 1
            assert bleed_ticks < 2000, "the brake never bled off"

            if session.commanded_brake_a <= 0.0:
                break  # brake reached zero this tick; the speed ramp may start

            this_tick = speed_frames(frames_from(bus, before))
            assert this_tick, (
                "no speed frame was sent on a STOPPING tick while brake current "
                f"was still {session.commanded_brake_a:.2f} A -- the drive's "
                "1000 ms firmware watchdog will expire and release it while the "
                "absorbers are still braking"
            )
            assert all(value == RUN_ERPM for _seq, value in this_tick), (
                f"the speed setpoint moved during the brake bleed-off: {this_tick}. "
                "It must be re-sent UNCHANGED -- holding the setpoint feeds the "
                "watchdog; ramping it breaks the brake-before-speed ordering"
            )

        assert bleed_ticks > 100, (
            f"the bleed-off took only {bleed_ticks} ticks; at "
            f"{LIMITS.brake_ramp_a_per_s} A/s from {RUN_BRAKE_A} A it should take "
            "hundreds, which is what makes the missing watchdog feed dangerous"
        )

        harness.run_until(session, clock, lambda s: s.state is State.IDLE)
        assert session.commanded_erpm == 0.0
        assert session.commanded_brake_a == 0.0

        # No gap between speed frames anywhere in the STOP window exceeds the
        # watchdog margin, and speed never fell while either absorber braked.
        stop_window = frames_from(bus, stop_index)
        stop_speed = speed_frames(stop_window)
        gaps = [b[0] - a[0] for a, b in zip(stop_speed, stop_speed[1:])]
        assert gaps and max(gaps) * DT <= MAX_WATCHDOG_GAP_S, (
            f"largest gap between speed frames during STOP was "
            f"{max(gaps) * DT:.3f}s; the watchdog expires at 1.0s"
        )

        speed = None
        brake = {DMC_L_ID: 0.0, DMC_R_ID: 0.0}
        for _seq, node, kind, value in stop_window:
            if kind == SET_RPM and node == MDC_ID:
                if speed is not None and value < speed:
                    assert brake[DMC_L_ID] == 0.0 and brake[DMC_R_ID] == 0.0, (
                        f"speed was commanded down to {value} ERPM while the "
                        f"absorbers still braked at L={brake[DMC_L_ID]}A "
                        f"R={brake[DMC_R_ID]}A"
                    )
                speed = value
            elif kind == SET_CURRENT_BRAKE and node in brake:
                brake[node] = value

        # ===================================================================
        # ACT 2 (T-1) -- the DUT runs away, and the stand stops itself.
        # ===================================================================
        session.arm()
        assert session.state is State.ARMED

        # Both setpoints are re-applied EXPLICITLY rather than inherited from the
        # ones Act 1's STOP left behind. The stand does currently retain them
        # across a STOP -- see design.md open question 7 -- but a trip test that
        # is loaded only by accident of that behaviour would silently become an
        # unloaded test the day it is fixed, and "the absorbers were released
        # before the drive" is only worth asserting under load.
        session.set_speed_setpoint_erpm(RUN_ERPM)
        session.set_brake_current_a(RUN_BRAKE_A)
        harness.run_until(session, clock, lambda s: s.commanded_erpm >= RUN_ERPM)
        harness.run_until(session, clock, lambda s: s.commanded_brake_a >= RUN_BRAKE_A)
        assert session.state is State.RUNNING
        assert session.commanded_brake_a == RUN_BRAKE_A, (
            "the stand must be under real regenerative load when it trips, or the "
            "release-ordering assertion below is about frame order only"
        )

        trip_index = len(bus.send.call_args_list)
        plant.mdc_velocity_override = OVERSPEED_ERPM
        session.tick(DT)
        clock.advance(DT)

        assert session.state is State.TRIPPED, (
            f"the DUT reported {OVERSPEED_ERPM} ERPM against a "
            f"{LIMITS.dut_overspeed_erpm} ERPM limit and the stand kept running"
        )
        assert session.trip is not None
        assert session.trip.reason == "dut_overspeed"
        assert session.trip.node == "mdc"

        # Every node was zeroed on the trip tick, bypassing the ramps entirely --
        # and the absorbers were released BEFORE the drive, so even the trip path
        # never leaves the driveline decelerating against live regen braking.
        zeroing = frames_from(bus, trip_index)
        brake_zeroed_at = {}
        drive_released_at = None
        for seq, node, kind, value in zeroing:
            if kind == SET_CURRENT_BRAKE and node in (DMC_L_ID, DMC_R_ID) and value == 0.0:
                brake_zeroed_at.setdefault(node, seq)
            elif kind == SET_CURRENT and node == MDC_ID and value == 0.0:
                drive_released_at = seq if drive_released_at is None else drive_released_at

        assert set(brake_zeroed_at) == {DMC_L_ID, DMC_R_ID}, (
            f"both absorbers must be commanded to zero brake current on the trip "
            f"tick; got {brake_zeroed_at} from {zeroing}"
        )
        assert drive_released_at is not None, (
            f"the drive motor was never released on the trip tick; frames were {zeroing}"
        )
        assert max(brake_zeroed_at.values()) < drive_released_at, (
            "the drive was released before the absorbers were -- a trip must "
            "take the load off first, same ordering as a graceful STOP"
        )

        # A tripped stand is silent on the wire. Nothing more is commanded, so the
        # firmware watchdog expires and every controller releases to freewheel --
        # a more definitive stop than a commanded zero.
        after_trip = len(bus.send.call_args_list)
        for _ in range(10):
            session.tick(DT)
            clock.advance(DT)
        assert len(bus.send.call_args_list) == after_trip, (
            "a TRIPPED session kept putting command frames on the bus: "
            f"{frames_from(bus, after_trip)}"
        )

        # Recovery is one specific control, and it is not disarm().
        session.disarm()
        assert session.state is State.TRIPPED, "disarm() must not clear a latched trip"
        assert not session.can_arm()
        session.arm()
        assert session.state is State.TRIPPED, "a latched trip must refuse arming"

        session.acknowledge_trip()
        assert session.state is State.IDLE
        assert session.trip is None

        # Acknowledging is a dismissal, not a fix: the machine is still
        # overspeeding, so it still cannot be armed.
        assert not session.can_arm(), (
            "arming was allowed while the DUT was still reporting "
            f"{OVERSPEED_ERPM} ERPM -- acknowledging clears the banner, not the fault"
        )
        session.arm()
        assert session.state is State.IDLE

        # The DUT coasts to rest. Now, and only now, the stand can be armed again.
        plant.mdc_velocity_override = 0.0
        session.tick(DT)
        clock.advance(DT)
        assert session.can_arm()

        session.arm()
        assert session.state is State.ARMED

        # And it re-arms AT REST: the trip cleared the operator's setpoints, so
        # the stand does not resume the 8000 ERPM request that was live when it
        # tripped.
        rearm_index = len(bus.send.call_args_list)
        session.tick(DT)
        clock.advance(DT)
        assert session.commanded_erpm == 0.0, (
            "re-arming after a trip resumed the pre-trip speed request; a trip "
            "must clear the operator's setpoints, not just the commanded values"
        )
        assert [v for _s, v in speed_frames(frames_from(bus, rearm_index))] == [0.0]

        # ===================================================================
        # ACT 3 (T-6) -- killing the supply mid-run stops the machine, the zeros
        # go out while the bus is still alive to receive them, and the supply's
        # OUTPUT is never disabled.
        #
        # "Disable the PSU" means "command the source voltage setpoint to 0 V",
        # NOT output_enable(False): on the EA PSB the output enable is a single
        # command to one shared DC terminal, so disabling it takes the SINK path
        # down with the source -- the manual's own "sink path disabled or
        # interrupted while the dynos brake" failure row. Commanding 0 V removes
        # the drive voltage while leaving the unit regulating, so it can still
        # absorb what a still-turning absorber pushes back. See design.md 10,
        # 10a and open question 4.
        #
        # CAVEAT, and it travels with the test deliberately: that reading is
        # code-level (instro's ea_psb10000.py on branch
        # wh/instro-511-513-ea-psb-10000-device), NOT a bench result. Nothing in
        # that driver's hardware suite pushes current in while the setpoint goes
        # to zero. design.md 10's caveat box names the check.
        # ===================================================================
        plant.mdc_velocity_override = None  # the plant follows commands again
        session.set_speed_setpoint_erpm(RUN_ERPM)
        session.set_brake_current_a(RUN_BRAKE_A)
        harness.run_until(session, clock, lambda s: s.commanded_erpm >= RUN_ERPM)
        harness.run_until(session, clock, lambda s: s.commanded_brake_a >= RUN_BRAKE_A)
        assert session.state is State.RUNNING

        # The supply is cut with the absorbers genuinely loaded. Without this the
        # "load off before the supply's voltage goes to zero" assertion below
        # would be satisfied by a brake frame that was already zero, i.e. it
        # would test frame order and not the invariant T-6's policy protects.
        assert session.commanded_brake_a == RUN_BRAKE_A

        # Record how many frames had gone out by the time each PSU write happened,
        # so the ordering between "zero the machine" and "take the supply to 0 V"
        # is observable rather than assumed.
        psu_writes: list[tuple[str, int]] = []

        def record_write(command, *args, **kwargs):
            psu_writes.append((command, len(bus.send.call_args_list)))

        visa.write.side_effect = record_write

        cut_index = len(bus.send.call_args_list)
        session.set_psu_output_enabled(False)

        assert session.state is State.TRIPPED, (
            "the supply was de-energised while the stand was RUNNING and the "
            "session kept commanding an unpowered machine"
        )
        assert session.trip is not None
        assert session.trip.reason == "psu_disabled"

        # The supply's output must be LEFT ON. Disabling it is the one command
        # that takes the sink path down with the source, and the absorbers may
        # still be turning.
        assert stand.psu_output_enabled(), (
            "the supply's output was disabled on the trip path; that is one "
            "OUTP OFF to the PSB's single shared terminal, which removes the "
            "sink for anything the absorbers push back"
        )
        assert not [cmd for cmd, _at in psu_writes if is_output_off_write(cmd)], (
            "an output-disable command was written to the supply; the trip path "
            f"must only command 0 V. Writes were {psu_writes}"
        )

        zero_v_writes = [(cmd, at) for cmd, at in psu_writes
                         if is_zero_volt_write(cmd)]
        assert zero_v_writes, (
            f"the supply was never commanded to 0 V; writes were {psu_writes}"
        )
        _command, frames_before_zero_v = zero_v_writes[0]

        pre_zero_v = frames_from(bus, cut_index)
        pre_zero_v = [f for f in pre_zero_v if f[0] < frames_before_zero_v]
        assert any(
            node in (DMC_L_ID, DMC_R_ID) and kind == SET_CURRENT_BRAKE and value == 0.0
            for _seq, node, kind, value in pre_zero_v
        ), (
            "the absorbers were not zeroed before the supply was taken to 0 V; "
            "the brake command must reach zero before the supply's behaviour "
            f"changes underneath it. Frames before the PSU write: {pre_zero_v}"
        )
        assert any(
            node == MDC_ID and kind == SET_CURRENT and value == 0.0
            for _seq, node, kind, value in pre_zero_v
        ), (
            "the drive was not released before the supply was taken to 0 V. "
            f"Frames before the PSU write: {pre_zero_v}"
        )

        # Same recovery path as any other trip -- and the stand stays unarmable
        # until the supply is back.
        session.acknowledge_trip()
        assert session.state is State.IDLE
        assert not session.can_arm(), "the supply is still commanded to 0 V"

        session.set_psu_output_enabled(True)
        session.tick(DT)
        clock.advance(DT)
        assert session.can_arm()

        stand.close()
        bus.shutdown.assert_called_once_with()
