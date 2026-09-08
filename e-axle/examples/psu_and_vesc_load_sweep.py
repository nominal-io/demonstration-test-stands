"""Example: PSU configuration plus a VESC6 CAN load sweep on the e-axle
stand, driven through StandSession's own operator state machine -- arm, run,
stop -- rather than raw driver calls. Standalone: no Connect, no main.py.

Configures the EA PSB 10000 supply's voltage and current limit directly
(configuration, not runtime state -- outside StandSession's own control
surface, which only ever toggles output on/off). Everything from there runs
through the session: enabling the PSU, waiting for fresh telemetry from all
three nodes, arming, holding a cautious DUT speed while both dyno absorbers
step through a short brake-current sweep, and stopping -- so the session's
own interlocks, watchdog resend, and STOP sequencing (brake before speed) do
the real work, exactly as main.py's Connect adapter does.

pole_pairs below is 1/7/7, matching constants.py. StandSession's interlocks
(check_dut_overspeed, the speed-tracking-error check) assume the DUT's own
VESC6 is built with pole_pairs=1, so its `velocity` field is already raw
ERPM -- see session.py's own comment ("mdc is pole_pairs=1, so velocity is
already ERPM"). Building the DUT with pole_pairs=4 instead (as instro's
generic motorcontroller example does) would compare a mechanical-RPM
reading against ERPM-scaled thresholds once run through the session; that
convention only applies to scripts that never touch StandSession.

VESC Tool prerequisites: a distinct VESC ID per controller, CAN baud 500
kbps, CAN status message mode including statuses 1/4/5 on every controller,
and FOC motor detection completed on every motor.

Edit PSU_VISA_RESOURCE, PSU_VOLTAGE_V, and PSU_CURRENT_LIMIT_A below to match
the stand before running.

Run:
    <e-axle venv python> examples/psu_and_vesc_load_sweep.py
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from instro.psu import InstroPSU
from instro.psu.drivers import EAPSB10000Visa
from instro.unstable.motorcontroller import InstroMotorController
from instro.unstable.motorcontroller.drivers import VESC6
from instro.unstable.transports import CanConfig, CanTransport

from session import StandSession, State  # noqa: E402
from stand import HardwareStand  # noqa: E402

# --- CAN wiring -- matches constants.py -------------------------------------
CAN_CHANNEL, CAN_INTERFACE, CAN_BITRATE = 0, "gs_usb", 500_000
MDC_ID, DMC_L_ID, DMC_R_ID = 0, 1, 2
MDC_POLE_PAIRS, DMC_POLE_PAIRS = 1, 7

# --- PSU wiring -- edit before running --------------------------------------
PSU_VISA_RESOURCE = "TCPIP0::192.168.1.50::5025::SOCKET"  # <-- edit: the stand's actual EA PSB 10000 address
PSU_CHANNEL = 1
PSU_VOLTAGE_V = 48.0  # <-- edit: the stand's nominal bus voltage
PSU_CURRENT_LIMIT_A = (
    5.0  # <-- edit: conservative for a first pass; raise once this runs clean
)

# --- Session/sweep parameters -- deliberately small for a first pass -------
TICK_HZ = 50.0
DT = 1.0 / TICK_HZ
DUT_ERPM = 3000.0  # <-- edit: raw ERPM (mdc's pole_pairs=1, so this IS the wire value)
BRAKE_STEPS_A = (0.0, 1.0, 0.0)  # unloaded baseline, one small step, release
HOLD_SECONDS = 3.0
ARM_TIMEOUT_S = 5.0


def _value(measurement):
    """Unwrap a one-channel Measurement (InstroPSU's get_voltage/get_current
    return exactly one channel) to its latest reading, or None."""
    if measurement is None:
        return None
    (values,) = measurement.channel_data.values()
    return values[-1]


bus = CanTransport(
    CanConfig(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=CAN_BITRATE)
)
mdc = InstroMotorController(
    "mdc", driver=VESC6(bus, pole_pairs=MDC_POLE_PAIRS, controller_id=MDC_ID)
)
dmc_l = InstroMotorController(
    "dmc_l", driver=VESC6(bus, pole_pairs=DMC_POLE_PAIRS, controller_id=DMC_L_ID)
)
dmc_r = InstroMotorController(
    "dmc_r", driver=VESC6(bus, pole_pairs=DMC_POLE_PAIRS, controller_id=DMC_R_ID)
)
psu = InstroPSU(
    name="psu", driver=EAPSB10000Visa(PSU_VISA_RESOURCE).source, num_channels=1
)


def configure_psu() -> None:
    """Voltage and current limit are configuration, set once before the
    session ever touches the PSU -- StandSession only enables/disables it."""
    psu.set_voltage(PSU_VOLTAGE_V, channel=PSU_CHANNEL)
    psu.set_current_limit(PSU_CURRENT_LIMIT_A, channel=PSU_CHANNEL)
    voltage = _value(psu.get_voltage(channel=PSU_CHANNEL))
    current = _value(psu.get_current(channel=PSU_CHANNEL))
    print(
        f"PSU configured: setpoint {PSU_VOLTAGE_V:.1f} V, limit {PSU_CURRENT_LIMIT_A:.1f} A"
    )
    if voltage is not None and current is not None:
        print(
            f"  (currently reading {voltage:.2f} V, {current:.2f} A -- output not yet enabled)"
        )


def wait_until_armed(session: StandSession) -> None:
    """Tick until three fresh nodes and the enabled PSU let can_arm() pass,
    then arm."""
    deadline = time.monotonic() + ARM_TIMEOUT_S
    while not session.can_arm():
        if time.monotonic() > deadline:
            raise RuntimeError(
                "stand never became armable; check CAN wiring and telemetry"
            )
        session.tick(DT)
        time.sleep(DT)
    session.arm()


def hold(session: StandSession, brake_a: float) -> None:
    """Hold DUT_ERPM and brake_a for HOLD_SECONDS. Ticking the session itself
    is what resends both setpoints every cycle (the VESC watchdog) and
    evaluates every interlock -- nothing here talks to a driver directly."""
    session.set_speed_setpoint_erpm(DUT_ERPM)
    session.set_brake_current_a(brake_a)
    deadline = time.monotonic() + HOLD_SECONDS
    while time.monotonic() < deadline:
        session.tick(DT)
        if session.trip is not None:
            raise RuntimeError(f"stand tripped: {session.trip.reason}")
        time.sleep(DT)


def wait_until_stopped(session: StandSession) -> None:
    while session.state not in (State.IDLE, State.TRIPPED):
        session.tick(DT)
        time.sleep(DT)
    if session.trip is not None:
        raise RuntimeError(f"stand tripped while stopping: {session.trip.reason}")


def report(session: StandSession, brake_a: float) -> None:
    print(
        f"loads {brake_a:.1f} A brake  ->  state={session.state.value}  "
        f"DUT {session.commanded_erpm:.0f} ERPM commanded, "
        f"{session.commanded_brake_a:.1f} A brake commanded"
    )


def main() -> None:
    configure_psu()
    stand = HardwareStand(mdc=mdc, dmc_l=dmc_l, dmc_r=dmc_r, psu=psu)
    session = StandSession(stand=stand, tick_hz=TICK_HZ)
    stand.open()
    try:
        session.set_psu_output_enabled(
            True
        )  # through the session, so can_arm() sees it
        wait_until_armed(session)

        for brake_a in BRAKE_STEPS_A:
            hold(session, brake_a)
            report(session, brake_a)

        session.stop()
        wait_until_stopped(session)
        report(session, session.commanded_brake_a)
    finally:
        print(
            "shutting down: zeroing the bus through the session, then disabling PSU output"
        )
        session.set_psu_output_enabled(
            False
        )  # zeroes bus voltage; T-6 -- never a raw output_enable(False) mid-run
        psu.output_enable(
            False, channel=PSU_CHANNEL
        )  # the script is exiting, so fully de-energize on the way out
        stand.close()


if __name__ == "__main__":
    main()
