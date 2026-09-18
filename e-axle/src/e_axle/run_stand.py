"""Stand up a real EAxleStand against the actual hardware and run a DUT torque sweep.

build.py is the composition root -- this script only drives the stand it hands back.
"""

import logging
from time import sleep

from instro.lib.transports.visa import VisaConfig, VisaDriver

from e_axle.build import build_stand
from e_axle.stand import EAxleStandState

logging.basicConfig(level=logging.INFO, format="%(name)s %(levelname)s %(message)s")

logger = logging.getLogger(__name__)


STEP_HOLD_S = 5.0
SWEEP_STEPS = 9
MAX_DRAINED_ERRORS = 50


def _sweep(max_value: float, steps: int) -> list[float]:
    """Ramp from 0 up to max_value and back down to 0, in `steps` increments each way."""
    step = max_value / steps
    up = [step * i for i in range(steps + 1)]
    return up + up[-2::-1]


def _drain_error_queue(visa_resource: VisaConfig | str) -> None:
    """Pop and discard errors left in the PSB's queue by a previous session that never cleanly closed."""
    visa = VisaDriver(visa_resource)
    visa.open()
    try:
        for _ in range(MAX_DRAINED_ERRORS):
            if visa.query("SYST:ERR?").startswith("0"):
                return
    finally:
        visa.close()


def main() -> None:
    with build_stand() as stand:
        # __enter__ already called open() (== arm()), landing in ARMED.
        stand.dut.control_mode.setpoint = "speed"
        stand.dut.velocity.setpoint = 1000.0  # mechanical RPM
        stand.right_load.control_mode.setpoint = "torque"
        stand.left_load.control_mode.setpoint = "torque"
        logger.info(f"Entering run state. speed setpoint: {stand.dut.velocity.setpoint}")
        stand.run()
        sleep(5)
        for torque in _sweep(
            min(stand.left_load.torque.maximum, stand.right_load.torque.maximum) * 0.95,
            SWEEP_STEPS,
        ):
            logger.info(f"Load sweep step started:  ={torque:.2f} Nm")
            stand.left_load.torque.setpoint = -torque
            stand.right_load.torque.setpoint = -torque
            sleep(STEP_HOLD_S)
            logger.info(
                f"dut speed: setpoint={stand.dut.velocity.setpoint:.1f} measured={stand.dut.velocity.measured}"
            )
            if stand.state == EAxleStandState.TRIPPED:
                logger.error("Stand tripped, aborting sweep")
                stand.reset()
                break

        stand.stop()
        stand.disarm()


if __name__ == "__main__":
    main()
