"""Stand up a real EAxleStand against the actual hardware and run a DUT torque sweep.

stand.py never references a concrete driver class -- this script is the composition root:
it owns building the shared CAN transport (all three motor controllers share one adapter)
and the shared EAPSB10000Visa device (source and sink are two quadrants of one physical
supply), then hands EAxleStand already-constructed InstroMotorController/InstroPSU/InstroELoad
instances plus config for everything else (channel bounds, defaults, timeouts).
"""

import logging
from pathlib import Path
from time import sleep

from instro.eload import InstroELoad
from instro.lib.publishers.nominal_core import NominalCorePublisher
from instro.lib.transports.visa import VisaConfig, VisaDriver
from instro.psu import InstroPSU
from instro.psu.drivers.ea_psb10000 import EAPSB10000Visa
from instro.unstable.motorcontroller import InstroMotorController
from instro.unstable.motorcontroller.drivers.vesc_6 import VESC6
from instro.unstable.transports.can import CanConfig, CanTransport

from e_axle.stand import EAxleStand, EAxleStandState
from e_axle.stand_config import EAxleStandConfig

logging.basicConfig(level=logging.INFO, format="%(name)s %(levelname)s %(message)s")

logger = logging.getLogger(__name__)


CONFIG_PATH = Path(__file__).parent.parent.parent / "nominal_config.yaml"
NETWORK_ADDRESS = "TCPIP0::192.168.0.3::5025::SOCKET"
STEP_HOLD_S = 5.0
SWEEP_STEPS = 9
MAX_DRAINED_ERRORS = 50
BACKGROUND_INTERVAL_S = 0.06  # VESC6 recommends >=10 Hz; its firmware times out a motor after 0.5s of silence

DATASET_RID = "ri.catalog.cerulean-staging.dataset.247ceceb-39dc-4581-a128-a15ed4788e38"


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


def build_stand() -> EAxleStand:
    config = EAxleStandConfig.from_yaml(CONFIG_PATH)

    # All three motor controllers share one physical CAN adapter (manual section 4.1).
    can = CanTransport(CanConfig(channel="0", interface="gs_usb"))
    dut_controller = InstroMotorController(
        "dut", driver=VESC6(channel=can, pole_pairs=4, controller_id=0)
    )
    left_load_controller = InstroMotorController(
        "left_load", driver=VESC6(channel=can, pole_pairs=7, controller_id=1)
    )
    right_load_controller = InstroMotorController(
        "right_load", driver=VESC6(channel=can, pole_pairs=7, controller_id=2)
    )
    for controller in (dut_controller, left_load_controller, right_load_controller):
        controller.add_publisher(NominalCorePublisher(DATASET_RID))
        controller.background_interval = BACKGROUND_INTERVAL_S

    # Source and sink are two quadrants of one physical bidirectional supply.
    visa = VisaConfig(visa_resource=NETWORK_ADDRESS, visa_backend="@py")
    # _drain_error_queue(visa)
    psb = EAPSB10000Visa(visa)
    source_driver = InstroPSU("source", driver=psb.source, num_channels=1)
    sink_driver = InstroELoad("sink", driver=psb.sink)
    for driver in (source_driver, sink_driver):
        driver.add_publisher(NominalCorePublisher(DATASET_RID))
        driver.background_interval = BACKGROUND_INTERVAL_S
    return EAxleStand.from_drivers(
        config,
        dut_controller,
        left_load_controller,
        right_load_controller,
        source_driver,
        sink_driver,
    )


def main() -> None:
    with build_stand() as stand:
        # __enter__ already called open() (== arm()), landing in ARMED.
        stand.dut.control_mode.setpoint = "speed"
        stand.dut.speed.setpoint = 1000.0  # mechanical RPM
        stand.right_load.control_mode.setpoint = "torque"
        stand.left_load.control_mode.setpoint = "torque"
        logger.info(f"Entering run state. speed setpoint: {stand.dut.speed.setpoint}")
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
                f"dut speed: setpoint={stand.dut.speed.setpoint:.1f} measured={stand.dut.speed.measured}"
            )
            if stand.state == EAxleStandState.TRIPPED:
                logger.error("Stand tripped, aborting sweep")
                stand.reset()
                break

        stand.stop()
        stand.disarm()


if __name__ == "__main__":
    main()
