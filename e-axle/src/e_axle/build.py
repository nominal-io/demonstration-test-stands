"""Composition root for a real EAxleStand wired to the actual hardware.

stand.py never references a concrete driver class, so this module owns building the
shared CAN transport and the shared EAPSB10000Visa device and hands EAxleStand the
already-constructed drivers.
"""

from instro.eload import InstroELoad
from instro.lib.transports.visa import VisaConfig
from instro.psu import InstroPSU
from instro.psu.drivers.ea_psb10000 import EAPSB10000Visa
from instro.unstable.motorcontroller import InstroMotorController
from instro.unstable.motorcontroller.drivers.vesc_6 import VESC6
from instro.unstable.transports.can import CanConfig, CanTransport
from instro.lib.publishers.nominal_core import NominalCorePublisher

from e_axle.stand import EAxleStand
from e_axle.stand_config import EAxleStandConfig

NETWORK_ADDRESS = "TCPIP0::192.168.0.3::5025::SOCKET"
MOTOR_INTERVAL_S = 0.06  # VESC6 recommends >=10 Hz; its firmware times out a motor after 0.5s of silence
PSB_INTERVAL_S = 0.5  # The PSB's SCPI interface cannot service the motor rate, and needs no resend to hold its setpoints

CORE_RID = "ri.catalog.cerulean-staging.dataset.247ceceb-39dc-4581-a128-a15ed4788e38"

def build_stand(
    config: EAxleStandConfig | None = None,
    *,
    network_address: str = NETWORK_ADDRESS,
    motor_interval_s: float = MOTOR_INTERVAL_S,
    psb_interval_s: float = PSB_INTERVAL_S,
) -> EAxleStand:
    """Build an EAxleStand against the real hardware, defaulting to the shipped config."""
    if config is None:
        config = EAxleStandConfig.default()

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

    # Source and sink are two quadrants of one physical bidirectional supply.
    visa = VisaConfig(visa_resource=network_address, visa_backend="@py")
    psb = EAPSB10000Visa(visa)
    source_driver = InstroPSU("source", driver=psb.source, num_channels=1)
    sink_driver = InstroELoad("sink", driver=psb.sink)

    motors = (dut_controller, left_load_controller, right_load_controller)
    for motor in motors:
        motor.background_interval = motor_interval_s
        motor.add_publisher(NominalCorePublisher(CORE_RID))
    for supply in (source_driver, sink_driver):
        supply.background_interval = psb_interval_s
        supply.add_publisher(NominalCorePublisher(CORE_RID))

    return EAxleStand.from_drivers(config, *motors, source_driver, sink_driver)
