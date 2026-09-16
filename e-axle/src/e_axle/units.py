def erpm_to_geared_rpm(erpm: float, ratio: float) -> float:
    """Convert electrical RPM to mechanical RPM after a gear reduction.

    erpm: Electrical RPM before the reduction.
    ratio: Gear ratio the mechanical output is reduced by.
    """
    return erpm / ratio


def geared_rpm_to_erpm(rpm: float, ratio: float) -> float:
    """Convert geared mechanical RPM back to electrical RPM.

    rpm: Mechanical RPM after the gear reduction.
    ratio: Gear ratio the mechanical output is reduced by.
    """
    return rpm * ratio


def current_to_torque_nm(amps: float, kt: float) -> float:
    """Convert motor current to torque using the motor's torque constant.

    amps: Current in amps.
    kt: Motor torque constant in newton-meters per amp.
    """
    return amps * kt


def mechanical_rpm_to_erpm(rpm: float, pole_pairs: int) -> float:
    """Convert mechanical RPM to electrical RPM for a motor's pole count.

    rpm: Mechanical shaft RPM.
    pole_pairs: Number of magnetic pole pairs the motor has.
    """
    return rpm * pole_pairs


def erpm_to_mechanical_rpm(erpm: float, pole_pairs: int) -> float:
    """Convert electrical RPM to mechanical RPM for a motor's pole count.

    erpm: Electrical RPM.
    pole_pairs: Number of magnetic pole pairs the motor has.
    """
    return erpm / pole_pairs
