def dut_erpm_to_stub_rpm(erpm: float, chain_ratio: float = 36.38) -> float:
    return erpm / chain_ratio


def stub_rpm_to_dut_erpm(stub_rpm: float, chain_ratio: float = 36.38) -> float:
    return stub_rpm * chain_ratio


def amps_to_brake_torque_nm(amps: float, kt: float = 0.1906) -> float:
    return amps * kt


def dyno_rpm_to_erpm(rpm: float, pole_pairs: int = 7) -> float:
    return rpm * pole_pairs
