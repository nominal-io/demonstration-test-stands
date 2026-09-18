"""Measured ground-truth constants for the E-Axle dynamometer stand.

This module is the single source of truth for the stand's mechanical and
electrical chain. Scripts, tests, and control code import from here rather
than redefining these values locally -- a wrong pole-pair/gear split silently
scales every derived RPM and torque figure without tripping anything.

Provenance is the commissioning data in
``reference/e-axle-dyno_user_manual_rev_1-0.md``, sections 4 and 5.

Deliberately excluded: test-profile settings and trip thresholds (top speed,
step size, overspeed limits). Those are policy choices belonging to the
individual scripts, not properties of the hardware.
"""

# --------------------------------------------------------------------------
# DUT chain
# --------------------------------------------------------------------------

# MEASURED: DUT_erpm / dyno_erpm = 5.1973 (sd 0.0021) across all load levels.
# With the dynos at 7 pole pairs that fixes the DUT chain at 36.38.
#
# ONLY THIS PRODUCT is measurable from CAN telemetry. Every stand-level
# quantity depends on the product of the DUT's pole pairs and its gear ratio,
# never on either alone (manual 5.2), so this is the number to trust.
DUT_CHAIN = 36.38

# The split below is PROVISIONAL and affects only motor-shaft-level figures.
# Separating the two requires a gearbox tooth count or an independent pole
# count on the DUT motor. 4 pole pairs is the working choice because it places
# the ratio near the nominal 9.5; 3 pole pairs would imply 12.13, which
# matches nothing.
DUT_POLE_PAIRS = 4
DUT_GEAR = DUT_CHAIN / DUT_POLE_PAIRS  # 9.095

# --------------------------------------------------------------------------
# Dyno chain
# --------------------------------------------------------------------------

DYNO_POLE_PAIRS = 7  # physically counted on the rotors, 12N14P
DYNO_GEAR = 1.0      # MP 8055 coupled 1:1 direct to the half shafts

# MEASURED: DUT ERPM / dyno ERPM, sd 0.0021, across all load levels.
DUT_ERPM_PER_DYNO_ERPM = DUT_CHAIN / (DYNO_POLE_PAIRS * DYNO_GEAR)  # 5.1973

# --------------------------------------------------------------------------
# Torque constants
# --------------------------------------------------------------------------

# Flux linkage comes from each controller's own back-EMF detection routine.
# Kt = 1.5 * pole_pairs * lambda is exact for a sinusoidally wound PMSM under
# field-oriented control, not an empirical fit (manual 5.4).
DUT_LAMBDA = 0.014423
DUT_KT = 1.5 * DUT_POLE_PAIRS * DUT_LAMBDA

DYNO_LAMBDA = 0.018148
DYNO_KT = 1.5 * DYNO_POLE_PAIRS * DYNO_LAMBDA  # 0.1906 Nm/A

# --------------------------------------------------------------------------
# CAN node map
# --------------------------------------------------------------------------

DUT_ID = 0
DYNO_IDS = (1, 2)

# vesc_id -> (display name, pole pairs, gear ratio from motor to output shaft)
NODES: dict[int, tuple[str, int, float]] = {
    DUT_ID: ("DUT", DUT_POLE_PAIRS, DUT_GEAR),
    1: ("DMC-L", DYNO_POLE_PAIRS, DYNO_GEAR),
    2: ("DMC-R", DYNO_POLE_PAIRS, DYNO_GEAR),
}

# vesc_id -> torque constant in Nm/A
KT_BY_ID: dict[int, float] = {
    DUT_ID: DUT_KT,
    1: DYNO_KT,
    2: DYNO_KT,
}
