"""The panel's operating procedures, from its operations sheet, as stand sequences.

Each sequence is a generator: it issues stand commands, then yields a WaitUntil to pause until
the panel responds. Start one with `start(stand, name, **kwargs)`: its first step runs
immediately (so a refused first command raises to the caller), then it advances once per
`stand.update()`. After that, a refused command or a timed-out wait trips the stand.
"""

import logging
from collections.abc import Callable

from fluids_panel.stand import FluidsStand, FluidsStandState, Sequence, WaitUntil

logger = logging.getLogger(__name__)


def startup(stand: FluidsStand) -> Sequence:
    """Startup steps 1.4-1.7: compressor on, main vent closed, wait for supply pressure."""
    stand.pressurize()
    yield WaitUntil(
        f"p_1 to reach {stand.config.ready_psi:g} psi",
        lambda: stand.state == FluidsStandState.READY,
        timeout_s=stand.config.pressurize_timeout_s + 5.0,
    )


def flow(stand: FluidsStand, lpm: float = 7.8, hold_s: float = 10.0) -> Sequence:
    """Flow control steps 2.1-2.4: command v_2, confirm on fm_1, hold, then close v_2."""
    stand.set_flow(lpm)
    tolerance = max(0.1 * lpm, stand.config.flowing_lpm)
    yield WaitUntil(
        f"fm_1 to reach {lpm:g} L/min",
        lambda: abs(stand.fm_1_lpm - lpm) <= tolerance,
        timeout_s=15.0,
    )
    yield stand.wait(hold_s)
    stand.set_flow(0.0)
    yield WaitUntil(
        "fm_1 to stop flowing",
        lambda: stand.fm_1_lpm < stand.config.flowing_lpm,
        timeout_s=15.0,
    )


def actuate(stand: FluidsStand, cycles: int = 1) -> Sequence:
    """Linear actuation steps 3.1-3.5: extend to sw_2, retract to sw_1, `cycles` times."""
    timeout_s = stand.config.actuator_timeout_s + 1.0
    for _ in range(int(cycles)):
        stand.extend()
        yield WaitUntil(
            "sw_2 (extended)",
            lambda: bool(stand.sw_2.measured and not stand.sw_1.measured),
            timeout_s=timeout_s,
        )
        stand.retract()
        yield WaitUntil(
            "sw_1 (retracted)",
            lambda: bool(stand.sw_1.measured and not stand.sw_2.measured),
            timeout_s=timeout_s,
        )


def proof_leak(
    stand: FluidsStand, psig: float = 72.5, hold_s: float = 60.0
) -> Sequence:
    """Proof and leak steps 4.1-4.7: fill the vessel, isolate it, and measure the decay.

    Ends by venting the vessel back through pr_2 rather than holding pressure behind a closed
    v_5, so the leg is left in its default state, as the shutdown sequence expects.
    """
    stand.isolate_vessel(False)
    stand.set_vessel_pressure(psig)
    yield WaitUntil(
        f"p_2 to settle at {psig:g} psi",
        # pr_2 can only deliver what the supply gives it.
        lambda: abs(stand.p_2_psi - min(psig, stand.p_1_psi)) <= 1.0,
        timeout_s=60.0,
    )

    stand.isolate_vessel(True)
    start_psi = stand.p_2_psi
    start_s = stand.now()
    yield stand.wait(hold_s)
    elapsed_min = (stand.now() - start_s) / 60.0
    stand.leak_rate_psi_min = (start_psi - stand.p_2_psi) / elapsed_min
    logger.info("proof_leak: p_2 leak rate %.2f psi/min", stand.leak_rate_psi_min)

    stand.set_vessel_pressure(0.0)
    stand.isolate_vessel(False)
    yield WaitUntil(
        "p_2 to vent",
        lambda: stand.p_2_psi <= stand.config.vented_psi,
        timeout_s=60.0,
    )


def shutdown(stand: FluidsStand) -> Sequence:
    """Shutdown steps 5.1-5.5: legs to default, compressor off, vent the supply."""
    stand.set_flow(0.0)
    stand.retract()
    stand.set_vessel_pressure(0.0)
    stand.isolate_vessel(False)
    if stand.state == FluidsStandState.READY:
        yield WaitUntil(
            "the legs to idle",
            lambda: bool(
                stand.fm_1_lpm < stand.config.flowing_lpm and stand.sw_1.measured
            ),
            timeout_s=stand.config.actuator_timeout_s + 1.0,
        )
    stand.vent()
    yield WaitUntil(
        "p_1 to vent",
        lambda: stand.state == FluidsStandState.VENTED,
        timeout_s=60.0,
    )
    logger.info(
        "shutdown: open the compressor's drain valve to bleed moisture (step 5.4)"
    )


SEQUENCES: dict[str, Callable[..., Sequence]] = {
    "startup": startup,
    "flow": flow,
    "actuate": actuate,
    "proof_leak": proof_leak,
    "shutdown": shutdown,
}


def start(stand: FluidsStand, name: str, **kwargs: float) -> None:
    """Start the named sequence on the stand."""
    if name not in SEQUENCES:
        raise ValueError(
            f"unknown sequence {name!r}; expected one of {list(SEQUENCES)}"
        )
    stand.start_sequence(name, SEQUENCES[name](stand, **kwargs))
