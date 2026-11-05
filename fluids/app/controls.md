**Click on the diagram**: every click toggles, a second click undoes it.

| Click | Does |
|---|---|
| Compressor or `v_1` | Pressurize, or vent the supply |
| `v_2` | Open at the flow setpoint, or close |
| `pr_2` | Pressurize the vessel to the setpoint, or zero it |
| `v_3` | Extend, or retract the actuator |
| `v_5` | Isolate the vessel, or reopen it |

Pressurizing needs the status to read **Vented** or **Venting**. Opening, extending, isolating
or pressurizing the vessel needs **Ready**. Closing, retracting, zeroing and venting always work.

While a sequence runs, only **Vent**, **Abort** and **Reset trip** are accepted.
**Vent** always works and stops a running sequence.
