**Click on the diagram**: every click toggles, a second click undoes it.

| Click | Does |
|---|---|
| Compressor or `v_1` | Pressurize, or vent the supply |
| `v_2` | Open at the flow setpoint, or close |
| `pr_2` | Pressurize the vessel to the setpoint, or zero it |
| `v_3` | Extend, or retract the actuator |
| `v_5` | Isolate the vessel, or reopen it |

Everything except venting needs the status to read **Ready**.
**Vent** always works and stops a running sequence.
