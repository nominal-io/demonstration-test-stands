# E-Axle Dynamometer Test Stand

## Design, Theory and Operation Manual

*Bench-scale regenerative dynamometer for electric axle characterization*

Prepared for Nominal
TR Techworks LLC
Responsible Engineer: Tyler Rowan
Revision 1.0 - 9/2/2026

## Contents

1. Purpose and Scope
   - 1.1 Intended audience
   - 1.2 Status
2. Safety
   - 2.1 Emergency stop
   - 2.2 Software abort is not a safety device
   - 2.3 Mechanical
   - 2.4 Electrical
3. System Overview
   - 3.1 Control topology
   - 3.2 Hardware inventory
   - 3.3 Signal and power paths
4. Machine Data
   - 4.1 Motors
   - 4.2 Derived stand constants
   - 4.3 Conversion equations
5. Theory of Operation
   - 5.1 ERPM and pole pairs
   - 5.2 The chain constant
   - 5.3 The open differential
   - 5.4 Torque estimation from motor current
     - 5.4.1 Accuracy of this method
   - 5.5 Why the dyno motors are the binding constraint
   - 5.6 Where the energy goes
6. Controller Configuration
   - 6.1 CAN node map
   - 6.2 Status broadcast
   - 6.3 Detected motor parameters
     - 6.3.1 The DUT sensor blend is deliberate
   - 6.4 Speed PID
7. Limits and Operating Envelope
   - 7.1 As-configured controller limits
   - 7.2 Operating policy limits
   - 7.3 Software trip thresholds
   - 7.4 Limits that are not yet set
8. CAN Interface
   - 8.1 Frame format
   - 8.2 Worked example
   - 8.3 Command frames
   - 8.4 Status frames
   - 8.5 Measurement caveats
   - 8.6 Adapter setup on Windows
   - 8.7 Bus termination
9. Thermal Behavior and Run Time
   - 9.1 Measured data
   - 9.2 Recommended run times
   - 9.3 DUT thermal
10. Software
    - 10.1 Tooling
    - 10.2 Architecture
      - 10.2.1 Receive runs on its own thread
      - 10.2.2 The transmit tick is the watchdog feed
    - 10.3 Sequence state machine
    - 10.4 Abort behavior
    - 10.5 Logging
    - 10.6 Bus health instrumentation
11. Standard Operating Procedure
    - 11.1 Pre-run checklist
    - 11.2 Running a loaded test
    - 11.3 Keyboard controls
    - 11.4 Stop conditions
12. Fault History and Troubleshooting
    - 12.1 CAN over-termination
    - 12.2 Host not keeping up with receive
    - 12.3 Detection and commutation faults
    - 12.4 USB and VESC Tool
    - 12.5 Diagnostic principle
13. Open Items
14. Recovery and Backup
15. Failure Mode Analysis
    - 15.1 First principles
    - 15.2 Protection layers
    - 15.3 Single-point failures
      - 15.3.1 Communications and host
      - 15.3.2 Controllers and motors
      - 15.3.3 Power
      - 15.3.4 Mechanical
    - 15.4 Combination failures
      - 15.4.1 Partial CAN loss
      - 15.4.2 Telemetry loss with commands intact
      - 15.4.3 Silent limit saturation
      - 15.4.4 Thermal derating during a sweep
      - 15.4.5 Two failures that share a cause
    - 15.5 Abort criteria
    - 15.6 Loss of CAN communication
    - 15.7 What has no automatic detection
16. Maintenance
    - 16.1 Before every run
    - 16.2 During every run (what to listen and watch for)
    - 16.3 Fastener torque
    - 16.4 Thread locking
    - 16.5 Scheduled maintenance
    - 16.6 After a fault or abort
    - 16.7 Accessing the stand
    - 16.8 Spares worth holding

---

## 1. Purpose and Scope

This manual describes the design, operating theory, configuration, and operation of the E-Axle Dynamometer Test Stand built by TR Techworks for Nominal. The stand spins an electric axle (the Device Under Test, or DUT) to a commanded speed while two absorption motors apply a controlled, measurable braking load.

It is a performance-characterization instrument. It maps speed, torque, current, power, and thermal behavior across an operating envelope. It is not an NVH stand, and it does not carry inline torque transducers, as torque is estimated from motor current as described in Section 5.4.

### 1.1 Intended audience

This revision is written for software engineers taking over or extending the stand. It assumes familiarity with Python and CAN, but not with motor control, so the motor-side theory is developed from first principles where it affects the software.

Everything a control or data-acquisition program needs is here: the CAN frame formats, the machine constants, the conversion equations, the limits that must not be exceeded, and the failure modes that have occurred on this hardware.

### 1.2 Status

All three motor controllers are configured and characterized. CAN communication is verified end to end. The stand has completed multiple ten-minute continuous loaded soak at 10 A of brake current per absorption motor with zero communication failures. The Python tooling described here is bench and development software; a production operator GUI is a planned follow-on.

> **Document conventions**
> ERPM means electrical RPM, which is what the controllers command and report. It is not mechanical RPM.
>
> Values quoted as "measured" come from logged CAN data. Values quoted as "detected" come from the controller's own parameter detection routine. Values quoted as "nominal" come from a datasheet and have not been independently verified.

## 2. Safety

The stand stores energy in two forms: a spinning driveline and a 48 V DC bus. Neither is lethal on its own, but both can cause injury and both can destroy hardware.

### 2.1 Emergency stop

> **The E-stop removes DC bus power from all three motor controllers.**
> It opens the contactor feeding the 48 V bus. All three controllers lose power simultaneously and every motor coasts to a stop under its own friction and inertia.

> **The E-stop is the only true safety device on this stand. Software aborts are a convenience.**
> Because power is removed rather than reduced, there is no controlled deceleration. The driveline freewheels. Unloaded coast-down has been observed to be quick (sub 2-seconds), but the operator should assume rotation continues for several seconds after the E-stop is pressed.

Before any run, confirm the E-stop is within reach of the operating position and that it actually drops the bus. Test it with the stand unpowered and spinning nothing.

### 2.2 Software abort is not a safety device

The test script provides a spacebar abort and a set of automatic trips. These depend on the Python process running, the USB-CAN adapter working, and the CAN bus being intact. Any of those can fail. Treat software aborts as an operational convenience that protects hardware, and the E-stop as the thing that protects people.

One specific limitation: if VESC Tool is actively driving a controller from its slider, it re-commands over USB and will fight the script. The software abort will not reliably hold against that. Use the E-stop.

### 2.3 Mechanical

- The 3/8 inch polycarbonate blast shield must be in place before any run.
- Frame casters locked.
- Both half-shaft couplings torqued and verified before applying load.
- Nothing loose near the hubs. At the 12,000 ERPM operating ceiling the half shafts turn about 330 RPM.

### 2.4 Electrical

- The controllers run a `no_hw_limits` firmware build. The firmware's built-in protection against out-of-range limit entries has been compiled out. Every current, voltage, and speed limit in Section 7 is the only line of defense. A typo in VESC Tool will be accepted and acted upon.
- The VESC USB port is not galvanically isolated. Its ground is bonded to DC bus negative. Plugging a mains-powered laptop into a controller bonds laptop earth to the bus and bypasses the CAN adapter's isolation. Run the laptop on battery, or use an isolated USB adapter, when connecting to a controller during a live run.
- The DC bus is supplied by a bidirectional laboratory supply that both sources and sinks. If the sink path is disabled or interrupted while the dyno motors are braking, regenerated energy has nowhere to go and bus voltage will rise.

## 3. System Overview

### 3.1 Control topology

The stand couples one electric axle to two independent absorption motors through the axle's own half shafts. The essential arrangement is one speed source working against two torque sinks:

- DUT: commanded to a target ERPM in closed-loop speed control. Its controller draws whatever current is needed to hold that speed as load is applied.
- Dyno motors: commanded a braking current. Each opposes rotation, absorbs mechanical power, and returns it to the DC bus as regeneration.

This split is forced by the mechanics. There is one mechanical degree of freedom (shaft speed) and two machines on it. Two speed loops would fight over the same shaft. Two torque commands would leave nothing regulating speed, and the system would accelerate or decelerate until torques happened to balance. One speed source and one torque sink gives a defined operating point that can be placed deliberately.

### 3.2 Hardware inventory

| Subsystem | Component | Notes |
|---|---|---|
| DUT (axle) | TDPRO TK204 / TD525 30" rear e-axle | 48 V 1000 W BLDC with integral open differential, 3-stud hubs, disc brake |
| Dyno motors (2) | MP 8055 50 Kv BLDC outrunner | 12N14P, 7 pole pairs, coupled 1:1 to the half shafts |
| Motor controllers (3) | Flipsky FSESC 75200 Pro V2 | `no_hw_limits` firmware, hardware target 75_300_R2 |
| Power supply | EA-PSB 10080-60 | Bidirectional, sources and sinks; 48 V bus setpoint |
| CAN adapter | DSD TECH SH-C31G | Isolated USB-CAN, factory candleLight (gs_usb), USB ID 1D50:606F |
| Host | Windows laptop | Python in a virtual environment |
| Frame | 80/20 T-slot, three tier | 51 x 27 in, casters, 3/8 in polycarbonate blast shield |
| Coupling | L090 spider coupler and custom hub adapter | 1045/4140 steel drum and cap, integral 10 mm stub, DIN 6885 keyway |

### 3.3 Signal and power paths

```
Host Laptop <--Ethernet--> Power Supply
     |                          |
    USB                    48VDC BUS
     |                          |
USB-CAN Adapter          Contactors <--Switched Current--> ESTOP
     |                          |
    CAN                    48VDC BUS
     |                          |
Motor Controllers <--48VDC BUS--+
     |
 48VDC BUS
     |
   Motors
```

## 4. Machine Data

These are the constants any control or analysis software needs. All values are as-detected on this specific hardware, not datasheet figures.

### 4.1 Motors

| Property | DUT | DMC-L (left dyno) | DMC-R (right dyno) |
|---|---|---|---|
| VESC node ID | 0 | 1 | 2 |
| Control mode | Speed (ERPM) | Brake current | Brake current |
| Pole pairs | 4 | 7 | 7 |
| Poles | 8 | 14 | 14 |
| Gear ratio to output shaft | 9.095 | 1:1 direct | 1:1 direct |
| Flux linkage lambda | 0.014404 Wb | 0.018148 Wb | 0.018099 Wb |
| Torque constant Kt | 0.0864 Nm/A | 0.1906 Nm/A | 0.1900 Nm/A |
| Winding resistance R | 9.8 mOhm | 202.0 mOhm | 211.4 mOhm |
| Inductance L | 28.8 uH | 355.4 uH | 341.1 uH |
| Lq - Ld | 14.58 uH | 164.63 uH | 160.41 uH |
| Direction inverted | No | No | Yes (mirrored mounting) |
| Motor temp sensor | NTC 10k 3590B (retrofit) | Fitted, NTC 10k | Fitted, NTC 10k |

### 4.2 Derived stand constants

| Quantity | Value | Basis |
|---|---|---|
| Chain constant (DUT pole pairs x gear ratio) | 36.38 | Measured from CAN, +/-0.04% |
| DUT ERPM / dyno ERPM | 5.1973 (SD 0.0021) | Measured across all load levels |
| Half-shaft RPM at 12,000 DUT ERPM | 330 RPM | ERPM / 36.38 |
| Dyno ERPM at 12,000 DUT ERPM | 2,309 ERPM | DUT ERPM / 5.1973 |
| No-load driveline drag | 198 W | Regression intercept, 24 load levels (0-12A, 12A-0) |
| Absorbed power per amp of brake | 12.6 W/A | Linear 1-12 A per dyno |
| Max axle torque at 20 A per dyno | 7.61 Nm | (Kt_L + Kt_R) x 20 A |
| Max absorbed power at 20 A, 330 RPM | 263 W | Torque x angular velocity |

### 4.3 Conversion equations

Speed:

```
motor RPM       = ERPM / pole_pairs
DUT shaft RPM   = ERPM / 36.38          (carrier / axle speed)
dyno shaft RPM  = ERPM / 7              (half-shaft speed)
```

Torque:

```
Kt        = 1.5 * pole_pairs * lambda
T_dyno    = |I_motor| * Kt              (Nm at the half shaft)
T_axle    = T_left + T_right
```

Power:

```
omega     = shaft_RPM * 2*pi / 60       (rad/s)
P_mech    = T * omega                   (W at the shaft)
P_elec    = V_in * I_in                 (W at the DC bus)
```

Tachometer, which the controllers report as commutation steps:

```
mechanical revolutions = tach / (6 * pole_pairs)
```

## 5. Theory of Operation

### 5.1 ERPM and pole pairs

VESC controllers command and report speed in electrical RPM. A permanent-magnet motor's magnets alternate north-south around the rotor, so poles come in pairs, and one mechanical revolution produces as many electrical cycles as there are pole pairs. A 7 pole-pair motor turning 330 RPM mechanically reports 2,310 ERPM.

This matters to software because every speed command and every speed reading on the CAN bus is electrical. Converting requires knowing the pole-pair count of the specific motor, and the two motor types on this stand differ.

### 5.2 The chain constant, and why the split is provisional

A finding from commissioning worth understanding before trusting any DUT-side number: every stand-level quantity depends only on the product of the DUT's pole pairs and its gear ratio, never on either alone.

```
shaft speed   = ERPM / (pp * N)
axle torque   = 1.5 * lambda * I * (pp * N)
```

The torque constant scales with pole pairs and the gear multiplication scales with ratio, so the two always appear together. From CAN telemetry that product was measured at 36.38, to within 0.04 percent invariant across every load level tested.

Separating the two cannot be done from CAN data. It requires the gearbox tooth count or an independent pole count on the DUT motor. The working split is 4 pole pairs with a 9.095 gear ratio, chosen because it places the ratio near the nominal 9.5; the alternative of 3 pole pairs would imply 12.13, which matches nothing.

> **What this means for software**
> Axle speed, axle torque, absorbed power, and efficiency depend only on the 36.38 product and are correct as computed.

### 5.3 The open differential

The DUT contains an open differential, and its behavior governs how the stand loads. It enforces two rules at once:

- Speed: the sum of the two half-shaft speeds is fixed by the input.
- Torque: it delivers equal torque to both output shafts. It physically cannot do otherwise.

```
omega_left + omega_right = 2 * omega_carrier
```

Three consequences, all observed on this stand:

1. Holding one half shaft does almost nothing. The differential sends all rotation to the free side, which spins at twice carrier speed, while torque stays shared. Same reason a car with one wheel on ice spins that wheel.
2. Loading both half shafts equally produces clean additive load, and the load reflects symmetrically back to the DUT.
3. Asymmetric braking causes speed splitting, not more load. Torque is equalized at the lesser-braked side, so the excess appears as a speed difference rather than added load.

> **Operating rule**
> Always command both dyno motors the same brake current. This is not a preference; it is what makes the load additive and predictable.
>
> A half-shaft speed spread appearing during an equal-command run is a symptom, not normal behavior. It means a coupling is slipping, a controller is not producing the current it was told to, or the node IDs are crossed. The software should trip on this; see Section 8.2.

### 5.4 Torque estimation from motor current

The stand has no inline torque transducers. Torque is estimated using the same field-oriented control relationship the motor controllers use internally:

```
T  = |I_motor| * Kt
Kt = 1.5 * pole_pairs * lambda
```

where I_motor is the reported phase current in amps, Kt is the torque constant in Nm/A, and lambda is the flux linkage in webers. The 1.5 factor comes from the amplitude-invariant three-phase-to-two-axis transform VESC uses; it is a constant of the convention, not a motor property, and is the same for every motor here.

| Motor | Pole pairs | lambda (Wb) | Kt (Nm/A) |
|---|---|---|---|
| DUT | 4 | 0.014404 | 0.0864 |
| DMC-L | 7 | 0.018148 | 0.1906 |
| DMC-R | 7 | 0.018099 | 0.1900 |

#### 5.4.1 Accuracy of this method

The torque equation is exact rather than empirical. For a sinusoidally wound permanent-magnet synchronous machine under field-oriented control, T = 1.5 x pp x lambda x Iq is the machine's governing relation. Both inputs are measured rather than assumed: pole pairs were counted directly on the dyno rotors (12 stator coils, 14 rotor magnets), and flux linkage comes from the controller's own back-EMF detection routine, which spins the motor and solves in volts and rad/s. As an independent check, the dyno flux linkage implies a Kv of about 52 against a 50 Kv nameplate.

Residual uncertainty comes from four places. Flux-linkage detection is a single-run measurement good to a few percent. Real windings carry some harmonic content, so instantaneous torque ripples around the average. The equation wants the quadrature current component, so any current sitting in the direct axis from commutation angle error produces heat rather than torque. Magnet strength falls with temperature and the dyno windings reach 66+ C in extended runs, so lambda at temperature sits slightly below the value detected cold. The last two bias the same direction, which is consistent with the stand's measured energy balance closing at about 104.5 percent incremental transfer, roughly 5 percent optimistic.

> **How to quote these numbers**
> Absolute torque should carry a +/-5 percent uncertainty band.
>
> Relative accuracy is considerably better. Repeat measurements at the same commanded brake current agreed within 1.5 percent, and absorbed power was linear across the full 1-12 A range.
>
> Efficiency comparisons, torque-versus-speed curves, and tare separation all depend on relative accuracy and are well supported. A certified absolute figure would require an inline rotating transducer.

### 5.5 Why the dyno motors are the binding constraint

The dyno motors sit downstream of the differential reduction, in the low-speed high-torque regime. For a given power they draw far more phase current than intuition suggests, so the dyno current limit rather than the DUT limit sets how much power the stand can process.

At the 20 A dyno ceiling, each motor produces about 3.8 Nm, giving 7.6 Nm at the axle. At the 330 RPM half-shaft speed corresponding to the 12,000 ERPM operating ceiling, that is roughly 263 W total absorbed. Raising speed does not help, because the stand is already at its speed ceiling; the only lever for more power is more dyno current.

### 5.6 Where the energy goes

At the 12 A hold point the DUT drew 332 W from the bus while the dynos returned 46 W, for a net 286 W from the supply. The bus therefore never sees net regeneration in steady-state operation; energy circulates between controllers and the supply tops up the difference.

The modest electrical return is explained by winding resistance. At about 205 mOhm, each dyno dissipates roughly 20 W of copper loss at 10 A and 30 W at 12 A. Of about 151 W absorbed mechanically at 12 A, roughly 88 W becomes heat in the windings before anything reaches the bus. This is expected for a 50 Kv high-turn wind and is why winding temperature, not the controllers, limits run duration.

Net regeneration onto the bus does occur during transients and if drive is removed while the dynos are still braking a spinning driveline. This is why the abort path always bleeds brake current off before removing drive.

## 6. Controller Configuration

All three controllers are configured in VESC Tool. Motor configuration and app configuration are separate writes; both must be written to flash. Exported XML for all three controllers is the recovery path if a controller is reset, and should be kept under version control.

### 6.1 CAN node map

| Node | Role | VESC ID | Command | TX EID | Status EID |
|---|---|---|---|---|---|
| DUT | E-axle, speed control | 0 | SET_RPM (3) | 0x300 | 0x900 |
| DMC-L | Dyno left, brake current | 1 | SET_CURRENT_BRAKE (2) | 0x201 | 0x901 |
| DMC-R | Dyno right, brake current | 2 | SET_CURRENT_BRAKE (2) | 0x202 | 0x902 |

### 6.2 Status broadcast

Each controller broadcasts telemetry unsolicited, with no polling required. Current configuration:

| Frame | Rate | Contents |
|---|---|---|
| STATUS_1 | 50 Hz | ERPM, motor current, duty cycle |
| STATUS_2 | Disabled | Amp-hours (not used by this stand) |
| STATUS_3 | Disabled | Watt-hours (not used by this stand) |
| STATUS_4 | 50 Hz | FET temp, motor temp, input current, PID position |
| STATUS_5 | 5 Hz | Tachometer, input voltage |
| STATUS_6 | Disabled | ADC inputs, PPM |

This gives about 105 frames per second per node, 315 total. STATUS_2 and STATUS_3 were disabled during commissioning because the host was not keeping up with receive at the original 205 Hz per node.

CAN baud rate is 500 kbit/s on all three controllers and must be matched in the Python bus initialization.

### 6.3 Detected motor parameters

These come from the controller's parameter detection routines and are recorded in the exported XML.

| Parameter | DUT | DMC-L | DMC-R |
|---|---|---|---|
| Motor type | FOC | FOC | FOC |
| Sensor mode | Hall sensors | Hall sensors | Hall sensors |
| Resistance | 9.8 mOhm | 202.0 mOhm | 211.4 mOhm |
| Inductance | 28.8 uH | 355.41 uH | 341.09 uH |
| Lq - Ld | 14.58 uH | 164.63 uH | 160.41 uH |
| Flux linkage | 0.014404 Wb | 0.018148 Wb | 0.018099 Wb |
| Current KP / KI | 0.0576 / 19.7 | 0.7108 / 404.08 | 0.6822 / 422.88 |
| Observer gain | 4.82 | 3.04 | 3.05 |
| Hall table | 255,133,0,165,72,93,31,255 | 255,119,55,87,188,156,19,255 | 255,58,190,24,124,90,157,255 |
| Sensored ERPM start | 0 | 2500 | 2500 |
| Sensorless ERPM | 4000 | 4000 | 4000 |
| Hall interpolation ERPM | 500 | 500 | 500 |
| Switching frequency | 30 kHz | 30 kHz | 30 kHz |

#### 6.3.1 The DUT sensor blend is deliberate

The DUT runs Sensored ERPM Start = 0, meaning the sensorless observer begins contributing to the rotor angle estimate from standstill and takes over fully at 4,000 ERPM. Why?

An experiment moved the blend window above the operating range (Sensored Start 12,600, Sensorless 14,000) to force pure hall commutation across 0-12,000 ERPM. Control became markedly worse: rough running, noisy current, and complete loss of commutation around 5,000 ERPM with current spikes to 34 A. The observer is doing real work above 4,000 ERPM on this motor. The configuration was reverted and this is the settled state.

### 6.4 Speed PID

All three controllers carry the firmware default speed PID gains (KP 0.004, KI 0.004, KD 0.0001, minimum ERPM 900, braking allowed).

The consequence is measurable droop: the DUT holds within 0.5 percent of setpoint unloaded but sags to 4 percent low at 12 A of brake per dyno, scaling linearly with load. This means the operating point moves as load changes, so a load sweep is not taken at constant speed. It is the most significant open item for anyone doing quantitative characterization; see Section 12.

The 900 ERPM minimum matters for the DUT only. The dyno motors run in brake-current mode with no speed loop, so the floor does not apply to them.

## 7. Limits and Operating Envelope

> **These limits are the only protection**
> The controllers run a `no_hw_limits` firmware build. Nothing in the firmware validates what is entered. These values are safety-critical configuration, not convenience settings.
>
> The firmware clamps incoming CAN commands against these limits silently. Commanding 60 A to a controller capped at 20 A produces 20 A with no fault and no warning. Log commanded current alongside measured current, or a saturated actuator will look like a compliant one.

### 7.1 As-configured controller limits

Read from the exported configuration XML.

| Limit | DUT | DMC-L / DMC-R | Note |
|---|---|---|---|
| Motor current max | 35 A | 20 A | Phase current |
| Motor current max brake | -35 A | -20 A | This is what does the work on the dynos |
| Absolute max current | 60 A | 40 A | Hard fault trip, not a clamp |
| Battery current max | 250 A (default) | 10 A | DUT value not yet set, see 7.4 |
| Battery current max regen | -200 A (default) | -10 A | DUT value not yet set, see 7.4 |
| Max ERPM | 12,500 | 100,000 (default) | Dyno value not yet set, see 7.4 |
| ERPM limit start | 80% | 80% | Current taper begins here |
| Max duty cycle | 95% | 95% | |
| Min input voltage | 12 V | 12 V | |
| Max input voltage | 60 V | 72 V | Inconsistent between nodes but not critical |
| Battery cutoff start / end | 42 / 40 V | 40.8 / 36 V | Inconsistent between nodes but not critical |
| FET temp cutoff start / end | 85 / 100 C | 85 / 100 C | |
| Motor temp cutoff start / end | 85 / 100 C | 85 / 100 C | |
| Timeout / timeout brake current | 1000 ms / 0 A | 1000 ms / 0 A | Coast on comms loss |

### 7.2 Operating policy limits

These are tighter than the controller limits and are enforced in software.

| Parameter | Limit | Rationale |
|---|---|---|
| DUT speed | 12,000 ERPM | About 330 RPM at the axle. Do not command higher. |
| Brake current per dyno | 20 A | Controller ceiling; also the stand's power ceiling based on winding heating. |
| Both dynos commanded equally | Always | Open differential; see 5.3 |
| Continuous run time at 10 A | 15 minutes | Dyno winding temperature; see Section 9 |

### 7.3 Software trip thresholds

Implemented in `dyno_test.py` bench testing script. Each fires an abort that commands zero torque to all three nodes.

| Trip | Threshold | Dwell | Rationale |
|---|---|---|---|
| CAN transmit failing | 25 consecutive send errors | - | Half a second at 50 Hz, inside the 1000 ms controller timeout |
| Dyno torque dropout | Measured < 30% of commanded | 0.3 s | Direct signature of a dyno losing output; fires before the shafts diverge |
| Half-shaft divergence | > 15% spread | 0.2 s | Healthy stand runs 0.68% mean, 3.17% peak |
| DUT overspeed | > 12,600 ERPM | Immediate | Just above the operating ceiling |
| Dyno overspeed | > 3,300 ERPM | Immediate | About 1.4x the normal operating point |
| DUT speed error | > 1,500 ERPM | 3.0 s | Normal droop is about 480 ERPM; this means load exceeded capability |
| FET temperature | > 80 C | Immediate | Any node |
| Motor temperature | > 100 C | Immediate | Nodes with a real sensor |
| Telemetry stale | > 0.5 s | Immediate | Any node |
| Data freshness gate | < 150 ms | - | Measurement trips are skipped on older data |

The freshness gate is important and non-obvious. If a node's most recent frame is older than 150 ms, its measurements are not evaluated for trips and the relevant dwell timers reset. Missing data must never look like a measurement of zero; that produced a false abort during commissioning. Between 150 and 500 ms the node is ignored; past 500 ms the staleness trip fires instead.

### 7.4 Limits that are not yet set

See the DUT battery current limits, dyno max ERPM, and related entries flagged in Section 7.1 and tracked in Section 13, Open Items.

## 8. CAN Interface

### 8.1 Frame format

VESC uses extended 29-bit CAN frames at 500 kbit/s. The identifier packs the command in the upper byte and the target node ID in the low byte:

```
EID = (command_id << 8) | vesc_id
```

CAN has no addressing in the network sense. Every frame reaches every node; each node compares the low byte of the identifier against its own configured ID and ignores anything else. Node ID 255 is a broadcast address that all nodes accept.

All multi-byte payload fields are big-endian (network byte order). This trips people up, since most automotive CAN is little-endian. In Python, struct format strings must use the ">" prefix.

A validated DBC file (`eaxle_dyno.dbc`) describes every command and status frame for all three nodes and can be loaded by cantools, Cangaroo, or any standard CAN tool.

### 8.2 Worked example

A speed command of 12,000 ERPM to the DUT:

```
Arbitration ID : 0x00000300   (29-bit extended)
DLC            : 4
Data           : 00 00 2E E0

  command_id = 3  (SET_RPM)  -> 0x03 in the upper byte
  vesc_id    = 0  (DUT)      -> 0x00 in the low byte
  payload    = 12000 as big-endian int32, no scaling
```

A brake command of 10.0 A to the left dyno:

```
Arbitration ID : 0x00000201
Data           : 00 00 27 10   (10000 = 10.0 A x 1000)
```

### 8.3 Command frames

| Command | ID | Payload | Scaling |
|---|---|---|---|
| SET_DUTY | 0 | int32 duty | x 100000 |
| SET_CURRENT | 1 | int32 current | x 1000 (mA) |
| SET_CURRENT_BRAKE | 2 | int32 brake current | x 1000 (mA), magnitude only |
| SET_RPM | 3 | int32 ERPM | direct, no scaling |
| SET_POS | 4 | int32 position | x 1000000 |
| SET_CURRENT_REL | 5 | int32 relative current | x 100000 |

SET_CURRENT_BRAKE always opposes rotation regardless of the sign of the value. This is a genuine structural safety property: a dyno commanded with brake current cannot drive the axle, only resist it.

### 8.4 Status frames

| Frame | ID | Fields, in order, big-endian |
|---|---|---|
| STATUS_1 | 9 | int32 ERPM, int16 motor current x10, int16 duty x1000 |
| STATUS_2 | 14 | int32 amp-hours x10000, int32 amp-hours charged x10000 |
| STATUS_3 | 15 | int32 watt-hours x10000, int32 watt-hours charged x10000 |
| STATUS_4 | 16 | int16 FET temp x10, int16 motor temp x10, int16 input current x10, int16 PID position x50 |
| STATUS_5 | 27 | int32 tachometer, int16 input voltage x10 |
| STATUS_6 | 28 | int16 ADC1 x1000, int16 ADC2 x1000, int16 ADC3 x1000, int16 PPM x1000 |

### 8.5 Measurement caveats

> **Input current is derived, not measured**
> The FSESC has no DC bus shunt. It computes input current from phase current and duty cycle, so the value is inherently noisy, worst at low duty, which is exactly where the dyno motors operate.
>
> Filter it before display or use. The scripts apply an exponential moving average with a 0.4 s time constant.
>
> Treat the power supply's own reading as the calibrated reference for power accounting. The controller values are for control and protection.

- Bus voltage differs by up to 0.5 V between controllers. This is ADC tolerance, not a real gradient.
- The tachometer counts commutation steps, six per electrical revolution. Divide by (6 x pole_pairs) for mechanical revolutions. It is cumulative and does not reset on power cycle, but does reset when a configuration is written.
- A motor temperature reading of exactly 0.0 or a large negative value means no sensor is fitted or the sensor is disabled, not a real temperature.

### 8.6 Adapter setup on Windows

The DSD TECH SH-C31G presents as a candleLight (gs_usb) device, USB ID 1D50:606F. No firmware flash is required. The working setup path:

1. Set the boot switch to work mode. Per the vendor, ON = DFU mode, OFF = work mode.
2. Install WinUSB via Zadig on the "canable2 gs_usb" identity (1D50:606F). Also install it on the bootloader identity "DFU in FS Mode" (0483:DF11) if firmware updates will ever be needed; the Chrome web updater cannot see the device otherwise.
3. Place libusb-1.0.dll into `C:\Windows\System32`, taken from libusb-1.0.30.7z, the VS2022\MS64\dll build. This resolves pyusb's "No backend available" error. The libusb-package pip module alone does not work.
4. Run the Python tooling with `--interface gs_usb`.

> **Startup order matters**
> On a fresh boot with both the VESC USB and the CAN adapter connected, the adapter sometimes fails to enumerate. Device Manager shows "Unknown USB Device (Port Reset Failed)" and python-can reports "Devices found: 0". Disregard if not using the VESC USB connection.

### 8.7 Bus termination

CAN requires exactly two 120 ohm terminators, one at each physical end, giving 60 ohms measured across CANH and CANL with power off.

This stand does not meet that. The FSESC controllers carry internal termination that cannot be disabled from VESC Tool, so with three controllers on the bus the measured resistance is about 36 ohms. With the adapter's terminator also enabled it was 28 ohms, which caused intermittent transmit failures under load; see Section 11.1.

The adapter's 120 ohm switch must remain off. The bus runs acceptably at 36 ohms with that mitigation, verified by a twelve-minute soak with zero transmit errors. A full fix would require physically removing a termination resistor from one of the controller boards.

## 9. Thermal Behavior and Run Time

Dyno winding temperature is the limiting factor for run duration. The controllers stay cool; the motors do not.

### 9.1 Measured data

| Run | Duration | Load | Dyno winding | Dyno FET | DUT FET |
|---|---|---|---|---|---|
| Load sweep | 8 min | 0-12 A sweep | 32 to 40 C | 32-34 C | 33 C |
| Soak test | 12 min | 10 A continuous | 32 to 66 C | 32-34 C | 34 C |

In the soak run the windings rose about 34 C in 12 minutes, roughly 2.8 C per minute, while the controller FETs moved less than 3 C total. Copper loss is the source: at about 205 mOhm winding resistance, each dyno dissipates 20.5 W at 10 A.

### 9.2 Recommended run times

Heating scales with the square of current, since copper loss is I squared times R. The table below extrapolates linearly from the single soak data point, which is conservative because real thermal rise is first-order and flattens over time.

| Brake current per dyno | Copper loss per dyno | Approx rise rate | Recommended max continuous | Cooldown |
|---|---|---|---|---|
| 5 A | 5 W | 0.7 C/min | 30 min or more | Not usually needed |
| 10 A | 20 W | 2.8 C/min | 15 min | 10 min |
| 15 A | 46 W | 6.3 C/min | 7 min | 15 min |
| 20 A | 82 W | 11 C/min | 4 min | 20 min |

**How to use the table above.** These are estimates from one measured run, not a validated thermal model. Watch the actual reading. Stop and cool down at 85 C winding. The controller begins current derating there, so data taken above it is not at the commanded load. The 100 C trip is a hardware protection limit, not an operating target.

The 10 A row is the only one backed by real data. The others are extrapolated and should be treated cautiously until confirmed. A useful improvement would be a long soak logging winding temperature to the plateau, which would give a proper thermal time constant and turn these estimates into a model.

### 9.3 DUT thermal

The DUT now carries a retrofit NTC 10k 3590B thermistor, wired between the controller's temperature input and hall harness ground. The controller supplies its own pull-up.

No long-run DUT winding data exists yet. Before extended characterization campaigns, repeat the 10 A soak and record DUT motor temperature alongside the dynos to establish which motor is actually the limiting node.

## 10. Software

### 10.1 Tooling

| Script | Purpose | Commands motion |
|---|---|---|
| `comms_check.py` | Passive telemetry monitor. Verifies all nodes present, frame rates correct, values sane. | Spacebar stop |
| `dut_step_test.py` | Automated stepped ERPM sequence for the DUT alone, uncoupled. | Yes |
| `dyno_test.py` | Loaded test. Ramps the DUT to speed, settles, then hands the operator manual brake trim. | Yes, with load |

Dependencies: python-can, rich, gs_usb, pyusb, cantools. Keep a `requirements.txt` under version control; the exact python-can version matters because the gs_usb backend argument shape has changed across releases.

### 10.2 Architecture

Two design decisions are load-bearing and should not be undone without understanding why they exist.

#### 10.2.1 Receive runs on its own thread

CAN receive uses `can.Notifier` with a listener that does nothing but decode frames into per-node state. No rendering, no logging, no control decisions. The main loop commands at a fixed rate, renders, and logs.

When receive shared the main loop, anything that stalled the process, like a console redraw or a Windows screenshot overlay, starved the receive buffer, and the script then made decisions on stale telemetry. This produced a false abort during commissioning and probably contributed to the earlier dropout events.

No locks are used. Each field is written by exactly one thread and read by the other, and attribute assignment is atomic under the GIL. Frame rates are sampled from counters rather than timestamp queues, because iterating a deque while another thread appends to it can raise.

#### 10.2.2 The transmit tick is the watchdog feed

Each controller releases its output if it receives no command for 1000 ms. The fixed-rate transmit loop is therefore not only how setpoints are delivered, it is what keeps the controllers alive. Anything that blocks that loop for a second will stop the stand.

Transmit rate is configurable with `--tx-hz`. The default is 50 Hz; 20 Hz is ample against a 1000 ms timeout and puts a third of the traffic on the bus.

### 10.3 Sequence state machine

`dyno_test.py` progresses through: IDLE, RAMP UP, SETTLE, MANUAL, RAMP DOWN, DONE, with ABORTED reachable from any active state.

- RAMP UP applies a smooth software ramp to the target ERPM, default 1,000 ERPM/s. This keeps acceleration current well inside the DUT limit; measured peak during a 12 s ramp was 11.5 A against a 35 A limit.
- SETTLE holds at speed with no load for a configurable period, default 10 s.
- MANUAL is the only state in which load can be applied. Arrow keys trim brake current in 1 A increments, applied equally to both dynos.
- RAMP DOWN bleeds brake current to zero first, then ramps speed down, then releases. Never the other way round; releasing drive while the dynos are still braking dumps the driveline and pushes regeneration onto an unloaded bus.

### 10.4 Abort behavior

An abort commands zero to every node, both SET_CURRENT and SET_CURRENT_BRAKE at zero, so a node releases regardless of which mode it was last commanded in. This is a release, not a hold: the driveline coasts down under its own friction. Commanding zero RPM instead would make the DUT actively brake to a standstill, which is not what is wanted in an abort.

A trip transmits immediately rather than waiting for the next scheduled tick. The exit path sends zero five times regardless of how the program terminated: operator quit, exception, or Ctrl-C.

### 10.5 Logging

Every run writes a timestamped CSV to `./logs` at 50 Hz, including aborted runs. Aborted runs are usually the informative ones.

Logged values are raw and unfiltered. Filtering is applied only to the live display. This is deliberate: a filter can always be applied to logged data afterward, but information removed by filtering before logging cannot be recovered.

Columns cover, per node: ERPM, shaft RPM, duty, motor current, input current, input voltage, FET and motor temperature, computed torque, mechanical power, receive rate, and telemetry age. Plus run-level columns: state, setpoints, absorbed power, DUT input power, efficiency, half-shaft spread, chain ratio, and bus health counters.

### 10.6 Bus health instrumentation

The Bus Health panel exists to separate host-side from controller-side failures, a distinction that cost significant time during commissioning:

| Symptom | Interpretation |
|---|---|
| TX failures or error frames climbing | Physical layer or adapter. Commands are not landing. |
| Adapter CAN state PASSIVE or BUS OFF | Adapter is accumulating transmit errors. Same conclusion. |
| RX rate collapses on one node | That controller stopped talking. |
| RX rates low on all nodes | The host is not keeping up. |
| RX and TX clean, but a dyno shows DROPOUT | Genuinely controller-side. |

## 11. Standard Operating Procedure

### 11.1 Pre-run checklist

1. Blast shield in place, casters locked, nothing loose near the hubs.
2. Both half-shaft couplings are secure. Both hubs turn freely by hand.
3. E-stop within reach of the operating position, and verified to drop the bus.
4. Power the EA-PSB. Confirm 48 V setpoint, source limit, sink limit enabled, and OVP set so the software interlock trips first.
5. Enumerate the CAN adapter, alone first if this is a fresh boot, then connect the VESC USB only if needed.
6. Run `comms_check.py`. Confirm all three nodes report, frame rates are about 105 Hz each, and bus voltage reads about 47 V.
7. Confirm current limits match Section 7.1: 35 A DUT, 20 A per dyno.

### 11.2 Running a loaded test

1. Run the unloaded ramp first (`dut_step_test.py`) if the driveline has been disturbed since the last run.
2. Launch `dyno_test.py`. Consider `--dry-run` first to confirm key handling and sequencing without transmitting.
3. Press UP to arm and start the ramp. Do not exceed 12,000 ERPM.
4. Wait for MANUAL. Load cannot be applied before this state.
5. Trim brake current up with RIGHT in 1 A increments. Watch dyno winding temperature and half-shaft spread as you go.
6. Hold at the desired load. Observe the run-time guidance in Section 9.2.
7. Press E to end. Brake ramps off first, then speed, then release. Do not simply quit.

### 11.3 Keyboard controls

| Key | Action | Available in |
|---|---|---|
| UP | Arm and start the ramp | IDLE |
| RIGHT | Brake +1 A on both dynos | MANUAL |
| LEFT | Brake -1 A on both dynos | MANUAL |
| DOWN | Drop load to zero, stay at speed | MANUAL |
| E | End run gracefully | Any active state |
| SPACE | Abort, zero torque to all nodes | Always |
| R | Reset back to idle | ABORTED or DONE |
| Q | Quit, zeroing all nodes on the way out | Always |

### 11.4 Stop conditions

| Observation | Action |
|---|---|
| Half-shaft spread rising above a few percent | Drop load. A coupling is slipping or a dyno is not producing commanded current. |
| Dyno winding temperature above 85 C | Drop load and cool down. Controller derating starts here. |
| DUT speed error growing | Reduce load. The DUT is running out of authority. |
| Bus voltage climbing | Abort. Check the supply sink limit. |
| Any node stale or missing | Abort. Do not continue on partial telemetry. |
| Unexpected noise, vibration, or smell | E-stop. |

## 12. Fault History and Troubleshooting

These are real faults encountered on this hardware. The diagnostic reasoning is included because the symptoms were misleading in each case.

### 12.1 CAN over-termination

Symptom: under load, controllers intermittently timed out waiting for commands. First one dyno would drop out, then all three within about 0.7 seconds, then recover. Telemetry receive continued perfectly throughout and no controller fault latched.

The combination of clean receive, no faults, and all three nodes releasing pointed at intermittent transmit failure rather than any controller problem. Controllers that time out do not log a fault; they simply release.

Cause: the bus was massively over-terminated. With the adapter's 120 ohm switch on, CANH to CANL measured 28 ohms, four terminators in parallel against a 60 ohm target. Below about 45 ohms the transceiver cannot pull the dominant state to full differential voltage, noise margin collapses, and bit errors appear. The load dependence follows: more phase current means more electrical noise.

Fix: disable the adapter's terminator, bringing the bus to 36 ohms. Verified by a twelve-minute soak with zero transmit failures and zero error frames.

### 12.2 Host not keeping up with receive

Symptom: after the termination fix, transmit was perfect but receive rates had collapsed to roughly a third of expected, and a false dropout abort fired immediately after a Windows screenshot shortcut was pressed.

Cause: receive shared the main loop. Anything that stalled the process starved the USB receive buffer, and the script then evaluated trips against stale telemetry that still held the last values received.

Fix: move receive to a background thread via `can.Notifier`, gate all measurement trips on data freshness, and disable the unused STATUS_2 and STATUS_3 frames to halve receive load.

### 12.3 Detection and commutation faults

| Symptom | Cause | Fix |
|---|---|---|
| ABS_OVER_CURRENT stalls | False flux linkage (0.040 mWb) from detection at too low a current (8.33 A); the driveline would not turn | Re-detect at 10 A. Breakaway sits between 8.33 and 10 A. |
| Loss of commutation above 4,000 ERPM | Bad hall table entry, index 2 reading 199 instead of about 0 | Re-run hall detection |
| Rough running, loss above 5,000 ERPM | Sensor blend window moved above the operating range, removing the observer | Revert to Sensored Start 0, Sensorless 4,000 |
| "Parameters truncated" dialog on limit write | Firmware enforces Absolute Max >= 1.5 x Motor Current Max | Set Absolute Max first, then the motor current limits |
| Motor temperature reads -99 C | Thermistor wired to the 5 V rail; the controller supplies its own pull-up | Wire the thermistor between the temperature input and ground only |

### 12.4 USB and VESC Tool

- "Could not read firmware version" after changing a CAN ID: CAN forwarding is still pointed at the old ID. Disconnect, clear forwarding, select the local device, re-select the COM port, reconnect. Changing a VESC ID cannot brick a board; the ID lives in app config, independent of USB enumeration.
- Adapter fails to enumerate on a fresh boot: unplug the VESC, enumerate the adapter alone, then reconnect the VESC.
- python-can reports "No backend available": libusb-1.0.dll is missing from System32.
- python-can reports "Devices found: 0": the adapter is not enumerated. Check Device Manager before anything else.

### 12.5 Diagnostic principle

Two general lessons from commissioning, both of which cost time:

- When several things change in one session, a symptom that appears afterward is not necessarily caused by the most recent change. Export a working configuration before each round of edits so reverting is possible.
- Distinguish "the actuator failed" from "the measurement of the actuator failed" before acting. Several apparent hardware faults on this stand were data-starvation artifacts.

## 13. Open Items

| Item | Impact | Priority |
|---|---|---|
| Speed PID never tuned. DUT droops 4% at full load, so load sweeps are not at constant speed. | Blocks quantitative characterization | High |
| DUT battery current limits at firmware defaults (250 A / -200 A) | No protection on the DUT bus current path | High |
| Dyno Max ERPM at firmware default (100,000) | No overspeed protection at the controller level | High |
| DUT ERPM limit start at 80%, so tapering begins inside the operating range | Top of the speed range is soft-limited | Medium |
| 5% systematic error between DUT-derived and dyno-measured power | Absolute accuracy band | Medium |
| DUT pole pair / gear ratio split unresolved (only the 36.38 product is known) | Motor-shaft figures carry an unknown factor | Medium |
| DUT motor temp sensor type and beta need verification against the fitted 3590B | Reading accuracy at high temperature | Medium |
| Bus termination at 36 ohms rather than 60 | Reduced noise margin; mitigated but not fixed | Low |
| Voltage limits inconsistent between DUT and dyno controllers | Cosmetic unless a fault condition is hit | Low |
| No long-duration thermal data; run times in Section 9.2 are extrapolated | Conservative but imprecise duty cycle guidance | Low |
| Operator GUI not started | Planned follow-on deliverable | Scheduled |

## 14. Recovery and Backup

The project folder was lost once during development. The following are the recovery-critical artifacts and should be under version control:

- Exported VESC configuration XML for all three controllers, motor and app configuration both. These contain the detection results and hall tables, which took significant effort to obtain and cannot be reconstructed without re-running detection.
- The three Python scripts and `requirements.txt`.
- `eaxle_dyno.dbc`.
- This document.

Logs need not be committed but should be retained outside the working folder. Add `./logs` to `.gitignore`.

If a controller must be recovered from factory state: write the exported motor configuration, write the exported app configuration, power cycle, then verify the VESC ID and hall table read back correctly before putting it on the bus.

## 15. Failure Mode Analysis

This section works from first principles: what energy the stand stores, what can release it in an uncontrolled way, and what stops it. It then walks each component and asks what happens when it fails, both alone and in combination.

### 15.1 First principles

The stand stores energy in exactly two places.

| Store | Magnitude | Release path if uncontrolled |
|---|---|---|
| Rotating inertia | Driveline plus two dyno rotors at up to 330 RPM at the half shafts | Continues spinning until friction absorbs it. Coast-down is seconds, not instant. |
| DC bus electrical | 48 V across the bus capacitance, plus whatever the supply will source or sink | Bus voltage rise if regeneration has nowhere to go; current into a fault if a short occurs. |

Every hazard on this stand is one of those two escaping control. That gives four questions worth asking of any failure:

1. Can it cause uncommanded torque or speed?
2. Can it cause energy to be dumped somewhere it should not go?
3. Can it cause the system to keep running when it should stop?
4. Can it cause the operator to believe something false about the system state?

The fourth is the one most often overlooked, and it is where several real events on this stand landed. A measurement that fails silently is more dangerous than an actuator that fails loudly, because the operator and the software both keep acting on a picture that is no longer true.

### 15.2 Protection layers

Four layers, in order of authority. Each is independent of the ones above it.

| Layer | Mechanism | Depends on | Response time |
|---|---|---|---|
| 1. Physical E-stop | Opens the contactor, removing DC bus power from all three controllers | Nothing but the operator and the contactor | Immediate; coast-down follows |
| 2. Controller limits | Firmware clamps current, speed, voltage and temperature per node | Controller powered and running its own firmware | Sub-millisecond, per node |
| 3. Command timeout | Each controller releases output after 1000 ms with no command | Controller powered; independent of the host | 1 second |
| 4. Software trips | Host script detects an out-of-bounds condition and commands zero | Host running, adapter working, CAN intact | 0.2 to 3 s depending on trip |

**Layer authority runs opposite to layer speed.** The software trips are the fastest to notice a problem but the least reliable, because they depend on the most components. The E-stop is the slowest to act as it needs a human, but depends on the fewest, which is why it is the last resort and the only true safety device.

Do not add software features that make the E-stop feel unnecessary. Its value comes from being independent of everything else.

### 15.3 Single-point failures

Each row assumes everything else is working.

#### 15.3.1 Communications and host

| Failure | Immediate effect | What stops it | Residual risk |
|---|---|---|---|
| Host script crashes or is killed | Commands stop. All three controllers time out after 1000 ms and release. Driveline coasts. | Layer 3 (command timeout) | Up to 1 s of continued operation on the last commanded setpoint |
| USB-CAN adapter unplugged or fails | Same as above; commands stop reaching all nodes. | Layer 3 | Same |
| CAN bus wiring open or shorted | Commands stop reaching some or all nodes. Affected nodes time out and release. | Layer 3 | If only some nodes lose comms, the stand runs asymmetrically for up to 1 s; see 15.4.1 |
| CAN bus degraded but not dead (noise, bad termination) | Intermittent command loss. Nodes may release and recover repeatedly. | Layer 3 releases; Layer 4 TX-failure trip aborts the run | This is the observed over-termination fault. It looks like a mechanical problem and is not. |
| Host stalls without crashing (blocked thread, OS interruption) | Commands stop for the stall duration; telemetry may go stale | Layer 3 if the stall exceeds 1 s; Layer 4 freshness gate prevents false trips | A stall shorter than 1 s leaves the stand running on a stale setpoint |
| Telemetry stops but commands continue | The stand keeps running while the operator sees frozen values | Layer 4 staleness trip at 0.5 s | Most dangerous comms failure; see 15.6 |

#### 15.3.2 Controllers and motors

| Failure | Immediate effect | What stops it | Residual risk |
|---|---|---|---|
| One dyno controller stops producing torque | That half shaft accelerates, the other decelerates. Sum of speeds is held by the differential, so total absorbed power collapses. | Layer 4 dropout trip (0.3 s) or divergence trip (0.2 s) | The freed side can reach roughly twice carrier speed before the trip fires |
| DUT controller faults or stops driving | Driveline decelerates under whatever brake current is still applied. Rapid stop rather than coast. | Layer 4 speed-error trip | Sudden deceleration loads the couplings; a still-braking dyno pushes regeneration onto an unloaded bus |
| A controller overheats | Firmware derates current from 85 C, cuts out at 100 C | Layer 2 | Derating is silent; the stand appears to be at commanded load and is not |
| A motor winding overheats | Same derating behavior via the motor temperature input | Layer 2, plus Layer 4 trip at 100 C | The DUT sensor is a retrofit; verify it before relying on it |
| Hall sensor fails or a connector backs out | Loss of commutation. Large current excursions, loss of torque, possible ABS_OVER_CURRENT | Layer 2 absolute current trip | Observed during commissioning. Load-dependent and progressive; it gets worse before it fails outright |
| A motor phase lead opens | Immediate loss of torque on that motor | Layer 4 dropout or divergence trip | Same asymmetry consequence as a dyno controller failure |

#### 15.3.3 Power

| Failure | Immediate effect | What stops it | Residual risk |
|---|---|---|---|
| Supply output drops or trips | All controllers lose bus voltage. Everything coasts. | Inherent | Equivalent to an E-stop; no additional hazard |
| Supply sink path disabled or interrupted while dynos brake | Regenerated energy has nowhere to go. Bus voltage rises. | Layer 2 over-voltage limits per controller | On a `no_hw_limits` build the configured max input voltage is the only guard. Verify it is set on every node. |
| E-stop contactor fails to open | Bus stays live. Operator has pressed the last-resort control and nothing happened. | Nothing above it | Test the E-stop before every session. This is the one failure with no backstop. |
| E-stop opens spuriously | Bus drops mid-run. Driveline coasts from speed with load applied. | Inherent | Loss of data, mechanical shock to couplings. Not a safety hazard. |

#### 15.3.4 Mechanical

| Failure | Immediate effect | What stops it | Residual risk |
|---|---|---|---|
| Coupling grub screw loosens | Progressive slip between shaft and hub. Torque reading falls below commanded; speed diverges. | Layer 4 dropout or divergence trip | Presents identically to an electrical dropout. Check mechanical first if a trip repeats on one side. |
| Coupling spider fails | Complete loss of connection on that side. That half shaft free-spins. | Layer 4 divergence trip | Debris inside the enclosure. Blast shield is the containment. |
| Motor mount fastener loosens | Growing vibration, misalignment, accelerated coupling wear | Operator observation only | No automatic detection. This is why the vibration check in Section 16 matters. |
| Bearing failure in a dyno motor | Rising drag, rising temperature, audible change | Layer 2 thermal, eventually | Slow onset. Detected by trend, not by trip. |
| Differential internal failure | Loss of drive to one or both half shafts, or seizure | Layer 4 divergence or speed-error trip | A seizure decelerates the driveline abruptly |

### 15.4 Combination failures

Single failures are largely covered. The cases worth attention are those where one failure disables the protection for another.

#### 15.4.1 Partial CAN loss

If communication is lost to one dyno but not the other, that node releases after 1000 ms while the other keeps braking. The differential then splits speed hard: the released side accelerates toward twice carrier speed while the braked side slows. This is the same signature as a dyno controller failure and is caught by the same trips, provided the host still has telemetry from both nodes.

If comms are lost to a node entirely, telemetry stops too, and the staleness trip fires at 0.5 s. That is faster than the 1000 ms command timeout, so in practice the host aborts before the asymmetry fully develops.

#### 15.4.2 Telemetry loss with commands intact

The dangerous asymmetry is the reverse: transmit works, receive does not. The stand continues executing the last commanded setpoint while the host sees frozen or stale values. Every measurement-based trip is blind.

This is not hypothetical; it occurred during commissioning when receive shared the main loop and a process stall starved the receive buffer. Two mitigations are now in place: receive runs on its own thread so a host stall cannot starve it, and measurement trips are gated on data freshness so stale values are never treated as measurements. The staleness trip fires at 0.5 s regardless.

#### 15.4.3 Silent limit saturation

The controllers clamp commands against their configured limits with no fault and no warning. If a limit is set wrongly, or left at a firmware default as several currently are, the stand will appear to obey a command it is not actually executing.

The defense is to log commanded values alongside measured values and compare. A saturated actuator and a compliant one look identical in the command stream alone.

#### 15.4.4 Thermal derating during a sweep

Above 85 C the controllers reduce current automatically. A load sweep that runs long enough to reach that temperature produces data at less than the commanded load, with nothing in the command stream indicating it. The result looks like a real efficiency change and is not.

The defense is the run-time guidance in Section 9.2 and watching winding temperature during the run, not only at the end.

#### 15.4.5 Two failures that share a cause

Both dyno controllers sharing a bus, a supply, and a CAN segment means a single upstream fault can take both out at once. This is benign for the dynos specifically; losing both load paths simply unloads the DUT, which then overspeeds slightly and is caught by the speed trip. It is worth noting only because the redundancy implied by having two absorption motors is not real redundancy.

### 15.5 Abort criteria

These are the conditions under which the host software automatically commands zero torque to all three nodes. They are implemented in `dyno_test.py` and repeated here from Section 7.3 for reference in a failure context.

| Condition | Threshold | Dwell | Failure it detects |
|---|---|---|---|
| CAN transmit failing | 25 consecutive send errors | - | Adapter, bus wiring, termination, bus-off state |
| Dyno torque dropout | Measured current below 30% of commanded | 0.3 s | Dyno controller fault, phase lead open, coupling slip, hall failure |
| Half-shaft divergence | Speed spread above 15% | 0.2 s | Same set, detected by mechanical consequence rather than cause |
| DUT overspeed | Above 12,600 ERPM | Immediate | Load released unexpectedly, speed loop instability, wrong setpoint |
| Dyno overspeed | Above 3,300 ERPM | Immediate | Differential dumping speed to one side |
| DUT speed error | Above 1,500 ERPM from setpoint | 3.0 s | Load exceeds DUT capability, DUT fault, mechanical binding |
| FET temperature | Above 80 C on any node | Immediate | Controller thermal limit approaching |
| Motor temperature | Above 100 C on any node with a sensor | Immediate | Winding thermal limit |
| Telemetry stale | No frame for more than 0.5 s from any node | Immediate | Node powered off, comms loss, host starvation |
| Operator abort | Spacebar | Immediate | Anything the operator sees that the software does not |

Two behaviors worth understanding:

- Measurement-based trips are gated on data freshness. If a node's most recent frame is older than 150 ms its values are not evaluated and the relevant dwell timers reset. Missing data must never be treated as a measurement of zero. Between 150 and 500 ms the node is ignored; past 500 ms the staleness trip fires instead.
- An abort commands zero to every node using both SET_CURRENT and SET_CURRENT_BRAKE, so a node releases regardless of which mode it was last in. This is a release, not a hold. The driveline coasts. Commanding zero RPM instead would make the DUT actively brake to a standstill, which is not wanted in an abort.

### 15.6 Loss of CAN communication

This case deserves separate treatment because it is the most likely fault and because the behavior is not obvious.

Each controller independently releases its output if no command arrives for 1000 ms. Timeout Brake Current is set to 0 A on all three, so the release is a coast rather than a brake. There is no coordination between controllers; each simply stops.

| Scenario | Behavior |
|---|---|
| Host or adapter fails cleanly | All three nodes stop receiving simultaneously and release within 1000 ms of each other. Driveline coasts. Telemetry also stops, so the host aborts at 0.5 s if it is still running. |
| Bus degrades intermittently | Nodes release and recover independently as commands come and go. The staggered dropout order observed during commissioning is the signature. The TX-failure trip is the intended catch. |
| One node loses comms | That node releases; the others keep running. On a dyno this creates immediate speed splitting. Staleness trip fires at 0.5 s, before the 1000 ms timeout. |
| Host stalls under 1 second | No node times out. The stand continues on the last setpoint. Telemetry backlogs and then catches up. |
| Host stalls over 1 second | Equivalent to a clean comms failure. |

**Why 1000 ms and not shorter.** A tighter timeout is better for script-driven operation. The stand runs at 1000 ms because VESC Tool sends one-shot commands that drop out at 250 ms, and VESC Tool is the preferred interface for bench debugging.

If the stand moves to script-only operation, reducing this to 200-300 ms would tighten the worst-case uncommanded-operation window by a factor of three or more. Change it on all three nodes together.

### 15.7 What has no automatic detection

Honest accounting of the gaps. None of these are covered by any layer.

- Loosening fasteners, before they produce a failure. Detected only by the operator noticing vibration or by scheduled inspection.
- Bearing degradation. Slow trend in drag and temperature, no discrete trip.
- Coupling wears short of slip.
- Blast shield removed or improperly secured.
- Wrong constants in software. A mistyped pole-pair count or gear ratio produces plausible but wrong torque and power numbers with no symptom. The chain-ratio check in the live display exists for this and it should read 5.197 and a drift indicates a constant is wrong.
- Thermistor failure reading plausibly. A disconnected sensor reads 0 or a large negative value and is caught, but a partially failed one can read a believable wrong temperature.
- E-stop contactor failing to open. There is no backstop above the last resort.

## 16. Maintenance

The stand has few consumable parts, but it is a rotating machine under repeated thermal and torsional cycling. Most of what follows is inspection rather than replacement.

### 16.1 Before every run

1. Blast shield in place and secured. Casters locked.
2. Turn both hubs by hand. They should turn freely with consistent drag and no notchiness or grinding.
3. Check both couplings by hand for rotational play between shaft and hub. This should be minimal (the E-Axle does have some play and doesn't have perfectly straight/balanced shafts.).
4. Visually check that no fastener has backed out and nothing is loose inside the enclosure.
5. Confirm the E-stop drops the bus.
6. Run `comms_check.py` and confirm all three nodes report at the expected rate with sane values.

### 16.2 During every run: what to listen and watch for

The stand has a normal sound and a normal set of readings. Deviation from either is the earliest warning available, and it is entirely operator-dependent.

| Observation | Likely meaning | Action |
|---|---|---|
| New vibration at a specific speed | Coupling misalignment, loosening mount, imbalance developing | Stop and inspect. Do not run through a resonance to see if it clears. |
| Vibration that grows with load rather than speed | Torsional slip or a fastener loosening under torque rather than centrifugal load | Stop. Check coupling grub screws and mount bolts. |
| New whine, growl, or rumble | Bearing degradation in a motor or the differential | Stop and identify which component. Compare left and right by hand after cool-down. |
| Clicking or knocking | Something loose, or a coupling spider failing | Stop immediately. |
| Half-shaft speed spread rising above a few percent | Coupling slip or a dyno not producing commanded current | Drop load. Investigate before continuing. |
| Drag or no-load power rising over time compared with earlier runs | Bearing wear, seal drag, or coupling misalignment | Compare against the 198 W no-load baseline. Investigate a sustained rise. |
| Winding temperature rising faster than the Section 9.2 estimate | Increased losses, or reduced cooling | Reduce load. Check for obstruction around the motors. |
| Burning or hot-insulation smell | Winding or connector overheating | E-stop. |

### 16.3 Fastener torque

All fasteners are torqued to standard values for their size and grade. Use these unless a component datasheet specifies otherwise, in which case the datasheet governs.

| Size | Class 8.8 steel into steel | Into aluminum / T-slot | Typical use on this stand |
|---|---|---|---|
| M3 | 1.3 Nm | 1.0 Nm | Small brackets, cable clamps |
| M4 | 3.0 Nm | 2.2 Nm | Dyno motor face mounting |
| M5 | 6.0 Nm | 4.5 Nm | Motor mounts, panel hardware |
| M6 | 10 Nm | 7.5 Nm | Frame brackets, mount plates |
| M8 | 25 Nm | 18 Nm | Frame structural, DUT mounting |
| M10 | 49 Nm | 35 Nm | Heavy structural connections |
| 1/4-20 | 11 Nm (8 ft-lb) | 8 Nm (6 ft-lb) | 80/20 T-slot fasteners |
| 5/16-18 | 23 Nm (17 ft-lb) | 16 Nm (12 ft-lb) | 80/20 structural |

### 16.4 Thread locking

| Location | Compound | Rationale |
|---|---|---|
| Coupling shaft grub screws | Blue (medium strength, removable) | Sees full torsional load and vibration. The most likely fastener on the stand to back out, and its failure mimics an electrical fault. |
| Dyno motor mounting fasteners | Blue | Continuous vibration, thermal cycling to 66 C or more |

Apply blue thread locker to clean, dry threads. Allow the specified cure time before applying load, typically several hours for full strength. Do not use red (permanent) anywhere on this stand; every joint here needs to come apart eventually.

### 16.5 Scheduled maintenance

| Interval | Task |
|---|---|
| Every run | Pre-run checklist, Section 16.1 |
| Every 10 run hours | Check coupling grub screw torque. Check motor mount fastener torque. Inspect coupling spiders for cracking or wear. |
| Every 25 run hours | Full fastener torque check across the frame and all mounts. Inspect all wiring for chafing, particularly where cable crosses moving or hot parts. Check CAN and phase lead routing has not shifted. |
| Every 50 run hours | Compare a no-load drag sweep against the 198 W baseline. A sustained rise indicates bearing or seal degradation. Inspect differential for leakage. |
| Annually or after any incident | Verify E-stop contactor operation under load. Re-verify controller configuration against the exported XML. Re-export if anything has changed. |
| After any disassembly | Re-torque to Section 16.3, re-apply blue thread locker per Section 16.4, then run an unloaded ramp before applying load. |

### 16.6 After a fault or abort

An abort is not by itself a reason to inspect the hardware, but some are.

- After a divergence or dropout trip: check the coupling on the affected side by hand before re-running. These trips cannot distinguish a mechanical slip from an electrical dropout, and mechanical is the cheaper thing to rule out.
- After an overspeed trip: check both couplings and inspect for anything that came loose at speed.
- After a thermal trip: allow full cool-down and confirm the temperature reading falls as expected. A sensor stuck high looks the same as a hot motor.
- After any E-stop under load: inspect couplings. An abrupt stop from speed under load is the highest torsional shock the stand sees.

### 16.7 Accessing the stand

> **Before opening**
> Press the E-stop and confirm the bus is dead.
>
> Confirm all rotation has stopped. Coast-down takes several seconds and the shield obscures slow rotation.
>
> Motors may be at 60 C or higher after a run. Allow cool-down before handling.

**Accessing DUT hardware.** Remove 8x 5/16" bolts from the rear blast shield using a 3/16" allen key socket.

**Accessing electrical enclosure.** Remove 6x 5/16" bolts from the electrical shield using a 3/16" allen key socket and use handles to safely remove.

### 16.8 Spares worth holding

- L090 coupling spiders are the most likely wear item.
- Coupling grub screws, correct size and length.
- A spare FSESC 75200 controller, pre-configured and with its VESC ID set to something other than 0.
- Hall sensor extension cable or connector kit.
- Spare NTC 10k 3590B thermistors.
- Micro-USB cable for the CAN adapter. Cable failure presents as an enumeration problem and is easy to misdiagnose.

---

*End of document: E-Axle Dynamometer Test Stand, Rev 1.0*
