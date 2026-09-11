# Field Oriented Control (FOC) Firmware for BLDC Motor

This repository contains the bare-metal C firmware for a 3-phase PMSM/BLDC motor controller built around the **Texas Instruments C2000 DSP (TMS320F28335)**. The project is being developed incrementally from peripheral bring-up to open-loop voltage-vector generation and, later, closed-loop Field-Oriented Control (FOC).

**Important:** the project is **not yet a closed-loop FOC implementation**. The current milestone is open-loop, resolver-angle-referenced voltage-vector generation with the major FOC mathematical and peripheral building blocks implemented.

The architecture is designed for deterministic, real-time execution, leveraging the DSP's Hardware Floating-Point Unit (FPU) to process complex control algorithms within a strict 50-microsecond deadline.

---

## Hardware Architecture

*   **Microcontroller:** TI TMS320F28335 (150 MHz)
*   **Gate Driver:** Texas Instruments DRV8323
    *   Communicates via McBSP-B (Configured as SPI in Clock Stop Mode)
    *   Drives the six MOSFET gate signals for the three inverter legs.
*   **Position & Velocity Feedback:** Analog Devices AD2S1210 Resolver-to-Digital Converter
    *   Communicates via SPI-A (current driver configures the SPI clock from the F28335 LSPCLK divider).
    *   Reads absolute mechanical angle and velocity through the resolver driver; final synchronized ISR integration is still pending.
*   **Modulation:** 3-Phase synchronized ePWM (Timers 1, 2, and 3)
    *   Active-high complementary mode.
    *   Hardcoded **1.0 µs hardware deadband** to prevent cross-conduction.
*   **Inverter:** 3 half-bridges / 6 MOSFETs, with phase outputs A/B/C and current shunts on the phase paths.
*   **Nominal DC bus:** 28 V (`P28P0V` in the hardware schematic).

---

## Current Development Stage

The current development stage is:

> **Open-loop voltage-vector generation and peripheral bring-up, before PI current control.**

Implemented so far:
- ePWM1/ePWM2/ePWM3 generation, synchronization, complementary outputs and deadband
- PWM trip-zone safety functions
- ePWM-triggered ADC conversion infrastructure
- AD2S1210 resolver communication and position/velocity acquisition
- Mechanical-to-electrical angle conversion
- Two-phase current acquisition, phase-C reconstruction and ADC offset calibration
- Clarke and Park transforms
- Inverse Park and inverse Clarke transforms
- Zero-Sequence Modulation (ZSM)
- Duty-cycle and CMPA generation

Not yet completed:
- Closed-loop PI current controllers
- Speed loop
- Final synchronized 20 kHz FOC ISR
- Resolver electrical-zero calibration
- Final current-sensor gain calibration
- Full gate-driver enable/fault integration
- Physical inverter and motor validation

The intended progression is:

```text
ePWM -> RDC -> ADC -> Clarke -> electrical angle -> Park
     -> inverse Park -> inverse Clarke -> ZSM -> duty/CMPA
     -> open-loop hardware test -> PI current loop -> speed loop
```

## Firmware & Control Architecture

The final controller is intended to execute a deterministic current loop synchronized to the PWM/ADC timing. That final architecture is **not yet integrated**.

For the current open-loop stage, the voltage path is:

```text
Resolver angle -> electrical angle -> inverse Park
                -> inverse Clarke -> ZSM
                -> duty cycle -> ePWM CMPA
```

The current-measurement path is already implemented separately:

```text
ADC phase A/B -> current scaling -> phase C reconstruction
               -> Clarke -> Park -> Id/Iq
```

The PI controllers are deliberately postponed until the open-loop voltage-generation path and hardware PWM/gate-driver behavior have been validated.

### The FOC Pipeline
At the current stage, the DSP does **not** yet execute the complete final 50 µs closed-loop sequence. The mathematical building blocks are being validated independently before they are tied to a synchronized control ISR.

The eventual closed-loop sequence will be:

1. Sample and scale phase currents.
2. Read resolver position and calculate electrical angle.
3. Clarke transform.
4. Park transform.
5. PI regulation of `Id` and `Iq`.
6. Inverse Park.
7. Inverse Clarke.
8. Zero-Sequence Modulation.
9. Convert duty cycles to ePWM compare values.
10. Update the three synchronized PWM legs.

## Development State / Milestones

The project is being developed in stages rather than enabling the full controller at once.

| Stage | Status | Description |
| :--- | :--- | :--- |
| 1 | Implemented | ePWM configuration, synchronization, complementary outputs and deadband |
| 2 | Implemented | PWM trip-zone safety functions |
| 3 | Implemented | AD2S1210 SPI communication and resolver data acquisition |
| 4 | Implemented | ADC current acquisition and offset-calibration infrastructure |
| 5 | Implemented | Clarke/Park and inverse transforms |
| 6 | Implemented | ZSM, duty-cycle and CMPA generation |
| 7 | **Current** | Open-loop voltage-vector validation and hardware PWM/gate-driver validation |
| 8 | Pending | Resolver electrical-zero calibration and controlled motor test |
| 9 | Pending | 20 kHz PI current loop |
| 10 | Pending | 2 kHz speed loop |
| 11 | Pending | Final closed-loop FOC integration |

## Safety & Fault Handling

The current firmware has software trip-zone functions which can force the ePWM outputs low. The gate-driver enable/fault path still needs to be fully validated before treating the inverter as production-safe.

The AD2S1210 code also reads resolver fault/status information and exposes the DOS/LOT hardware inputs.

Initial motor testing should use a current-limited supply and a low commanded voltage. Oscilloscope verification of complementary PWM and deadtime should be completed before applying motor power.



*   **Resolver Fault Monitoring:** The background CPU loop continuously monitors the `RDC_DOS` (Degradation of Signal) and `RDC_LOT` (Loss of Tracking) hardware pins. If a fault is detected, the system immediately drops to `IDLE`.
*   **Integral Anti-Windup:** PI controllers feature strict upper and lower saturation limits (`Umax`, `Umin`).
*   **Overcurrent Protection (OCP):** The DRV8323 is initialized to Latched Fault mode via SPI to protect the MOSFETs from overcurrent events.

---

## Current Status & TODOs

### Completed in software

**PWM:**
- GPIO0-GPIO5 mapped to ePWM1A/B, ePWM2A/B and ePWM3A/B
- Up-down / center-aligned PWM
- 20 kHz target frequency with `TBPRD = 3750` at the configured 150 MHz time-base
- 50% initial compare value (`CMPA = 1875`)
- Complementary output generation
- Approximately 1 µs deadband
- Master/slave synchronization
- One-shot trip-zone force/clear functions
- ePWM SOCA configuration for ADC triggering

**Resolver:**
- SPI-A initialization
- AD2S1210 configuration
- Startup fault-clearing sequence
- Position and velocity reads
- 12-bit mechanical angle extraction
- Mechanical-to-electrical angle conversion

**ADC:**
- Simultaneous A/B sampling configuration
- ADC interrupt infrastructure
- 1000-sample zero-current offset calibration
- Phase A/B current scaling
- Phase C reconstruction using `Ic = -(Ia + Ib)`

**FOC mathematics:**
- Clarke
- Park
- Inverse Park
- Inverse Clarke
- Zero-Sequence Modulation
- Duty generation
- Duty-to-CMPA conversion

### Immediate TODOs

1. Validate the resolver-angle-dependent voltage calculations in CCS.
2. Connect the calculated CMPA values to `PWM_UpdateDuty()` in the open-loop path.
3. Verify all six ePWM signals on an oscilloscope without motor power applied.
4. Verify gate-driver enable, fault and all six gate outputs.
5. Validate current-sensor gain and polarity.
6. Calibrate the motor electrical-angle offset.
7. Perform a cautious low-voltage/current-limited open-loop motor test.
8. Replace the uncontrolled background-loop execution with a PWM/ADC-synchronized control ISR.
9. Add PI current controllers.
10. Add the outer speed controller.

### Important implementation caveats

- The present `FOC.c` calculates `CMPA_a/b/c` but the uploaded version does not yet call `PWM_UpdateDuty()` from `FOC_OpenLoopStep()`.
- The current open-loop test uses `Vd = 0 V` and `Vq = 3 V`. There are no PI controllers yet.
- `FOC_OpenLoopStep()` currently calls Clarke/Park before overwriting `Vd/Vq`; because the uploaded `FOC.c` does not call `Read_ADC_Currents()` in that path, those current transforms are not yet part of the active voltage-control feedback path.
- The uploaded `RDC.c` currently initializes `Motor_PolePairs` to `1`; this is a configurable placeholder and must be set to the actual motor pole-pair count before final electrical-angle testing.
- `VDC` is currently set to 28 V as the nominal inverter bus value; final modulation scaling must be validated against the actual inverter topology and PWM polarity.
- The current `CURRENT_GAIN` must be verified from the actual shunt/amplifier hardware before interpreting current values as calibrated amperage.
- `Angle_Offset` is currently initialized to zero and must be calibrated against the physical motor.
- The final control loop should be synchronized to the 20 kHz PWM/ADC timing rather than run from an arbitrary `while(1)` loop.

## Control Equations Currently Implemented

### Clarke

```text
Iα = Ia
Iβ = (Ia + 2Ib) / √3
```

### Park

```text
Id = Iα cos(θe) + Iβ sin(θe)
Iq = -Iα sin(θe) + Iβ cos(θe)
```

### Inverse Park

```text
Vα = Vd cos(θe) - Vq sin(θe)
Vβ = Vd sin(θe) + Vq cos(θe)
```

### Inverse Clarke

```text
Va = Vα
Vb = -0.5Vα + (√3/2)Vβ
Vc = -0.5Vα - (√3/2)Vβ
```

### Zero-Sequence Modulation

```text
Vmax = max(Va, Vb, Vc)
Vmin = min(Va, Vb, Vc)
Voffset = -(Vmax + Vmin)/2

Va_mod = Va + Voffset
Vb_mod = Vb + Voffset
Vc_mod = Vc + Voffset
```

### Duty generation

The current implementation uses:

```text
Duty = 0.5 + Vmod / VDC
```

which corresponds to the chosen normalized modulation convention. This scaling is still subject to hardware validation.

### CMPA generation

```text
CMPA = Duty × TBPRD
     = Duty × 3750
```

For example, with `Vd = 0`, `Vq = 3 V`, `θe = 0` and `VDC = 28 V`:

```text
Vα = 0
Vβ = 3 V
Va = 0 V
Vb ≈ +2.598 V
Vc ≈ -2.598 V

Duty_A = 0.5000
Duty_B ≈ 0.5928
Duty_C ≈ 0.4072

CMPA_A = 1875
CMPA_B ≈ 2223
CMPA_C ≈ 1527
```

This provides a concrete numerical sanity check for the open-loop signal chain.

## Project Direction

The development philosophy is to prove each layer before adding feedback: **peripherals → measurements → angle → transforms → modulation → PWM → hardware → current loop → speed loop**. This keeps hardware failures and control-algorithm failures distinguishable.

**Last updated:** 11 September 2026
