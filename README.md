# Field Oriented Control (FOC) Firmware for PMSM/BLDC Motor

Bare-metal C firmware for a 3-phase PMSM/BLDC motor controller based on the **Texas Instruments TMS320F28335**. The firmware is being developed incrementally from peripheral bring-up to open-loop voltage-vector generation and, later, closed-loop Field-Oriented Control (FOC).

> **Current status:** The project is **not yet a closed-loop FOC implementation**. The current milestone is open-loop voltage-vector generation with resolver-angle processing, current measurement, FOC mathematical transforms, SVPWM-style zero-sequence modulation, PWM generation, and hardware bring-up.

---

## 1. Hardware

- **MCU:** TI TMS320F28335
- **Board oscillator:** 25 MHz
- **Intended SYSCLKOUT:** 125 MHz
- **Gate driver:** TI DRV8323
- **Resolver-to-digital converter:** Analog Devices AD2S1210
- **Resolver interface:** SPI-A plus GPIO control signals
- **Current sensing:** ADC phase-A and phase-B measurements; phase-C reconstructed in software
- **PWM:** ePWM1/ePWM2/ePWM3, complementary A/B outputs
- **Nominal inverter DC bus:** 28 V (`P28P0V`)
- **Inverter:** 3 half-bridges / 6 MOSFETs

### MCU clock configuration

The hardware uses a 25 MHz oscillator. The intended PLL configuration is:

```text
PLLCR.DIV   = 10
PLLSTS.DIVSEL = 2

SYSCLKOUT = 25 MHz × 10 / 2
          = 125 MHz
```

The older 150 MHz assumptions in legacy code are **not the current basis** for the FOC PWM configuration.

---

# 2. Current Development Stage

The current development stage is:

> **Open-loop voltage-vector generation + peripheral bring-up + hardware timing validation, before PI current control.**

### Implemented

- ePWM1/ePWM2/ePWM3 generation
- Center-aligned/up-down PWM
- PWM synchronization
- Complementary PWM outputs
- Programmable deadband
- PWM one-shot Trip Zone force/clear
- ePWM SOCA → ADC trigger infrastructure
- ADC interrupt infrastructure
- ADC offset calibration
- Phase-A/phase-B current acquisition
- Phase-C current reconstruction
- AD2S1210 SPI communication
- Resolver angle and velocity acquisition
- 12-bit mechanical angle extraction
- Mechanical-to-electrical angle conversion
- Clarke transform
- Park transform
- Inverse Park transform
- Inverse Clarke transform
- Zero-sequence modulation
- Duty-cycle generation
- Duty-to-CMPA conversion
- Open-loop FOC execution path
- `Motor_Enable` / Trip Zone safety gating

### Not yet completed

- Closed-loop PI current controllers
- Outer speed loop
- Resolver electrical-zero calibration
- Final motor-specific pole-pair configuration validation
- Final current-sensor calibration/validation
- Full gate-driver enable/fault validation
- Controlled rotating open-loop angle ramp
- Physical motor validation
- Final closed-loop FOC integration

---

# 3. Development Sequence

The current development sequence is:

```text
Clock
  ↓
ePWM
  ↓
PWM SOCA
  ↓
ADC
  ↓
ADC ISR
  ↓
ADC currents + RDC angle
  ↓
Clarke
  ↓
Electrical angle
  ↓
Park
  ↓
Open-loop Vd/Vq
  ↓
Inverse Park
  ↓
Inverse Clarke
  ↓
Zero-Sequence Modulation
  ↓
Duty
  ↓
CMPA
  ↓
ePWM
  ↓
Open-loop motor test
  ↓
PI current loop
  ↓
Speed loop
```

The PI and speed loops are deliberately postponed until the open-loop path and hardware interfaces have been validated.

---

# 4. PWM Configuration

The intended PWM frequency is **20 kHz**.

For a 125 MHz time-base clock:

```text
TBCLK = 125 MHz
TBPRD = 3125
CTRMODE = up-down

fPWM = TBCLK / (2 × TBPRD)
     = 125 MHz / (2 × 3125)
     = 20 kHz
```

Therefore:

```text
PWM period = 50 µs
```

### Current PWM settings

```text
TBPRD       = 3125
CTRMODE     = 2          // up-down
HSPCLKDIV   = 0          // /1
CLKDIV      = 0          // /1
```

All three PWM modules should use:

```text
ePWM1 → Phase A
ePWM2 → Phase B
ePWM3 → Phase C
```

ePWM1 is the master time base; ePWM2 and ePWM3 are synchronized slaves.

### Initial duty

50% duty corresponds approximately to:

```text
CMPA = 1562/1563
```

Do not use the old `1875` initial value for the current 3125-TBPRD configuration, because that corresponds to 60%.

### Deadband

Current register value:

```text
DBRED = 150
DBFED = 150
```

At 125 MHz:

```text
1 TBCLK = 8 ns
150 × 8 ns = 1.2 µs
```

Therefore the configured deadband is approximately **1.2 µs**.

---

# 5. PWM → ADC Trigger Chain

The intended timing chain is:

```text
ePWM1 at 20 kHz
      ↓
SOCA once per PWM cycle
      ↓
ADC conversion
      ↓
ADCINT
      ↓
adc_isr()
```

Current trigger configuration:

```text
EPwm1Regs.ETSEL.bit.SOCAEN  = 1
EPwm1Regs.ETSEL.bit.SOCASEL = 2
EPwm1Regs.ETPS.bit.SOCAPRD  = 1

AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1
AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1   = 1
```

The first hardware timing test should verify this chain **before adding FOC computation to the ISR**.

---

# 6. ADC ISR Timing Test

GPIO10 is used as an ISR execution marker.

At ISR entry:

```c
GpioDataRegs.GPASET.bit.GPIO10 = 1;
```

At ISR exit:

```c
GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
```

Therefore:

- GPIO10 repetition frequency should be approximately **20 kHz**
- GPIO10 period should be approximately **50 µs**
- GPIO10 pulse width represents approximately the ISR execution time

`isr_counter` should increase at approximately:

```text
20,000 counts/second
```

### Initial timing-test ISR

For the first trigger-chain test, temporarily use:

```c
__interrupt void adc_isr(void)
{
    GpioDataRegs.GPASET.bit.GPIO10 = 1;

    isr_counter++;

    Read_ADC_Currents();
    Read_Resolver_Data();

    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;

    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
```

`FOC_OpenLoopStep()` is temporarily omitted only to isolate:

```text
PWM → SOCA → ADC → ADC ISR
```

After this timing chain is verified, `FOC_OpenLoopStep()` can be restored and the ISR execution time rechecked.

---

# 7. ADC Current Acquisition

The ADC measures phase A and phase B currents.

Software variables:

```text
Raw_Current_A
Raw_Current_B

Offset_Current_A
Offset_Current_B

Current_A
Current_B
Current_C
```

Raw ADC conversion:

```text
Raw_Current_A = ADCRESULT0 >> 4
Raw_Current_B = ADCRESULT1 >> 4
```

Current conversion:

```text
Current_A = (Raw_Current_A - Offset_Current_A) × CURRENT_GAIN
Current_B = (Raw_Current_B - Offset_Current_B) × CURRENT_GAIN
Current_C = -(Current_A + Current_B)
```

The ADC offset calibration uses 1000 samples with the motor current inactive.

`CURRENT_GAIN` is treated as a validated project parameter and should not be changed during the current timing/bring-up test unless hardware testing indicates otherwise.

---

# 8. Resolver / AD2S1210

The AD2S1210 communicates through SPI-A.

Important signals:

```text
GPIO16 = SPI SIMO
GPIO17 = SPI SOMI
GPIO18 = SPI CLK
GPIO19 = CS
GPIO20 = A0
GPIO21 = A1
GPIO23 = SAMPLE
GPIO9  = FSYNC

GPIO84 = DOS
GPIO85 = LOT
```

### Resolver data variables

```text
Rotor_Angle_Raw
Rotor_Angle_12

Rotor_Velocity_Raw
Rotor_Velocity_12

Resolver_Fault_Register
```

The current validated angle extraction is:

```text
Rotor_Angle_12 = Rotor_Angle_Raw >> 4
```

with a 12-bit range:

```text
0 ... 4095
```

The current resolver configuration uses:

```text
AD2S1210 resolution control = 0x7E
```

and the existing velocity extraction is retained.

---

# 9. Resolver Fault Handling

The background loop checks:

```c
GpioDataRegs.GPCDAT.bit.GPIO84
GpioDataRegs.GPCDAT.bit.GPIO85
```

These correspond to:

```text
GPIO84 = DOS
GPIO85 = LOT
```

The current software assumes:

```text
DOS HIGH = healthy
LOT HIGH = healthy
DOS LOW  = fault
LOT LOW  = fault
```

A resolver fault forces:

```text
Motor_Enable = 0
PWM_ForceTripZone()
```

The actual hardware polarity should be confirmed during bring-up.

---

# 10. Mechanical and Electrical Angle

Mechanical angle:

```text
theta_mech = Rotor_Angle_12 × 2π / 4096
```

Electrical angle:

```text
theta_e = theta_mech × Motor_PolePairs + Angle_Offset
```

The result is wrapped to:

```text
0 ≤ theta_e < 2π
```

Current variables:

```text
Motor_PolePairs
Angle_Offset
theta_e
```

Current default values:

```text
Motor_PolePairs = 1
Angle_Offset = 0
```

These are configuration/calibration values and must ultimately match the actual motor.

---

# 11. FOC Mathematical Chain

## Clarke

```text
Iα = Ia

Iβ = (Ia + 2Ib) / √3
```

Variables:

```text
I_alpha
I_beta
```

## Park

```text
Id = Iα cos(θe) + Iβ sin(θe)

Iq = -Iα sin(θe) + Iβ cos(θe)
```

Variables:

```text
Id
Iq
```

PI current control is **not implemented yet**.

---

# 12. Open-Loop Voltage Generation

Current open-loop command:

```text
Vd = 0 V
Vq = 3 V
```

The present voltage path is:

```text
Vd/Vq
  ↓
Inverse Park
  ↓
Inverse Clarke
  ↓
Zero-Sequence Modulation
  ↓
Duty
  ↓
CMPA
  ↓
ePWM
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

---

# 13. Zero-Sequence Modulation

The current implementation calculates:

```text
Vmax = max(Va, Vb, Vc)
Vmin = min(Va, Vb, Vc)

Voffset = -(Vmax + Vmin)/2
```

Then:

```text
Va_mod = Va + Voffset
Vb_mod = Vb + Voffset
Vc_mod = Vc + Voffset
```

---

# 14. Duty Generation

The current implementation uses:

```text
Duty_A = 0.5 + Va_mod / VDC
Duty_B = 0.5 + Vb_mod / VDC
Duty_C = 0.5 + Vc_mod / VDC
```

Each duty is clamped to:

```text
0 ... 1
```

Current nominal software value:

```text
VDC = 28 V
```

Final modulation scaling and PWM polarity must be validated against the actual inverter.

---

# 15. CMPA Generation

<<<<<<< HEAD
Current conversion:

```text
CMPA_a = Duty_A × 3125
CMPA_b = Duty_B × 3125
CMPA_c = Duty_C × 3125
```

Variables:

```text
CMPA_a
CMPA_b
CMPA_c
```

These are written to:

```text
EPwm1Regs.CMPA.half.CMPA
EPwm2Regs.CMPA.half.CMPA
EPwm3Regs.CMPA.half.CMPA
```

through:

```c
PWM_UpdateDuty(CMPA_a, CMPA_b, CMPA_c);
```

---

# 16. Motor Enable and Trip Zone

The global control variable is:

```c
volatile Uint16 Motor_Enable = 0;
```

It is defined in `main.c` and declared elsewhere with:

```c
extern volatile Uint16 Motor_Enable;
```

Behavior:

```text
Motor_Enable = 0
    ↓
PWM_ForceTripZone()

Motor_Enable = 1
    ↓
PWM_ClearTripZone()
```

provided resolver fault inputs indicate healthy operation.

The Trip Zone configuration is intended to force PWM outputs low during a fault/disabled state.

---

# 17. Current FOC Variables for CCS Debugging

A compact watch window should contain:

```text
Raw_Current_A
Raw_Current_B
Offset_Current_A
Offset_Current_B
Current_A
Current_B
Current_C

Rotor_Angle_12
Rotor_Velocity_12
Motor_PolePairs
Angle_Offset
theta_e

I_alpha
I_beta
Id
Iq

VDC
Vd
Vq
V_alpha
V_beta
Va
Vb
Vc

V_max
V_min
V_offset
Va_mod
Vb_mod
Vc_mod

Duty_A
Duty_B
Duty_C

CMPA_a
CMPA_b
CMPA_c

Motor_Enable
isr_counter
```

---

# 18. 30-Minute Hardware Validation Checklist

Only the following items are required for the first short hardware session.

## Clock

Check:

```text
SysCtrlRegs.PLLCR.bit.DIV
SysCtrlRegs.PLLSTS.bit.DIVSEL
```

Expected:

```text
10
2
```

giving:

```text
SYSCLK = 125 MHz
```

## PWM

Check:

```text
EPwm1Regs.TBPRD
EPwm1Regs.TBCTL.bit.CTRMODE
EPwm1Regs.TBCTL.bit.HSPCLKDIV
EPwm1Regs.TBCTL.bit.CLKDIV
EPwm2Regs.TBPRD
EPwm3Regs.TBPRD
```

Expected:

```text
TBPRD = 3125
CTRMODE = 2
HSPCLKDIV = 0
CLKDIV = 0
```

Scope ePWM1A:

```text
20 kHz
50 µs period
```

## ADC trigger / ISR

Check:

```text
EPwm1Regs.ETSEL.bit.SOCAEN
EPwm1Regs.ETSEL.bit.SOCASEL
EPwm1Regs.ETPS.bit.SOCAPRD

AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1
AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1

isr_counter
```

Expected trigger settings:

```text
SOCAEN = 1
SOCASEL = 2
SOCAPRD = 1
EPWM_SOCA_SEQ1 = 1
INT_ENA_SEQ1 = 1
```

Scope GPIO10:

```text
~20 kHz repetition
~50 µs period
```

Measure GPIO10 pulse width to estimate ISR execution time.

## ADC

Check:

```text
Raw_Current_A
Raw_Current_B
Offset_Current_A
Offset_Current_B
Current_A
Current_B
Current_C
```

With zero current:

```text
Current_A ≈ 0
Current_B ≈ 0
Current_C ≈ 0
```

## Resolver

Check:

```text
Rotor_Angle_12
Rotor_Velocity_12
GPIO84 / DOS
GPIO85 / LOT
```

Manually rotate the rotor and verify that the angle changes smoothly and remains within:

```text
0 ... 4095
```

## FOC calculation

Check:

```text
theta_e
I_alpha
I_beta
Id
Iq
V_alpha
V_beta
Va
Vb
Vc
Duty_A
Duty_B
Duty_C
```

Confirm:

```text
No NaN/invalid values
0 ≤ theta_e < 2π
0 ≤ Duty_A/B/C ≤ 1
```

## PWM update

Check:

```text
CMPA_a
CMPA_b
CMPA_c

EPwm1Regs.CMPA.half.CMPA
EPwm2Regs.CMPA.half.CMPA
EPwm3Regs.CMPA.half.CMPA
```

and verify:

```text
CMPA ≈ Duty × 3125
```

---

# 19. Hardware Test Safety

Before applying the 28 V inverter bus:

1. Verify firmware builds with no errors.
2. Verify PWM frequency and synchronization.
3. Verify Trip Zone behavior.
4. Verify ADC trigger and ISR timing.
5. Verify resolver angle and DOS/LOT status.
6. Verify calculated duty/CMPA values.
7. Verify complementary PWM and deadtime on the oscilloscope.
8. Verify gate-driver enable/fault behavior.

Initial motor testing should use a current-limited supply and conservative voltage commands.

Do not rely on `Motor_Enable = 1` as the only safety mechanism. Resolver faults and Trip Zone behavior must be verified first.

---

# 20. Current Milestone Plan

| Stage | Status | Description |
|---|---|---|
| 1 | Complete | ePWM configuration and synchronization |
| 2 | Complete | PWM complementary outputs and deadband |
| 3 | Complete | PWM Trip Zone safety functions |
| 4 | Complete | AD2S1210 communication and resolver acquisition |
| 5 | Complete | ADC current acquisition and offset calibration infrastructure |
| 6 | Complete | Clarke/Park and inverse transforms |
| 7 | Complete | ZSM, duty and CMPA generation |
| 8 | **Current** | Verify 125 MHz clock and 20 kHz PWM |
| 9 | **Current** | Verify PWM → ADC → ISR timing chain |
| 10 | **Current** | Verify resolver/current data on hardware |
| 11 | **Current** | Validate open-loop voltage-vector path |
| 12 | Pending | Controlled rotating open-loop angle generation |
| 13 | Pending | Low-voltage open-loop motor test |
| 14 | Pending | Resolver electrical-zero calibration |
| 15 | Pending | 20 kHz PI current loop |
| 16 | Pending | 2 kHz speed loop |
| 17 | Pending | Final closed-loop FOC integration |

---

# 21. Known Implementation Notes

- The current hardware oscillator is **25 MHz**; the intended F28335 system clock is **125 MHz** using `PLLCR = 10` and `DIVSEL = 2`.
- The current PWM target is **20 kHz**, using `TBPRD = 3125` in up-down mode.
- Do not retain legacy `3750`/150 MHz PWM calculations in the active FOC path.
- `CMPA = 1562/1563` corresponds to approximately 50% duty with `TBPRD = 3125`.
- `DBRED/DBFED = 150` corresponds to approximately **1.2 µs** deadtime at 125 MHz.
- `CURRENT_GAIN` is treated as validated for the current development stage.
- AD2S1210 resolution/control value `0x7E` is the current validated configuration.
- Resolver velocity extraction is treated as validated.
- `Motor_PolePairs` and `Angle_Offset` still require motor-specific confirmation/calibration.
- The current open-loop voltage command is `Vd = 0`, `Vq = 3 V`.
- The current open-loop path uses the resolver electrical angle for inverse Park; a deliberately advancing open-loop angle ramp is a later motor-start step.
- PI controllers and the speed loop are not yet implemented.
- The ADC ISR is intended to be synchronized to the 20 kHz PWM/ADC trigger.
- GPIO10 is an **ADC ISR execution marker**, not the raw ePWM SOCA signal.
- `isr_counter` is the software count used to verify ADC ISR repetition.
- Standalone test source files containing their own `main()` must be excluded from the production/main build.
- `DSP2833x_Gpio.c` must be included in the project because `InitGpio()` is used by `main.c`.

---

# 22. Project Philosophy

The firmware is intentionally developed one layer at a time:

```text
Peripherals
    ↓
Measurements
    ↓
Resolver angle
    ↓
Transforms
    ↓
Voltage vector
    ↓
Modulation
    ↓
PWM
    ↓
Gate driver
    ↓
Open-loop motor test
    ↓
Current loop
    ↓
Speed loop
    ↓
Closed-loop FOC
```

The immediate objective is **not PI tuning**. The immediate objective is to prove that the clock, PWM timing, ADC trigger, current acquisition, resolver acquisition, angle processing, voltage-vector mathematics, modulation, PWM update, and hardware safety mechanisms all behave correctly on the actual controller.

---

**Last updated: 21 September 2026**
=======
**Last updated:** 11 September 2026
>>>>>>> branch 'master' of https://github.com/grim-reaper-original/Honeywell_Pupa
