# Field Oriented Control (FOC) Firmware for BLDC Motor

This repository contains the bare-metal C firmware for a high-performance 3-phase Brushless DC (BLDC) motor controller. It implements closed-loop Field Oriented Control (FOC) running on a **Texas Instruments C2000 DSP (TMS320F28335)**.

The architecture is designed for deterministic, real-time execution, leveraging the DSP's Hardware Floating-Point Unit (FPU) to process complex control algorithms within a strict 50-microsecond deadline.

---

## ⚙️ Hardware Architecture

*   **Microcontroller:** TI TMS320F28335 (150 MHz)
*   **Gate Driver:** Texas Instruments DRV8323
    *   Communicates via McBSP-B (Configured as SPI in Clock Stop Mode)
    *   Configured for 100mA source / 200mA sink gate drive strength.
*   **Position & Velocity Feedback:** Analog Devices AD2S1210 Resolver-to-Digital Converter
    *   Communicates via SPI-A at 7.5 MHz.
    *   Reads absolute mechanical angle and velocity back-to-back in the ISR.
*   **Modulation:** 3-Phase synchronized ePWM (Timers 1, 2, and 3)
    *   Active-high complementary mode.
    *   Hardcoded **1.0 µs hardware deadband** to prevent cross-conduction.

---

## 🧠 Firmware & Control Architecture

The core FOC pipeline executes entirely within a deterministic **20 kHz hardware interrupt** (`adc_isr`), triggered synchronously by the ePWM1 zero-event to ensure noise-free ADC sampling.

### Control Loops
*   **Inner Current Loops (20 kHz):** Two independent PI controllers regulate the D-axis (Magnetic Flux) and Q-axis (Torque) currents.
*   **Outer Speed Loop (2 kHz):** A prescaled PI controller regulates shaft RPM, outputting a torque/current command to the Q-axis loop. 

### The FOC Pipeline
Every 50 µs, the DSP performs the following sequence:
1.  **Sample & Scale:** Reads raw 3-phase ADC counts and scales them to real floating-point Amperage.
2.  **Sensor Read:** Fetches resolver position/velocity and converts mechanical angle to electrical angle.
3.  **Forward Transforms:** Applies **Clarke** and **Park** transforms to convert 3-phase AC currents into stationary DC vectors ($D$ and $Q$).
4.  **PI Regulation:** Executes the anti-windup PI controllers to minimize error.
5.  **Reverse Transforms:** Applies **Inverse Park** and **Inverse Clarke** transforms to convert DC voltage commands back into 3-phase AC voltages.
6.  **Duty Cycle Generation:** Saturates the voltage commands and loads them into the ePWM `CMPA` shadow registers.

---

## 🚦 System State Machine

To prevent integral windup and ensure safe startup, the motor control logic is governed by a 3-state machine:

| State | Name | Description |
| :--- | :--- | :--- |
| **0** | `IDLE` | **Safe State.** ePWM outputs commanded to 0V (50% duty cycle). All PI controller memory (integrals) are continuously cleared to prevent windup. |
| **1** | `ALIGN` | **Rotor Zeroing.** Injects a static DC current (2.0A) solely into the D-axis to electromagnetically lock the rotor to the 0° position. After 1 second, the mechanical offset is recorded. |
| **2** | `RUN` | **Closed-Loop.** Normal FOC operation. The outer speed loop and inner current loops take full control of the motor. |

---

## 🛡️ Safety & Fault Handling

*   **Resolver Fault Monitoring:** The background CPU loop continuously monitors the `RDC_DOS` (Degradation of Signal) and `RDC_LOT` (Loss of Tracking) hardware pins. If a fault is detected, the system immediately drops to `IDLE`.
*   **Integral Anti-Windup:** PI controllers feature strict upper and lower saturation limits (`Umax`, `Umin`).
*   **Overcurrent Protection (OCP):** The DRV8323 is initialized to Latched Fault mode via SPI to protect the MOSFETs from overcurrent events.

---

## 🚀 Current Status & TODOs

The software logic is fully implemented, compiling with 0 errors. Development is currently paused pending physical hardware integration.

**Pending Hardware Tasks:**
- [ ] **Hardware Trip Zone (Milestone 10):** Map the physical `GD_FAULT` GPIO pin to the DSP's Trip Zone (TZ) module for instant, hardware-level PWM kill on short circuit.
- [ ] **PI Tuning:** Connect to a physical motor to perform step-response tuning on `Kp` and `Ki` values for the Id, Iq, and Speed controllers.

---
