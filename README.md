# Field Oriented Control (FOC) BLDC Motor Implementation

This repository contains the firmware for a 3-phase BLDC motor utilizing Field Oriented Control (FOC) running on a TI C2000 DSP (TMS320F28335).

## System Architecture
*   **Execution Rate:** 20 kHz Hardware Interrupt (`adc_isr`)
*   **Sensor Package:** AD2S1210 Resolver (SPI-A at 7.5 MHz)
*   **Power Stage:** DRV8323 Gate Driver (SPI over McBSP-B)
*   **Modulation:** Master/Slave synchronized ePWM with 1 us hardware deadband.
