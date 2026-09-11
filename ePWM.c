#include "pwm.h"


int main(void)
{
    InitSysCtrl();
    InitGpio();
    Init_ePWM_MotorControl();

    DELAY_US(5000000);

    PWM_ForceTripZone();

    DELAY_US(5000000);

    PWM_ClearTripZone();

    while(1)
    {
    }
}

void Init_ePWM_MotorControl(void)
{
    EALLOW;

    // 1. Configure GPIO pins for ePWM1, ePWM2, ePWM3 (A and B)
    GpioCtrlRegs.GPAPUD.all &= ~0x0000003F;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;

    // Halt time-base clocks before configuration
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    // -----------------------------------------------------
    // MASTER: ePWM 1
    // -----------------------------------------------------
    EPwm1Regs.TBPRD = 3750;
    EPwm1Regs.TBPHS.half.TBPHS = 0;
    EPwm1Regs.TBCTL.bit.CTRMODE = 2; // Up-down counting upto 3750 (Centre aligned PWM)
    EPwm1Regs.TBCTL.bit.PHSEN = 0;   // Master module
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 1; // Send sync pulse at TBCTR = 0x0000

    EPwm1Regs.CMPA.half.CMPA = 1875; // Start at 50% duty
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;

    EPwm1Regs.AQCTLA.bit.CAU = 2; //Set bit on upcount
    EPwm1Regs.AQCTLA.bit.CAD = 1; //Clear bit on downcount

    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;   //full dead band generation (delay enabled for both rising and falling edge for A and B respectively)
    EPwm1Regs.DBCTL.bit.POLSEL = 2; // Active High Complementary (PWM1 - Active High. PWM2 - Active low for complementary A and B High and Low)
    EPwm1Regs.DBRED = 150;          // 1.0us Deadband
    EPwm1Regs.DBFED = 150;

    EPwm1Regs.TZCTL.bit.TZA = 2;    // Force LOW on trip
    EPwm1Regs.TZCTL.bit.TZB = 2;    // Force LOW on trip

    // ADC Trigger (SOCA) at TBCTR = 0
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;     //actually enables Start of Conversion for A
    EPwm1Regs.ETSEL.bit.SOCASEL = 2;    //SOCA pulse is generated when TBCTR = TBPRD (3750) i.e middle of the centre-aligned pwm pulse
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;    //generates SOCA pulse on first event. Does not wait for more than one

    // -----------------------------------------------------
    // SLAVE: ePWM 2
    // -----------------------------------------------------
    EPwm2Regs.TBPRD = 3750;
    EPwm2Regs.TBPHS.half.TBPHS = 0;
    EPwm2Regs.TBCTL.bit.CTRMODE = 2;
    EPwm2Regs.TBCTL.bit.PHSEN = 1;    // Slave module
    EPwm2Regs.TBCTL.bit.SYNCOSEL = 0; // Pass sync out

    EPwm2Regs.CMPA.half.CMPA = 1875;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;

    EPwm2Regs.AQCTLA.bit.CAU = 2;  // Set bit (0->1) on upcount (here = 1875)
    EPwm2Regs.AQCTLA.bit.CAD = 1;  // Clear bit (1->0) on downcount (here = 1875)

    EPwm2Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm2Regs.DBCTL.bit.POLSEL = 2;
    EPwm2Regs.DBRED = 150;
    EPwm2Regs.DBFED = 150;

    EPwm2Regs.TZCTL.bit.TZA = 2;
    EPwm2Regs.TZCTL.bit.TZB = 2;

    // -----------------------------------------------------
    // SLAVE: ePWM 3
    // -----------------------------------------------------
    EPwm3Regs.TBPRD = 3750;
    EPwm3Regs.TBPHS.half.TBPHS = 0;
    EPwm3Regs.TBCTL.bit.CTRMODE = 2;
    EPwm3Regs.TBCTL.bit.PHSEN = 1;    // Slave module

    EPwm3Regs.CMPA.half.CMPA = 1875;
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = 0;

    EPwm3Regs.AQCTLA.bit.CAU = 2;
    EPwm3Regs.AQCTLA.bit.CAD = 1;

    EPwm3Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm3Regs.DBCTL.bit.POLSEL = 2;
    EPwm3Regs.DBRED = 150;
    EPwm3Regs.DBFED = 150;

    EPwm3Regs.TZCTL.bit.TZA = 2;
    EPwm3Regs.TZCTL.bit.TZB = 2;

    // Restart time-base clocks perfectly in sync
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    EDIS;
}

// =========================================================
// PWM SAFETY FUNCTIONS
// =========================================================

// Instantly pulls all 6 PWM signals to 0V using One-Shot Trip (OST)
void PWM_ForceTripZone(void)
{
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm3Regs.TZFRC.bit.OST = 1;
    EDIS;
}

// Clears the trip condition so the motor can spin again
void PWM_ClearTripZone(void)
{
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EDIS;
}


void PWM_UpdateDuty(Uint16 cmp_A, Uint16 cmp_B, Uint16 cmp_C)
{
    EPwm1Regs.CMPA.half.CMPA = cmp_A;
    EPwm2Regs.CMPA.half.CMPA = cmp_B;
    EPwm3Regs.CMPA.half.CMPA = cmp_C;
}
