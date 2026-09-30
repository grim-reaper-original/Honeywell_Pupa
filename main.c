#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "ADC.h"
#include "RDC.h"
#include "ePWM.h"
#include "FOC.h"

volatile Uint16 Motor_Enable = 0;

extern __interrupt void adc_isr(void);

void main(void)
{
    InitSysCtrl();

    DINT;

    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    // Link ADC interrupt to ISR
    EALLOW;
    PieVectTable.ADCINT = &adc_isr;
    EDIS;

    // GPIO
    InitGpio();

    // ADC trigger/ISR debug marker
    Init_ADC_Trigger_Marker();

    // ADC
    InitAdc();
    Init_ADC_CurrentSensors();

    // Motor MUST be off during calibration
    Calibrate_ADC_Offsets();

    // Resolver
    Init_SPI_RDC();
    AD2S1210_Configure();

    DELAY_US(25000);

    AD2S1210_Clear_Startup_Faults();

    // PWM
    Init_ePWM_MotorControl();

    // Force PWM outputs into safe state
    PWM_ForceTripZone();

    // Initialize FOC startup state
    FOC_ResetStartup();

    // Keep motor disabled for initial testing
    Motor_Enable = 0;

    // Enable ADC interrupt
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    IER |= M_INT1;

    EINT;
    ERTM;

    // Background safety loop
    while(1)
    {
        if(Motor_Enable == 0)
        {
            PWM_ForceTripZone();
        }
        else
        {
            PWM_ClearTripZone();
        }
    }
}
