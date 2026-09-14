#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "ADC.h"
#include "RDC.h"
#include "pwm.h"
#include "FOC.h"

extern __interrupt void adc_isr(void);

void main(void)
{
    InitSysCtrl();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    // Link the ADC interrupt to your ISR
    EALLOW;
    PieVectTable.ADCINT = &adc_isr;
    EDIS;

    // Initialize all hardware modules
    InitGpio();
    InitAdc();
    Init_ADC_CurrentSensors();
    Calibrate_ADC_Offsets(); // Motor MUST be off here!

    Init_SPI_RDC();
    AD2S1210_Configure();
    DELAY_US(25000);
    AD2S1210_Clear_Startup_Faults();

    Init_ePWM_MotorControl();

    // Enable Interrupts
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    IER |= M_INT1;
    EINT;
    ERTM;

    // Background Safety Loop
    while(1)
    {
        // Monitor AD2S1210 hardware fault pins
        // (Assuming READ_RDC_DOS and READ_RDC_LOT are in a header file or read here)
        if (GpioDataRegs.GPCDAT.bit.GPIO84 == 0 || GpioDataRegs.GPCDAT.bit.GPIO85 == 0)
        {
            PWM_ForceTripZone();
        }
    }
}
