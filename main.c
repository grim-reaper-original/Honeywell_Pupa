#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "ADC.h"
#include "RDC.h"
#include "ePWM.h"
#include "FOC.h"

volatile Uint16 Motor_Enable = 0;
volatile Uint32 Main_Loop_Counter = 0;


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
    Init_ADC_Trigger_Marker(); //creates marker that enables every time adc triggers


    InitAdc();
    Init_ADC_CurrentSensors();
    Calibrate_ADC_Offsets(); // Motor MUST be off here!

    Init_SPI_RDC();
    AD2S1210_Configure();
    DELAY_US(25000);
    AD2S1210_Clear_Startup_Faults();

    Init_ePWM_MotorControl();

    Motor_Enable = 1;
    // Enable Interrupts
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    IER |= M_INT1;
    EINT;
    ERTM;

    // Background Safety Loop
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
