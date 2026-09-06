#include "ADC.h"

Uint16 Raw_Current_A = 0;
Uint16 Raw_Current_B = 0;
Uint16 Raw_Current_C = 0;
float Offset_Current_A = 2048.0f;
float Offset_Current_B = 2048.0f;



void Init_ADC_CurrentSensors(void)
{
    EALLOW;

    AdcRegs.ADCTRL1.bit.ACQ_PS = 0x0F;
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;

    AdcRegs.ADCTRL3.bit.ADCCLKPS = 0x03;
    AdcRegs.ADCTRL3.bit.SMODE_SEL = 1; //simultaneous sampling of ADCINA0-A7 and ADCINB0-B7

    AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0; //conversions=maxconv_value+1 = 1 which means 1 pair of conversions (because of simultaneous sampling)

    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x8;
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0xA;

    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;

    EDIS;
}

void Calibrate_ADC_Offsets(void)
{
    Uint32 Sum_A = 0;
    Uint32 Sum_B = 0;
    Uint16 i;

    // Trigger the ADC 1000 times to get a solid average while the inverter is OFF
    for(i = 0; i < 1000; i++)
    {
        // Force a software trigger to the ADC
        AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;

        // Wait for conversion to complete
        while(AdcRegs.ADCST.bit.INT_SEQ1 == 0) {}
        AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1; // Clear flag

        Sum_A += (AdcRegs.ADCRESULT0 >> 4);
        Sum_B += (AdcRegs.ADCRESULT1 >> 4);

        DELAY_US(100);
    }

    Offset_Current_A = (float)Sum_A / 1000.0f;
    Offset_Current_B = (float)Sum_B / 1000.0f;

    // Phase C is no longer needed since we calculate it mathematically!
}
