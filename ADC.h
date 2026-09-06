#ifndef ADC_H
#define ADC_H

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

extern Uint16 Raw_Current_A;
extern Uint16 Raw_Current_B;
extern Uint16 Raw_Current_C;

void Init_ADC_CurrentSensors(void);
void Calibrate_ADC_Offsets(void);

#endif
