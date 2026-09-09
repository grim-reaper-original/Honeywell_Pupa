#ifndef ADC_H
#define ADC_H

#define CURRENT_GAIN 0.003222656f

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

extern Uint16 Raw_Current_A;
extern Uint16 Raw_Current_B;

extern float Offset_Current_A;
extern float Offset_Current_B;

extern float Current_A;
extern float Current_B;
extern float Current_C;

void Init_ADC_CurrentSensors(void);
void Calibrate_ADC_Offsets(void);
void Read_ADC_Currents(void);

#endif
