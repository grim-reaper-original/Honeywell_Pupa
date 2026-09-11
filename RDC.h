#ifndef RDC_H
#define RDC_H

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

// =========================================================
// Public RDC data
// =========================================================

extern Uint16 Rotor_Angle_Raw;
extern int16  Rotor_Velocity_Raw;
extern Uint16 Resolver_Fault_Register;

extern Uint16 Rotor_Angle_12;
extern int16  Rotor_Velocity_12;

extern volatile Uint16 Motor_PolePairs;
extern volatile float Angle_Offset;


// =========================================================
// Public RDC functions
// =========================================================

void Init_SPI_RDC(void);

void AD2S1210_Configure(void);
void AD2S1210_Clear_Startup_Faults(void);

void Read_Resolver_Data(void);
float RDC_GetMechanicalAngle(void);
float RDC_GetElectricalAngle(void);

#endif
