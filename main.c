#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include <math.h>

#define MOTOR_POLE_PAIRS      4
#define ONE_DIVIDED_BY_SQRT3  0.57735026919f
#define CURRENT_GAIN          0.015f
#define RAD_PER_TICK          0.00009587379f

// FOC Structures
typedef struct { float As, Bs, Cs, Alpha, Beta; } CLARKE_T;
typedef struct { float Alpha, Beta, Sine, Cosine, Ds, Qs; } PARK_T;
typedef struct { float Ds, Qs, Sine, Cosine, Alpha, Beta; } IPARK_T;

void Init_ADC_CurrentSensors(void);
void Init_SPI_RDC(void);
void Init_SPI_GateDriver(void);
void Init_ePWM_MotorControl(void);
void Calc_Clarke(CLARKE_T *v);
void Calc_Park(PARK_T *v);
__interrupt void adc_isr(void);

Uint16 Raw_Current_A = 0;
Uint16 Raw_Current_B = 0;
Uint16 Raw_Current_C = 0;
float Offset_Current_A = 2048.0f;
float Offset_Current_B = 2048.0f;
float Offset_Current_C = 2048.0f;
Uint16 Rotor_Angle_Raw = 0;
Uint16 Rotor_Angle_Elec = 0;

CLARKE_T clarke_calc;
PARK_T park_calc;

void main(void)
{
    DisableDog();
    InitPll(6, 3);
    InitPeripheralClocks();

    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCINT = &adc_isr;
    EDIS;

    InitAdc();
    Init_ADC_CurrentSensors();
    Init_SPI_RDC();
    Init_SPI_GateDriver();
    Init_ePWM_MotorControl();

    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    IER |= M_INT1;
    EINT;
    ERTM;

    while(1) {}
}

__interrupt void adc_isr(void)
{
    Raw_Current_A = (AdcRegs.ADCRESULT0 >> 4);
    Raw_Current_B = (AdcRegs.ADCRESULT1 >> 4);
    Raw_Current_C = (AdcRegs.ADCRESULT2 >> 4);

    clarke_calc.As = ((float)Raw_Current_A - Offset_Current_A) * CURRENT_GAIN;
    clarke_calc.Bs = ((float)Raw_Current_B - Offset_Current_B) * CURRENT_GAIN;

    // Basic Resolver Read (Placeholder)
    Rotor_Angle_Elec = Rotor_Angle_Raw * MOTOR_POLE_PAIRS;
    float angle_rad = (float)Rotor_Angle_Elec * RAD_PER_TICK;

    park_calc.Sine   = sinf(angle_rad);
    park_calc.Cosine = cosf(angle_rad);

    Calc_Clarke(&clarke_calc);

    park_calc.Alpha = clarke_calc.Alpha;
    park_calc.Beta  = clarke_calc.Beta;
    Calc_Park(&park_calc);

    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// Implementations omitted for historical brevity
void Init_ADC_CurrentSensors(void) {}
void Init_SPI_RDC(void) {}
void Init_SPI_GateDriver(void) {}
void Init_ePWM_MotorControl(void) {}
void Calc_Clarke(CLARKE_T *v) {}
void Calc_Park(PARK_T *v) {}
