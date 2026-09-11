#include "FOC.h"
#include "ADC.h"
#include "RDC.h"
#include "ePWM.h"
#include <math.h>

volatile float VDC = 28.0f;

volatile float Id = 0.0f;
volatile float Iq = 0.0f;
volatile float Vd = 0.0f;
volatile float Vq = 3.0f;

volatile float V_alpha = 0.0f;
volatile float V_beta = 0.0f;

volatile float Va = 0.0f;
volatile float Vb = 0.0f;
volatile float Vc = 0.0f;


//modulation variables
volatile float V_offset=0.0f;

volatile float Va_mod=0.0f;
volatile float Vb_mod=0.0f;
volatile float Vc_mod=0.0f;

volatile float V_max = 0.0f;
volatile float V_min = 0.0f;

volatile float Duty_A=0.0f;
volatile float Duty_B=0.0f;
volatile float Duty_C=0.0f;

volatile Uint16 CMPA_a = 1875;
volatile Uint16 CMPA_b = 1875;
volatile Uint16 CMPA_c = 1875;



//functions
void Clarke_Transform(void)
{
    I_alpha = Current_A;

    I_beta = (Current_A + 2.0f * Current_B)
             * 0.577350269f;
}


void Park_Transform(void)
{
    float theta_e;
    float sin_theta;
    float cos_theta;

    theta_e = RDC_GetElectricalAngle();

    sin_theta = sinf(theta_e);
    cos_theta = cosf(theta_e);

    Id = I_alpha * cos_theta
       + I_beta * sin_theta;

    Iq = -I_alpha * sin_theta
       + I_beta * cos_theta;
}

void Inverse_Park(void)
{
    float theta_e;
    float sin_theta;
    float cos_theta;

    theta_e = RDC_GetElectricalAngle();

    sin_theta=sinf(theta_e);
    cos_theta=cosf(theta_e);

    V_alpha = V_d*cos_theta - V_q*sin_theta;
    V_beta = V_d*sin_theta + V_q*cos_theta;
}


void Inverse_Clarke(void)
{
    Va = V_alpha;

    Vb = -0.5f * V_alpha
         + 0.8660254038f * V_beta;

    Vc = -0.5f * V_alpha
         - 0.8660254038f * V_beta;
}


void Zero_Sequence_Modulation(void)
{
    V_max= Va;
    if (Vb>V_max)
        V_max=Vb;
    if (Vc>V_max)
        V_max=Vc;     //finding Vmax

    V_min=Va;
    if (Vb<V_min)
        V_min=Vb;
    if (Vc<V_min)
        V_min=Vc;   //finding Vmin

    V_offset = -((V_max+V_min)/2.0f);

    Va_mod = Va + V_offset;
    Vb_mod = Vb + V_offset;
    Vc_mod = Vc + V_offset;
}

void Modulation_to_Duty(void)
{
    Duty_A = 0.5f + (Va_mod/VDC);
    Duty_B = 0.5f + (Vb_mod/VDC);
    Duty_C = 0.5f + (Vc_mod/VDC);

    if (Duty_A > 1.0f)
        Duty_A = 1.0f;
    if (Duty_A < 0.0f)
        Duty_A = 0.0f;

    if (Duty_B > 1.0f)
        Duty_B = 1.0f;
    if (Duty_B < 0.0f)
        Duty_B = 0.0f;

    if (Duty_C > 1.0f)
        Duty_C = 1.0f;
    if (Duty_C < 0.0f)
        Duty_C = 0.0f;
}

void Duty_to_CMPA(void)
{
    CMPA_a = (Uint16)(Duty_A * 3750.0f);
    CMPA_b = (Uint16)(Duty_B * 3750.0f);
    CMPA_c = (Uint16)(Duty_C * 3750.0f);

}

void FOC_OpenLoopStep(void)
{
    Clarke_Transform();
    Park_Transform();

    Vd = 0.0f;
    Vq = 3.0f;

    Inverse_Park();
    Inverse_Clarke();

    Zero_Sequence_Modulation();
    Modulation_to_Duty();
    Duty_to_CMPA();
}


