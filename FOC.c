#include "FOC.h"
#include "ADC.h"
#include "RDC.h"
#include "ePWM.h"
#include <math.h>

extern volatile Uint16 Motor_Enable;

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

volatile Uint16 CMPA_a = 3125;
volatile Uint16 CMPA_b = 3125;
volatile Uint16 CMPA_c = 3125;

volatile float theta_e = 0.0f;

// =========================================================
// Open-loop startup variables
// =========================================================
volatile float theta_res = 0.0f;       // Actual rotor electrical angle
volatile float theta_cmd = 0.0f;       // Commanded stator electrical angle
volatile float theta_error = 0.0f;     // theta_cmd - theta_res

volatile float omega_cmd = 0.0f;       // Commanded electrical angular speed

volatile Uint16 FOC_Startup_State = 0;
volatile Uint32 FOC_Startup_Count = 0;

// 20 kHz control loop
#define FOC_TS              0.00005f

// Startup states
#define FOC_STATE_ALIGN     0
#define FOC_STATE_RAMP      1

// Initial alignment
#define ALIGN_TIME          5.0f
#define ALIGN_VOLTAGE       2.0f

// Open-loop rotation
#define OPENLOOP_VQ         3.0f
#define OPENLOOP_OMEGA      50.0f
#define OPENLOOP_ACCEL      20.0f



//functions
void Clarke_Transform(void)
{
    I_alpha = Current_A;

    I_beta = (Current_A + 2.0f * Current_B)
             * 0.577350269f;
}


void Park_Transform(void)
{
    float sin_theta;
    float cos_theta;

    sin_theta = sinf(theta_res);
    cos_theta = cosf(theta_res);

    Id = I_alpha * cos_theta
       + I_beta * sin_theta;

    Iq = -I_alpha * sin_theta
       + I_beta * cos_theta;
}

void Inverse_Park(void)
{
    float sin_theta;
    float cos_theta;

    // Use commanded electrical angle for open-loop voltage vector
    sin_theta = sinf(theta_cmd);
    cos_theta = cosf(theta_cmd);

    V_alpha = Vd * cos_theta
            - Vq * sin_theta;

    V_beta  = Vd * sin_theta
            + Vq * cos_theta;
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
    CMPA_a = (Uint16)(Duty_A * 3125.0f);
    CMPA_b = (Uint16)(Duty_B * 3125.0f);
    CMPA_c = (Uint16)(Duty_C * 3125.0f);

}

void FOC_AlignmentStep(void)
{
    /*
     * Hold the commanded stator field at a fixed electrical angle.
     *
     * theta_cmd = 0 rad
     * Vd = ALIGN_VOLTAGE
     * Vq = 0
     */

    theta_cmd = 0.0f;

    Vd = ALIGN_VOLTAGE;
    Vq = 0.0f;

    FOC_Startup_Count++;

    /*
     * 20 kHz loop:
     *
     * 5.0 s × 20000 = 100000 cycles
     */
    if (FOC_Startup_Count >=
        (Uint32)(ALIGN_TIME / FOC_TS))
    {
        FOC_Startup_Count = 0;

        // Start the open-loop ramp from zero speed
        omega_cmd = 0.0f;

        // Start commanded angle from the alignment angle
        theta_cmd = 0.0f;

        FOC_Startup_State = FOC_STATE_RAMP;
    }
}


void FOC_OpenLoopRampStep(void)
{
    /*
     * Gradually increase electrical angular speed.
     */
    omega_cmd += OPENLOOP_ACCEL * FOC_TS;

    /*
     * Limit the commanded speed.
     */
    if (omega_cmd > OPENLOOP_OMEGA)
        omega_cmd = OPENLOOP_OMEGA;

    /*
     * Advance the commanded electrical angle.
     *
     * theta_cmd[k+1] = theta_cmd[k] + omega_cmd * Ts
     */
    theta_cmd += omega_cmd * FOC_TS;

    /*
     * Wrap theta_cmd to 0 ... 2*pi.
     */
    if (theta_cmd >= 6.283185307f)
        theta_cmd -= 6.283185307f;

    if (theta_cmd < 0.0f)
        theta_cmd += 6.283185307f;

    /*
     * Open-loop voltage command.
     */
    Vd = 0.0f;
    Vq = OPENLOOP_VQ;
}

void FOC_UpdateResolverAngle(void)
{
    theta_res = RDC_GetElectricalAngle();
    theta_e = theta_res;
}


void FOC_UpdateAngleError(void)
{
    theta_error = theta_cmd - theta_res;

    if (theta_error > 3.141592654f)
        theta_error -= 6.283185307f;

    if (theta_error < -3.141592654f)
        theta_error += 6.283185307f;
}


void FOC_ResetStartup(void)
{
    theta_cmd = 0.0f;
    theta_res = 0.0f;
    theta_error = 0.0f;

    omega_cmd = 0.0f;

    FOC_Startup_State = FOC_STATE_ALIGN;
    FOC_Startup_Count = 0;

    Vd = 0.0f;
    Vq = 0.0f;
}


void FOC_OpenLoopStep(void)
{
    /*
     * Always update resolver angle so that resolver
     * operation can be tested while the motor is disabled.
     */
    FOC_UpdateResolverAngle();

    /*
     * Keep startup state reset while motor is disabled.
     * Do not generate/update motor-control PWM commands.
     */
    if (Motor_Enable == 0)
    {
        FOC_ResetStartup();
        return;
    }

    /*
     * Measure currents and transform them.
     */
    Clarke_Transform();
    Park_Transform();

    /*
     * Startup state machine.
     */
    if (FOC_Startup_State == FOC_STATE_ALIGN)
    {
        FOC_AlignmentStep();
    }
    else if (FOC_Startup_State == FOC_STATE_RAMP)
    {
        FOC_OpenLoopRampStep();
    }

    /*
     * Compare commanded field angle with actual rotor angle.
     */
    FOC_UpdateAngleError();

    /*
     * Convert commanded dq voltage into alpha-beta voltage.
     */
    Inverse_Park();

    /*
     * alpha-beta -> three phase
     */
    Inverse_Clarke();

    /*
     * Common-mode / zero-sequence modulation
     */
    Zero_Sequence_Modulation();

    /*
     * Voltage -> duty
     */
    Modulation_to_Duty();

    /*
     * Duty -> PWM compare values
     */
    Duty_to_CMPA();

    /*
     * Update ePWM1/2/3
     */
    PWM_UpdateDuty(CMPA_a, CMPA_b, CMPA_c);
}




