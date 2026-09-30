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
volatile float Vq = 0.0f;

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
#define ALIGN_TIME          2.0f
#define ALIGN_VOLTAGE       1.0f

// Open-loop V/f parameters
volatile float Target_RPM = 1500.0f;       // Set your desired open-loop speed here
volatile float Measured_RPM = 0.0f;        // Measured speed in RPM (watch this in CCS)
volatile float theta_res_prev = 0.0f;      // Previous angle for velocity math

#define OPENLOOP_ACCEL_RPM  500.0f         // Acceleration rate (RPM/second)
#define VOLTS_PER_RPM       0.001575f        // V/f scalar to overcome Back-EMF
#define MIN_VQ_VOLTS         1.0f           // Minimum voltage at zero speed



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
    // 1. Convert Target_RPM to electrical rad/s (for 1 Pole Pair)
    float target_omega = Target_RPM * 0.104719755f * (float)Motor_PolePairs;

    // 2. Gradually ramp omega_cmd up to target_omega
    if (omega_cmd < target_omega)
    {
        omega_cmd += (OPENLOOP_ACCEL_RPM * 0.104719755f) * FOC_TS;
        if (omega_cmd > target_omega) omega_cmd = target_omega;
    }
    else if (omega_cmd > target_omega)
    {
        omega_cmd -= (OPENLOOP_ACCEL_RPM * 0.104719755f) * FOC_TS;
        if (omega_cmd < target_omega) omega_cmd = target_omega;
    }

    // 3. Advance commanded electrical angle
    theta_cmd += omega_cmd * FOC_TS;

    // 4. Wrap theta_cmd to 0 ... 2*pi
    if (theta_cmd >= 6.283185307f) theta_cmd -= 6.283185307f;
    if (theta_cmd < 0.0f)          theta_cmd += 6.283185307f;

    // 5. V/f Control: Automatically scale Vq with speed to overcome Back-EMF
    float current_commanded_rpm = omega_cmd * 9.54929658f;
    Vd = 0.0f;
    Vq = current_commanded_rpm * VOLTS_PER_RPM;

    // Maintain minimum holding voltage at low speed
    if (Vq < MIN_VQ_VOLTS)
    {
        Vq = MIN_VQ_VOLTS;
    }

    // Clamp Vq to safe maximum duty limit
    if (Vq > (VDC * 0.577f))
    {
        Vq = VDC * 0.577f;
    }
}

void FOC_UpdateResolverAngle(void)
{
    theta_res = RDC_GetElectricalAngle();
    theta_e = theta_res;

    // --- Calculate Actual Velocity in RPM ---
    float delta_res = theta_res - theta_res_prev;

    // Handle 0 to 2*PI boundary wrapping
    if (delta_res < -3.141592654f) delta_res += 6.283185307f;
    if (delta_res >  3.141592654f) delta_res -= 6.283185307f;

    // Raw RPM calculation (20 kHz ISR -> FOC_TS = 50us)
    float raw_rpm = (delta_res / FOC_TS) * (9.54929658f / Motor_PolePairs);

    // 100 Hz Low-Pass Filter (Prevents jittery numbers in CCS Expressions window)
    Measured_RPM = (Measured_RPM * 0.95f) + (raw_rpm * 0.05f);

    // Save previous angle
    theta_res_prev = theta_res;
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




