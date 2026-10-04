#include "FOC.h"
#include "ADC.h"
#include "RDC.h"
#include "ePWM.h"
#include <math.h>

//General constants
#define PI                  3.14159265358979323846f
#define TWO_PI              6.28318530717958647692f

#define MOTOR_POLE_PAIRS    2.0f

#define MIN_DC_BUS_V        18.5f
#define NOMINAL_DC_BUS_V    28.0f

// Startup states
#define FOC_STATE_ALIGN     0
#define FOC_STATE_RAMP      1

// Initial alignment
#define ALIGN_TIME          2.0f
#define ALIGN_VOLTAGE       1.0f

// 20 kHz control loop
#define FOC_TS              0.00005f

// Open-loop V/f parameters
volatile float Target_RPM = 1500.0f;       // Set your desired open-loop speed here
volatile float Ramped_Target_RPM = 0.0f;
volatile float Measured_RPM = 0.0f;        // Measured speed in RPM (watch this in CCS)
volatile float theta_res_prev = 0.0f;      // Previous angle for velocity math

#define OPENLOOP_ACCEL_RPM  500.0f         // Acceleration rate (RPM/second)
#define VOLTS_PER_RPM       0.001575f        // V/f scalar to overcome Back-EMF

// Calibration Constants
#define CAL_ALIGN_VOLTAGE       0.50f
#define CAL_ALIGN_TIME_S        0.50f
#define CAL_CAPTURE_SAMPLES     400U
#define CAL_CURRENT_LIMIT_A     2.0f

#define FOC_MODE_IDLE          0U
#define FOC_MODE_CALIBRATION  1U
#define FOC_MODE_CLOSED_LOOP  2U



extern volatile Uint16 Motor_Enable;

volatile float VDC = NOMINAL_DC_BUS_V;
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

volatile Uint16 CMPA_a = 1562;
volatile Uint16 CMPA_b = 1562;
volatile Uint16 CMPA_c = 1562;

volatile float theta_e = 0.0f;

// =========================================================
//  startup variables
// =========================================================
volatile float theta_res = 0.0f;       // Actual rotor electrical angle
volatile float theta_cmd = 0.0f;       // Commanded stator electrical angle
volatile float theta_error = 0.0f;     // theta_cmd - theta_res
volatile float theta_res_prev = 0.0f;
volatile float omega_e = 0.0f;

volatile float Measured_RPM = 0.0f;
volatile float Measured_Electrical_RPM = 0.0f;

volatile float omega_cmd = 0.0f;       // Commanded electrical angular speed

volatile Uint16 FOC_Startup_State = 0;
volatile Uint32 FOC_Startup_Count = 0;

static Uint16 Cal_State = 0;
static Uint32 Cal_Count = 0;
static Uint16 Cal_SampleCount = 0;

static float Cal_SinSum = 0.0f;
static float Cal_CosSum = 0.0f;

volatile float Cal_Raw_Electrical_Angle = 0.0f;
volatile float Cal_Calculated_Offset = 0.0f;
volatile float Cal_Average_Iq = 0.0f;

volatile Uint16 Angle_Calibration_Request = 0;

volatile Uint16 FOC_Mode = FOC_MODE_IDLE;






//functions

static float clampf_local(float x, float lo, float hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}


static float wrap_angle(float x)
{
    while (x >= TWO_PI_F) x -= TWO_PI_F;
    while (x < 0.0f) x += TWO_PI_F;
    return x;
}

static float wrap_angle_signed(float x)
{
    while (x > PI_F) x -= TWO_PI_F;
    while (x < -PI_F) x += TWO_PI_F;
    return x;
}



void Clarke_Transform(void)
{
    I_alpha = Current_A;

    I_beta = (Current_A + 2.0f * Current_B)
             * 0.577350269f;
}


void Park_Transform(void)
{
    float s = sinf(theta_res);
    float c = cosf(theta_res);

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
    sin_theta = sinf(theta_res);
    cos_theta = cosf(theta_res);

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

    Duty_A = clampf_local(Duty_A, 0.02f, 0.98f);
    Duty_B = clampf_local(Duty_B, 0.02f, 0.98f);
    Duty_C = clampf_local(Duty_C, 0.02f, 0.98f);
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


static void FOC_AngleCalibrationStep(void)
{
    float theta_mech;
    float theta_elec_raw;

    if (Cal_State == 0)
    {
        Cal_Count = 0;
        Cal_SampleCount = 0;

        Cal_SinSum = 0.0f;
        Cal_CosSum = 0.0f;
        Cal_Average_Iq = 0.0f;

        theta_cmd = 0.0f;

        Vd = CAL_ALIGN_VOLTAGE;
        Vq = 0.0f;

        Cal_State = 1;
        return;
    }

    if (Cal_State == 1)
    {
        theta_cmd = 0.0f;

        Vd = CAL_ALIGN_VOLTAGE;
        Vq = 0.0f;

        Cal_Count++;

        if (sqrtf(Id * Id + Iq * Iq) > CAL_CURRENT_LIMIT_A)
        {
            Vd = 0.0f;
            Vq = 0.0f;

            PWM_UpdateDuty(1562, 1562, 1562);

            Cal_State = 0;
            Angle_Calibration_Request = 0;
            Motor_Enable = 0;

            return;
        }

        if (Cal_Count >=
            (Uint32)(CAL_ALIGN_TIME_S / FOC_TS))
        {
            Cal_Count = 0;
            Cal_State = 2;
        }

        return;
    }

    if (Cal_State == 2)
    {
        float mean_angle;
        float offset;

        theta_mech = RDC_GetMechanicalAngle();

        theta_elec_raw =
            wrap_angle(theta_mech * MOTOR_POLE_PAIRS);

        Cal_SinSum += sinf(theta_elec_raw);
        Cal_CosSum += cosf(theta_elec_raw);

        Cal_Average_Iq += Iq;

        Cal_SampleCount++;

        if (Cal_SampleCount >= CAL_CAPTURE_SAMPLES)
        {
            mean_angle =
                atan2f(Cal_SinSum, Cal_CosSum);

            if (mean_angle < 0.0f)
                mean_angle += TWO_PI_F;

            offset = wrap_angle(-mean_angle);

            Cal_Raw_Electrical_Angle = mean_angle;
            Cal_Calculated_Offset = offset;

            Cal_Average_Iq /=
                (float)CAL_CAPTURE_SAMPLES;

            Angle_Offset = offset;

            Vd = 0.0f;
            Vq = 0.0f;

            PWM_UpdateDuty(1562, 1562, 1562);

            Cal_State = 0;
            Angle_Calibration_Request = 0;
            Motor_Enable = 0;
        }
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

        omega_e =
            wrap_angle_signed(theta_res - theta_res_prev)
            / FOC_TS;

        Measured_Electrical_RPM =
            omega_e * 9.54929658f;

        Measured_RPM =
            (Measured_RPM * 0.95f) +
            ((Measured_Electrical_RPM / MOTOR_POLE_PAIRS) * 0.05f);

        theta_res_prev = theta_res;
}


void FOC_UpdateAngleError(void)
{
    theta_error =
        wrap_angle_signed(theta_cmd - theta_res);
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




