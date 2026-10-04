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


//current control constants
#define CURRENT_LIMIT_A      5.0f
#define CURRENT_TRIP_A       8.0f

#define CURRENT_PI_KP        1.3509f
#define CURRENT_PI_KI        1482.83f

// Speed control constants
#define TARGET_RPM              1500.0f    //desired mechanical rotor speed.
#define SPEED_LOOP_DIVIDER     20U         //speed PI executes every 1kHz
#define SPEED_PI_KP            0.0020f
#define SPEED_PI_KI            0.0200f


// 20 kHz control loop
#define FOC_TS              0.00005f




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

//current references
volatile float Id_ref = 0.0f;
volatile float Iq_ref = 0.0f;


//integrators
static float Id_integrator = 0.0f;
static float Iq_integrator = 0.0f;

// Speed controller
volatile float Target_RPM = TARGET_RPM;
static float Speed_integrator = 0.0f;
static Uint16 Speed_Loop_Count = 0;



// =========================================================
// Resolver feedback variables
// =========================================================
volatile float theta_res = 0.0f;       // Actual rotor electrical angle
volatile float theta_cmd = 0.0f;       // Commanded stator electrical angle
volatile float theta_error = 0.0f;     // theta_cmd - theta_res
volatile float theta_res_prev = 0.0f;
volatile float omega_e = 0.0f;

volatile float Measured_RPM = 0.0f;
volatile float Measured_Electrical_RPM = 0.0f;


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
    while (x >= TWO_PI) x -= TWO_PI;
    while (x < 0.0f) x += TWO_PI;
    return x;
}

static float wrap_angle_signed(float x)
{
    while (x > PI) x -= TWO_PI;
    while (x < -PI) x += TWO_PI;
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
    float sin_theta = sinf(theta_res);
    float cos_theta = cosf(theta_res);

    Id = I_alpha * cos_theta
       + I_beta * sin_theta;

    Iq = -I_alpha * sin_theta
       + I_beta * cos_theta;
}

void Inverse_Park(void)
{
    float sin_theta;
    float cos_theta;

    // Use measured rotor electrical angle
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

static float AvailableVoltage(void)
{
    float vmax = VDC * 0.577350269f;

    if (vmax < 0.0f)
        vmax = 0.0f;

    return vmax;
}

static void LimitVoltageVector(void)
{
    float vmax = AvailableVoltage();
    float mag = sqrtf(Vd * Vd + Vq * Vq);

    if (mag > vmax && mag > 0.001f)
    {
        float scale = vmax / mag;

        Vd *= scale;
        Vq *= scale;
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
                mean_angle += TWO_PI;

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

static float CurrentPI_Update(
    float error,
    float *integrator,
    float kp,
    float ki,
    float output_limit)
{
    float proportional = kp * error;
    float output;

    output = proportional + *integrator;

    if (output < output_limit &&
        output > -output_limit)
    {
        *integrator +=
            ki * error * FOC_TS;
    }
    else if (output >= output_limit &&
             error < 0.0f)
    {
        *integrator +=
            ki * error * FOC_TS;
    }
    else if (output <= -output_limit &&
             error > 0.0f)
    {
        *integrator +=
            ki * error * FOC_TS;
    }

    *integrator = clampf_local(
        *integrator,
        -output_limit,
        output_limit);

    output = proportional + *integrator;

    return clampf_local(
        output,
        -output_limit,
        output_limit);
}

static float SpeedPI_Update(
    float speed_error)
{
    float proportional;
    float output;

    proportional = SPEED_PI_KP * speed_error;

    output = proportional + Speed_integrator;

    if (output < CURRENT_LIMIT_A &&
        output > -CURRENT_LIMIT_A)
    {
        Speed_integrator +=
            SPEED_PI_KI * speed_error * (FOC_TS * SPEED_LOOP_DIVIDER);
    }
    else if (output >= CURRENT_LIMIT_A &&
             speed_error < 0.0f)
    {
        Speed_integrator +=
            SPEED_PI_KI * speed_error * (FOC_TS * SPEED_LOOP_DIVIDER);
    }
    else if (output <= -CURRENT_LIMIT_A &&
             speed_error > 0.0f)
    {
        Speed_integrator +=
            SPEED_PI_KI * speed_error * (FOC_TS * SPEED_LOOP_DIVIDER);
    }

    Speed_integrator = clampf_local(
        Speed_integrator,
        -CURRENT_LIMIT_A,
        CURRENT_LIMIT_A);

    output = proportional + Speed_integrator;

    return clampf_local(
        output,
        -CURRENT_LIMIT_A,
        CURRENT_LIMIT_A);
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



void FOC_ResetStartup(void)
{
    theta_cmd = 0.0f;
    theta_error = 0.0f;

    Vd = 0.0f;
    Vq = 0.0f;

    Id_ref = 0.0f;
    Iq_ref = 0.0f;

    Id_integrator = 0.0f;
    Iq_integrator = 0.0f;

    Speed_integrator = 0.0f;
    Speed_Loop_Count = 0;
}


void FOC_OpenLoopStep(void)
{
    /*
     * Always update resolver angle.
     */
    FOC_UpdateResolverAngle();

    /*
     * Keep controller inactive while motor is disabled.
     */
    if (Motor_Enable == 0)
        {
            Vd = 0.0f;
            Vq = 0.0f;

            Id_ref = 0.0f;
            Iq_ref = 0.0f;

            Id_integrator = 0.0f;
            Iq_integrator = 0.0f;

            Speed_integrator = 0.0f;
            Speed_Loop_Count = 0;

            return;
        }

    /*
     * Measure phase currents and transform them
     * into the rotating d-q reference frame.
     */
    Clarke_Transform();
    Park_Transform();

    /*
         * Outer speed loop.
         *
         * Speed PI runs at 1 kHz while the
         * current loop continues at 20 kHz.
         */
    Speed_Loop_Count++;

    if (Speed_Loop_Count >= SPEED_LOOP_DIVIDER)
        {
            float speed_error;

            Speed_Loop_Count = 0;

            speed_error =
                Target_RPM - Measured_RPM;

            Iq_ref = SpeedPI_Update(speed_error);
        }

        Id_ref = 0.0f;

    /*
     * Current PI controllers.
     *
     * Convert current errors into d-q voltage commands.
     */
    {
        float voltage_limit = AvailableVoltage();

        Vd = CurrentPI_Update(
            Id_ref - Id,
            &Id_integrator,
            CURRENT_PI_KP,
            CURRENT_PI_KI,
            voltage_limit);

        Vq = CurrentPI_Update(
            Iq_ref - Iq,
            &Iq_integrator,
            CURRENT_PI_KP,
            CURRENT_PI_KI,
            voltage_limit);
    }

    /*
     * Limit total d-q voltage vector.
     */
    LimitVoltageVector();

    /*
     * d-q -> alpha-beta
     */
    Inverse_Park();

    /*
     * alpha-beta -> three-phase
     */
    Inverse_Clarke();

    /*
     * Common-mode / zero-sequence modulation
     */
    Zero_Sequence_Modulation();

    /*
     * Phase voltage -> duty ratio
     */
    Modulation_to_Duty();

    /*
     * Duty ratio -> PWM compare values
     */
    Duty_to_CMPA();

    /*
     * Update ePWM1/2/3.
     */
    PWM_UpdateDuty(CMPA_a, CMPA_b, CMPA_c);
}




