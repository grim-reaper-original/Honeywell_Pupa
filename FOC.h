#ifndef FOC_H
#define FOC_H

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

#define TWO_PI                 6.28318530717958647692f

#define FOC_MODE_IDLE          0U
#define FOC_MODE_CALIBRATION   1U
#define FOC_MODE_CLOSED_LOOP   2U
#define FOC_MODE_FAULT         3U
#define PWM_PERIOD          3125U
#define PWM_HALF            1562U
#define MOTOR_POLE_PAIRS       2.0f
#define RESOLVER_POLE_PAIRS    2.0f

#define CURRENT_TRIP_A         6.0f
#define CURRENT_PI_KP          0.675f
#define CURRENT_PI_KI          741.0f
#define CAL_ALIGN_VOLTAGE      0.18f
#define SPEED_RAMP_RPM_PER_S   1500.0f
#define FOC_TS                 (1.0f / 20000.0f)

#define FOC_FAULT_OVERCURRENT  0x0001U
#define FOC_FAULT_UNDERVOLTAGE 0x0002U
#define FOC_FAULT_CALIBRATION  0x0004U
#define FOC_FAULT_RESOLVER     0x0008U


extern volatile float Target_RPM;
extern volatile float Measured_RPM;

extern volatile float VDC;

extern volatile float Id;
extern volatile float Iq;

extern volatile float Vd;
extern volatile float Vq;

extern volatile float V_alpha;
extern volatile float V_beta;

extern volatile float Va;
extern volatile float Vb;
extern volatile float Vc;

extern volatile float V_offset;

extern volatile float Va_mod;
extern volatile float Vb_mod;
extern volatile float Vc_mod;

extern volatile float V_max;
extern volatile float V_min;

extern volatile float Duty_A;
extern volatile float Duty_B;
extern volatile float Duty_C;

extern volatile Uint16 CMPA_a;
extern volatile Uint16 CMPA_b;
extern volatile Uint16 CMPA_c;

extern volatile float theta_e;
extern volatile float theta_res;
extern volatile float theta_cmd;
extern volatile float theta_error;
extern volatile float omega_e;

extern volatile Uint16 FOC_Mode;
extern volatile Uint16 FOC_Fault;
extern volatile Uint16 Angle_Calibration_Request;
extern volatile Uint16 Angle_Calibrated;

void FOC_UpdateResolverAngle(void);
void FOC_ResetStartup(void);
void FOC_ResetFault(void);

void Clarke_Transform(void);
void Park_Transform(void);
void Inverse_Park_Angle(float theta);
void Inverse_Park(void);
void Inverse_Clarke(void);
void Zero_Sequence_Modulation(void);
void Modulation_to_Duty(void);
void Duty_to_CMPA(void);
void FOC_OpenLoopStep(void);

#endif
