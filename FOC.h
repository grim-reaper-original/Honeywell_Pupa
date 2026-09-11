#ifndef FOC_H
#define FOC_H

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

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

extern volatile float Duty_A;
extern volatile float Duty_B;
extern volatile float Duty_C;

extern volatile Uint16 CMPA_a;
extern volatile Uint16 CMPA_b;
extern volatile Uint16 CMPA_c;


void Clarke_Transform(void);
void Park_Transform(void);
void Inverse_Park(void);
void Inverse_Clarke(void);
void Zero_Sequence_Modulation(void);
void Modulation_to_Duty(void);
void Duty_to_CMPA(void);

#endif
