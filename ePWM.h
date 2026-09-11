#ifndef PWM_H
#define PWM_H

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

// =========================================================
// Public PWM Functions
// =========================================================

// Configures the 3-phase FOC ePWMs (20 kHz, center-aligned, 1us deadband)
void Init_ePWM_MotorControl(void);

// Hardware Safety Functions
void PWM_ForceTripZone(void);
void PWM_ClearTripZone(void);
void PWM_UpdateDuty(Uint16 cmp_a, Uint16 cmp_b, Uint16 cmp_c);

#endif // PWM_H
