#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include <math.h>
#include "ADC.c"
#include "RDC.c"

// ---------------------------------------------------------
// RESOLVER HARDWARE PIN MAPPINGS (Confirmed from Schematic)
// ---------------------------------------------------------

// ---------------------------------------------------------
// GATE DRIVER HARDWARE PIN MAPPINGS
// ---------------------------------------------------------
// SPI-B Chip Select (GPIO27 - Port A)
#define DRV_CS_LOW()      (GpioDataRegs.GPACLEAR.bit.GPIO27 = 1)
#define DRV_CS_HIGH()     (GpioDataRegs.GPASET.bit.GPIO27 = 1)

// DRV8323 Calibration Pin (GPIO38 - Port B)
#define DRV_CAL_LOW()     (GpioDataRegs.GPBCLEAR.bit.GPIO38 = 1)
#define DRV_CAL_HIGH()    (GpioDataRegs.GPBSET.bit.GPIO38 = 1)

// DRV8323 Enable Pin (GPIO39 - Port B)
#define DRV_ENABLE_LOW()  (GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1)
#define DRV_ENABLE_HIGH() (GpioDataRegs.GPBSET.bit.GPIO39 = 1)

// ---------------------------------------------------------
// MATH & MOTOR CONSTANTS
// ---------------------------------------------------------
#define MOTOR_POLE_PAIRS      1
#define ONE_DIVIDED_BY_SQRT3  0.57735026919f
#define CURRENT_GAIN          0.003222656f
#define RAD_PER_TICK          0.00009587379f

// =========================================================
// FIELD ORIENTED CONTROL (FOC) STRUCTURES
// =========================================================
typedef struct {
    float As;
    float Bs;
    float Cs;
    float Alpha;
    float Beta;
} CLARKE_T;

typedef struct {
    float Alpha;
    float Beta;
    float Sine;
    float Cosine;
    float Ds;
    float Qs;
} PARK_T;

typedef struct {
    float Ds;
    float Qs;
    float Sine;
    float Cosine;
    float Alpha;
    float Beta;
} IPARK_T;

typedef struct {
    float Alpha;
    float Beta;
    float Va;
    float Vb;
    float Vc;
} INV_CLARKE_T;

typedef struct {
    float Ref;
    float Fbk;
    float Err;
    float Kp;
    float Ki;
    float Umax;
    float Umin;
    float Ui;
    float Out;
} PI_CONTROLLER_T;

// =========================================================
// FUNCTION PROTOTYPES
// =========================================================
void Init_ADC_CurrentSensors(void);
void Init_SPI_GateDriver(void);
void Init_ePWM_MotorControl(void);
void Init_PI_Controllers(void);
void Calc_InvClarke(INV_CLARKE_T *v);
void Read_Resolver_Data(void);
Uint16 SPI_ReadWrite_16(Uint16 tx_data);
void DRV8323_WakeUp(void);
void DRV8323_WriteRegister(Uint16 address, Uint16 data);
Uint16 DRV8323_ReadRegister(Uint16 address);
void DRV8323_Init_GateDriveStrength(void);
void DRV8323_Init_OCP(void);
Uint16 SPI_B_ReadWrite_16(Uint16 tx_data);
void Calc_Clarke(CLARKE_T *v);
void Calc_Park(PARK_T *v);
void Calc_InvPark(IPARK_T *v);
void Calc_PI(PI_CONTROLLER_T *v);
__interrupt void adc_isr(void);

// =========================================================
// GLOBAL VARIABLES
// =========================================================

Uint16 GD_Test_Readback = 0;
Uint16 GD_Fault_Status = 0;


CLARKE_T clarke_calc;
PARK_T park_calc;
IPARK_T ipark_calc;
INV_CLARKE_T inv_clarke_calc;
PI_CONTROLLER_T pi_id;
PI_CONTROLLER_T pi_iq;

// STATE MACHINE VARIABLES ---
Uint16 System_State = 0;      // 0 = IDLE, 1 = ALIGN, 2 = RUN
Uint32 State_Timer = 0;       // Counts ISR ticks (50us each)
Uint16 Resolver_Offset = 0;   // The mechanical zero-degree offset

// --- SPEED CONTROLLER ---
PI_CONTROLLER_T pi_speed;     // Outer Speed Loop
PI_CONTROLLER_T pi_id;        // Inner Flux Loop
PI_CONTROLLER_T pi_iq;        // Inner Torque Loop



// =========================================================
// MAIN PROGRAM
// =========================================================
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
    DRV8323_WakeUp();
    DRV8323_Init_GateDriveStrength();
    DRV8323_Init_OCP();

    GD_Test_Readback = DRV8323_ReadRegister(0x03);
    GD_Fault_Status = DRV8323_ReadRegister(0x00);

    Init_PI_Controllers();
    Init_ePWM_MotorControl();

    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    IER |= M_INT1;
    EINT;
    ERTM;

    while(1)
    {
        if (READ_RDC_DOS() == 0 || READ_RDC_LOT() == 0)
        {
            RDC_Fault_Flag = 1;
            System_State = 0; // Drop to Idle/Safe state

            // TODO (Milestone 10):
            // 1. Force ePWMs to 0% duty cycle via Trip Zone
            // 2. Pull Gate Driver ENABLE pin low
        }
        else
        {
            RDC_Fault_Flag = 0;
        }
    }
}

// =========================================================
// INTERRUPT SERVICE ROUTINES (Runs at 20 kHz)
// =========================================================
__interrupt void adc_isr(void)
{
    // --- 1. SENSOR FEEDBACK ---
    Raw_Current_A = (AdcRegs.ADCRESULT0 >> 4);
    Raw_Current_B = (AdcRegs.ADCRESULT1 >> 4);

    clarke_calc.As = ((float)Raw_Current_A - Offset_Current_A) * CURRENT_GAIN;
    clarke_calc.Bs = ((float)Raw_Current_B - Offset_Current_B) * CURRENT_GAIN;
    clarke_calc.Cs = -(clarke_calc.As+clarke_calc.Bs);

    Read_Resolver_Data();
    Uint16 compensated_angle = Rotor_Angle_Raw - Resolver_Offset;
    Rotor_Angle_Elec = compensated_angle * MOTOR_POLE_PAIRS;

    float angle_rad = (float)Rotor_Angle_Elec * RAD_PER_TICK;
    park_calc.Sine   = sinf(angle_rad);
    park_calc.Cosine = cosf(angle_rad);

    // --- 2. FORWARD TRANSFORMS (AC to DC) ---
    Calc_Clarke(&clarke_calc);

    park_calc.Alpha = clarke_calc.Alpha;
    park_calc.Beta  = clarke_calc.Beta;
    Calc_Park(&park_calc);

    // --- 3. PI CURRENT CONTROLLERS ---
    pi_id.Fbk = park_calc.Ds;
    pi_iq.Fbk = park_calc.Qs;

    static Uint16 speed_loop_prescaler = 0;

        switch(System_State)
        {
            case 0: // IDLE (Safe State)
                ipark_calc.Ds = 0.0f;
                ipark_calc.Qs = 0.0f;
                pi_id.Ui = 0.0f;
                pi_iq.Ui = 0.0f;
                pi_speed.Ui = 0.0f;
                State_Timer = 0;
                break;

            case 1: // ALIGN (Inject DC current to lock rotor to 0 degrees)
                // Command 2.0 Amps on the D-axis, 0 Amps on Q-axis
                pi_id.Ref = 2.0f;
                pi_iq.Ref = 0.0f;

                Calc_PI(&pi_id);
                Calc_PI(&pi_iq);

                ipark_calc.Ds = pi_id.Out;
                ipark_calc.Qs = pi_iq.Out;

                // Force the math angle to 0 so the magnetic field freezes in place
                park_calc.Sine = 0.0f;
                park_calc.Cosine = 1.0f;

                State_Timer++;
                // Wait 1 second (20,000 ticks at 50us) for rotor to settle physically
                if (State_Timer > 20000)
                {
                    Resolver_Offset = Rotor_Angle_Raw; // Lock in the calibration
                    System_State = 2;                  // Transition to RUN
                }
                break;

            case 2: // RUN (Full Closed-Loop FOC)
                // 1. Run Outer Speed Loop (at 2 kHz)
                if (++speed_loop_prescaler >= 10)
                {
                    speed_loop_prescaler = 0;
                    pi_speed.Fbk = (float)Rotor_Velocity_Raw;
                    Calc_PI(&pi_speed);
                }

                // 2. Output of Speed Loop becomes Input of Torque Loop
                pi_iq.Ref = pi_speed.Out;
                pi_id.Ref = 0.0f; // Always 0 Amps for surface-mount BLDCs

                // 3. Run Inner Current Loops (at 20 kHz)
                Calc_PI(&pi_id);
                Calc_PI(&pi_iq);

                ipark_calc.Ds = pi_id.Out;
                ipark_calc.Qs = pi_iq.Out;
                break;
        }

    // --- 4. REVERSE TRANSFORMS (DC to AC) ---
    ipark_calc.Sine = park_calc.Sine;
    ipark_calc.Cosine = park_calc.Cosine;
    Calc_InvPark(&ipark_calc);

    // --- 5. DUTY CYCLE GENERATION ---
    EPwm1Regs.CMPA.half.CMPA = (Uint16)(1875.0f + (inv_clarke_calc.Va * 1875.0f));
    EPwm2Regs.CMPA.half.CMPA = (Uint16)(1875.0f + (inv_clarke_calc.Vb * 1875.0f));
    EPwm3Regs.CMPA.half.CMPA = (Uint16)(1875.0f + (inv_clarke_calc.Vc * 1875.0f));

    // --- 6. CLEAR INTERRUPT FLAGS ---
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

// =========================================================
// HARDWARE INITIALIZATION FUNCTIONS
// =========================================================


void Init_PI_Controllers(void)
{
    pi_id.Ref = 0.0f;
    pi_id.Fbk = 0.0f;
    pi_id.Err = 0.0f;
    pi_id.Ui  = 0.0f;
    pi_id.Kp  = 0.5f;
    pi_id.Ki  = 0.01f;
    pi_id.Umax = 1.0f;
    pi_id.Umin = -1.0f;

    pi_iq.Ref = 0.0f;
    pi_iq.Fbk = 0.0f;
    pi_iq.Err = 0.0f;
    pi_iq.Ui  = 0.0f;
    pi_iq.Kp  = 0.5f;
    pi_iq.Ki  = 0.01f;
    pi_iq.Umax = 1.0f;
    pi_iq.Umin = -1.0f;

    // Speed Controller (Regulates RPM, Outputs Target Amps to pi_iq)
    pi_speed.Ref = 0.0f;       // Target Speed in RPM
    pi_speed.Fbk = 0.0f;
    pi_speed.Err = 0.0f;
    pi_speed.Ui  = 0.0f;
    pi_speed.Kp  = 0.05f;      // Placeholder speed gain
    pi_speed.Ki  = 0.001f;     // Placeholder speed gain
    pi_speed.Umax = 10.0f;     // Max current command (+10 Amps)
    pi_speed.Umin = -10.0f;    // Min current command (-10 Amps)


}

void Init_ePWM_MotorControl(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.all &= ~0x0000003F;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;

    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    // Master ePWM 1
    EPwm1Regs.TBPRD = 3750;
    EPwm1Regs.TBPHS.half.TBPHS = 0; //  phase register TBPHS = 0
    EPwm1Regs.TBCTL.bit.CTRMODE = 2; // up down counting
    EPwm1Regs.TBCTL.bit.PHSEN = 0;  // no phase loading for pwm1, making it master
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 1;  //synchronisation select such that TBCTR = 0x0000
    EPwm1Regs.CMPA.half.CMPA = 1875; //sets initial duty cycle to 50%
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0; //enable shadow registers
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 0; //
    EPwm1Regs.AQCTLA.bit.CAU = 2;
    EPwm1Regs.AQCTLA.bit.CAD = 1;
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm1Regs.DBCTL.bit.POLSEL = 2;
    EPwm1Regs.DBRED = 150;  // 150*6.67ns = 1us delay
    EPwm1Regs.DBFED = 150;
    EPwm1Regs.TZCTL.bit.TZA = 2;
    EPwm1Regs.TZCTL.bit.TZB = 2;
    EPwm1Regs.ETSEL.bit.SOCAEN = 1; //enable start of conversion A
    EPwm1Regs.ETSEL.bit.SOCASEL = 1; //enable event TBCTR = 0
    EPwm1Regs.ETPS.bit.SOCAPRD = 1; //generate SOCA pulse on first SOCA event

    // Slave ePWM 2
    EPwm2Regs.TBPRD = 3750;
    EPwm2Regs.TBPHS.half.TBPHS = 0;
    EPwm2Regs.TBCTL.bit.CTRMODE = 2;
    EPwm2Regs.TBCTL.bit.PHSEN = 1;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = 0;
    EPwm2Regs.CMPA.half.CMPA = 1875;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;
    EPwm2Regs.AQCTLA.bit.CAU = 2;
    EPwm2Regs.AQCTLA.bit.CAD = 1;
    EPwm2Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm2Regs.DBCTL.bit.POLSEL = 2;
    EPwm2Regs.DBRED = 150;
    EPwm2Regs.DBFED = 150;
    EPwm2Regs.TZCTL.bit.TZA = 2;
    EPwm2Regs.TZCTL.bit.TZB = 2;

    // Slave ePWM 3
    EPwm3Regs.TBPRD = 3750;
    EPwm3Regs.TBPHS.half.TBPHS = 0;
    EPwm3Regs.TBCTL.bit.CTRMODE = 2;
    EPwm3Regs.TBCTL.bit.PHSEN = 1;
    EPwm3Regs.CMPA.half.CMPA = 1875;
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = 0;
    EPwm3Regs.AQCTLA.bit.CAU = 2;
    EPwm3Regs.AQCTLA.bit.CAD = 1;
    EPwm3Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm3Regs.DBCTL.bit.POLSEL = 2;
    EPwm3Regs.DBRED = 150;
    EPwm3Regs.DBFED = 150;
    EPwm3Regs.TZCTL.bit.TZA = 2;
    EPwm3Regs.TZCTL.bit.TZB = 2;

    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

void Init_SPI_GateDriver(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.all &= ~0x07000000;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 3;
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 3;
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 3;
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 3;
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO38 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;
    EDIS;

    DRV_CS_HIGH();
    DRV_CAL_LOW();
    DRV_ENABLE_LOW();

    McbspbRegs.SPCR2.all = 0x0000;
    McbspbRegs.SPCR1.all = 0x0000;
    McbspbRegs.SPCR1.bit.CLKSTP = 3;
    McbspbRegs.PCR.all = 0x0F08;
    McbspbRegs.PCR.bit.CLKXP = 0;
    McbspbRegs.PCR.bit.CLKRP = 0;
    McbspbRegs.RCR1.bit.RWDLEN1 = 2;
    McbspbRegs.XCR1.bit.XWDLEN1 = 2;
    McbspbRegs.SRGR2.bit.CLKSM = 1;
    McbspbRegs.SRGR1.bit.CLKGDV = 36;
    McbspbRegs.SPCR2.bit.GRST = 1;
    DELAY_US(10);
    McbspbRegs.SPCR2.bit.XRST = 1;
    McbspbRegs.SPCR1.bit.RRST = 1;
    McbspbRegs.SPCR2.bit.FRST = 1;
}



Uint16 SPI_B_ReadWrite_16(Uint16 tx_data)
{
    while(McbspbRegs.SPCR2.bit.XRDY == 0) { }
    DRV_CS_LOW();
    McbspbRegs.DXR1.all = tx_data;
    while(McbspbRegs.SPCR1.bit.RRDY == 0) { }
    Uint16 rx_data = McbspbRegs.DRR1.all;
    DRV_CS_HIGH();
    return rx_data;
}



void DRV8323_WakeUp(void)
{
    DRV_ENABLE_HIGH();
    DELAY_US(2000);
}

void DRV8323_WriteRegister(Uint16 address, Uint16 data)
{
    Uint16 payload = (address << 11) | (data & 0x07FF);
    SPI_B_ReadWrite_16(payload);
}

Uint16 DRV8323_ReadRegister(Uint16 address)
{
    Uint16 tx_payload = 0x8000 | (address << 11);
    Uint16 rx_payload = SPI_B_ReadWrite_16(tx_payload);
    return (rx_payload & 0x07FF);
}

void DRV8323_Init_GateDriveStrength(void)
{
    DRV8323_WriteRegister(0x02, 0x0344);
    DRV8323_WriteRegister(0x03, 0x0344);
}

void DRV8323_Init_OCP(void)
{
    DRV8323_WriteRegister(0x05, 0x0159);
}

void Calc_Clarke(CLARKE_T *v)
{
    v->Alpha = v->As;
    v->Beta = (v->As + 2.0f * v->Bs) * ONE_DIVIDED_BY_SQRT3;
}

void Calc_Park(PARK_T *v)
{
    v->Ds = (v->Alpha * v->Cosine) + (v->Beta * v->Sine);
    v->Qs = (v->Beta * v->Cosine) - (v->Alpha * v->Sine);
}

void Calc_InvPark(IPARK_T *v)
{
    v->Alpha = (v->Ds * v->Cosine) - (v->Qs * v->Sine);
    v->Beta  = (v->Qs * v->Cosine) + (v->Ds * v->Sine);
}

void Calc_InvClarke(INV_CLARKE_T *v)
{
    v->Va = v->Alpha;
    v->Vb = -0.5f * v->Alpha + 0.8660254f * v->Beta;
    v->Vc = -0.5f * v->Alpha - 0.8660254f * v->Beta;
}

void Calc_PI(PI_CONTROLLER_T *v)
{
    v->Err = v->Ref - v->Fbk;
    float Up = v->Kp * v->Err;
    v->Ui = v->Ui + (v->Ki * v->Err);

    if (v->Ui > v->Umax) { v->Ui = v->Umax; }
    else if (v->Ui < v->Umin) { v->Ui = v->Umin; }

    v->Out = Up + v->Ui;

    if (v->Out > v->Umax) { v->Out = v->Umax; }
    else if (v->Out < v->Umin) { v->Out = v->Umin; }
}
