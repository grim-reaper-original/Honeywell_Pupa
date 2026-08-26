#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

// ---------------------------------------------------------
// GATE DRIVER HARDWARE PIN MAPPINGS
// ---------------------------------------------------------
#define DRV_CS_LOW()      (GpioDataRegs.GPACLEAR.bit.GPIO27 = 1)
#define DRV_CS_HIGH()     (GpioDataRegs.GPASET.bit.GPIO27 = 1)
#define DRV_CAL_LOW()     (GpioDataRegs.GPBCLEAR.bit.GPIO38 = 1)
#define DRV_CAL_HIGH()    (GpioDataRegs.GPBSET.bit.GPIO38 = 1)
#define DRV_ENABLE_LOW()  (GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1)
#define DRV_ENABLE_HIGH() (GpioDataRegs.GPBSET.bit.GPIO39 = 1)

// ---------------------------------------------------------
// RESOLVER HARDWARE PIN MAPPINGS
// ---------------------------------------------------------
#define RDC_CS_LOW()      (GpioDataRegs.GPACLEAR.bit.GPIO19 = 1)
#define RDC_CS_HIGH()     (GpioDataRegs.GPASET.bit.GPIO19 = 1)
#define RDC_A0_LOW()      (GpioDataRegs.GPACLEAR.bit.GPIO20 = 1)
#define RDC_A0_HIGH()     (GpioDataRegs.GPASET.bit.GPIO20 = 1)
#define RDC_A1_LOW()      (GpioDataRegs.GPACLEAR.bit.GPIO21 = 1)
#define RDC_A1_HIGH()     (GpioDataRegs.GPASET.bit.GPIO21 = 1)
#define RDC_SAMPLE_LOW()  (GpioDataRegs.GPACLEAR.bit.GPIO23 = 1)
#define RDC_SAMPLE_HIGH() (GpioDataRegs.GPASET.bit.GPIO23 = 1)

// Function Prototypes
void Init_ADC_CurrentSensors(void);
void Init_SPI_RDC(void);
void Init_SPI_GateDriver(void);
Uint16 SPI_ReadWrite_16(Uint16 tx_data);
Uint16 SPI_B_ReadWrite_16(Uint16 tx_data);
Uint16 Read_Resolver_Position(void);
void AD2S1210_SetResolution_12Bit(void);
void DRV8323_WakeUp(void);
void DRV8323_WriteRegister(Uint16 address, Uint16 data);
Uint16 DRV8323_ReadRegister(Uint16 address);
void DRV8323_Init_GateDriveStrength(void);
void DRV8323_Init_OCP(void);
__interrupt void adc_isr(void);

// Global variables
Uint16 Raw_Current_A = 0;
Uint16 Raw_Current_B = 0;
Uint16 Raw_Current_C = 0;
Uint16 Rotor_Angle_Raw = 0;
Uint16 GD_Test_Readback = 0;
Uint16 GD_Fault_Status = 0;

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
    AD2S1210_SetResolution_12Bit();

    Init_SPI_GateDriver();
    DRV8323_WakeUp();
    DRV8323_Init_GateDriveStrength();
    DRV8323_Init_OCP();

    GD_Test_Readback = DRV8323_ReadRegister(0x03);
    GD_Fault_Status = DRV8323_ReadRegister(0x00);

    // --- ePWM HARDWARE TRIGGER ---
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EPwm1Regs.TBPRD = 3750;
    EPwm1Regs.TBCTL.bit.CTRMODE = 2;
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = 1;
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

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

    Rotor_Angle_Raw = Read_Resolver_Position();

    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void Init_ADC_CurrentSensors(void)
{
    EALLOW;
    AdcRegs.ADCTRL1.bit.ACQ_PS = 0x0F;
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;
    AdcRegs.ADCTRL3.bit.ADCCLKPS = 0x03;
    AdcRegs.ADCTRL3.bit.SMODE_SEL = 0;
    AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 2;
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x8;
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0xA;
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;
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

void Init_SPI_RDC(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.all &= ~0x00070000;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;
    EDIS;

    RDC_CS_HIGH();
    RDC_SAMPLE_HIGH();
    RDC_A0_LOW();
    RDC_A1_LOW();

    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    SpiaRegs.SPICCR.all = 0x000F;
    SpiaRegs.SPICTL.all = 0x0006;
    SpiaRegs.SPIBRR = 36;
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x2044;
    SpiaRegs.SPIFFCT.all = 0x0;
}

Uint16 SPI_ReadWrite_16(Uint16 tx_data)
{
    while(SpiaRegs.SPIFFTX.bit.TXFFST != 0) { }
    RDC_CS_LOW();
    SpiaRegs.SPITXBUF = tx_data;
    while(SpiaRegs.SPIFFRX.bit.RXFFST == 0) { }
    Uint16 rx_data = SpiaRegs.SPIRXBUF;
    RDC_CS_HIGH();
    return rx_data;
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

void AD2S1210_SetResolution_12Bit(void)
{
    RDC_A0_LOW();
    RDC_A1_HIGH();
    DELAY_US(10);
    SPI_ReadWrite_16(0x9277);
    DELAY_US(10);
    RDC_A0_LOW();
    RDC_A1_LOW();
    DELAY_US(100);
}

Uint16 Read_Resolver_Position(void)
{
    Uint16 raw_angle;
    RDC_SAMPLE_LOW();
    DELAY_US(1);
    RDC_A0_LOW();
    RDC_A1_LOW();
    raw_angle = SPI_ReadWrite_16(0x0000);
    RDC_SAMPLE_HIGH();
    return raw_angle;
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
