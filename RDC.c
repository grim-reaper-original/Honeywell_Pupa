#include "RDC.h"


#define READ_RDC_DOS()    (GpioDataRegs.GPCDAT.bit.GPIO84)
#define READ_RDC_LOT()    (GpioDataRegs.GPCDAT.bit.GPIO85)
#define RDC_WR_FSYNC_LOW()   (GpioDataRegs.GPACLEAR.bit.GPIO9 = 1)
#define RDC_WR_FSYNC_HIGH()  (GpioDataRegs.GPASET.bit.GPIO9 = 1)
#define RDC_CS_LOW()      (GpioDataRegs.GPACLEAR.bit.GPIO19 = 1)
#define RDC_CS_HIGH()     (GpioDataRegs.GPASET.bit.GPIO19 = 1)
#define RDC_A0_LOW()      (GpioDataRegs.GPACLEAR.bit.GPIO20 = 1)
#define RDC_A0_HIGH()     (GpioDataRegs.GPASET.bit.GPIO20 = 1)
#define RDC_A1_LOW()      (GpioDataRegs.GPACLEAR.bit.GPIO21 = 1)
#define RDC_A1_HIGH()     (GpioDataRegs.GPASET.bit.GPIO21 = 1)
#define RDC_SAMPLE_LOW()  (GpioDataRegs.GPACLEAR.bit.GPIO23 = 1)
#define RDC_SAMPLE_HIGH() (GpioDataRegs.GPASET.bit.GPIO23 = 1)


Uint16 Rotor_Angle_Raw = 0;
int16  Rotor_Velocity_Raw = 0;
Uint16 Resolver_Fault_Register = 0;
Uint16 Rotor_Angle_12 = 0;
int16 Rotor_Velocity_12 = 0;

void Init_SPI_RDC(void);
static Uint16 SPI_A_Transfer_8(Uint16 tx_data);
static void AD2S1210_WriteRegister(Uint16 address, Uint16 data);
void AD2S1210_Configure(void);
void AD2S1210_Clear_Startup_Faults(void);
void Read_Resolver_Data(void);


//THIS ENTIRE SECTION BELONGS IN MAIN.C

//void main(void)
//{
//    // 1. Basic DSP Setup
//    DisableDog();
//    InitPll(6, 3);
//    InitPeripheralClocks();
//    DINT;
//
//    // 2. Initialize SPI-A
//    Init_SPI_RDC();
//
//    // 3. Configure AD2S1210 (12-bit, 1.0V LOS, 4.1V DOS, 10kHz Excitation)
//    AD2S1210_Configure();
//
//    // 4. Clear power-up step-response faults from the AD2S1210
//    AD2S1210_Clear_Startup_Faults();
//
//    // 5. Safe Polling Loop
//    while(1)
//    {
//        // Read the absolute angle, velocity, and fault byte continuously
//        Read_Resolver_Data();
//        DELAY_US(10000); // Poll at 100 Hz
//    }
//}



//// =========================================================
//// RESOLVER FUNCTIONS
//// =========================================================

void Init_SPI_RDC(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.all &= ~0x00070000;
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;   // GPIO9 function

    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;

    GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 0;
    GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO84 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO85 = 0;


    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // SPISOMIA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // SPICLKA

    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0; // CS
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0; // A0
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0; // A1
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0; // SAMPLE

    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;    // GPIO9 output
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;
    EDIS;

    RDC_WR_FSYNC_HIGH();
    RDC_CS_HIGH();
    RDC_SAMPLE_HIGH();
    RDC_A0_LOW();
    RDC_A1_LOW();

    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    SpiaRegs.SPICCR.all = 0x0007;
    SpiaRegs.SPICTL.all = 0x000E;        // CPOL=0, CPHA=1
    SpiaRegs.SPIBRR = 4;                 // SPI Clock = LSPCLK / 5
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;

    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x2044;
    SpiaRegs.SPIFFCT.all = 0x0;
}

static Uint16 SPI_A_Transfer_8(Uint16 tx_data)
{
    while(SpiaRegs.SPIFFTX.bit.TXFFST != 0) {}
    SpiaRegs.SPITXBUF = (tx_data << 8);
    while(SpiaRegs.SPIFFRX.bit.RXFFST == 0) {}
    return (SpiaRegs.SPIRXBUF & 0x00FF);
}

// Helper function to correctly frame 8-bit Configuration Writes
static void AD2S1210_WriteRegister(Uint16 address, Uint16 data)
{
    RDC_A0_HIGH();
    RDC_A1_HIGH();
    DELAY_US(1);

    RDC_CS_LOW();
    RDC_WR_FSYNC_LOW();
    SPI_A_Transfer_8(address);
    RDC_WR_FSYNC_HIGH();

    DELAY_US(1);

    RDC_WR_FSYNC_LOW();
    SPI_A_Transfer_8(data);
    RDC_WR_FSYNC_HIGH();
}

void AD2S1210_Configure(void)
{
    AD2S1210_WriteRegister(0x92, 0x7E);

    // --- 2. Set LOS Threshold to 1.0 Volts ---
    AD2S1210_WriteRegister(0x88, 0x1A);

    // --- 3. Set DOS Overrange to 4.1 Volts ---
    AD2S1210_WriteRegister(0x89, 0x6C);

    // --- 4. Set Excitation Frequency to 10 kHz ---
    AD2S1210_WriteRegister(0x91, 0x28);
}

void AD2S1210_Clear_Startup_Faults(void)
{
    RDC_SAMPLE_LOW();
    DELAY_US(1);
    RDC_SAMPLE_HIGH();

    RDC_A0_HIGH();
    RDC_A1_HIGH();
    DELAY_US(1);

    RDC_WR_FSYNC_LOW();
    SPI_A_Transfer_8(0xFF);
    RDC_WR_FSYNC_HIGH();

    DELAY_US(1);

    RDC_WR_FSYNC_LOW();
    Resolver_Fault_Register = SPI_A_Transfer_8(0x00);
    RDC_WR_FSYNC_HIGH();

    RDC_SAMPLE_LOW();
    DELAY_US(1);
    RDC_SAMPLE_HIGH();

    RDC_A0_LOW();
    RDC_A1_LOW();
    DELAY_US(100);
}

void Read_Resolver_Data(void)
{
    Uint16 high_byte, low_byte;

    // 1. Lock the internal position/velocity registers
    RDC_SAMPLE_LOW();
    DELAY_US(1);
    RDC_SAMPLE_HIGH();

    // --- 2. READ POSITION (Mode A0=0, A1=0) ---
    RDC_A0_LOW();
    RDC_A1_LOW();

    RDC_CS_LOW();
    RDC_WR_FSYNC_LOW();
    // Execute three continuous 8-bit transfers to form the 24-bit word without resetting SPI
    high_byte = SPI_A_Transfer_8(0x00);
    low_byte  = SPI_A_Transfer_8(0x00);
    Resolver_Fault_Register = SPI_A_Transfer_8(0x00);
    RDC_WR_FSYNC_HIGH();

    Rotor_Angle_Raw = (high_byte << 8) | low_byte;
    Rotor_Angle_12 = Rotor_Angle_Raw >> 4;

    // --- 3. READ VELOCITY (Mode A0=0, A1=1) ---
    RDC_A0_LOW();
    RDC_A1_HIGH();

    RDC_CS_LOW(); // Takes SDO out of High-Z[cite: 4]
    RDC_WR_FSYNC_LOW();
    high_byte = SPI_A_Transfer_8(0x00);
    low_byte  = SPI_A_Transfer_8(0x00);
    Resolver_Fault_Register = SPI_A_Transfer_8(0x00);
    RDC_WR_FSYNC_HIGH();

    Rotor_Velocity_Raw = (int16)((high_byte << 8) | low_byte);
    Rotor_Velocity_12 = Rotor_Velocity_Raw >> 4;

    // 4. Release the internal registers
    RDC_SAMPLE_HIGH();
}
