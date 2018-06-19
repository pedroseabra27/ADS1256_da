#define _POSIX_C_SOURCE  200113L
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <unistd.h>
#include <getopt.h>
#include <time.h>
#include <signal.h>

#include <bcm2835.h>

using namespace std;

// CS     -----   SPICS
// DIN    -----   MOSI
// DOUT   -----   MISO
// SCLK   -----   SCLK
// DRDY   -----   ctl_IO     data  starting
// RST    -----   ctl_IO     reset


#define  DRDY   RPI_GPIO_P1_11  // P0
#define  RST    RPI_GPIO_P1_12  // P1
#define  SPICS  RPI_GPIO_P1_15  // P3

volatile int break_loop = 0;

void sigint_handler( int signum );

void sigint_handler( int signum )
{
  ++break_loop;
}

inline void CS_1() { bcm2835_gpio_write( SPICS, HIGH ); }
inline void CS_0() { bcm2835_gpio_write( SPICS, LOW );  }

class CS_guard {
  public:
   CS_guard()  { bcm2835_gpio_write( SPICS, LOW  ); };
   ~CS_guard() { bcm2835_gpio_write( SPICS, HIGH ); };
};

inline bool DRDY_IS_LOW() { return bcm2835_gpio_lev( DRDY ) == 0; };

inline void RST_1() {  bcm2835_gpio_write( RST, HIGH ); }
inline void RST_0() {  bcm2835_gpio_write( RST, LOW );  }

inline void  bsp_DelayUS( uint64_t micros )
{
  bcm2835_delayMicroseconds( micros );
}

class ADS1256 {
  public:
   enum AdcGain {
     GAIN_1      = 0,
     GAIN_2      = 1,
     GAIN_4      = 2,
     GAIN_8      = 3,
     GAIN_16     = 4,
     GAIN_32     = 5,
     GAIN_64     = 6,
     GAIN_NUM    // 7
   };
   struct AdcGainInfo {
     AdcGain idx;
     uint8_t val;
   };
   enum Drate {
     SPS_30000 = 0,
     SPS_15000,
     SPS_7500,
     SPS_3750,
     SPS_2000,
     SPS_1000,
     SPS_500,
     SPS_100,
     SPS_60,
     SPS_50,
     SPS_30,
     SPS_25,
     SPS_15,
     SPS_10,
     SPS_5,
     SPS_2d5,
     SPS_MAX
   };
   struct AdcDrateInfo {
     Drate    idx;
     uint16_t val;
     uint8_t  regval;
   };

   enum RegNum { //* Register definitions Table 23. Register Map --- ADS1256 datasheet Page 30
                 //  Register address, followed by reset the default values
     REG_STATUS =  0, // x1H
     REG_MUX    =  1, // 01H
     REG_ADCON  =  2, // 20H
     REG_DRATE  =  3, // F0H
     REG_IO     =  4, // E0H
     REG_OFC0   =  5, // xxH
     REG_OFC1   =  6, // xxH
     REG_OFC2   =  7, // xxH
     REG_FSC0   =  8, // xxH
     REG_FSC1   =  9, // xxH
     REG_FSC2   = 10, // xxH
   };

   //* Command definitions TTable 24. Command Definitions --- ADS1256 datasheet Page 34
   enum Commands  {
     CMD_WAKEUP  = 0x00, //* Completes SYNC and Exits Standby Mode 0000  0000 (00h)
     CMD_RDATA   = 0x01, //* Read Data 0000  0001 (01h)
     CMD_RDATAC  = 0x03, //* Read Data Continuously 0000   0011 (03h)
     CMD_SDATAC  = 0x0F, //* Stop Read Data Continuously 0000   1111 (0Fh)
     CMD_RREG    = 0x10, //* Read from REG rrr 0001 rrrr (1xh)
     CMD_WREG    = 0x50, //* Write to REG rrr 0101 rrrr (5xh)
     CMD_SELFCAL = 0xF0, //* Offset and Gain Self-Calibration 1111    0000 (F0h)
     CMD_SELFOCAL= 0xF1, //* Offset Self-Calibration 1111    0001 (F1h)
     CMD_SELFGCAL= 0xF2, //* Gain Self-Calibration 1111    0010 (F2h)
     CMD_SYSOCAL = 0xF3, //* System Offset Calibration 1111   0011 (F3h)
     CMD_SYSGCAL = 0xF4, //* System Gain Calibration 1111    0100 (F4h)
     CMD_SYNC    = 0xFC, //* Synchronize the A/D Conversion 1111   1100 (FCh)
     CMD_STANDBY = 0xFD, //* Begin Standby Mode 1111   1101 (FDh)
     CMD_RESET   = 0xFE, //* Reset to Power-Up Values 1111   1110 (FEh)
   };

   ADS1256();
   void StartScan( uint8_t ucScanMode );
   void Send8Bit( uint8_t data );
   static AdcGain findGain( int g );
   static Drate   findDrate( int sps );
   int  CfgADC( AdcGain gain, Drate drate );
   void DelayDATA() { bsp_DelayUS( 10 ); } // The minimum time delay 6.5us
   uint8_t Recive8Bit() {  return bcm2835_spi_transfer( 0xFF ); }
   void WriteReg( uint8_t RegID, uint8_t RegValue );
   uint8_t ReadReg( uint8_t RegID );
   void WriteCmd( uint8_t cmd );
   uint8_t ReadChipID();
   void SetChannal( uint8_t ch );
   void SetDiffChannal( uint8_t ch );
   int  WaitDRDY();
   int32_t ReadData();

   int32_t GetAdc( uint8_t ch );
   void ISR();
   uint8_t Scan();
   void clear();
  protected:
   static const uint8_t s_tabDataRate[SPS_MAX];
   static const unsigned ch_n = 8;
   static const AdcGainInfo gainInfo[GAIN_NUM];
   static const AdcDrateInfo drateInfo[SPS_MAX];
   AdcGain Gain   = GAIN_1;
   Drate DataRate = SPS_15;
   int32_t AdcNow[ch_n];  //* ADC  Conversion value
   uint8_t Channel =  0;  //* The current channel
   uint8_t ScanMode = 0;  //* Scanning mode, 0=Single-ended input 8 channels; 1=Differential input  4 channels
};

const ADS1256::AdcGainInfo ADS1256::gainInfo[ADS1256::GAIN_NUM] = {
  { ADS1256::GAIN_1,   1 },
  { ADS1256::GAIN_2,   2 },
  { ADS1256::GAIN_4,   4 },
  { ADS1256::GAIN_8,   8 },
  { ADS1256::GAIN_16, 16 },
  { ADS1256::GAIN_32, 32 },
  { ADS1256::GAIN_64, 64 }
};

const ADS1256::AdcDrateInfo ADS1256::drateInfo[ADS1256::SPS_MAX] = {
  {  ADS1256::SPS_30000, 30000, 0xF0 },
  {  ADS1256::SPS_15000, 15000, 0xE0 },
  {  ADS1256::SPS_7500,   7500, 0xD0 },
  {  ADS1256::SPS_3750,   3750, 0xC0 },
  {  ADS1256::SPS_2000,   2000, 0xB0 },
  {  ADS1256::SPS_1000,   1000, 0xA1 },
  {  ADS1256::SPS_500,     500, 0x92 },
  {  ADS1256::SPS_100,     100, 0x82 },
  {  ADS1256::SPS_60,       60, 0x72 },
  {  ADS1256::SPS_50,       50, 0x63 },
  {  ADS1256::SPS_30,       30, 0x53 },
  {  ADS1256::SPS_25,       25, 0x43 },
  {  ADS1256::SPS_15,       15, 0x33 },
  {  ADS1256::SPS_10,       10, 0x23 },
  {  ADS1256::SPS_5,         5, 0x13 },
  {  ADS1256::SPS_2d5,       2, 0x03 } // really 2.5
};

const uint8_t ADS1256::s_tabDataRate[SPS_MAX] =
{    /*reset the default values                                                   23?     */
  0xF0, 0xE0, 0xD0, 0xC0, 0xB0, 0xA1, 0x92, 0x82, 0x72, 0x63, 0x53, 0x43, 0x33, 0x20, 0x13, 0x03
};

/* Sampling speed choice*/
/*
   11110000 = 30,000SPS (default)
   11100000 = 15,000SPS
   11010000 = 7,500SPS
   11000000 = 3,750SPS
   10110000 = 2,000SPS
   10100001 = 1,000SPS
   10010010 = 500SPS
   10000010 = 100SPS
   01110010 = 60SPS
   01100011 = 50SPS
   01010011 = 30SPS
   01000011 = 25SPS
   00110011 = 15SPS
   00100011 = 10SPS
   00010011 = 5SPS
   00000011 = 2.5SPS
   */


ADS1256::ADS1256()
{
}



/*
 *********************************************************************************************************
 *  name: bsp_InitADS1256
 *  function: Configuration of the STM32 GPIO and SPI interfaces The connection ADS1256
 *  parameter: NULL
 *  The return value: NULL
 *********************************************************************************************************
 */


void bsp_InitADS1256()
{
#ifdef SOFT_SPI
  CS_1();
  SCK_0();
  DI_0();
#endif

  //ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_1000SPS);  /*  1KHz */
}


ADS1256::AdcGain ADS1256::findGain( int g )
{
  for( auto v : ADS1256::gainInfo ) {
    if( v.val == g ) {
      return v.idx;
    }
  }
  return GAIN_NUM;
}

ADS1256::Drate  ADS1256::findDrate( int sps )
{
  for( auto v : drateInfo ) {
    if( v.val == sps ) {
      return v.idx;
    }
  }
  return SPS_MAX;
}

/*
 *********************************************************************************************************
 *  name: ADS1256::StartScan
 *  function: Configuration DRDY PIN for external interrupt is triggered
 *  parameter: ucDiffMode : 0  Single-ended input  8 channels 1 Differential input  4 channe
 *  The return value: NULL
 *********************************************************************************************************
 */
void ADS1256::StartScan( uint8_t ucScanMode )
{
  ScanMode = ucScanMode;
  Channel = 0;
  clear();
}

/*
 *********************************************************************************************************
 *  name: ADS1256::Send8Bit
 *  function: SPI bus to send 8 bit data
 *  parameter: data:  data
 *  The return value: NULL
 *********************************************************************************************************
 */
void ADS1256::Send8Bit( uint8_t data )
{
  bsp_DelayUS( 2 );
  bcm2835_spi_transfer( data );
}

/*
 *********************************************************************************************************
 *  name: ADS1256::CfgADC
 *  function: The configuration parameters of ADC, gain and data rate
 *  parameter: gain:gain 1-64
 *                      drate:  data  rate
 *  The return value: 1 - ok, 0 - error
 *********************************************************************************************************
 */
int  ADS1256::CfgADC( AdcGain gain, Drate drate )
{
  Gain = gain;
  DataRate = drate;

  if( ! WaitDRDY() ) {
    return 0;
  }

  uint8_t buf[4];    /* Storage ads1256 register configuration parameters */

  /*Status register define
    Bits 7-4 ID3, ID2, ID1, ID0  Factory Programmed Identification Bits (Read Only)

    Bit 3 ORDER: Data Output Bit Order
    0 = Most Significant Bit First (default)
    1 = Least Significant Bit First
    Input data  is always shifted in most significant byte and bit first. Output data is always shifted out most significant
    byte first. The ORDER bit only controls the bit order of the output data within the byte.

    Bit 2 ACAL : Auto-Calibration
    0 = Auto-Calibration Disabled (default)
    1 = Auto-Calibration Enabled
    When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes
    the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register)
    values.

    Bit 1 BUFEN: Analog Input Buffer Enable
    0 = Buffer Disabled (default)
    1 = Buffer Enabled

    Bit 0 DRDY :  Data Ready (Read Only)
    This bit duplicates the state of the DRDY pin.

    ACAL=1  enable  calibration
    */
  //buf[0] = (0 << 3) | (1 << 2) | (1 << 1);//enable the internal buffer

  buf[0] = (0 << 3) | (1 << 2) | (0 << 1);  // The internal buffer is prohibited

  //ADS1256::WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

  buf[1] = 0x08;

  /*  ADCON: A/D Control Register (Address 02h)
      Bit 7 Reserved, always 0 (Read Only)
      Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
      00 = Clock Out OFF
      01 = Clock Out Frequency = fCLKIN (default)
      10 = Clock Out Frequency = fCLKIN/2
      11 = Clock Out Frequency = fCLKIN/4
      When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.

      Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources
      00 = Sensor Detect OFF (default)
      01 = Sensor Detect Current = 0.5 uA
      10 = Sensor Detect Current = 2 uA
      11 = Sensor Detect Current = 10 uA
      The Sensor Detect Current Sources can be activated to verify  the integrity of an external sensor supplying a signal to the
      ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.

      Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
      000 = 1 (default)
      001 = 2
      010 = 4
      011 = 8
      100 = 16
      101 = 32
      110 = 64
      111 = 64
      */
  buf[2] = (0 << 5) | (0 << 3) | ( gain << 0 );
  //WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));  /*choose 1: gain 1 ;input 5V/
  buf[3] = s_tabDataRate[drate];  // DRATE_10SPS;

  CS_guard csg;

  Send8Bit( CMD_WREG | 0 );  /* Write command register, send the register address */
  Send8Bit( 0x03 );      /* Register number 4,Initialize the number  -1*/

  Send8Bit( buf[0] );  /* Set the status register */
  Send8Bit( buf[1] );  /* Set the input channel parameters */
  Send8Bit( buf[2] );  /* Set the ADCON control register,gain */
  Send8Bit( buf[3] );  /* Set the output rate */

  bsp_DelayUS( 50 );
  // TODO: calibrate
  return 1;
}


/*
 *********************************************************************************************************
 *  name: ADS1256::WriteReg
 *  function: Write the corresponding register
 *  parameter: RegID: register  ID
 *       RegValue: register Value
 *  The return value: NULL
 *********************************************************************************************************
 */
void ADS1256::WriteReg(uint8_t RegID, uint8_t RegValue)
{
  CS_guard csg;

  Send8Bit( CMD_WREG | RegID );  //* Write command register
  Send8Bit( 0x00 );              //* Write the register number
  Send8Bit( RegValue );          //* send register value
}

/*
 *********************************************************************************************************
 *  name: ADS1256::ReadReg
 *  function: Read  the corresponding register
 *  parameter: RegID: register  ID
 *  The return value: read register value
 *********************************************************************************************************
 */
uint8_t ADS1256::ReadReg( uint8_t RegID )
{
  CS_guard csg;

  Send8Bit( CMD_RREG | RegID );  // Write command register
  Send8Bit( 0x00 );              // Write the register number
  DelayDATA();

  return Recive8Bit();
}

/*
 *********************************************************************************************************
 *  name: ADS1256::WriteCmd
 *  function: Sending a single byte order
 *  parameter: cmd : command
 *********************************************************************************************************
 */
void ADS1256::WriteCmd( uint8_t cmd )
{
  CS_guard csg;
  Send8Bit( cmd );
}

/*
 *********************************************************************************************************
 *  name: ADS1256::ReadChipID
 *  function: Read the chip ID
 *  parameter: cmd : NULL
 *  The return value: four high status register
 *********************************************************************************************************
 */
uint8_t ADS1256::ReadChipID()
{
  if( !WaitDRDY() ) {
    return 0;
  }
  int8_t id = ReadReg( REG_STATUS );
  return (id >> 4);
}

/*
 *********************************************************************************************************
 *  name: ADS1256::SetChannal
 *  function: Configuration channel number
 *  parameter:  ch:  channel number  0--7
 *********************************************************************************************************
 */
void ADS1256::SetChannal( uint8_t ch )
{
  /*
     Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
     0000 = AIN0 (default)
     0001 = AIN1
     0010 = AIN2 (ADS1256 only)
     0011 = AIN3 (ADS1256 only)
     0100 = AIN4 (ADS1256 only)
     0101 = AIN5 (ADS1256 only)
     0110 = AIN6 (ADS1256 only)
     0111 = AIN7 (ADS1256 only)
     1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are dont care)

    NOTE: When using an ADS1255 make sure to only select the available inputs.

    Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
    0000 = AIN0
    0001 = AIN1 (default)
    0010 = AIN2 (ADS1256 only)
    0011 = AIN3 (ADS1256 only)
    0100 = AIN4 (ADS1256 only)
    0101 = AIN5 (ADS1256 only)
    0110 = AIN6 (ADS1256 only)
    0111 = AIN7 (ADS1256 only)
    1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are dont care)
    */

  if( ch >= ch_n ) {
    return;
  }
  WriteReg( REG_MUX, (ch << 4) | (1 << 3) );  /* Bit3 = 1, AINN connection AINCOM */
}

/*
 *********************************************************************************************************
 *  name: ADS1256::SetDiffChannal
 *  function: The configuration difference channel
 *  parameter:  ch:  channel number  0--3
 *  The return value:  four high status register
 *********************************************************************************************************
 */
void ADS1256::SetDiffChannal( uint8_t ch )
{
  /*
     Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
     0000 = AIN0 (default)
     0001 = AIN1
     0010 = AIN2 (ADS1256 only)
     0011 = AIN3 (ADS1256 only)
     0100 = AIN4 (ADS1256 only)
     0101 = AIN5 (ADS1256 only)
     0110 = AIN6 (ADS1256 only)
     0111 = AIN7 (ADS1256 only)
     1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are dont care)

    NOTE: When using an ADS1255 make sure to only select the available inputs.

    Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
    0000 = AIN0
    0001 = AIN1 (default)
    0010 = AIN2 (ADS1256 only)
    0011 = AIN3 (ADS1256 only)
    0100 = AIN4 (ADS1256 only)
    0101 = AIN5 (ADS1256 only)
    0110 = AIN6 (ADS1256 only)
    0111 = AIN7 (ADS1256 only)
    1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are dont care)
    */

  if( ch == 0) {
    WriteReg( REG_MUX, (0 << 4) | 1 );  //* DiffChannal  AIN0 AIN1
  } else if( ch == 1 )  {
    WriteReg( REG_MUX, (2 << 4) | 3 );  //* DiffChannal   AIN2 AIN3
  } else if( ch == 2 ) {
    WriteReg( REG_MUX, (4 << 4) | 5 );  //* DiffChannal    AIN4 AIN5
  } else if( ch == 3 ) {
    WriteReg( REG_MUX, (6 << 4) | 7 );  //* DiffChannal   AIN6 AIN7
  }
}

/*
 *********************************************************************************************************
 *  name: ADS1256::WaitDRDY
 *  function: delay time  wait for automatic calibration
 *  parameter:  NULL
 *  The return value:  0 - timeout, 1 = ok
 *********************************************************************************************************
 */
int ADS1256::WaitDRDY()
{
  for( uint32_t i = 0; i < 400000; ++i ) {
    if( DRDY_IS_LOW() ) {
      return 1;
    }
  }
  cerr << "WaitDRDY() Time Out ..." << endl;
  return 0;
}

/*
 *********************************************************************************************************
 *  name: ADS1256::ReadData
 *  function: read ADC value
 *  parameter: NULL
 *  The return value:  NULL
 *********************************************************************************************************
 */
int32_t ADS1256::ReadData()
{
  uint8_t buf[3];

  CS_guard csg;

  Send8Bit( CMD_RDATA );  // read ADC command

  DelayDATA();  // delay time

  /*Read the sample results 24bit*/
  buf[0] = Recive8Bit();
  buf[1] = Recive8Bit();
  buf[2] = Recive8Bit();

  uint32_t
  read  =  ( (uint32_t)buf[0] << 16 ) & 0x00FF0000;
  read |=  ( (uint32_t)buf[1] <<  8 );  /* Pay attention to It is wrong   read |= (buf[1] << 8) */
  read |=  buf[2];

  if( read & 0x800000 ) { // Extend a signed number
    read |= 0xFF000000;
  }

  return (int32_t)read;
}


/*
 *********************************************************************************************************
 *  name: ADS1256::GetAdc
 *  function: read ADC value
 *  parameter:  channel number 0--7
 *  The return value:  ADC vaule (signed number)
 *********************************************************************************************************
 */
int32_t ADS1256::GetAdc( uint8_t ch )
{
  if( ch >= ch_n ) {
    return 0;
  }

  return  AdcNow[ch];
}

/*
 *********************************************************************************************************
 *  name: ISR
 *  function: Collection procedures
 *  parameter: NULL
 *  The return value:  NULL
 *********************************************************************************************************
 */
void ADS1256::ISR()
{
  if( ScanMode == 0 ) {    //  0=Single-ended input, 8 channel;  1= Differential input  4 channels
    SetChannal( Channel ); //  Switch channel mode
    bsp_DelayUS( 5 );

    WriteCmd( CMD_SYNC );
    bsp_DelayUS( 5 );

    WriteCmd(CMD_WAKEUP);
    bsp_DelayUS( 25 );

    if( Channel == 0 ) { // What??
      AdcNow[7] = ReadData();
    } else {
      AdcNow[Channel-1] = ReadData();
    }

    if( ++Channel >= 8 )  {
      Channel = 0;
    }
  }  else  { //* DiffChannal

    SetDiffChannal( Channel );  //* change DiffChannal
    bsp_DelayUS( 5 );

    WriteCmd( CMD_SYNC );
    bsp_DelayUS( 5 );

    WriteCmd( CMD_WAKEUP );
    bsp_DelayUS( 25 );

    if( Channel == 0 ) {
      AdcNow[3] = ReadData();
    } else {
      AdcNow[Channel-1] = ReadData();
    }

    if( ++Channel >= 4 ) {
      Channel = 0;
    }
  }
}

/*
 *********************************************************************************************************
 *  name: ADS1256::Scan
 *  function:
 *  parameter:NULL
 *  The return value:  1
 *********************************************************************************************************
 */
uint8_t ADS1256::Scan()
{
  clear();
  if( DRDY_IS_LOW() ) {
    ISR();
    return 1;
  }
  // WaitDRDY();
  // ISR();

  return 0;
}

void ADS1256::clear()
{
  for( auto &v : AdcNow ) {
    v = 0;
  }
}

/*
 *********************************************************************************************************
 *  name: Write_DAC8552
 *  function:  DAC send data
 *  parameter: channel : output channel number
 *         data : output DAC value
 *  The return value:  NULL
 *********************************************************************************************************
 */
// void Write_DAC8552( uint8_t channel, uint16_t Data )
// {
//   // uint8_t i;
//   CS_1() ;
//   CS_0() ;
//   bcm2835_spi_transfer( channel);
//   bcm2835_spi_transfer( Data>>8 );
//   bcm2835_spi_transfer( Data&0xff );
//   CS_1() ;
// }

/*
 *********************************************************************************************************
 *  name: Voltage_Convert
 *  function:  Voltage value conversion function
 *  parameter: Vref : The reference voltage 3.3V or 5V
 *         voltage : output DAC value
 *  The return value:  NULL
 *********************************************************************************************************
 */
// uint16_t Voltage_Convert( float Vref, float voltage )
// {
//   return (uint16_t)( 65536 * voltage / Vref );
// }

int init_hw()
{
  if( !bcm2835_init() ) {
    return 0;
  }
  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST );    // The default
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                  // The default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024); // The default
  bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP);//
  bcm2835_gpio_write(SPICS, HIGH);
  bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP);
  return 1;
}

void show_help()
{
  cout << "ads1256_da usage: \n";
  cout << "ads1256_da [-h] [-d] [-t t_dly,us ] [ -c channels ] [ -g gain ] [ -D drate ] [ -n iterations ] [ -r ref_volt ] [ -o file ]\n";
}


// -----------------------------------------------------------------------------------------------

int main( int argc, char **argv )
{
  int debug = 0;             // -d
  uint32_t t_dly = 1000;     // in ms, -t
  uint32_t N = 1000;         // -n
  uint8_t  n_ch = 8;         // -c
  int      gain = 1;         // -c
  int      drate = -1;       // -D -1 = auto
  double   ref_volt = 2.484; // -r
  string ofn;                // -o
  bool do_fout = false;

  int op; // TODO: -q 0 1 2   -C 1,5,4-2,3 -T - ideal time output
  while( ( op = getopt( argc, argv, "hdt:n:g:c:D:r:o:" ) ) != -1 ) {
    switch( op ) {
      case 'h' : show_help(); return 0;
      case 'd' : ++debug; break;
      case 't' : t_dly = strtol( optarg, 0, 0 ); break;
      case 'n' : N     = strtol( optarg, 0, 0 ); break;
      case 'c' : n_ch  = (uint8_t) strtol( optarg, 0, 0 ); break;
      case 'g' : gain  = strtol( optarg, 0, 0 ); break;
      case 'D' : drate  = strtol( optarg, 0, 0 ); break;
      case 'r' : ref_volt  = strtod( optarg, 0 ); break;
      case 'o' : ofn  = optarg; break;
      default:
        cerr << "Error: unknown or bad option '" << (char)(optopt) << endl;
        show_help();
        return 1;
    }
  }

  auto gain_idx = ADS1256::findGain( gain );
  if( gain_idx >= ADS1256::GAIN_NUM ) {
    cerr << "Bad gain value " << gain << ", must be 1,2...64" << endl;
    return 1;
  }

  // TODO: auto drate
  if( drate < 1 ) {
    drate = 500;
  }

  auto drate_idx = ADS1256::findDrate( drate );
  if( drate_idx >= ADS1256::SPS_MAX ) {
    cerr << "Bad drate value " << drate << ", must be 2,5,10,20,25,50,60,100 ... 30000" << endl;
    return 1;
  }

  // TODO: check drate with t_dly

  uint32_t t_add_sec = t_dly / 1000;
  uint32_t t_add_ns = ( t_dly % 1000 ) * 1000000;

  if( debug > 0 ) {
    cerr << "N= " << N << " t_dly= " << t_dly << "n_ch= " << n_ch
         << " gain= " << gain << " gain_idx= " << (int)(gain_idx) <<" ref_volt= " << ref_volt
         << " t_add_sec= " << t_add_sec << " t_add_ns= " << t_add_ns << endl;
  }

  ofstream os;
  if( ! ofn.empty() ) {
    os.open( ofn );
    if( os ) {
      do_fout = true;
    }
  }

  if( ! init_hw() ) {
    cerr << "Fail to init hardware" << endl;
    return 2;
  }

  ADS1256 adc;
  uint8_t id = adc.ReadChipID();
  cerr << "\r\n ID= " << (int)id << " (must be 3)"<< endl;
  if( id != 3 )  {
    cerr << "Bad chip ID " << id << endl;
    return 3;
  }

  if( ! adc.CfgADC( gain_idx, drate_idx ) ) {
    cerr << "Fail to config ADC" << endl;
    return 5;
  }

  adc.StartScan( 0 );

  //if( Scan() == 0)
  //{
  //continue;
  //}

  struct timespec ts0, ts1, tsc;
  clock_gettime( CLOCK_MONOTONIC, &ts0 ); ts1 = ts1; // just to make GCC happy

  int32_t adc_d[8];
  double volt[8];

  string obuf;
  obuf.reserve( 256 );
  ostringstream s_os( obuf ); // .str( )
  if( do_fout ) {
    os << "# start" << endl;
  }

  signal( SIGINT, sigint_handler );


  for( uint32_t i_n=0; i_n < N && ! break_loop; ++i_n ) {

    unsigned n_wait = 0;
    while( adc.Scan() == 0 ) { ++n_wait; /* NOP */ };

    clock_gettime( CLOCK_MONOTONIC, &tsc );
    if( i_n == 0 ) {
      ts0 = tsc; ts1 = tsc;
    }
    double dt = tsc.tv_sec - ts0.tv_sec + 1e-9 * (tsc.tv_nsec - ts0.tv_nsec);

    cout << setfill('0') << setw(8) << i_n << ' ' << showpoint  << setw(8) << setprecision(4) << dt;
    for( int i = 0; i < n_ch; i++ ) {
      adc_d[i] = adc.GetAdc(i);
      volt[i] = (double)adc_d[i] * ref_volt / gain / 0x400000;
      cout << ' ' << setw(10) << setprecision(8) << volt[i];
    }
    cout << ' ' << n_wait << endl;

    ts1.tv_sec  += t_add_sec;
    ts1.tv_nsec += t_add_ns; // compensation
    if( ts1.tv_nsec > 1000000000L ) {
      ts1.tv_nsec -=  1000000000L;
      ++ts1.tv_sec;
    }
    clock_nanosleep( CLOCK_MONOTONIC, TIMER_ABSTIME, &ts1, 0 );
  }
  cout << endl;
  if( break_loop ) {
    cerr << "Loop was terminated" << endl;
  }

  bcm2835_spi_end();
  bcm2835_close();

  return 0;
}

