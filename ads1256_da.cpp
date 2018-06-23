#define _POSIX_C_SOURCE  200113L
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <regex>

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
#ifdef __arm__
  bcm2835_delayMicroseconds( micros );
#else
  usleep( micros );
#endif
}

class ADS1256 {
  public:
   enum AdcTimes {
     time_send = 2,
     time_postcfg = 50,
     time_delayData  = 10,
     time_postChan = 5,
     time_wakeup = 25
   };
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
   void sendByte( uint8_t data );
   void sendBytes( uint8_t d0, uint8_t d1 );
   void sendBytes( uint8_t d0, uint8_t d1, uint8_t d2 );
   void sendBytes( const uint8_t *data, unsigned n );
   static AdcGain findGain( int g );
   static Drate   findDrate( int sps );
   static uint8_t calc_reg_mux( uint8_t c1, uint8_t c2 );
   int calc_muxs_n( int n );
   int calc_muxs_spec( const string &spec );
   int  CfgADC( AdcGain gain, Drate drate );
   void DelayDATA() { bsp_DelayUS( time_delayData ); } // The minimum time delay 6.5us
   uint8_t recvByte() {  return bcm2835_spi_transfer( 0xFF ); }
   void WriteReg( uint8_t RegID, uint8_t RegValue );
   uint8_t ReadReg( uint8_t RegID );
   void WriteCmd( uint8_t cmd );
   uint8_t ReadChipID();
   int  WaitDRDY();
   int32_t ReadData();
   int measureLine();
   void setRefVolt( double rv ) { ref_volt = rv; }
   double getRefVolt() const { return ref_volt; }

   const vector<double>& getVolts() const { return volts; }
   void clear();
   int get_ch_n() const { return muxs.size(); };
   const vector<uint8_t>& getMuxs() const { return muxs; }
  protected:
   static const unsigned ch_max = 8;
   static const AdcGainInfo gainInfo[GAIN_NUM];
   static const AdcDrateInfo drateInfo[SPS_MAX];
   double ref_volt = 2.48;
   AdcGain Gain   = GAIN_1;
   int gainval = 1;
   uint8_t curr_reg_mux = 0xFF;
   vector<double> volts;
   vector<uint8_t> muxs;
   Drate DataRate = SPS_15;
   bool need_start = true;
   int writeMux( uint8_t m );
};

const ADS1256::AdcGainInfo ADS1256::gainInfo[ADS1256::GAIN_NUM] = {
  { GAIN_1,   1 },
  { GAIN_2,   2 },
  { GAIN_4,   4 },
  { GAIN_8,   8 },
  { GAIN_16, 16 },
  { GAIN_32, 32 },
  { GAIN_64, 64 }
};

const ADS1256::AdcDrateInfo ADS1256::drateInfo[ADS1256::SPS_MAX] = {
  { SPS_30000, 30000, 0xF0 },
  { SPS_15000, 15000, 0xE0 },
  { SPS_7500,   7500, 0xD0 },
  { SPS_3750,   3750, 0xC0 },
  { SPS_2000,   2000, 0xB0 },
  { SPS_1000,   1000, 0xA1 },
  { SPS_500,     500, 0x92 },
  { SPS_100,     100, 0x82 },
  { SPS_60,       60, 0x72 },
  { SPS_50,       50, 0x63 },
  { SPS_30,       30, 0x53 },
  { SPS_25,       25, 0x43 },
  { SPS_15,       15, 0x33 },
  { SPS_10,       10, 0x23 },
  { SPS_5,         5, 0x13 },
  { SPS_2d5,       2, 0x03 } // really 2.5
};


ADS1256::ADS1256()
{
  volts.reserve( 32 );
  volts.assign( 32, 0.0 );
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
 *  function: calculate REG_MUX value for channel or pair of channels
 *  parameters: c1 - first channel [0:7], c2 - secons channel [0:7], if > 7 - single-ended
 *  The return value: register valus or 0xFF if bad params ( both > 7 )
 *********************************************************************************************************
 */
uint8_t ADS1256::calc_reg_mux( uint8_t c1, uint8_t c2 )
{
  uint8_t v = 0;
  if( c1 >= ch_max ) {
     v |= 0xF0; // AINCOM
  } else {
     v |= ( c1 << 4 ) & 0x70;
  }

  if( c2 >= ch_max ) {
     v |= 0x0F; // AINCOM
  } else {
     v |= c2 & 0x07;
  }

  return v;
}

int ADS1256::calc_muxs_n( int n )
{
  if( n > (int)ch_max ) {
    n = ch_max;
  }
  muxs.clear();
  for( int i=0; i<n; ++i ) {
    uint8_t m = calc_reg_mux( (uint8_t)(i), 0xFF );
    if( m != 0xFF ) {
      muxs.emplace_back( m );
    }
  }
  clear();
  curr_reg_mux = 0xFF;
  return muxs.size();
}

int ADS1256::calc_muxs_spec( const string &spec )
{
  muxs.clear();
  if( spec.empty() ) {
    return 0;
  }
  regex decuns( "^\\d+$" );
  regex decdiff( "^(\\d+)-(\\d+)$" );
  regex decrange( "^(\\d+):(\\d+)$" );

  bool need_next = true;
  string badstr;
  for( auto c = spec.cbegin(); need_next;  ) {
    auto f = find( c, spec.cend(), ',' );
    if( f == spec.cend() ) {
      need_next = false;
    }
    string t1 ( c, f );
    smatch sm;
    // cerr << t1 << endl;
    if( regex_search( t1, decuns ) ) {
      int ch1 = stoi( t1, nullptr, 0 );
      // cout << " ch1= " << ch1;
      uint8_t m = calc_reg_mux( (uint8_t)(ch1), 0xFF );
      if( m != 0xFF ) {
        muxs.emplace_back( m );
      } else {
        badstr = t1; break;
      };
    } else if( regex_search( t1, sm, decdiff ) ) {
      int ch1 = stoi( sm[1], nullptr, 0 );
      int ch2 = stoi( sm[2], nullptr, 0 );
      // cout << " ch1= " << ch1 << " ch2= " << ch2;
      uint8_t m = calc_reg_mux( (uint8_t)(ch1), (uint8_t)(ch2) );
      if( m != 0xFF ) {
        muxs.emplace_back( m );
      } else {
        badstr = t1; break;
      };
    } else if( regex_search( t1, sm, decrange ) ) {
      int ch1 = stoi( sm[1], nullptr, 0 );
      int ch2 = stoi( sm[2], nullptr, 0 );
      // cout << " ch1= " << ch1 << " ch2= " << ch2;
      for( int c = ch1; c <= ch2 && badstr.empty(); ++c ) {
        uint8_t m = calc_reg_mux( (uint8_t)(c), 0xFF );
        if( m != 0xFF ) {
          muxs.emplace_back( m );
        } else {
          badstr = t1; break;
        };
      }
    } else {
      badstr = t1; break;
    }
    ++f; c = f;
    // cerr << endl;
  }

  if( ! badstr.empty() ) {
      cerr << "Error: bad channel spec string \"" << badstr << "\"" << endl;
      muxs.clear();
      return 0;
  }

  curr_reg_mux = 0xFF;
  return muxs.size();
}

int ADS1256::writeMux( uint8_t m )
{
  WaitDRDY();
  if( m == curr_reg_mux ) {
    return 1;
  }
  WriteReg( REG_MUX, m );
  bsp_DelayUS( time_postChan );
  WriteCmd( CMD_SYNC );
  bsp_DelayUS( time_postChan );

  WriteCmd( CMD_WAKEUP );
  bsp_DelayUS( time_wakeup );
  curr_reg_mux = m;
  return 2;
}

int ADS1256::measureLine()
{
  int n = 0;
  int mc = muxs.size();
  clear();
  if( mc < 1 ) {
    return 0;
  }

  if( need_start ) {
    WriteCmd( CMD_SYNC );
    bsp_DelayUS( time_postChan );

    WriteCmd( CMD_WAKEUP );
    bsp_DelayUS( time_wakeup );
    need_start = false;
  }

  writeMux( muxs[0] );

  for( int i=0; i<mc; ++i ) {
    int32_t vi = ReadData();
    volts[i] = (double) vi * ref_volt / gainval / 0x400000;
    ++n;
    int j = i+1;
    if( j >= mc ) { j  = 0; }
    writeMux( muxs[j] );
  }

  return n;
}

int32_t ADS1256::ReadData()
{
  uint8_t buf[3];

  CS_guard csg;

  sendByte( CMD_RDATA );  // read ADC command

  DelayDATA();  // delay time
  WaitDRDY();

  /*Read the sample results 24bit*/
  buf[0] = recvByte();
  buf[1] = recvByte();
  buf[2] = recvByte();

  uint32_t
  read  =  ( (uint32_t)buf[0] << 16 ) & 0x00FF0000;
  read |=  ( (uint32_t)buf[1] <<  8 );  /* Pay attention to It is wrong   read |= (buf[1] << 8) */
  read |=  buf[2];

  if( read & 0x800000 ) { // Extend a signed number
    read |= 0xFF000000;
  }

  return (int32_t)read;
}


void ADS1256::sendByte( uint8_t d0 )
{
  bsp_DelayUS( time_send );
  bcm2835_spi_transfer( d0 );
}

void ADS1256::sendBytes( uint8_t d0, uint8_t d1 )
{
  bsp_DelayUS( time_send );
  bcm2835_spi_transfer( d0 );
  bcm2835_spi_transfer( d1 );
}

void ADS1256::sendBytes( uint8_t d0, uint8_t d1, uint8_t d2 )
{
  bsp_DelayUS( time_send );
  bcm2835_spi_transfer( d0 );
  bcm2835_spi_transfer( d1 );
  bcm2835_spi_transfer( d2 );
}

void ADS1256::sendBytes( const uint8_t *data, unsigned n )
{
  bsp_DelayUS( time_send );
  for( unsigned i=0; i<n; ++i ) {
    bcm2835_spi_transfer( data[i] );
  }
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
  if( gain >= GAIN_NUM || drate >= SPS_MAX ) {
    return 0;
  }

  Gain = gain;
  gainval = gainInfo[gain].val;
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
  buf[2] = (0 << 5) | (0 << 3) |  gain;
  buf[3] = drateInfo[drate].regval;

  CS_guard csg;

  sendBytes( CMD_WREG | 0, 0x03 );  /* Write command register, send the register address, write */

  sendBytes( buf, 4 );  /* Set the status, input channel, ADCON, outrate regs */

  bsp_DelayUS( time_postcfg );
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
void ADS1256::WriteReg( uint8_t RegID, uint8_t RegValue )
{
  CS_guard csg;

  sendBytes( CMD_WREG | RegID, 0, RegValue );  //* Write command register, num, value
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

  sendBytes( CMD_RREG | RegID, 0 );  // Write command register

  DelayDATA();

  return recvByte();
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
  sendByte( cmd );
}

uint8_t ADS1256::ReadChipID()
{
  if( ! WaitDRDY() ) {
    return 0;
  }
  int8_t id = ReadReg( REG_STATUS );
  return ( id >> 4 );
}


/*
 *  name: ADS1256::WaitDRDY
 *  function: wait for data ready signal
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


void ADS1256::clear()
{
  volts.assign( muxs.size(), 0.0 );
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
  if( ! bcm2835_spi_begin() )  {
    return 0;
  }
  bcm2835_spi_setBitOrder( BCM2835_SPI_BIT_ORDER_LSBFIRST );    // The default LSBFIRST
  bcm2835_spi_setDataMode( BCM2835_SPI_MODE1 );                  // The default = MODE1
  bcm2835_spi_setClockDivider( BCM2835_SPI_CLOCK_DIVIDER_1024 ); // The default = 1024
  bcm2835_gpio_fsel( SPICS, BCM2835_GPIO_FSEL_OUTP );
  bcm2835_gpio_write( SPICS, HIGH );
  bcm2835_gpio_fsel( DRDY, BCM2835_GPIO_FSEL_INPT );
  bcm2835_gpio_set_pud( DRDY, BCM2835_GPIO_PUD_UP );
  return 1;
}

void show_help()
{
  cout << "ads1256_da usage: \n";
  cout << "ads1256_da [-h] [-d] [-t t_dly,us ] [ -c channels ] \n";
  cout << "   or [ -C c1-c2,c3 ] [ -g gain ] [ -D drate ] [ -n iterations ] [ -r ref_volt ] [ -o file ]\n";
}


// -----------------------------------------------------------------------------------------------

int main( int argc, char **argv )
{
  int debug = 0;             // -d
  uint32_t t_dly = 1000;     // in ms, -t
  uint32_t N = 1000;         // -n
  int   n_ch = -1;           // -c
  string   ch_specs;         // -C
  int      gain = 1;         // -c
  int      drate = -1;       // -D -1 = auto
  double   ref_volt = 2.484; // -r
  string ofn;                // -o
  bool do_fout = false;

  int op; // TODO: -q 0 1 2, -B - buffer
  while( ( op = getopt( argc, argv, "hdt:n:g:c:C:D:r:o:" ) ) != -1 ) {
    switch( op ) {
      case 'h' : show_help(); return 0;
      case 'd' : ++debug; break;
      case 't' : t_dly = strtol( optarg, 0, 0 ); break;
      case 'n' : N     = strtol( optarg, 0, 0 ); break;
      case 'c' : n_ch  = strtol( optarg, 0, 0 ); break;
      case 'g' : gain  = strtol( optarg, 0, 0 ); break;
      case 'D' : drate  = strtol( optarg, 0, 0 ); break;
      case 'r' : ref_volt  = strtod( optarg, 0 ); break;
      case 'o' : ofn  = optarg; break;
      case 'C' : ch_specs  = optarg; break;
      default:
        cerr << "Error: unknown or bad option '" << (char)(optopt) << endl;
        show_help();
        return 1;
    }
  }

  ADS1256 adc;
  adc.setRefVolt( ref_volt );

  if( n_ch > 0 ) {
    if( ! ch_specs.empty() ) {
      cerr << "Error: both n_ch and ch_specs found" << endl;
      return 1;
    }
    adc.calc_muxs_n( n_ch );
  } else {
    if( ! ch_specs.empty() ) {
      adc.calc_muxs_spec( ch_specs );
    } else {
      n_ch = 8;
      adc.calc_muxs_n( n_ch );
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
    cerr << "N= " << N << " t_dly= " << t_dly << " n_ch= " << adc.get_ch_n()
         << " gain= " << gain << " gain_idx= " << (int)(gain_idx) <<" ref_volt= " << ref_volt
         << " t_add_sec= " << t_add_sec << " t_add_ns= " << t_add_ns << endl;
  }
  if( debug > 1 ) {
    auto m = adc.getMuxs();
    cerr << hex;
    for( auto v : m ) {
      cerr << (unsigned)v << ' ';
    }
    cerr << dec << endl;
  }

  if( adc.get_ch_n() < 1 ) {
    cerr << "Error: nothing to measure" << endl;
    return 1;
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

  struct timespec ts0, ts1, tsc;
  clock_gettime( CLOCK_MONOTONIC, &ts0 ); ts1 = ts1; // just to make GCC happy

  string obuf;
  obuf.reserve( 256 );
  ostringstream s_os( obuf ); // .str( )
  if( do_fout ) {
    os << "# start" << endl;
  }

  signal( SIGINT, sigint_handler );


  for( uint32_t i_n=0; i_n < N && ! break_loop; ++i_n ) {

    clock_gettime( CLOCK_MONOTONIC, &tsc );
    if( i_n == 0 ) {
      ts0 = tsc; ts1 = tsc;
    }
    double dt = tsc.tv_sec - ts0.tv_sec + 1e-9 * (tsc.tv_nsec - ts0.tv_nsec);

    s_os.str().clear();
    s_os << setfill('0') << setw(8) << i_n << ' ' << showpoint  << setw(8) << setprecision(4) << dt; // TODO: theor time

    adc.measureLine();

    for( auto v : adc.getVolts() ) {
       s_os << ' ' << setw(10) << setprecision(8) << v;
    }
    s_os << ' ' << endl;
    cout << s_os.str();
    if( do_fout ) {
      os << s_os.str();
    }

    ts1.tv_sec  += t_add_sec;
    ts1.tv_nsec += t_add_ns; // compensation
    if( ts1.tv_nsec > 1000000000L ) {
      ts1.tv_nsec -=  1000000000L;
      ++ts1.tv_sec;
    }
    clock_nanosleep( CLOCK_MONOTONIC, TIMER_ABSTIME, &ts1, 0 );
  }
  cout << endl;
  if( do_fout ) {
    os << endl;
  }
  if( break_loop ) {
    cerr << "Loop was terminated" << endl;
  }

  bcm2835_spi_end();
  bcm2835_close();

  return 0;
}


