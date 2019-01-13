/**
 * @file AD-DA-WS-RPI.h
 * @author Boris Bocquet <b.bocquet@akeoplus.com>
 * @date May, 2018
 * @brief This work is based on provided codes by Waveshare High-Precision AD-DA raspberry pi hat/shield.
 *
 * @details Usefull to perform Analog to Digital(ADC, using ADS1256 circuit) and Digital to Analog (DAC, using DAC8552).
 * @todo More code especially for DAC8552.
 */

// This Source Code Form is subject to the terms of the
// GNU Lesser General Public License V 3.0.
// If a copy of the GNU GPL was not distributed
// with this file, You can obtain one at https://www.gnu.org/licenses/lgpl-3.0.en.html

/*
             define from bcm2835.h                       define from Board DVK511
                 3.3V | | 5V               ->                 3.3V | | 5V
    RPI_V2_GPIO_P1_03 | | 5V               ->                  SDA | | 5V 
    RPI_V2_GPIO_P1_05 | | GND              ->                  SCL | | GND
       RPI_GPIO_P1_07 | | RPI_GPIO_P1_08   ->                  IO7 | | TX
                  GND | | RPI_GPIO_P1_10   ->                  GND | | RX
       RPI_GPIO_P1_11 | | RPI_GPIO_P1_12   ->                  IO0 | | IO1
    RPI_V2_GPIO_P1_13 | | GND              ->                  IO2 | | GND
       RPI_GPIO_P1_15 | | RPI_GPIO_P1_16   ->                  IO3 | | IO4
                  VCC | | RPI_GPIO_P1_18   ->                  VCC | | IO5
       RPI_GPIO_P1_19 | | GND              ->                 MOSI | | GND
       RPI_GPIO_P1_21 | | RPI_GPIO_P1_22   ->                 MISO | | IO6
       RPI_GPIO_P1_23 | | RPI_GPIO_P1_24   ->                  SCK | | CE0
                  GND | | RPI_GPIO_P1_26   ->                  GND | | CE1

::if your raspberry Pi is version 1 or rev 1 or rev A
RPI_V2_GPIO_P1_03->RPI_GPIO_P1_03
RPI_V2_GPIO_P1_05->RPI_GPIO_P1_05
RPI_V2_GPIO_P1_13->RPI_GPIO_P1_13
::
*/

#ifndef AD_DA_WS_RPI_H_
#define AD_DA_WS_RPI_H_

#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <stdlib.h>

//CS      -----   SPI_CS_ADC1256
//DIN     -----   MOSI
//DOUT  -----   MISO
//SCLK   -----   SCLK
//DRDY  -----   ctl_IO     data  starting
//RST     -----   ctl_IO     reset

#define DRDY RPI_GPIO_P1_11			  //P0
#define RST RPI_GPIO_P1_12			  //P1
#define SPI_CS_ADC1256 RPI_GPIO_P1_15 //P3   ads1256  cs
#define SPI_CS_DAC8552 RPI_GPIO_P1_16 //P4   DAC8552 CS

#define CS_ADC_1() bcm2835_gpio_write(SPI_CS_ADC1256, HIGH)
#define CS_ADC_0() bcm2835_gpio_write(SPI_CS_ADC1256, LOW)

#define CS_DAC_1() bcm2835_gpio_write(SPI_CS_DAC8552, HIGH)
#define CS_DAC_0() bcm2835_gpio_write(SPI_CS_DAC8552, LOW)

#define DRDY_IS_LOW() ((bcm2835_gpio_lev(DRDY) == 0))

#define RST_1() bcm2835_gpio_write(RST, HIGH);
#define RST_0() bcm2835_gpio_write(RST, LOW);

/* Unsigned integer types  */
#define uint8_t unsigned char
#define uint16_t unsigned short
#define uint32_t unsigned long

/**
 * @brief Channel A for the DAC8552.
 * 
 */
#define channel_A 0x30

/**
 * @brief Channel B for the DAC8552.
 * 
 */
#define channel_B 0x34

typedef enum
{
	FALSE = 0,
	TRUE = !FALSE
} bool;

/**
 * @brief Defined ADC gain for channels.
 * 
 */
typedef enum
{
	ADS1256_GAIN_1 = (0),  /* GAIN   1 */
	ADS1256_GAIN_2 = (1),  /*GAIN   2 */
	ADS1256_GAIN_4 = (2),  /*GAIN   4 */
	ADS1256_GAIN_8 = (3),  /*GAIN   8 */
	ADS1256_GAIN_16 = (4), /* GAIN  16 */
	ADS1256_GAIN_32 = (5), /*GAIN    32 */
	ADS1256_GAIN_64 = (6), /*GAIN    64 */
} ADS1256_GAIN_E;

/**
 * @brief Defined data rates for ADC1256. See datasheet table 11 and table 12.
 * 
 */
typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,

	ADS1256_DRATE_MAX
} ADS1256_DRATE_E;

/**
 * @brief If you want your ADC inputs to be single ended (i.e. you have 8 inputs) or differential (i.e. 4 inputs).
 * 
 */
typedef enum
{
	SINGLE_ENDED_INPUTS_8 = (uint8_t)0,
	DIFFERENTIAL_INPUTS_4 = (uint8_t)1,
} ADS1256_SCAN_MODE;

/**
 * @brief Register map of ADS1256. See ADS1256 datasheet table 23.
 * 
 */
enum
{
	/*Register address, followed by reset the default values */
	REG_STATUS = 0, // x1H
	REG_MUX = 1,	// 01H
	REG_ADCON = 2,  // 20H
	REG_DRATE = 3,  // F0H
	REG_IO = 4,		// E0H
	REG_OFC0 = 5,   // xxH
	REG_OFC1 = 6,   // xxH
	REG_OFC2 = 7,   // xxH
	REG_FSC0 = 8,   // xxH
	REG_FSC1 = 9,   // xxH
	REG_FSC2 = 10,  // xxH
};

/* Command definition£º TTable 24. Command Definitions --- ADS1256 datasheet Page 34 */

/**
 * @brief Command definitions. See ADS1256 datasheet table 24. 
 * 
 */
enum
{
	CMD_WAKEUP = 0x00,   // Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA = 0x01,	// Read Data 0000  0001 (01h)
	CMD_RDATAC = 0x03,   // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC = 0x0F,   // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG = 0x10,	 // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG = 0x50,	 // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0,  // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL = 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL = 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3,  // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4,  // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC = 0xFC,	 // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD,  // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET = 0xFE,	// Reset to Power-Up Values 1111   1110 (FEh)
};

static const uint8_t s_tabDataRate[ADS1256_DRATE_MAX] =
	{
		0xF0, /*reset the default values  */
		0xE0,
		0xD0,
		0xC0,
		0xB0,
		0xA1,
		0x92,
		0x82,
		0x72,
		0x63,
		0x53,
		0x43,
		0x33,
		0x20,
		0x13,
		0x03};

/**
 * @brief Cycling throughput (period in microseconds) you can expect for ADS1256 according to SPS setting. See ADS1256 documentation table 14. I took 5% additionnal safety margin.
 * 
 */
static const uint32_t CYCLING_TROUGHPUT_USEC[ADS1256_DRATE_MAX] =
	{
		240,	//approx 1/(4374 Hz)
		275,	//approx 1/3817
		345,	//approx 1/3043
		485,	//approx 1/2165
		730,	//approx 1/1438
		1254,   //approx 1/837
		2303,   //approx 1/456
		10714,  //approx 1/98
		17797,  //approx 1/59
		21000,  //approx 1/50
		35000,  //approx 1/30
		42000,  //approx 1/25
		70000,  //approx 1/15
		105000, //approx 1/10
		210000, //approx 1/5
		420000  //approx 1/2.5
};

/**
 * @brief The so called T18 (in datasheet) settling time (in microseconds). you can expect for ADS1256 according to SPS setting. See ADS1256 documentation table 13.
 * 
 */
static const uint32_t SETTLING_TIME_T18_USEC[ADS1256_DRATE_MAX] =
	{
		210,
		250,
		310,
		440,
		680,
		1180,
		2180,
		10180,
		16840,
		20180,
		33510,
		40180,
		66840,
		100180,
		200180,
		400180};

/**
 * @brief The master clock period time in microseconds multiplied by 24. 24*0.13 # 4 microseconds.
 * 
 */
static const uint64_t MASTER_CLOCK_PERIOD_USEC_TIMES_24 = 4; //24*0.13

/**
 * @brief The master clock period time in microseconds multiplied by 50. 50*0.13 # 7 microseconds.
 * 
 */
static const uint64_t MASTER_CLOCK_PERIOD_USEC_TIMES_50 = 7; //50*0.13

bool DRDYIsLow();
bool DRDYIsHigh();
int ADS1256_WaitDRDY_LOW();
int ADS1256_WaitDRDY_HIGH();
void ADS1256_Send8Bit(uint8_t _data);
int ADS1256_ConfigureADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);
void ADS1256_DelayDATA(void);
uint8_t ADS1256_Receive8Bit(void);
void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue);
uint8_t ADS1256_ReadReg(uint8_t _RegID);
void ADS1256_WriteCmd(uint8_t _cmd);
uint8_t ADS1256_ReadChipID(void);
void ADS1256_SetChannel(uint8_t _ch);
void ADS1256_SetDiffChannel(uint8_t _ch);
int32_t ADS1256_ReadData(void);
uint8_t ADS1256_RemapChannelIndex(uint8_t ch);

int ADS1256_ReadAdcValues(uint8_t **Channels, int NbChannels, ADS1256_SCAN_MODE mode, int32_t **AdcVals);

int ADC_DAC_Init(int *id, ADS1256_GAIN_E aGain, ADS1256_DRATE_E aDrate);
double ADS1256_AdcToMicroVolts(int32_t adcValue, double scalingFactor);
double *ADS1256_AdcArrayToMicroVolts(int32_t *adcValue, int NbVals, double scalingFactor);
int ADC_DAC_Close();

/***************************************************/

void bsp_DelayUS(uint64_t micros);
int WaitCondition(bool (*f)());

/***************************************************/
void DAC8552_Write(uint8_t channel, uint16_t Data);
uint16_t Voltage_Convert(double Vref, double voltage);

#endif // AD_DA_WS_RPI_H_
