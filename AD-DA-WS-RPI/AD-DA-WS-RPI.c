/**
 * @file AD-DA-WS-RPI.c
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

#include "AD-DA-WS-RPI.h"

/**
 * @brief Wait microseconds. @sa bcm2835_delayMicroseconds.
 * 
 * @param micros 
 */
void bsp_DelayUS(uint64_t micros)
{
	bcm2835_delayMicroseconds(micros);
}

/**
 * @brief Returns if /DRDY is currently low.
 * 
 * @return true : /DRDY is currently low.
 * @return false : /DRDY is NOT currently low (i.e. it is high).
 */
bool DRDYIsLow()
{
	return DRDY_IS_LOW();
}

/**
 * @brief Returns if /DRDY is currently high.
 * 
 * @return true : /DRDY is currently high.
 * @return false : /DRDY is NOT currently high (i.e. it is low).
 */
bool DRDYIsHigh()
{
	return !(DRDY_IS_LOW());
}

/**
 * @brief Wait for /DRDY input to be low. 
 * 
 * @return 0 : Wait successful.
 * @return -1 : A timeout occured. @sa WaitCondition.
 */
int ADS1256_WaitDRDY_LOW()
{
	return WaitCondition(DRDYIsLow);
}

/**
 * @brief Wait for /DRDY input to be high. 
 * 
 * @return 0 : Wait successful.
 * @return -1 : A timeout occured. @sa WaitCondition.
 */
int ADS1256_WaitDRDY_HIGH()
{
	return WaitCondition(DRDYIsHigh);
}

/**
 * @brief Simple for loop waiting for a condition (given in the function as argument) to be true.
 * 
 * @param f : the function evaluated. If f returns true, the loop stops and return 0.
 * @return 0 : the function returned true (after a while).
 * @return -1 :  timeout : the function never returned true even after several microseconds.
 */
int WaitCondition(bool (*f)())
{
	int i;
	for (i = 0; i < 410002; i++) //TODO find a better way. E.g. with time.h clock_gettime as you know you shall timeout if time is over CYCLING_TROUGHPUT_USEC
	{
		if ((*f)())
			return 0;

		bcm2835_delayMicroseconds(1);
	}

	return -1;
}

/**
 * @brief SPI bus to send 8 bit data.
 * 
 * @param _data : what to send
 */
void ADS1256_Send8Bit(uint8_t _data)
{
	bsp_DelayUS(2);
	bcm2835_spi_transfer(_data);
}

/**
 * @brief The configuration parameters of ADC, gain and data rate.
 * 
 * @param _gain : gain 1-64
 * @param _drate : data rate. @sa ADS1256_DRATE_E
 * @return 0 : success
 * @return -1 : ADS1256_WaitDRDY_LOW failed. 
 */
int ADS1256_ConfigureADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
	int w = ADS1256_WaitDRDY_LOW();
	if (w != 0)
	{
		printf("In ADS1256_ConfigureADC, ADS1256_WaitDRDY_LOW returned %d\r\n", w);
		return -1;
	}

	{
		uint8_t buf[4]; /* Storage ads1256 register configuration parameters */

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
		buf[0] = (0 << 3) | (1 << 2) | (0 << 1); // The internal buffer is prohibited

		//ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

		buf[1] = 0x08;

		/*	ADCON: A/D Control Register (Address 02h)
			Bit 7 Reserved, always 0 (Read Only)
			Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
				00 = Clock Out OFF
				01 = Clock Out Frequency = fCLKIN (default)
				10 = Clock Out Frequency = fCLKIN/2
				11 = Clock Out Frequency = fCLKIN/4
				When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.

			Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources
				00 = Sensor Detect OFF (default)
				01 = Sensor Detect Current = 0.5 ŠÌ A
				10 = Sensor Detect Current = 2 ŠÌ A
				11 = Sensor Detect Current = 10ŠÌ A
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
		buf[2] = (0 << 5) | (0 << 3) | (_gain << 0);
		//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));	/*choose 1: gain 1 ;input 5V/
		buf[3] = s_tabDataRate[_drate]; // e.g. DRATE_10SPS;

		CS_ADC_0();						/* SPIÆ¬Ñ¡ = 0 */
		ADS1256_Send8Bit(CMD_WREG | 0); /* Write command register, send the register address */
		ADS1256_Send8Bit(0x03);			/* Register number 4,Initialize the number  -1*/

		ADS1256_Send8Bit(buf[0]); /* Set the status register */
		ADS1256_Send8Bit(buf[1]); /* Set the input channel parameters */
		ADS1256_Send8Bit(buf[2]); /* Set the ADCON control register,gain */
		ADS1256_Send8Bit(buf[3]); /* Set the output rate */

		CS_ADC_1(); /* SPI  cs = 1 */
	}

	bsp_DelayUS(50);

	return 0;
}

/**
 * @brief The necessarry delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands. 
 * it represents 50 x the clock period.
 * 
 */
void ADS1256_DelayDATA(void)
{
	/*
		Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
		min  50   CLK = 50 * 0.13uS = 6.5uS
	*/
	bsp_DelayUS(MASTER_CLOCK_PERIOD_USEC_TIMES_50);
}

/**
 * @brief SPI bus receive function.
 * 
 * @return uint8_t : the received bits.
 */
uint8_t ADS1256_Receive8Bit(void)
{
	uint8_t read = 0;
	read = bcm2835_spi_transfer(0xff);
	return read;
}

/**
 * @brief Write the corresponding register. See WREG: Write to Register figure 34 in ADS1256 datasheet.
 * 
 * @param _RegID : register ID.
 * @param _RegValue : register value to be written.
 */
void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
	CS_ADC_0();							 /* SPI  cs  = 0 */
	ADS1256_Send8Bit(CMD_WREG | _RegID); /*Write command register */
	ADS1256_Send8Bit(0x00);				 /*Write the register number */

	ADS1256_Send8Bit(_RegValue); /*send register value */
	CS_ADC_1();					 /* SPI   cs = 1 */
}

/**
 * @brief Read  the corresponding register. See RREG: Read from Registers figure 33 in ADS1256 datasheet.
 * 
 * @param _RegID : register ID.
 * @return uint8_t : read value in the register.
 */
uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	CS_ADC_0();							 /* SPI  cs  = 0 */
	ADS1256_Send8Bit(CMD_RREG | _RegID); /* Write command register */
	ADS1256_Send8Bit(0x00);				 /* Write the register number */

	ADS1256_DelayDATA(); /*delay time */

	read = ADS1256_Receive8Bit(); /* Read the register values */
	CS_ADC_1();					  /* SPI   cs  = 1 */

	return read;
}

/**
 * @brief Sending a single byte order.
 * 
 * @param _cmd : command to send.
 */
void ADS1256_WriteCmd(uint8_t _cmd)
{
	CS_ADC_0(); /* SPI   cs = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_ADC_1(); /* SPI  cs  = 1 */
}

/**
 * @brief Read the chip ID.
 * 
 * @return uint8_t : The chip id.
 */
uint8_t ADS1256_ReadChipID(void)
{
	uint8_t id;
	id = ADS1256_ReadReg(REG_STATUS);
	return (id >> 4);
}

/**
 * @brief Configure the single ended channel number for the multiplexer (MUX).
 * 
 * @param channel : The channel to read the value on.
 */
void ADS1256_SetChannel(uint8_t channel)
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
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ¡°don¡¯t care¡±)

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
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ¡°don¡¯t care¡±)
	*/

	uint8_t _ch = ADS1256_RemapChannelIndex(channel); //it seems that there is either a wiring error or a software error in the channels. So I remap the correct this error 'manu militari'. BB 20180508

	if (_ch > 7)
	{
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3)); /* Bit3 = 1, AINN connection AINCOM */
}

/**
 * @brief Configure the differential channel number for the multiplexer (MUX).
 * 
 * @param channel : The channel to read the value on.
 */
void ADS1256_SetDiffChannel(uint8_t channel)
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
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ¡°don¡¯t care¡±)

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
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ¡°don¡¯t care¡±)
	*/

	uint8_t _ch = ADS1256_RemapChannelIndex(channel); //it seems that there is either a wiring error or a software error in the channels. So I remap the correct this error 'manu militari'. BB 20180508

	if (_ch == 0)
	{
		ADS1256_WriteReg(REG_MUX, (0 << 4) | 1); /* DiffChannal  AIN0£¬ AIN1 */
	}
	else if (_ch == 1)
	{
		ADS1256_WriteReg(REG_MUX, (2 << 4) | 3); /*DiffChannal   AIN2£¬ AIN3 */
	}
	else if (_ch == 2)
	{
		ADS1256_WriteReg(REG_MUX, (4 << 4) | 5); /*DiffChannal    AIN4£¬ AIN5 */
	}
	else if (_ch == 3)
	{
		ADS1256_WriteReg(REG_MUX, (6 << 4) | 7); /*DiffChannal   AIN6£¬ AIN7 */
	}
}

/**
 * @brief read ADC value. Please do not use this : call ADS1256_ReadAdcValues instead. @sa ADS1256_ReadAdcValues.
 * 
 * @return int32_t : the ADC value.
 */
int32_t ADS1256_ReadData(void)
{
	uint32_t read = 0;
	static uint8_t buf[3];

	CS_ADC_0(); /* SPI   cs = 0 */

	ADS1256_Send8Bit(CMD_RDATA); /* read ADC command  */

	ADS1256_DelayDATA(); /*delay time  */

	/*Read the sample results 24bit*/
	buf[0] = ADS1256_Receive8Bit();
	buf[1] = ADS1256_Receive8Bit();
	buf[2] = ADS1256_Receive8Bit();

	read = ((uint32_t)buf[0] << 16) & 0x00FF0000;
	read |= ((uint32_t)buf[1] << 8); /* Pay attention to It is wrong   read |= (buf[1] << 8) */
	read |= buf[2];

	CS_ADC_1(); /* SPIÆ¬Ñ¡ = 1 */

	/* Extend a signed number*/
	if (read & 0x800000)
	{
		read |= 0xFF000000;
	}

	return (int32_t)read;
}

/**
 * @brief Complete sequence to get the current ADC value on a channel / input. See figure 19 in ADS1256 datasheet.
 * 
 * @param Channels : Pass an array whith all channels values you want to read. e.g. [0,1,2,3].
 * @param NbChannels : The size of array Channels.
 * @param mode : Scan mode of the inputs (single ended or differential).
 * @param AdcVals : Current read values of each channel.
 * @return 0 : no error. 
 * @return -1 : ADS1256_WaitDRDY_LOW failed.
 */
int ADS1256_ReadAdcValues(uint8_t **Channels, int NbChannels, ADS1256_SCAN_MODE mode, int32_t **AdcVals)
{
	*AdcVals = malloc(NbChannels * sizeof(int32_t));

	int i;
	for (i = 0; i < NbChannels; i++)
	{
		int w = ADS1256_WaitDRDY_LOW();
		if (w != 0)
		{
			printf("In ADS1256_ReadAdcValues, ADS1256_WaitDRDY_LOW returned %d\r\n", w);
			return -1;
		}

		uint8_t CurChannel = *(*Channels + i);

		if (mode == SINGLE_ENDED_INPUTS_8)
			ADS1256_SetChannel(CurChannel);
		else
			ADS1256_SetDiffChannel(CurChannel);

		bsp_DelayUS(MASTER_CLOCK_PERIOD_USEC_TIMES_24);

		ADS1256_WriteCmd(CMD_SYNC);
		bsp_DelayUS(MASTER_CLOCK_PERIOD_USEC_TIMES_24);

		ADS1256_WriteCmd(CMD_WAKEUP);
		bsp_DelayUS(MASTER_CLOCK_PERIOD_USEC_TIMES_24);

		int32_t RD = ADS1256_ReadData();
		*(*AdcVals + i) = RD;
	}

	return 0;
}

/**
 * @brief It looks like there is either a wiring error on the bord or in this code (please help) 
 * because when you ask channel 0, you have channel 1 ... channel 7 is channel 0. 
 * I don't know why. Boris Bocquet 08/05/2018
 * 
 * @param ch The channel you wired your signal to 
 * @return uint8_t The remapped channel to compensate the wiring / code error.
 */
uint8_t ADS1256_RemapChannelIndex(uint8_t ch)
{
	switch (ch)
	{
	case 0:
		return 1;
	case 1:
		return 2;
	case 2:
		return 3;
	case 3:
		return 4;
	case 4:
		return 5;			
	case 5:
		return 6;
	case 6:
		return 7;
	case 7:
		return 0;
	default:
		return 0;
	}
}

/**
 * @brief Init the ADS1256 and DAC 8552 SPI communication and configure the conversion (gain and data rate).
 * 
 * @param id : the returned chip ID. ID should be == 3.
 * @param aGain : Channel gain. @sa ADS1256_ConfigureADC.
 * @param aDrate : Data rate. @sa ADS1256_ConfigureADC.
 * @return 0 : sucess.
 * @return -1 : bcm2835_init failed.
 * @return -2 : bcm2835_spi_begin failed.
 * @return -3 : ADS1256_ReadChipID failed (id should be == 3).
 */
int ADC_DAC_Init(int *id, ADS1256_GAIN_E aGain, ADS1256_DRATE_E aDrate)
{
	int initBcm = bcm2835_init();
	if (initBcm != 1)
	{
		printf("In ADS1256_Init, bcm2835_init returned %d\r\n", initBcm);
		return -1;
	}

	int spiBegin = bcm2835_spi_begin();

	if (spiBegin != 1)
	{
		printf("In ADS1256_Init, bcm2835_spi_begin returned %d\r\n", spiBegin);
		return -2;
	}

	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);	 //Since bcm2835 V1.56
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);					 // The default
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024); // The default

	//ADS1256
	bcm2835_gpio_fsel(SPI_CS_ADC1256, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(SPI_CS_ADC1256, HIGH);
	bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP);

	//DAC8552
	bcm2835_gpio_fsel(SPI_CS_DAC8552, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(SPI_CS_DAC8552, HIGH);

	//ADS1256_WriteReg(REG_MUX,0x01);
	//ADS1256_WriteReg(REG_ADCON,0x20);

	*id = ADS1256_ReadChipID();

	if (*id != 3)
	{
		printf("In ADS1256_Init, Error, ASD1256 Chip ID = 0x%d which is != 3\r\n", (int)(*id));
		return -3;
	}

	ADS1256_ConfigureADC(aGain, aDrate);

	return 0;
}

/**
 * @brief Convert ADC value (from ADS1256_ReadAdcValues) to voltage uV value. @sa ADS1256_ReadAdcValues.
 * 
 * @param adcValue : read with ADS1256_ReadAdcValues.
 * @param scalingFactor : if you want the microvolt value to be scaled. E.g. pass 1.0/1000000.0 to have Volts.
 * @return double : The scaled microvolt value.
 */
double ADS1256_AdcToMicroVolts(int32_t adcValue, double scalingFactor)
{
	//24 bits randing from -5 to 5v
	double uV = scalingFactor * ((double)(adcValue)) * 5000000.0 / 8388608; //5 volts on 23 bits (which is approximatly 100/167 as you can sse in Waveshare original code
	return uV;
}

/**
 * @brief Same as ADS1256_AdcToMicroVolts but for an array. @sa ADS1256_AdcToMicroVolts.
 * 
 * @param adcValue : Same as ADS1256_AdcToMicroVolts but for an array. @sa ADS1256_AdcToMicroVolts.
 * @param NbVals : Number of passed values in adcValue.
 * @param scalingFactor :  
 * @return double* : Same as ADS1256_AdcToMicroVolts but for an array. @sa ADS1256_AdcToMicroVolts.
 */
double *ADS1256_AdcArrayToMicroVolts(int32_t *adcValue, int NbVals, double scalingFactor)
{
	double *microVoltVals = malloc(NbVals * sizeof(double));

	int i;
	for (i = 0; i < NbVals; i++)
	{
		*(microVoltVals + i) = ADS1256_AdcToMicroVolts(*(adcValue + i), scalingFactor);
	}

	return microVoltVals;
}

/**
 * @brief STANDBY the ADS1256 and close SPI communication.
 * 
 * @return 0 : no error.
 * @return -1 : bcm2835_close failed.
 */
int ADC_DAC_Close()
{
	//Not 100% sure this is a good idea to call CMD_STANDBY
	ADS1256_WriteCmd(CMD_STANDBY);
	bsp_DelayUS(25);

	bcm2835_spi_end();
	int closingBcm = bcm2835_close();

	if (closingBcm != 1)
	{
		printf("In ADS1256_Close, bcm2835_close returned %d\r\n", closingBcm);
		return -1;
	}

	return 0;
}

/**
 * @brief DAC8552 send data.
 * 
 * @param channel : output where to send the data. 
 * @param Data : data to convert to analog value.
 */
void DAC8552_Write(uint8_t channel, uint16_t Data)
{
	CS_DAC_1();
	CS_DAC_0();
	bcm2835_spi_transfer(channel);
	bcm2835_spi_transfer((Data >> 8));
	bcm2835_spi_transfer((Data & 0xff));
	CS_DAC_1();
}

/**
 * @brief Voltage value conversion function.
 * 
 * @param Vref  : The reference voltage 3.3V or 5V.
 * @param voltage : Desired voltage to send (in volts).
 * @return uint16_t : output DAC value that you can write with DAC8552_Write.
 */
uint16_t Voltage_Convert(double Vref, double voltage)
{
	uint16_t _D_;
	_D_ = (uint16_t)(65536 * voltage / Vref);

	return _D_;
}
