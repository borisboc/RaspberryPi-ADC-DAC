/**
 * @file mainTestAdda.c
 * @author Boris Bocquet <b.bocquet@akeoplus.com>
 * @date May, 2018
 * @brief Testing AD-DA-WS-RPI lib
 *
 * @details Usefull to perform Analog to Digital(ADC, using ADS1256 circuit) and Digital to Analog (DAC, using DAC8552).
 * @todo More code especially for DAC8552.
 */

// This Source Code Form is subject to the terms of the
// GNU Lesser General Public License V 3.0.
// If a copy of the GNU GPL was not distributed
// with this file, You can obtain one at https://www.gnu.org/licenses/lgpl-3.0.en.html

#include "AD-DA-WS-RPI/AD-DA-WS-RPI.h"

/**
 * @brief Simple code to get the voltage on each 8 inputs (single ended) using the ADC1256 and write the input 1 value to output 1 using DAC8552.
 * You can use the jumper of the waveshare board to connect to 5V (power supply and vref), and connect ADO to ADJ (the potentiometer) and DAC to LEDA.
 * 
 * @return int 
 */
int main()
{
	printf("Starting Test ADDA\r\n");

	int initBcm = bcm2835_init();
	if (initBcm != 1)
	{
		printf("bcm init failed with code %d", initBcm);
		return -55;
	}

	//Prepare an array with the 8 single ended inputs to read [0,1,2,3,4,5,6,7]
	int NbChannels = 8;
	int MainLoop = 0;
	int RetCode = 0;
	uint8_t *Channels = malloc(NbChannels * sizeof(uint8_t));
	int Ch;
	for (Ch = 0; Ch < NbChannels; Ch++)
	{
		*(Channels + Ch) = Ch;
	}

	//Write 0 to ouputs A and B
	DAC8552_Write(channel_A, Voltage_Convert(5.0, 0));
	DAC8552_Write(channel_B, Voltage_Convert(5.0, 0));

	while (1 == 1)
	{

		printf("ADC_DAC_Init\r\n");
		int Id = 0;
		int Init = ADC_DAC_Init(&Id, ADS1256_GAIN_1, ADS1256_100SPS);
		if (Init != 0)
		{
			RetCode = -1;
			break;
		}
		printf("init done !\r\n");
		int Loop;
		for (Loop = 0; Loop < 100; Loop++)
		{
			int32_t *AdcValues = NULL;
			ADS1256_ReadAdcValues(&Channels, NbChannels, SINGLE_ENDED_INPUTS_8, &AdcValues);

			double *volt = ADS1256_AdcArrayToMicroVolts(AdcValues, NbChannels, 1.0 / 1000000.0);

			printf("0 : %f V   1 : %f V   2 : %f V   3 : %f V   4 : %f V   5 : %f V   6 : %f V   7 : %f V \r\n",
				   volt[0], volt[1], volt[2], volt[3], volt[4], volt[5], volt[6], volt[7]);

			DAC8552_Write(channel_A, Voltage_Convert(5.0, volt[0]));

			free(AdcValues);
			free(volt);

			bsp_DelayUS(1000);
		}

		//Write 0 to ouputs A and B
		DAC8552_Write(channel_A, Voltage_Convert(5.0, 0));
		DAC8552_Write(channel_B, Voltage_Convert(5.0, 0));

		printf("ADC_DAC_Close\r\n");
		int CloseCode = ADC_DAC_Close();
		if (CloseCode != 0)
		{
			RetCode = -2;
			break;
		}

		MainLoop++;

		//This loop proves that you can close and re-init pacefully the librairie. Prove it several times (e.g. 3) and then finish the code.
		if (MainLoop >= 3)
			break;
	}

	printf("Test ADDA finished with returned code %d\r\n", RetCode);

	return RetCode;
}
