/**
 * @file AD-DA-BCM.c
 * @author Boris Bocquet <b.bocquet@akeoplus.com>
 * @date January, 2019
 * @brief The implementation of SPI abstraction using BCM2835 library. See http://www.airspayce.com/mikem/bcm2835/
 *
 * @details The implementation of SPI abstraction using BCM2835 library. See http://www.airspayce.com/mikem/bcm2835/
 * @todo
 */

 // This Source Code Form is subject to the terms of the
 // GNU Lesser General Public License V 3.0.
 // If a copy of the GNU GPL was not distributed
 // with this file, You can obtain one at https://www.gnu.org/licenses/lgpl-3.0.en.html

#include "AD-DA-BCM.h"

/**
 * @brief @sa bcm2835_delayMicroseconds.
 *
 * @param dalay expressed in microseconds.
 */
void spi_delay_us(uint64_t micros)
{
	bcm2835_delayMicroseconds(micros);
}

/**
 * @brief @sa bcm2835_spi_transfer.
 *
 * @param data to be transfered.
 * @return answer.
 */
uint8_t spi_transfer(uint8_t data)
{
	return bcm2835_spi_transfer(data);
}

/**
 * @brief @sa bcm2835_init.
 *
 * @return int
 */
int spi_init()
{
	return bcm2835_init();
}

/**
 * @brief @sa bcm2835_spi_begin.
 *
 * @return int
 */
int spi_begin()
{
	return bcm2835_spi_begin();
}

/**
 * @brief Every actions that should be performed in order to use the ADS1256 and the DAC8552 properly. E.g. setting modes, frequencies and wiring.
 *
 * @return int. 1 is "no error".
 */
int spi_init_adc_dac_board()
{
	//SPI params
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

	return 1;
}

/**
 * @brief @sa bcm2835_spi_end.
 *
 */
void spi_end()
{
	bcm2835_spi_end();
}

/**
 * @brief @sa bcm2835_close.
 *
 * @return int
 */
int spi_close()
{
	return bcm2835_close();
}

