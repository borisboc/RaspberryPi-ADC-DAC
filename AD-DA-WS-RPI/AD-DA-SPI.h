/**
 * @file AD-DA-SPI.h
 * @author Boris Bocquet <b.bocquet@akeoplus.com>
 * @date January, 2019
 * @brief The sort of "interface" SPI abstractions must implement. See http://www.airspayce.com/mikem/bcm2835/
 *
 * @details The sort of "interface" SPI abstractions must implement. See http://www.airspayce.com/mikem/bcm2835/
 * @todo 
 */

// This Source Code Form is subject to the terms of the
// GNU Lesser General Public License V 3.0.
// If a copy of the GNU GPL was not distributed
// with this file, You can obtain one at https://www.gnu.org/licenses/lgpl-3.0.en.html

#ifndef AD_DA_SPI_H_
#define AD_DA_SPI_H_

#include "AD-DA-TYPES.h"

void spi_delay_us(uint64_t micros);
uint8_t spi_transfer(uint8_t data);
int spi_init();
int spi_begin();
int spi_init_adc_dac_board();
void spi_end();
int spi_close();

#endif //AD_DA_SPI_H_