/**
 * @file AD-DA-BCM.h
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

//CS      -----   SPI_CS_ADC1256
//DIN     -----   MOSI
//DOUT  -----   MISO
//SCLK   -----   SCLK
//DRDY  -----   ctl_IO     data  starting
//RST     -----   ctl_IO     reset

#ifndef AD_DA_BCM_H_
#define AD_DA_BCM_H_

//#ifdef ADDA_WS_RPI_USE_BCM_SPI

#include "AD-DA-TYPES.h"
#include "AD-DA-SPI.h"
#include "AD-DA-RPI.h"
#include <bcm2835.h>

#define CS_ADC_1() bcm2835_gpio_write(SPI_CS_ADC1256, HIGH)
#define CS_ADC_0() bcm2835_gpio_write(SPI_CS_ADC1256, LOW)

#define CS_DAC_1() bcm2835_gpio_write(SPI_CS_DAC8552, HIGH)
#define CS_DAC_0() bcm2835_gpio_write(SPI_CS_DAC8552, LOW)

#define DRDY_IS_LOW() ((bcm2835_gpio_lev(DRDY) == 0))

#define RST_1() bcm2835_gpio_write(RST, HIGH);
#define RST_0() bcm2835_gpio_write(RST, LOW);

//#endif //ADDA_WS_RPI_USE_BCM_SPI

#endif //AD_DA_BCM_H_
