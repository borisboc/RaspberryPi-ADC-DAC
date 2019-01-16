/**
 * @file AD-DA-RPI.h
 * @author Boris Bocquet <b.bocquet@akeoplus.com>
 * @date January, 2019
 * @brief The implementation of Raspberry Pi abstraction.
 *
 * @details The implementation of Raspberry Pi abstraction.
 * @todo 
 */

// This Source Code Form is subject to the terms of the
// GNU Lesser General Public License V 3.0.
// If a copy of the GNU GPL was not distributed
// with this file, You can obtain one at https://www.gnu.org/licenses/lgpl-3.0.en.html

#ifndef AD_DA_RPI_H_
#define AD_DA_RPI_H_

#define DRDY RPI_GPIO_P1_11			  //P0
#define RST RPI_GPIO_P1_12			  //P1
#define SPI_CS_ADC1256 RPI_GPIO_P1_15 //P3   ads1256  cs
#define SPI_CS_DAC8552 RPI_GPIO_P1_16 //P4   DAC8552 CS

#endif //AD_DA_RPI_H_