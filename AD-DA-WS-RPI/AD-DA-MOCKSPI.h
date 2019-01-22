/**
 * @file AD-DA-MOCKSPI.h
 * @author Boris Bocquet <b.bocquet@akeoplus.com>
 * @date January, 2019
 * @brief A mock for SPI interface. Simply prints in console.
 *
 * @details A mock for SPI interface. Simply prints in console.
 * @todo 
 */

// This Source Code Form is subject to the terms of the
// GNU Lesser General Public License V 3.0.
// If a copy of the GNU GPL was not distributed
// with this file, You can obtain one at https://www.gnu.org/licenses/lgpl-3.0.en.html

#ifndef AD_DA_MOCKSPI_H_
#define AD_DA_MOCKSPI_H_

#include "AD-DA-TYPES.h"
#include "AD-DA-SPI.h"
#include <stdio.h>

//int cpt_drdy = 0;
bool spi_mock_modulo_cpt(int modulo);

#define CS_ADC_1() printf("CS_ADC_1()\n")
#define CS_ADC_0() printf("CS_ADC_0()\n")

#define CS_DAC_1() printf("CS_DAC_1()\n")
#define CS_DAC_0() printf("CS_DAC_0()\n")

#define DRDY_IS_LOW() (spi_mock_modulo_cpt(5))

#define RST_1() printf("RST_1()\n")
#define RST_0() printf("RST_0()\n")

#endif //AD_DA_MOCKSPI_H_
