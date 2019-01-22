/**
 * @file AD-DA-TYPES.h
 * @author Boris Bocquet <b.bocquet@akeoplus.com>
 * @date January, 2019
 * @brief The usefull types, defines, common to all (several) files in this projet.
 *
 * @details The usefull types, defines, common to all (several) files in this projet.
 * @todo 
 */

// This Source Code Form is subject to the terms of the
// GNU Lesser General Public License V 3.0.
// If a copy of the GNU GPL was not distributed
// with this file, You can obtain one at https://www.gnu.org/licenses/lgpl-3.0.en.html

#ifndef AD_DA_TYPES_H_
#define AD_DA_TYPES_H_

#include "stdint.h"
#include "stdbool.h"

///* Unsigned integer types  */
//#define uint8_t unsigned char
//#define uint16_t unsigned short
//#define uint32_t unsigned long

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

//typedef enum
//{
//	FALSE = 0,
//	TRUE = !FALSE
//} bool;

#endif //AD_DA_TYPES_H_
