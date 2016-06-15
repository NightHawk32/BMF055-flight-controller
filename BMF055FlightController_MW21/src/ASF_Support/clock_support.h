/*
 * This file is part of the MW21 adaption for BMF055.
 *
 * BMF055 flight controller is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BMF055 flight controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BMF055 flight controller.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#ifndef CLOCK_SUPPORT_H_
#define CLOCK_SUPPORT_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include <clock.h>
#include <gclk.h>

/************************************************************************/
/* Function Declarations                                                */
/************************************************************************/

/*!
* @brief		Initializes clock sources, generic clock generators and system main clock of the MCU
*
* This function calls configuration functions for DFLL48M and OSC8M clock sources,
* generic clock generators 1 and 2 and0 the main clock of the system (generic clock generator 0).
* After initialization, the clock sources’ and generic clock generators’ frequencies are as follows:
*
*	Clock Source OSC8M   :	8 MHz clock source of MCU used as the source for multiple modules (Frequency 2 MHz)
*	Clock Source DFLL48M :	DFLL clock source of MCU used as the source for multiple modules (Frequency 48 MHz – Open Loop)
*	GCLK 0				 :	Generates the main system clock, using DFLL as its source (Frequency 24 MHz)
*	GCLK 1				 :	Generates clock signal for TC4, using OSC8M as its source (Frequency 500 KHz)
*	GCLK 2				 :	Generates clock signal for USART, using OSC8M as its source (Frequency 2 MHz)
*
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void clock_initialize(void);

/*!
* @brief		Configures the DFLL48M clock source of the MCU
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void clock_configure_dfll(void);

void clock_configure_xosc32k(void);
/*!
* @brief		Configures the 8 MHz internal clock source of the MCU
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void clock_configure_osc8m(void);

/*!
* @brief		Configures system main clock source
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void clock_configure_system_clock(void);

/*!
* @brief		Configures generic clock generator 1
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void clock_configure_gclk_generator_1(void);

/*!
* @brief		Configures generic clock generator 2
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void clock_configure_gclk_generator_2(void);


#endif /* CLOCK_SUPPORT_H_ */