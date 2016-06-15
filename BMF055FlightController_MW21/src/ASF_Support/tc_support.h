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



#ifndef TC_SUPPORT_H_
#define TC_SUPPORT_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include <tc.h>
#include <tc_interrupt.h>

/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/

/*! Maximum value of a 32-bit counter */
#define COUNT_MAX_32BIT			UINT32_C(0xFFFFFFFF)
/*! TC6 count value to overflow after 1000 milliseconds */
#define TC6_PERIOD_1000MS		COUNT_MAX_32BIT - UINT32_C(500000)
/*! TC6 count value to overflow after 100 milliseconds */
#define TC6_PERIOD_100MS		COUNT_MAX_32BIT - UINT32_C(50000)
/*! TC6 count value to overflow after 10 milliseconds */
#define TC6_PERIOD_10MS			COUNT_MAX_32BIT - UINT32_C(5000)
/*! the value loaded onto TC6 count register */
#define TC6_COUNT_VALUE			TC6_PERIOD_100MS

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

/*! Instantiates a TC software instance structure, used to retain
* software state information of the associated hardware module instance TC4 */
struct tc_module tc4_instance;
/*! Instantiates a TC software instance structure, used to retain
* software state information of the associated hardware module instance TC6*/
struct tc_module tc6_instance;

/*! Callback flag for TC4 (Compare Flag) */
volatile bool tc4_callback_flag;
/*! Callback flag for TC6 (Compare Flag) */
volatile bool tc6_callback_flag;

/************************************************************************/
/* Function Declarations                                                */
/************************************************************************/

/*!
* @brief		Initializes TC4 and TC6 timer/counter modules of the MCU
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc_initialize(void);

/*!
* @brief		Configures TC4 of the MCU
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc4_configure(void);

/*!
* @brief		Configures TC4 callback register
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc4_configure_callbacks(void);

/*!
* @brief		Called when TC4 counter is equal to its capture channel 0 value
*
* @param[in]	module_inst_ptr	Pointer to the TC module which triggered the interrupt
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc4_callback(struct tc_module *const module_inst_ptr);

/*!
* @brief		Implements a delay of the length of the argument in milliseconds
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc4_wait_for_msec(uint32_t msec);

/*!
* @brief		Configures TC6 of the MCU
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc6_configure(void);

/*!
* @brief		Configures TC6 callback register
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc6_configure_callbacks(void);

/*!
* @brief		Called when TC6 counter is equal to its capture channel 0 value
*
* @param[in]	module_inst_ptr	Pointer to the TC module which triggered the interrupt
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc6_callback(struct tc_module *const module_inst_ptr);

/*!
* @brief		Stops TC6 counter
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc6_stop_counter(void);

/*!
* @brief		Starts TC3 counter
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc6_start_counter(void);


#endif /* TC_SUPPORT_H_ */

