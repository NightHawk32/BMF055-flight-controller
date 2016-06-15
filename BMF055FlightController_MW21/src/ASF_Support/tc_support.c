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


/************************************************************************/
/* Include Own Header                                                   */
/************************************************************************/

#include "tc_support.h"
#include "globals.h"

volatile unsigned long tc6_overflows = 0;

/************************************************************************/
/* Function Definitions                                                 */
/************************************************************************/

unsigned long micros(void)
{
	system_interrupt_disable_global();
	uint32_t tc_count = tc_get_count_value(&tc6_instance);
	system_interrupt_enable_global();
	return (((tc6_overflows<<16)+tc_count)*(1/8.0f));
}

unsigned long millis(void)
{
	system_interrupt_disable_global();
	uint32_t tc_count = tc_get_count_value(&tc6_instance);
	system_interrupt_enable_global();
	return (((tc6_overflows<<16)+tc_count)*(1/8000.0));
}

/*!
* @brief		Initializes timer/counter modules of the MCU that are needed
*
* @param[in]	tc6_callback_function	Pointer to the interrupt handler function for TC6
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc_initialize(void)
{
	/* Configure TC4 to generate delays */
	//tc4_configure ();
	//tc4_configure_callbacks();
	
	/* Configure TC6 to schedule tasks */
	tc6_configure();
	tc6_configure_callbacks();
}

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
void tc4_configure (void)
{
	/* TC's configuration structure */
	struct tc_config config_tc;
	
	/* Get TC configuration default */
	tc_get_config_defaults(&config_tc);
	
	/* set TC GLCK */
	config_tc.clock_source = GCLK_GENERATOR_1;
	
	/* Set the initial compare value */
	config_tc.counter_16_bit.compare_capture_channel[TC_COMPARE_CAPTURE_CHANNEL_0] = 500;
	
	/* initialize TC4 with current configurations */
	tc_init(&tc4_instance, TC4, &config_tc);
	
	/* enable the TC module */
	tc_enable(&tc4_instance);
	
	/* Stop the counter */
	tc_stop_counter(&tc4_instance);
}

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
void tc4_configure_callbacks (void)
{
	tc_register_callback(&tc4_instance, tc4_callback, TC_CALLBACK_CC_CHANNEL0);
	tc4_callback_flag = false;
	tc_enable_callback(&tc4_instance, TC_CALLBACK_CC_CHANNEL0);
}

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
void tc4_callback (struct tc_module *const module_inst_ptr)
{
	/* Set the corresponding interrupt flag */
	tc4_callback_flag = true;
}

/*!
* @brief		Implements a delay of the length of the argument in 
econds
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc4_wait_for_msec (uint32_t msec)
{
	/* Set the compare value */
	tc_set_compare_value(&tc4_instance, TC_COMPARE_CAPTURE_CHANNEL_0, (msec *500));
	
	/* start counting */
	tc_start_counter(&tc4_instance);
	
	/* delay until required time is elapsed */
	while (!tc4_callback_flag);
	
	/* stop the counter */
	tc_stop_counter(&tc4_instance);
	/* reset the interrupt flag */
	tc4_callback_flag = false;
}

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
void tc6_configure(void)
{
	/* TC's configuration structure */
	struct tc_config config_tc;
	
	/* Get TC configuration default */
	tc_get_config_defaults(&config_tc);
	
	/* set the counter size */
	config_tc.counter_size = TC_COUNTER_SIZE_16BIT;
	/* set TC GLCK */
	config_tc.clock_source = GCLK_GENERATOR_1;
	
	/* initialize TC6 with current configurations */
	tc_init(&tc6_instance,TC6,&config_tc);
	
	/* enable the TC module */
	tc_enable(&tc6_instance);
}

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
void tc6_configure_callbacks(void)
{
	tc_register_callback(&tc6_instance, tc6_callback, TC_CALLBACK_OVERFLOW);
	//tc6_callback_flag = false;
	tc_enable_callback(&tc6_instance, TC_CALLBACK_OVERFLOW);
}

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
void tc6_callback (struct tc_module *const module_inst_ptr)
{	
	/* Set the corresponding interrupt flag */
	//tc6_callback_flag = true;
	tc6_overflows++;
}

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
void tc6_stop_counter (void)
{
	tc_stop_counter(&tc6_instance);
}

/*!
* @brief		Starts TC6 counter
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void tc6_start_counter (void)
{
	tc_set_count_value(&tc6_instance, TC6_COUNT_VALUE);
	tc_start_counter(&tc6_instance);
}
