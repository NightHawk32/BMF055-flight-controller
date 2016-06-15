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

#include "clock_support.h"

/************************************************************************/
/* Function Definitions                                                 */
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
void clock_initialize(void)
{
	//clock_configure_dfll();
	//clock_configure_xosc32k();
	clock_configure_osc8m();
	//clock_configure_system_clock ();
	clock_configure_gclk_generator_1();
	clock_configure_gclk_generator_2();
}

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
void clock_configure_dfll(void)
{
	/* Structure for DFLL's configuration */
	struct system_clock_source_dfll_config config_dfll;
	
	/*Get and set the default dfll configuration*/
	system_clock_source_dfll_get_config_defaults(&config_dfll);
	config_dfll.loop_mode = SYSTEM_CLOCK_DFLL_LOOP_MODE_CLOSED;
	//config_dfll.multiply_factor = (48000000 / 32768);
	system_clock_source_dfll_set_config(&config_dfll);
	
	/*Enable DFLL as clock source*/
	enum status_code dfll_status = system_clock_source_enable(SYSTEM_CLOCK_SOURCE_DFLL);

	while (dfll_status != STATUS_OK) {
		/* TODO: Handle clock setting error */
	}
}

void clock_configure_xosc32k(void)
{
	/* Structure for OSC8M's configuration */
	struct system_clock_source_xosc32k_config config_xosc32k;
	
	/* Get the default OSCM8 configuration */
	system_clock_source_xosc32k_get_config_defaults(&config_xosc32k);
	/* Set the prescaler factor to 4 */
	//config_xosc32k.prescaler = SYSTEM_OSC8M_DIV_1;
	
	/* initialize OSC8M with current configuration */
	system_clock_source_xosc32k_set_config(&config_xosc32k);
}

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
void clock_configure_osc8m(void)
{
	/* Structure for OSC8M's configuration */
	struct system_clock_source_osc8m_config config_osc8m;
	
	/* Get the default OSCM8 configuration */
	system_clock_source_osc8m_get_config_defaults(&config_osc8m);
	/* Set the prescaler factor to 4 */
	config_osc8m.prescaler = SYSTEM_OSC8M_DIV_1;
	
	/* initialize OSC8M with current configuration */
	system_clock_source_osc8m_set_config(&config_osc8m);
}

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
void clock_configure_system_clock(void)
{
	/* Set waitstates to 2 as we are using a high system clock rate */
	system_flash_set_waitstates(2);
	
	/* structure for GCLK's configuration */
	struct system_gclk_gen_config config_gclock_gen;
	
	/* Get GLCK configuration defaults */
	system_gclk_gen_get_config_defaults(&config_gclock_gen);
	
	/* Set the DFLL as the clock source for system GCLK */
	config_gclock_gen.source_clock = SYSTEM_CLOCK_SOURCE_DFLL;
	
	/* set devision factor to 2 */
	config_gclock_gen.division_factor = 1;
	
	/* Initialize GCLK0 with current configurations */
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &config_gclock_gen);
}

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
void clock_configure_gclk_generator_1(void)
{
	struct system_gclk_gen_config gclock_gen_conf;
	system_gclk_gen_get_config_defaults(&gclock_gen_conf);
	gclock_gen_conf.source_clock    = SYSTEM_CLOCK_SOURCE_OSC8M;
	//gclock_gen_conf.source_clock = SYSTEM_CLOCK_SOURCE_DFLL;
	//gclock_gen_conf.division_factor = 6;
	
	system_gclk_gen_set_config(GCLK_GENERATOR_1, &gclock_gen_conf);
	system_gclk_gen_enable(GCLK_GENERATOR_1);
}

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
void clock_configure_gclk_generator_2(void)
{
	/* structure for GCLK's configuration */
	struct system_gclk_gen_config gclock_gen_conf;
	
	/* Get GCLK configuration defaults */
	system_gclk_gen_get_config_defaults(&gclock_gen_conf);
	//gclock_gen_conf.source_clock = SYSTEM_CLOCK_SOURCE_DFLL;
	//gclock_gen_conf.division_factor = 6;
	
	/* Initialize GCLK2 with current configurations */
	system_gclk_gen_set_config(GCLK_GENERATOR_2, &gclock_gen_conf);
	
	/* Enable GCLK2 */
	system_gclk_gen_enable(GCLK_GENERATOR_2);
}