/**
*
*
**************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH. All Rights Reserved.
*
* File:		clock_support.c
*
* Date:		2015/02/02
*
* Revision:	1.0
*
* Usage:	Part of BMF055 Data Stream Project
*
**************************************************************************
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
*
*
*************************************************************************/
/*!
*
* @file		clock_support.c
* @author	Bosch Sensortec
*
* @brief	Functions declared in clock_support.h file are defined here.
*
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