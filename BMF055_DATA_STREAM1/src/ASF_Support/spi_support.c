/**
*
*
**************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH. All Rights Reserved.
*
* File:		spi_support.c
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
* @file		spi_support.c
* @author	Bosch Sensortec
*
* @brief	Functions declared in spi_support.h file are defined here.
*
*
*/


/************************************************************************/
/* Include Own Header                                                   */
/************************************************************************/

#include "spi_support.h"

/************************************************************************/
/* Function Definitions                                                 */
/************************************************************************/

void spi_initialize(void)
{
	spi_configure_master();
}

/*!
* @brief		Configures SPI master module of the MCU
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void spi_configure_master(void)
{
	/* SPI master's configuration structure */
	struct spi_config config_spi_master;
	
	/* get SPI configuration defaults */
	spi_get_config_defaults(&config_spi_master);
	
	/* set SPI Baudrate*/
	config_spi_master.mode_specific.master.baudrate = SPI_BAUDRATE;
	/* Configure pad 0 for data out (MOSI) */
	config_spi_master.pinmux_pad0 = PINMUX_PA16C_SERCOM1_PAD0;
	/* Configure pad 1 as clock (SCKL) */
	config_spi_master.pinmux_pad1 = PINMUX_PA17C_SERCOM1_PAD1;
	/* Configure pad 2 for unused */
	config_spi_master.pinmux_pad2 = PINMUX_UNUSED;
	/* Configure pad 3 for data in (MISO) */
	config_spi_master.pinmux_pad3 = PINMUX_PA19C_SERCOM1_PAD3;
	
	config_spi_master.generator_source = GCLK_GENERATOR_0;
	
	/* initialize SERCOM3 as an SPI master */
	spi_init(&spi_master_instance, SERCOM1, &config_spi_master);
	/* enable the SPI module */
	spi_enable(&spi_master_instance);
}

/*!
* @brief		Configure and initialize software device instance of peripheral slave
*
* @param[in]	ss_pin			SPI slave-select pin number
*
* @param[out]	slave_inst_ptr	Pointer to the SPI slave software instance struct
*
* @return		NULL
*
*/
void spi_configure_slave(struct spi_slave_inst *slave_inst_ptr, uint8_t const ss_pin)
{
	/* SPI slave's configuration structure */
	struct spi_slave_inst_config slave_config;
	
	/* get SPI slave's default configuration */
	spi_slave_inst_get_config_defaults(&slave_config);
	
	/* Assign a slave slect pin */
	slave_config.ss_pin = ss_pin;
	
	/* initialize the SPI slave instance */
	spi_attach_slave(slave_inst_ptr, &slave_config);
}
