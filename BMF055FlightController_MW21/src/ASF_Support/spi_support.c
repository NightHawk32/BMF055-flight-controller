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
