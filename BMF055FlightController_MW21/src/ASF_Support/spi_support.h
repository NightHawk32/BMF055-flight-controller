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


#ifndef SPI_SUPPORT_H_
#define SPI_SUPPORT_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include "spi.h"
#include "spi_interrupt.h"

/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/

/*! This is a value of 10 million that can be used to set SPI frequency to 10 MHz. */
#define SPI_BAUDRATE_10M	UINT32_C(10000000)
/*! The default value loaded onto SPI baud rate register is 10 MHz. */
#define SPI_BAUDRATE		SPI_BAUDRATE_10M

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

/*! Instantiates a SERCOM SPI driver software structure, used to retain
* software state information of the associated hardware module instance. */
struct spi_module spi_master_instance;

/************************************************************************/
/* Function Declarations                                                */
/************************************************************************/


/*!
* @brief		Initializes SPI module of the MCU
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void spi_initialize(void);

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
void spi_configure_master(void);

/*!
* @brief		Configures an SPI slave
*
* @param[in]	ss_pin			SPI slave-select pin number
*
* @param[out]	slave_inst_ptr	Pointer to the SPI slave software instance struct
*
* @return		NULL
*
*/
void spi_configure_slave(struct spi_slave_inst *slave_inst_ptr, uint8_t const ss_pin);


#endif /* SPI_SUPPORT_H_ */