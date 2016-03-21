/**
*
*
**************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH. All Rights Reserved.
*
* File:		spi_support.h
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
* @file		spi_support.h
* @author	Bosch Sensortec
*
* @brief	TC ASF Driver Support Header File
*
* TC support uses ASF timer/counter driver modules and defines
* initialization, configuration and callback functions for
* the microcontroller’s timer/counter peripherals that that are needed
* for scheduling tasks or initiating delays. In addition to these
* some wrapper functions are defined that are needed.
*
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