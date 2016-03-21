/**
*
*
**************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH. All Rights Reserved.
*
* File:		bmm050_support.h
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
* @file		bmm050_support.h
* @author	Bosch Sensortec
*
* @brief	BMM050 support defines functions to interface the sensor API
*			with the actual BMM050 magnetometer via SPI.
*			It implements bus read/ write and delay functions that are needed
*			for this communication. It also defines the sensor initialization routine.
*
*
*/


#ifndef BMM050_SUPPORT_H_
#define BMM050_SUPPORT_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include "bmm050.h"
#include "spi_support.h"
#include "tc_support.h"

/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/

/*! BMM050 SPI slave select pin */
#define BMM050_SS_PIN PIN_PA18

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

/*! Instantiates a bmm050 software instance structure, which holds
* relevant information about BMM050 and links communication to the SPI bus. */
struct bmm050 bmm050;
/*! It instantiates an SPI slave software instance structure, used to configure
* the correct SPI transfer mode settings for an attached slave (here BMM050 is the slave).
* For example it holds the SS pin number of the corresponding slave. */
struct spi_slave_inst bmm050_spi_slave;

/************************************************************************/
/* Function Declarations                                                */
/************************************************************************/

/*!
* @brief		Initializes BMM050 magnetometer sensor and its required connections
*
* @param [in]	NULL
*
* @param [out]	NULL
*
* @return		NULL
*
*/
void bmm_init(void);

/*!
* @brief		Sends data to BMM050 via SPI
*
* @param[in]	dev_addr	Device I2C slave address (not used)
*
* @param[in]	reg_addr	Address of destination register
*
* @param[in]	reg_data	Pointer to data buffer to be sent
*
* @param[in]	length		Length of the data to be sent
*
* @retval		0			BMM050_SUCCESS
* @retval		-1			BMM050_ERROR
*
*/
int8_t bmm_spi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t length);

/*!
* @brief		Receives data from BMM050 on SPI
*
* @param[in]	dev_addr	Device I2C slave address (not used)
*
* @param[in]	reg_addr	Address of destination register
*
* @param[out]	reg_data	Pointer to data buffer to be received
*
* @param[in]	length		Length of the data to be received
*
* @retval		0			BMM050_SUCCESS
* @retval		-1			BMM050_ERROR
*
*/
int8_t bmm_spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *rx_data, uint8_t length);

/*!
* @brief		Initiates a delay of the length of the argument in milliseconds
*
* @param[in]	msec	Delay length in terms of milliseconds
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void bmm_delay_msec(uint32_t msec);

#endif /* BMM050_SUPPORT_H_ */