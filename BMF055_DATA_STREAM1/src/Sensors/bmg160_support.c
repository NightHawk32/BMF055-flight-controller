/**
*
*
**************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH. All Rights Reserved.
*
* File:		bmg160_support.c
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
* @file		bmg160_support.c
* @author	Bosch Sensortec
*
* @brief	Functions declared in bmg160_support.h file are defined here.
*
*
*/


/************************************************************************/
/* Include Own Header                                                   */
/************************************************************************/

#include "bmg160_support.h"

/************************************************************************/
/* Function Definitions                                                 */
/************************************************************************/

/*!
* @brief		Initializes BMA280 accelerometer sensor and its required connections
*
* @param [in]	NULL
*
* @param [out]	NULL
*
* @return		NULL
*
*/
void bmg_init(void)
{
	/* Configure an SPI slave instance for the sensor */
	spi_configure_slave(&bmg160_spi_slave, BMG160_SS_PIN);
	
	/* Assign functions required by sensor API */
	bmg160.bus_write = &bmg_spi_write;
	bmg160.bus_read = &bmg_spi_read;
	bmg160.delay_msec = &bmg_delay_msec;
	
	/* Call sensor API initialization function */
	bmg160_init(&bmg160);
}

/*!
* @brief		Sends data to BMG160 via SPI
*
* @param[in]	dev_addr	Device I2C slave address (not used)
*
* @param[in]	reg_addr	Address of destination register
*
* @param[in]	reg_data	Pointer to data buffer to be sent
*
* @param[in]	length		Length of the data to be sent
*
* @retval		0			BMG160_SUCCESS
* @retval		-1			BMG160_ERROR
*
*/
int8_t bmg_spi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t length)
{
	enum status_code bmg_write_stat = STATUS_NO_CHANGE;
	
	/* This variable is used to avoid infinite loops. */
	uint16_t loop_count;
	
	spi_select_slave(&spi_master_instance, &bmg160_spi_slave, true);
	
	loop_count = 0;
	do
	{
		bmg_write_stat = spi_write_buffer_wait(&spi_master_instance, &reg_addr, 1);
		loop_count++;
	}while(bmg_write_stat != STATUS_OK && loop_count < 100);
	
	loop_count = 0;
	do
	{
		bmg_write_stat = spi_write_buffer_wait(&spi_master_instance, reg_data, length);
		loop_count++;
	}while(bmg_write_stat != STATUS_OK && loop_count < 100);
	
	spi_select_slave(&spi_master_instance, &bmg160_spi_slave, false);
	
	if (bmg_write_stat != STATUS_OK)
	{
		return -1;
	}
	return 0;
}

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
* @retval		0			BMG160_SUCCESS
* @retval		-1			BMG160_ERROR
*
*/
int8_t bmg_spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *rx_data, uint8_t length)
{
	enum status_code bmg_read_stat = STATUS_NO_CHANGE;
	
	/* This variable is used to avoid infinite loops. */
	uint16_t loop_count;
	
	uint16_t dummy = 0;
	
	reg_addr = reg_addr | 0x80;
	
	spi_select_slave(&spi_master_instance, &bmg160_spi_slave, true);
	
	loop_count = 0;
	do
	{
		bmg_read_stat = spi_write_buffer_wait(&spi_master_instance, &reg_addr, 1);
		loop_count++;
	}while(bmg_read_stat != STATUS_OK && loop_count < 100);
	
	loop_count = 0;
	do
	{
		bmg_read_stat = spi_read_buffer_wait(&spi_master_instance, rx_data, length, dummy);
		loop_count++;
	}while(bmg_read_stat != STATUS_OK && loop_count < 100);
	
	spi_select_slave(&spi_master_instance, &bmg160_spi_slave, false);
	
	if (bmg_read_stat != STATUS_OK)
	{
		return -1;
	}
	return 0;
}

/*!
* @brief		Initializes BMG160 gyroscope sensor and its required connections
*
* @param[in]	msec	Delay length in terms of milliseconds
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void bmg_delay_msec(uint32_t msec)
{
	tc4_wait_for_msec(msec);
}