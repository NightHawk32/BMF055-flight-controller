/**
*
*
**************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH. All Rights Reserved.
*
* File:		bmf055.h
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
* @file		bmf055.h
* @author	Bosch Sensortec
*
* @brief	In this file Atmel drivers, sensor APIs and driver support facilities
*			are used to implement the desired application.
*			The application specific functions, constants and variables are all defined here.
*
*
*/


#ifndef BMF055_H_
#define BMF055_H_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/
#include "asf.h"

#include "clock_support.h"
#include "spi_support.h"
#include "tc_support.h"
#include "usart_support.h"

#include "bma2x2_support.h"
#include "bmg160_support.h"
#include "bmm050_support.h"


/************************************************************************/
/* Type Definitions                                                     */
/************************************************************************/

/*! These type values are states of USART input command process. */
typedef enum usart_input_state_type
{
	USART_INPUT_STATE_PRINT_DATA,
	USART_INPUT_STATE_STOPPED,
	USART_INPUT_STATE_ACC_RANGE,
	USART_INPUT_STATE_ACC_BW,
	USART_INPUT_STATE_ACC_MODE,
	USART_INPUT_STATE_GYR_RANGE,
	USART_INPUT_STATE_GYR_BW,
	USART_INPUT_STATE_GYR_MODE,
	USART_INPUT_STATE_MAG_PRESET,
	USART_INPUT_STATE_MAG_BW,
	USART_INPUT_STATE_MAG_MODE,
	USART_INPUT_STATE_LAST = USART_INPUT_STATE_MAG_MODE
}usart_input_state_t;

/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/

/*! Sensors’ data are read in accordance with TC6 callback. */
#define READ_SENSORS_FLAG				tc6_callback_flag
/*! USART command process is executed in accordance with USART receive callback. */
#define USART_COMMAND_PROCESS_FLAG		usart_callback_receive_flag

/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

/*! This type holds the state of USART input command process. */
usart_input_state_t bmf055_input_state;

/************************************************************************/
/* Function Declarations                                                */
/************************************************************************/

/*!
* @brief		Initializes the internal sensors of BMF055.
*
* @param [in]	NULL
*
* @param [out]	NULL
*
* @return		NULL
*
*/
void bmf055_sensors_initialize(void);


/*!
* @brief		Reads output data of the internal sensors and sends sensor data via USART.
*
* @param [in]	NULL
*
* @param [out]	NULL
*
* @return		NULL
*
*/
void bmf055_sensors_data_print(void);

/*!
* @brief		Processes USART input commands.
*
* @param [in]	bmf055_usart_input_buf	Holds the received byte to be processed
*
* @param [out]	NULL
*
* @return		NULL
*
*/
void bmf055_usart_read_process(uint16_t bmf055_usart_input_buf);



#endif /* BMF055_H_ */