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
