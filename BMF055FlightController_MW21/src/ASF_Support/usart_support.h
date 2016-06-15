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


#ifndef usart_support_h_
#define usart_support_h_

/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include "usart.h"
#include "usart_interrupt.h"

/************************************************************************/
/* Macro Definitions                                                    */
/************************************************************************/

/*! value of 115200 that is used to set USART baud rate */
#define USART_BAUDRATE_115200	UINT32_C(115200)
/*! loaded onto USART baud rate register initially */
#define USART_BAUDRATE			USART_BAUDRATE_115200


/*! SERCOM USART driver software instance structure, used to retain
* software state information of the associated hardware module instance */
struct usart_module usart_instance;
/*! USART receive callback flag (set after each USART reception) */
volatile bool usart_callback_receive_flag;

/*! USART receive callback flag (set after each USART transmission) */
volatile bool usart_callback_transmit_flag;

/*! USART Rx buffer */
uint16_t usart_rx_string[64];
/*! USART Rx byte */
uint16_t usart_rx_byte;
/*! USART Rx buffer length */
uint16_t usart_rx_count;
/************************************************************************/
/* Function Declarations                                                */
/************************************************************************/

/*!
* @brief		Initializes the USART module of the MCU
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void usart_initialize(void);

/*!
* @brief		Configures the USART module of the MCU
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void usart_configure(void);

/*!
* @brief		Configures USART callback register
*
* @param[in]	NULL
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void usart_configure_callbacks(void);

/*!
* @brief		Called after USART receptions
*
* @param[in]	usart_module_ptr	Pointer to the USART module which triggers the interrupt
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void usart_callback_receive(struct usart_module *const usart_module_ptr);

/*!
* @brief		Called after USART transmissions
*
* @param[in]	usart_module_ptr	Pointer to the USART module which triggers the interrupt
*
* @param[out]	NULL
*
* @return		NULL
*
*/
void usart_callback_transmit(struct usart_module *const usart_module_ptr);


#endif /* usart_support_h_ */