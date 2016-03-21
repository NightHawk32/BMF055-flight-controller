/**
*
*
**************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH. All Rights Reserved.
*
* File:		usart_support.h
*
* Date:		2015/08/26
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
* @file		usart_support.h
* @author	Bosch Sensortec
*
* @brief	USART ASF Driver Support Header File
*
* USART support uses ASF USART driver modules and defines
* initialization, configuration and callback functions for
* the microcontroller’s USART peripheral that is needed
* to communicate with an external device (here a serial terminal).
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