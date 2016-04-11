/**
*
*
**************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH. All Rights Reserved.
*
* File:		usart_support.c
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
* @file		usart_support.c
* @author	Bosch Sensortec
*
* @brief	Functions declared in usart_support.h file are defined here.
*
*
*/


/************************************************************************/
/* Include Own Header                                                   */
/************************************************************************/

#include "usart_support.h"
#include "serial.h"

/************************************************************************/
/* Function Definitions                                                 */
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
void usart_initialize(void)
{
	/* Initialize the variables */
	usart_rx_byte = 0;
	usart_rx_count = 0;
	
	/* Configure the USART Module */
	usart_configure();
	
	/* Configure USART callbacks */
	usart_configure_callbacks();
	
	/* Enable the interrupt to receive the first byte */
	usart_read_job(&usart_instance, &usart_rx_byte);
}

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
void usart_configure(void)
{
	/* USART's configuration structure */
	struct usart_config config_usart;
	
	/* get USART configuration defaults */
	usart_get_config_defaults(&config_usart);
	
	/* set USART Baudrate*/
	config_usart.baudrate = USART_BAUDRATE;
	/* Set USART GCLK */
	config_usart.generator_source = GCLK_GENERATOR_2;
	/* Se USART MUX setting */
	config_usart.mux_setting = USART_RX_1_TX_0_XCK_1;
	/* Configure pad 0 for Tx */
	config_usart.pinmux_pad0 = PINMUX_PB16C_SERCOM5_PAD0;
	/* Configure pad 1 for Rx */
	config_usart.pinmux_pad1 = PINMUX_PB17C_SERCOM5_PAD1;
	/* Configure pad 2 for unused */
	config_usart.pinmux_pad2 = PINMUX_UNUSED;
	/* Configure pad 3 for unused */
	config_usart.pinmux_pad3 = PINMUX_UNUSED;
	
	/* Initialize SERCOM5 as a USART module*/
	while (usart_init(&usart_instance,SERCOM5, &config_usart) != STATUS_OK) ;
	
	/* Enable the USART module */
	usart_enable(&usart_instance);
}

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
void usart_configure_callbacks(void)
{
	/* Configure USART receive callback */
	usart_register_callback(&usart_instance, usart_callback_receive, USART_CALLBACK_BUFFER_RECEIVED);
	usart_callback_receive_flag = false;
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
	
	/* Configure USART transmit callback */
	usart_register_callback(&usart_instance, usart_callback_transmit, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_callback_transmit_flag = true;
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
}

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
void usart_callback_receive(struct usart_module *const usart_module_ptr)
{
	store_uart_in_buf(usart_rx_byte);
	usart_read_job(&usart_instance, &usart_rx_byte);
}

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
void usart_callback_transmit(struct usart_module *const usart_module_ptr)
{
	/* Set the corresponding flag */
	usart_callback_transmit_flag = true;
}

