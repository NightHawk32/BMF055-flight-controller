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

