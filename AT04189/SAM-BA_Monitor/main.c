/**
 * \file
 *
 * \brief SAM D20 SAM-BA Bootloader
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * --------------------
 * SAM-BA Implementation on SAMD20
 * --------------------
 * Requirements to use SAM-BA :
 *
 * Supported communication interfaces :
 * --------------------
 *
 * SERCOM3 : TX:PA24 RX:PA25
 * Baudrate : 115200 8N1
 *
 * Pins Usage
 * --------------------
 * The following pins are used by the program :
 * PA25 : input
 * PA24 : output
 * PA15 : input
 *
 * The application board shall avoid driving the PA25,PA24 and PA15 signals
 * while the boot program is running (after a POR for example)
 *
 * Memory Mapping
 * --------------------
 * SAM-BA code will be located at 0x0 and executed before any applicative code.
 *
 * Applications compiled to be executed along with the bootloader will start at
 * 0x800
 * Before jumping to the application, the bootloader changes the VTOR register
 * to use the interrupt vectors of the application @0x800.<- not required as
 * application code is taking care of this
 *
*/

#include "samd20j18.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "conf_bootloader.h"
#include "sam_ba_monitor.h"
#include "usart_sam_ba.h"

static void check_start_application(void);

/**
 * \brief Check the application startup condition
 *
 */
static void check_start_application(void)
{
	uint32_t app_start_address;

	/* Load the Reset Handler address of the application */
	app_start_address = *(uint32_t *)(APP_START_ADDRESS + 4);

	/**
	 * Test reset vector of application @APP_START_ADDRESS+4
	 * Stay in SAM-BA if *(APP_START+0x4) == 0xFFFFFFFF
	 * Application erased condition
	 */
	if (app_start_address == 0xFFFFFFFF) {
		/* Stay in bootloader */
		return;
	}

	volatile PortGroup *boot_port = (volatile PortGroup *)(&(PORT->Group[BOOT_LOAD_PIN / 32]));
	volatile bool boot_en;

	/* Enable the input mode in Boot GPIO Pin */
	boot_port->DIRCLR.reg = GPIO_BOOT_PIN_MASK;
	boot_port->PINCFG[BOOT_LOAD_PIN & 0x1F].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	boot_port->OUTSET.reg = GPIO_BOOT_PIN_MASK;
	/* Read the BOOT_LOAD_PIN status */
	boot_en = (boot_port->IN.reg) & GPIO_BOOT_PIN_MASK;

	/* Check the bootloader enable condition */
	if (!boot_en) {
		/* Stay in bootloader */
		return;
	}

	/* Rebase the Stack Pointer */
	__set_MSP(*(uint32_t *) APP_START_ADDRESS);

	/* Rebase the vector table base address */
	SCB->VTOR = ((uint32_t) APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

	/* Jump to application Reset Handler in the application */
	asm("bx %0"::"r"(app_start_address));
}


#if DEBUG_ENABLE
#	define DEBUG_PIN_HIGH 	PORT->Group[0].OUTSET.reg=(1u<<BOOT_LED)
#	define DEBUG_PIN_LOW 	PORT->Group[0].OUTCLR.reg=(1u<<BOOT_LED)
#else
#	define DEBUG_PIN_HIGH 	do{}while(0)
#	define DEBUG_PIN_LOW 	do{}while(0)
#endif


/**
 *  \brief SAMD20 SAM-BA Main loop.
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
  
#if DEBUG_ENABLE
        /* Set BOOT_LED pin as output */
        PORT->Group[0].DIRSET.reg=(1u<<BOOT_LED);
#endif
	DEBUG_PIN_HIGH;
        
	/* Jump in application if condition is satisfied */
	check_start_application();
        
        /* Make OSC8M prescalar to zero rather divide by 8 */
        SYSCTRL->OSC8M.bit.PRESC = 0;

	/* UART is enabled in all cases */
	usart_open();

	/* Wait for a complete enum on usb or a '#' char on serial line */
	while (1) {
		/* Check if a '#' has been received */
		if (usart_sharp_received()) {
                        DEBUG_PIN_LOW;
			sam_ba_monitor_init(SAM_BA_INTERFACE_USART);
			/* SAM-BA on UART loop */
			while(1) {
				sam_ba_monitor_run();
			}
		}
	}
}
