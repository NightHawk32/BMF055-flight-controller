/**
 * \file
 *
 * \brief Monitor functions for SAM-BA on SAM D20
 * Port of rom monitor functions from legacy sam-ba software
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
 *
 * \page License
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

#include <string.h>
#include <intrinsics.h>
#include "samd20j18.h"
#include "sam_ba_monitor.h"
#include "usart_sam_ba.h"

#define min(a, b)           (((a) < (b)) ?  (a) : (b))
   
const char RomBOOT_Version[] = SAM_BA_VERSION;

/* Provides one common interface to handle both USART and USB-CDC */
typedef struct
{
	/* send one byte of data */
	int (*putc)(uint8_t value);
	/* Get one byte */
	uint8_t (*getc)(void);
	/* Receive buffer not empty */
	bool (*is_rx_ready)(void);
	/* Send given data (polling) */
	uint32_t (*putdata)(void const* data, uint32_t length);
	/* Get data from comm. device */
	uint32_t (*getdata)(void* data, uint32_t length);
	/* Send given data (polling) using xmodem (if necessary) */
	uint32_t (*putdata_xmd)(void const* data, uint32_t length);
	/* Get data from comm. device using xmodem (if necessary) */
	uint32_t (*getdata_xmd)(void* data, uint32_t length);
} t_monitor_if;

/* Initialize structures with function pointers from supported interfaces */
const t_monitor_if uart_if =
{ usart_putc, usart_getc, usart_is_rx_ready, usart_putdata, usart_getdata,
		usart_putdata_xmd, usart_getdata_xmd };

/* The pointer to the interface object use by the monitor */
t_monitor_if * ptr_monitor_if;

/* b_terminal_mode mode (ascii) or hex mode */
volatile bool b_terminal_mode = false;


/**
 * \brief This function initializes the SAM-BA monitor
 *
 * \param com_interface  Communication interface to be used.
 */
void sam_ba_monitor_init(uint8_t com_interface)
{
	/* Selects the requested interface for future actions */
	if (com_interface == SAM_BA_INTERFACE_USART)
		ptr_monitor_if = (t_monitor_if*) &uart_if;
}


/**
 * \brief This function allows data rx by USART
 *
 * \param *data  Data pointer
 * \param length Length of the data
 */
void sam_ba_putdata_term(uint8_t* data, uint32_t length)
{
	uint8_t temp, buf[12], *data_ascii;
	uint32_t i, int_value;

	if (b_terminal_mode)
	{
		if (length == 4)
			int_value = *(uint32_t *) data;
		else if (length == 2)
			int_value = *(uint16_t *) data;
		else
			int_value = *(uint8_t *) data;

		data_ascii = buf + 2;
		data_ascii += length * 2 - 1;

		for (i = 0; i < length * 2; i++)
		{
			temp = (uint8_t) (int_value & 0xf);

			if (temp <= 0x9)
				*data_ascii = temp | 0x30;
			else
				*data_ascii = temp + 0x37;

			int_value >>= 4;
			data_ascii--;
		}
		buf[0] = '0';
		buf[1] = 'x';
		buf[length * 2 + 2] = '\n';
		buf[length * 2 + 3] = '\r';
		ptr_monitor_if->putdata(buf, length * 2 + 4);
	}
	else
		ptr_monitor_if->putdata(data, length);
	return;
}


/**
 * \brief Execute an applet from the specified address
 *
 * \param address Applet address
 */
// void call_applet(uint32_t address)
// {
	// uint32_t *ptr_reset_vector;
	// uint32_t app_start_add;
	// /* Get the pointer to reset handler from application exception table */
	// ptr_reset_vector = (uint32_t *)(address + 4);
	// /* Get the application reset handler address */
	// app_start_add = (*ptr_reset_vector);
	// /* Disable global interrupts */
	// cpu_irq_disable();
	// /* Jump to the application */
	// asm("bx %0"::"r"(app_start_add));
// }
void call_applet(uint32_t address)
{
	uint32_t app_start_address;

	/* Rebase the Stack Pointer */
	__set_MSP(*(uint32_t *) address);

	/* Rebase the vector table base address */
	SCB->VTOR = ((uint32_t) address & SCB_VTOR_TBLOFF_Msk);

	/* Load the Reset Handler address of the application */
	app_start_address = *(uint32_t *)(address + 4);

	/* Jump to application Reset Handler in the application */
	asm("bx %0"::"r"(app_start_address));
}


uint32_t current_number;
uint32_t i, length;
uint8_t command, *ptr_data, *ptr, data[SIZEBUFMAX];
uint8_t j;
uint32_t u32tmp;


/**
 * \brief This function starts the SAM-BA monitor.
 */
void sam_ba_monitor_run(void)
{
	ptr_data = NULL;
	command = 'z';
	
	// Start waiting some cmd
	while (1)
	{
		length = ptr_monitor_if->getdata(data, SIZEBUFMAX);
		ptr = data;
		for (i = 0; i < length; i++)
		{
			if (*ptr != 0xff)
			{
				if (*ptr == '#')
				{
					if (b_terminal_mode)
					{
						ptr_monitor_if->putdata("\n\r", 2);
					}
					if (command == 'S')
					{
						//Check if some data are remaining in the "data" buffer
						if(length>i)
						{
							//Move current indexes to next avail data (currently ptr points to "#")
							ptr++;
							i++;
							//We need to add first the remaining data of the current buffer already read from usb
							//read a maximum of "current_number" bytes
							u32tmp=min((length-i),current_number);
							for(j=0;j<u32tmp;j++)
							{
								*ptr_data = *ptr; 
								ptr_data++;
								ptr++;
								i++;
							}
						}
						//update i with the data read from the buffer
						i--;
						ptr--;
						//Do we expect more data ?
						if(j<current_number)
							ptr_monitor_if->getdata_xmd(ptr_data, current_number-j);
						
						__asm("nop");
					}
					else if (command == 'R')
					{
						ptr_monitor_if->putdata_xmd(ptr_data, current_number);
					}
					else if (command == 'O')
					{
						*ptr_data = (char) current_number;
					}
					else if (command == 'H')
					{
						*((uint16_t *) ptr_data) = (uint16_t) current_number;
					}
					else if (command == 'W')
					{
						*((int *) ptr_data) = current_number;
					}
					else if (command == 'o')
					{
						sam_ba_putdata_term(ptr_data, 1);
					}
					else if (command == 'h')
					{
						current_number = *((uint16_t *) ptr_data);
						sam_ba_putdata_term((uint8_t*) &current_number, 2);
					}
					else if (command == 'w')
					{
						current_number = *((uint32_t *) ptr_data);
						sam_ba_putdata_term((uint8_t*) &current_number, 4);
					}
					else if (command == 'G')
					{
						call_applet(current_number);
                                                ptr_monitor_if->putc(0x6);
					}
					else if (command == 'T')
					{
						b_terminal_mode = 1;
						ptr_monitor_if->putdata("\n\r", 2);
					}
					else if (command == 'N')
					{
						if (b_terminal_mode == 0)
						{
							ptr_monitor_if->putdata("\n\r", 2);
						}
						b_terminal_mode = 0;
					}
					else if (command == 'V')
					{
						ptr_monitor_if->putdata("v", 1);
						ptr_monitor_if->putdata((uint8_t *) RomBOOT_Version,
								strlen(RomBOOT_Version));
						ptr_monitor_if->putdata(" ", 1);
						ptr = (uint8_t*) &(__DATE__);
						i = 0;
						while (*ptr++ != '\0')
							i++;
						ptr_monitor_if->putdata((uint8_t *) &(__DATE__), i);
						ptr_monitor_if->putdata(" ", 1);
						i = 0;
						ptr = (uint8_t*) &(__TIME__);
						while (*ptr++ != '\0')
							i++;
						ptr_monitor_if->putdata((uint8_t *) &(__TIME__), i);
						ptr_monitor_if->putdata("\n\r", 2);
					}

					command = 'z';
					current_number = 0;

					if (b_terminal_mode)
					{
						ptr_monitor_if->putdata(">", 1);
					}
				}
				else
				{
					if (('0' <= *ptr) && (*ptr <= '9'))
					{
						current_number = (current_number << 4) | (*ptr - '0');

					}
					else if (('A' <= *ptr) && (*ptr <= 'F'))
					{
						current_number = (current_number << 4)
								| (*ptr - 'A' + 0xa);

					}
					else if (('a' <= *ptr) && (*ptr <= 'f'))
					{
						current_number = (current_number << 4)
								| (*ptr - 'a' + 0xa);

					}
					else if (*ptr == ',')
					{
						ptr_data = (uint8_t *) current_number;
						current_number = 0;

					}
					else
					{
						command = *ptr;
						current_number = 0;
					}
				}
				ptr++;
			}
		}
	}
}
