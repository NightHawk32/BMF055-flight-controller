/**
 * \file
 *
 * \brief USART functions for SAM-BA on SAM D20
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

#include "samd20j18.h"
#include "usart_sam_ba.h"
#include "conf_bootloader.h"

/* Variable to let the main task select the appropriate communication interface */
volatile uint8_t b_sharp_received;

/* RX and TX Buffers + rw pointers for each buffer */
volatile uint8_t buffer_rx_usart[USART_BUFFER_SIZE];

volatile uint8_t idx_rx_read;
volatile uint8_t idx_rx_write;

volatile uint8_t buffer_tx_usart[USART_BUFFER_SIZE];

volatile uint8_t idx_tx_read;
volatile uint8_t idx_tx_write;

/* Test for timeout in AT91F_GetChar */
uint8_t error_timeout;
uint16_t size_of_data;
uint8_t mode_of_transfer;

void uart_disable(Sercom *sercom)
{
        while(sercom->USART.STATUS.bit.SYNCBUSY);
	sercom->USART.CTRLA.bit.ENABLE = 0;
}

void uart_write_byte(Sercom *sercom, uint8_t data)
{
	while(!sercom->USART.INTFLAG.bit.DRE);
	sercom->USART.DATA.reg = (uint16_t)data;
}

uint8_t uart_read_byte(Sercom *sercom)
{
	while(!sercom->USART.INTFLAG.bit.RXC);
	return((uint8_t)(sercom->USART.DATA.reg & 0x00FF));
}

/**
 * \brief Open the given USART
 */
void usart_open()
{
        /* Enable & configure alternate function C for pins PA24 & PA25 */
        PORT->Group[1].PINCFG[16].bit.PMUXEN = 1;
        PORT->Group[1].PINCFG[17].bit.PMUXEN = 1;
        PORT->Group[1].PMUX[8].reg = 0x22;
        /* Enable APB clock for SERCOM3 */
        PM->APBCMASK.reg |= (1u << 7);
        /* Configure GCLK generator 0 as clock source for SERCOM3 */
        GCLK->CLKCTRL.reg = 0x4012;
        /* Configure SERCOM3 USART with baud 115200 8-N-1 settings */
        BOOT_USART_MODULE->USART.CTRLA.reg = SERCOM_USART_CTRLA_RXPO(1) | SERCOM_USART_CTRLA_MODE(1) | SERCOM_USART_CTRLA_DORD;
	while(BOOT_USART_MODULE->USART.STATUS.bit.SYNCBUSY);
	BOOT_USART_MODULE->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_CHSIZE(0);
	BOOT_USART_MODULE->USART.BAUD.reg = 50436;
	while(BOOT_USART_MODULE->USART.STATUS.bit.SYNCBUSY);
	BOOT_USART_MODULE->USART.CTRLA.bit.ENABLE = 1;
        
	//Initialize flag
	b_sharp_received = false;
	idx_rx_read = 0;
	idx_rx_write = 0;
	idx_tx_read = 0;
	idx_tx_write = 0;

	error_timeout = 0;
}

/**
 * \brief Configures communication line
 *
 */
void usart_close(void)
{
	uart_disable(BOOT_USART_MODULE);
}

/**
 * \brief Puts a byte on usart line
 * The type int is used to support printf redirection from compiler LIB.
 *
 * \param value      Value to put
 *
 * \return \c 1 if function was successfully done, otherwise \c 0.
 */
int usart_putc(uint8_t value)
{
        uart_write_byte(BOOT_USART_MODULE, value);
	return 1;
}



uint8_t usart_getc(void) {
	uint8_t retval;
	//Wait until input buffer is filled
        retval = uart_read_byte(BOOT_USART_MODULE);
	return (retval);
}

int usart_sharp_received(void) {
	if (usart_is_rx_ready()) {
		if (usart_getc() == SHARP_CHARACTER)
			return (true);
	}
	return (false);
}

bool usart_is_rx_ready(void) {
	return (BOOT_USART_MODULE->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC);
}

int usart_readc(void) {
	int retval;
	retval = buffer_rx_usart[idx_rx_read];
	idx_rx_read = (idx_rx_read + 1) & (USART_BUFFER_SIZE - 1);
	return (retval);
}

//Send given data (polling)
uint32_t usart_putdata(void const* data, uint32_t length) {
	uint32_t i;
	uint8_t* ptrdata;
	ptrdata = (uint8_t*) data;
	for (i = 0; i < length; i++) {
		usart_putc(*ptrdata);
		ptrdata++;
	}
	return (i);
}

//Get data from comm. device
uint32_t usart_getdata(void* data, uint32_t length) {
	uint8_t* ptrdata;
	ptrdata = (uint8_t*) data;
	*ptrdata = usart_getc();
	return (1);
}

//*----------------------------------------------------------------------------
//* \fn    add_crc
//* \brief Compute the CRC
//*----------------------------------------------------------------------------
unsigned short add_crc(char ptr, unsigned short crc) {

	unsigned short cmpt;

	crc = crc ^ (int) ptr << 8;

	for (cmpt = 0; cmpt < 8; cmpt++) {
		if (crc & 0x8000)
			crc = crc << 1 ^ CRC16POLY;
		else
			crc = crc << 1;
	}

	return (crc & 0xFFFF);
}

//*----------------------------------------------------------------------------
//* \fn    getbytes
//* \brief
//*----------------------------------------------------------------------------
static uint16_t getbytes(uint8_t *ptr_data, uint16_t length) {
	uint16_t crc = 0;
	uint16_t cpt;
	uint8_t c;

	for (cpt = 0; cpt < length; ++cpt) {
		c = usart_getc();
		if (error_timeout)
			return 1;
		crc = add_crc(c, crc);
		//crc = (crc << 8) ^ xcrc16tab[(crc>>8) ^ c];
		if (size_of_data || mode_of_transfer) {
			*ptr_data++ = c;
			if (length == PKTLEN_128)
				size_of_data--;
		}
	}

	return crc;
}

//*----------------------------------------------------------------------------
//* \fn    putPacket
//* \brief Used by Xup to send packets.
//*----------------------------------------------------------------------------
static int putPacket(uint8_t *tmppkt, uint8_t sno) {
	uint32_t i;
	uint16_t chksm;
	uint8_t data;

	chksm = 0;

	usart_putc(SOH);

	usart_putc(sno);
	usart_putc((uint8_t) ~(sno));

	for (i = 0; i < PKTLEN_128; i++) {
		if (size_of_data || mode_of_transfer) {
			data = *tmppkt++;
			size_of_data--;
		} else
			data = 0x00;

		usart_putc(data);

		//chksm = (chksm<<8) ^ xcrc16tab[(chksm>>8)^data];
		chksm = add_crc(data, chksm);
	}

	/* An "endian independent way to extract the CRC bytes. */
	usart_putc((uint8_t) (chksm >> 8));
	usart_putc((uint8_t) chksm);

	return (usart_getc()); /* Wait for ack */
}

//*----------------------------------------------------------------------------
//* \fn    getPacket
//* \brief Used by Xdown to retrieve packets.
//*----------------------------------------------------------------------------
uint8_t getPacket(uint8_t *ptr_data, uint8_t sno) {
	uint8_t seq[2];
	uint16_t crc, xcrc;

	getbytes(seq, 2);
	xcrc = getbytes(ptr_data, PKTLEN_128);
	if (error_timeout)
		return (false);

	/* An "endian independent way to combine the CRC bytes. */
	crc = (uint16_t) usart_getc() << 8;
	crc += (uint16_t) usart_getc();

	if (error_timeout == 1)
		return (false);

	if ((crc != xcrc) || (seq[0] != sno) || (seq[1] != (uint8_t) (~sno))) {
		usart_putc(CAN);
		return (false);
	}

	usart_putc(ACK);
	return (true);
}

//*----------------------------------------------------------------------------
//* \fn    Xup
//* \brief Called when a transfer from target to host is being made (considered
//*        an upload).
//*----------------------------------------------------------------------------
//static void Xup(char *ptr_data, uint16_t length)
//Send given data (polling) using xmodem (if necessary)
uint32_t usart_putdata_xmd(void const* data, uint32_t length) {
	uint8_t c, sno = 1;
	uint8_t done;
	uint8_t * ptr_data = (uint8_t *) data;
	error_timeout = 0;
	if (!length)
		mode_of_transfer = 1;
	else {
		size_of_data = length;
		mode_of_transfer = 0;
	}

	if (length & (PKTLEN_128 - 1)) {
		length += PKTLEN_128;
		length &= ~(PKTLEN_128 - 1);
	}

	/* Startup synchronization... */
	/* Wait to receive a NAK or 'C' from receiver. */
	done = 0;
	while (!done) {
		c = (uint8_t) usart_getc();
		if (error_timeout) { // Test for timeout in usart_getc
			error_timeout = 0;
			c = (uint8_t) usart_getc();
			if (error_timeout) {
				error_timeout = 0;
				return (0);
//                return -2;
			}
		}
		switch (c) {
		case NAK:
			done = 1;
			// ("CSM");
			break;
		case 'C':
			done = 1;
			// ("CRC");
			break;
		case 'q': /* ELS addition, not part of XMODEM spec. */
			return (0);
//           return(0);
		default:
			break;
		}
	}

	done = 0;
	sno = 1;
	while (!done) {
		c = (uint8_t) putPacket((uint8_t *) ptr_data, sno);
		if (error_timeout) { // Test for timeout in usart_getc
			error_timeout = 0;
			return (0);
//            return (-1);
		}
		switch (c) {
		case ACK:
			++sno;
			length -= PKTLEN_128;
			ptr_data += PKTLEN_128;
			// ("A");
			break;
		case NAK:
			// ("N");
			break;
		case CAN:
		case EOT:
		default:
			done = 0;
			break;
		}
		if (!length) {
			usart_putc(EOT);
			usart_getc(); /* Flush the ACK */
			break;
		}
		// ("!");
	}

	mode_of_transfer = 0;
	// ("Xup_done.");
	return (1);
	//    return(0);
}

//*----------------------------------------------------------------------------
//* \fn    Xdown
//* \brief Called when a transfer from host to target is being made (considered
//*        an download).
//*----------------------------------------------------------------------------
//static void Xdown(char *ptr_data, uint16_t length)
//Get data from comm. device using xmodem (if necessary)
uint32_t usart_getdata_xmd(void* data, uint32_t length) {
	uint32_t timeout;
	char c;
	uint8_t * ptr_data = (uint8_t *) data;
	uint32_t b_run, nbr_of_timeout = 100;
	uint8_t sno = 0x01;
	uint32_t data_transfered = 0;

	//Copied from legacy source code ... might need some tweaking
	uint32_t loops_per_second = CPU_CLOCK_FREQ / 10;

	error_timeout = 0;

	if (length == 0)
		mode_of_transfer = 1;
	else {
		size_of_data = length;
		mode_of_transfer = 0;
	}

	/* Startup synchronization... */
	/* Continuously send NAK or 'C' until sender responds. */
	// ("Xdown");
	while (1) {
		usart_putc('C');
		timeout = loops_per_second;
		while (!(usart_is_rx_ready()) && timeout)
			timeout--;
		if (timeout)
			break;

		if (!(--nbr_of_timeout))
			return (0);
//            return -1;
	}

	b_run = true;
	// ("Got response");
	while (b_run != false) {
		c = (char) usart_getc();
		if (error_timeout) { // Test for timeout in usart_getc
			error_timeout = 0;
			return (0);
//            return (-1);
		}
		switch (c) {
		case SOH: /* 128-byte incoming packet */
			// ("O");
			b_run = getPacket(ptr_data, sno);
			if (error_timeout) { // Test for timeout in usart_getc
				error_timeout = 0;
				return (0);
//                return (-1);
			}
			if (b_run == true) {
				++sno;
				ptr_data += PKTLEN_128;
				data_transfered += PKTLEN_128;
			}
			break;
		case EOT: // ("E");
			usart_putc(ACK);
			b_run = false;
			break;
		case CAN: // ("C");
		case ESC: /* "X" User-invoked abort */
		default:
			b_run = false;
			break;
		}
		// ("!");
	}
	mode_of_transfer = 0;
	return (true);
//    return(b_run);
}

