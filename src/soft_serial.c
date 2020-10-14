/* Copyright (C) 
 * 2020 - Gabriel Durante
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 * 
 */


/**
 * @file soft_serial.c 
 *
 * @brief Soft Serial API Implementation
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>

#include <soft_serial.h>

/* ================================================================================ */

#define SOFT_SERIAL_RX_BITS (8)
#define SOFT_SERIAL_TX_BITS (10)
#define SOFT_SERIAL_TX_MASK(X) ((X << 1)|0x200)
#define SOFT_SERIAL_RX_START_GAP (4)

/**
 * @brief software serial context declaration
 */
struct soft_serial_ctx {
	// soft serial bus
	volatile struct soft_serial *bus;

	// soft serial rx flags
	volatile uint8_t rx_ready;
	volatile uint8_t rx_timer_counter;
	volatile uint8_t rx_bits_left;
	volatile uint8_t rx_mask;
	volatile uint8_t rx_in;
	volatile uint8_t rx_waiting_for_stop_bit;

	// soft serial tx flags
	volatile uint8_t tx_busy;
	volatile uint8_t tx_timer_counter;
	volatile uint8_t tx_bits_left;
	volatile unsigned short tx_out;

	// internal timer control
	volatile uint32_t baud_timer_compare;

} ctx;

/**
 * @brief soft_serial RX Ring Buffer definition
 */
static volatile t_ssbuffer g_rx_buff;

/**
 * @brief soft_serial TX Ring Buffer definition
 */
static volatile t_ssbuffer g_tx_buff;

/* ================================================================================ */

/**
 * @brief Soft Serial interrupt
 *
 * Data is sent/received in this ISR and placed in the TX/RX ring buffer - if there is still space available.
 *  If statistics support is enabled those will be updated in this ISR as well
 *
 * @param USART_RX_vect
 */
ISR(TIMER2_COMPA_vect)
{
	// tx section
	{
		// proceed if there still is data to be send
		if (g_tx_buff.u.r.head != g_tx_buff.u.r.tail) {
			// check if tx is busy
			if (ctx.tx_busy) {
				// check baud_timer_compare expire
				if (--ctx.tx_timer_counter == 0) {
					// send bits
					if (ctx.tx_out & 0x01) {
						GPIO_SET_HIGH(&ctx.bus->tx);
					}
					else {
						GPIO_SET_LOW(&ctx.bus->tx);
					}
					// shift tx data
					ctx.tx_out >>= 1;
					// prepare next bit
					ctx.tx_timer_counter = ctx.baud_timer_compare;
					// check end of bits
					if (--ctx.tx_bits_left == 0) {
						// update ring
						g_tx_buff.u.r.tail = (g_tx_buff.u.r.tail + 1) % SOFT_SERIAL_TX_RING_SIZE;
						ctx.tx_busy = 0;
					}
				}
			}
			else {
				// prepare transfer
				ctx.tx_out = SOFT_SERIAL_TX_MASK(g_tx_buff.u.r.ring[g_tx_buff.u.r.tail]);
				ctx.tx_bits_left = SOFT_SERIAL_TX_BITS;
				ctx.tx_timer_counter = ctx.baud_timer_compare;
				ctx.tx_busy = 1;
			}
#if SOFT_SERIAL_COLLECT_STATS == 1
			g_tx_buff.stats.ok++;
#endif
		}
	}

	// rx section
	{
		// if waiting for stop bit
		if (ctx.rx_waiting_for_stop_bit) {
			// check baud_timer_compare expire
			if (--ctx.rx_timer_counter == 0) {
				// calculate the next available ring buffer data bucket index
				volatile unsigned char next =
					((g_rx_buff.u.r.head + 1) % SOFT_SERIAL_RX_RING_SIZE);

				// do not overflow the buffer
				if (next != g_rx_buff.u.r.tail) {
					g_rx_buff.u.r.ring[g_rx_buff.u.r.head] = ctx.rx_in;
					g_rx_buff.u.r.head = next;			
#if SOFT_SERIAL_COLLECT_STATS == 1
					g_rx_buff.stats.ok++;
#endif
				}
				else {
#if SOFT_SERIAL_COLLECT_STATS == 1
					// increase the dropped counter
					g_rx_buff.stats.dropped++;
#endif
				}
				ctx.rx_waiting_for_stop_bit = 0;
				ctx.rx_ready = 0;
			}
		}
		else {
			// check if rx is ready
			if (ctx.rx_ready == 0) {
				// check for start bit
				if (GPIO_GET(&ctx.bus->rx) == 0) {
					// prepare receive
					ctx.rx_bits_left = SOFT_SERIAL_RX_BITS;
					ctx.rx_timer_counter = (ctx.baud_timer_compare + SOFT_SERIAL_RX_START_GAP);
					ctx.rx_in = 0;
					ctx.rx_waiting_for_stop_bit = 0;
					ctx.rx_mask = 1;
					ctx.rx_ready = 1;
				}
			}
			else {
				// check baud_timer_compare expire
				if (--ctx.rx_timer_counter == 0) {
					// get bit
					if (GPIO_GET(&ctx.bus->rx)) {
						ctx.rx_in |= ctx.rx_mask;
					}

					// shift rx data
					ctx.rx_mask <<= 1;

					// check end of bits
					if (--ctx.rx_bits_left == 0) {
						ctx.rx_waiting_for_stop_bit = 1;
					}

					// prepare next bit
					ctx.rx_timer_counter = ctx.baud_timer_compare;
				}
			}
		}
	}
}

/* ================================================================================ */

/**
 * @brief wrapper for stdio, just to make the soft_serial API interface compatible
 *
 * @param c character to 'print'
 * @param stream file stream
 */
static int _soft_serial_putc(char c, FILE *stream) {
	if ('\n' == c) {
		_soft_serial_putc('\r', stream);
	}

	while (!soft_serial_sendc(c));

	return 0;
}


/**
 * @brief wrapper for stdio, just to make the soft_serial API interface compatible
 *
 * @param stream 
 *
 * @return character received
 */
static int _soft_serial_getc(FILE *stream) {
	unsigned char c = 0x00;

	while (!soft_serial_getc(&c));

	return (char)c;
}


/* ================================================================================ */


e_return soft_serial_init(volatile struct soft_serial *a_bus) {
	// reset context structure
	memset(&ctx, 0, sizeof(ctx));

	// set bus structure
	ctx.bus = a_bus;

	// set baudrate timing parameters
	switch (a_bus->baudrate)
	{
	case E_SSBAUD_2400:
		ctx.baud_timer_compare = 41;
		break;

	case E_SSBAUD_4800:
		ctx.baud_timer_compare = 20;
		break;

	case E_SSBAUD_9600:
		ctx.baud_timer_compare = 10;
		break;

	case E_SSBAUD_14400:
		ctx.baud_timer_compare = 7;
		break;

	case E_SSBAUD_19200:
	default:
		ctx.baud_timer_compare = 5;
		break;
	}

	// rx/tx configure
	GPIO_CONFIGURE_AS_OUTPUT(&a_bus->tx);
	GPIO_CONFIGURE_AS_INPUT(&a_bus->rx);

	/* set tx to high to avoid garbage on init */
	GPIO_SET_HIGH(&a_bus->tx);

	// clear the ring
	uint16_t val = sizeof(g_rx_buff);
	common_zero_mem(&g_rx_buff, val);

	val = sizeof(g_tx_buff);
	common_zero_mem(&g_tx_buff, val);

	// timer configure to interrupt in 10us
	uint32_t pocr = _timer_freq_prescale(E_TIMER2, (100000/2), 255);

	_timer_init_ctc(E_TIMER2);
	_timer_setup_ctc(E_TIMER2, pocr);

	// enable interrupt
	_timer_en_compa_int(E_TIMER2);

	return RET_OK;
}

void soft_serial_install_stdio() {
	static FILE uart_stdout = FDEV_SETUP_STREAM(_soft_serial_putc, NULL, _FDEV_SETUP_WRITE);
	static FILE uart_stdin = FDEV_SETUP_STREAM(NULL, _soft_serial_getc, _FDEV_SETUP_READ);

	stdout = &uart_stdout;
	stdin = &uart_stdin;
}


inline unsigned char soft_serial_available() {
	return (SOFT_SERIAL_RX_RING_SIZE + g_rx_buff.u.r.head - g_rx_buff.u.r.tail) % SOFT_SERIAL_RX_RING_SIZE;
}


unsigned char soft_serial_peek(void *a_data, unsigned char a_size) {
	unsigned char read = soft_serial_available();
	if (read > a_size)
		read = a_size;

	for (unsigned char i = 0; i<read; i++) {
		((unsigned char *)a_data)[i] =
			g_rx_buff.u.r.ring[ (g_rx_buff.u.r.tail + i) % SOFT_SERIAL_RX_RING_SIZE ];
	}

	return read;
}


unsigned int soft_serial_recv(void *a_data, unsigned int a_size, unsigned char a_waitall) {
	unsigned int read = 0x00;

	if (!a_waitall && g_rx_buff.u.r.head == g_rx_buff.u.r.tail) {
		return 0;
	}

	while (read < a_size) {

		while (soft_serial_available() && (read < a_size)) 
		{
			((unsigned char *)a_data)[read] = g_rx_buff.u.r.ring[ g_rx_buff.u.r.tail ];
			g_rx_buff.u.r.tail = (g_rx_buff.u.r.tail + 1) % SOFT_SERIAL_RX_RING_SIZE;
			read++;
		}

		if (!a_waitall)
			break;
	}

	return read;
}


unsigned char soft_serial_getc(unsigned char *a_data) {
	
	if (g_rx_buff.u.r.head == g_rx_buff.u.r.tail)
		return 0;

	*a_data = g_rx_buff.u.r.ring[g_rx_buff.u.r.tail];
	g_rx_buff.u.r.tail = (g_rx_buff.u.r.tail + 1) % SOFT_SERIAL_RX_RING_SIZE;

	return 1;
}


unsigned char soft_serial_send(void *a_data, unsigned int a_size, unsigned char a_waitall) {
	uint8_t n = 0x00;
	uint8_t *data = (uint8_t *)a_data;
	uint8_t initiated = 0x00;

	while (a_size) {
		volatile unsigned char next =
		   	((g_tx_buff.u.r.head + 1) % SOFT_SERIAL_TX_RING_SIZE);

		/// do not overflow the buffer
		if (next != g_tx_buff.u.r.tail) {
			g_tx_buff.u.r.ring[g_tx_buff.u.r.head] = *data;
			g_tx_buff.u.r.head = next;			
		}
		else {
			if (a_waitall) {
				if (!initiated) {
					initiated = 0x01;
				}
				continue;
			}
			else
				break;
		}

		a_size--;
		n++;
		data++;
	}

	if (!a_waitall || !initiated) {
		initiated = 0x01;
	}

	return n;
}


unsigned char soft_serial_sendc(unsigned char a_data) {

	uint8_t n = 0x00;
	uint8_t next =
		((g_tx_buff.u.r.head + 1) % SOFT_SERIAL_TX_RING_SIZE);

	/// do not overflow the buffer
	if (next != g_tx_buff.u.r.tail) {
		g_tx_buff.u.r.ring[g_tx_buff.u.r.head] = a_data;
		g_tx_buff.u.r.head = next;
		n = 1;
	}
#if SOFT_SERIAL_COLLECT_STATS == 1
	else {
		g_tx_buff.stats.dropped++;
	}
#endif
	return n;
}


volatile t_ssbuffer* soft_serial_get_rx_state() {
	return &g_rx_buff;
}


volatile t_ssbuffer* soft_serial_get_tx_state() {
	return &g_tx_buff;
}
