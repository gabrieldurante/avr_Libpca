#ifndef __SOFT_SERIAL_H__
#define __SOFT_SERIAL_H__

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
 * @file soft_serial.h 
 *
 * @brief API over SOFT UART configured as RS232. Contains all the necessary routines in order to provide basic means
 *  of communication over software serial port
 *
 * Using macro definitions here you can enable/disable some of the library implementation features, aw well
 *  as change the default values for some of the important configuration constants.
 *
 */

#include <config.h>
#include <common.h>
#include <gpio.h>
#include <ring.h>
#include <timer_common.h>

/**
 * @brief module version definition (just for internal use, to determine API incompatibilities)
 */
#define SOFT_SERIAL_VERSION "0.010"


/**
 * @brief soft_serial pre calculated supported USART baud rates
 */
typedef enum _e_soft_serial_speed {
	E_SSBAUD_2400 = 2400,
	E_SSBAUD_4800 = 4800,
	E_SSBAUD_9600 = 9600,
	E_SSBAUD_14400 = 14400,
	E_SSBAUD_19200 = 19200,
	E_SSBAUD_LAST
} e_soft_serial_speed;


#if SOFT_SERIAL_COLLECT_STATS == 1
/**
 * @brief IO buffer statistics type  declaration
 */
typedef struct _t_stats {

	/// number of a valid rx/tx bytes
	volatile uint32_t ok;

	/// number of dropped data bytes due to buffer being full
	volatile uint32_t dropped;

	/// number of bytes received with frame error indicator
	volatile uint32_t frame_error;
} t_stats;
#endif


/**
 * @brief TX/RX ring buffer declaration
 */
typedef struct _t_ssbuffer {

	/// data storage space for the ring buffer
	union {
		volatile uint8_t raw[SOFT_SERIAL_RX_RING_SIZE + RING_SIZE];
		volatile ring_buffer r;
	} u;

#if SOFT_SERIAL_COLLECT_STATS == 1

	/// statistics for the buffer
	volatile t_stats stats;
#endif

} t_ssbuffer;

/**
 * @brief software serial declaration
 */
struct soft_serial {
	
	// soft serial tx gpio
	volatile gpio_pin tx;

	// soft serial tx gpio
	volatile gpio_pin rx;

	// soft serial baudrate
	e_soft_serial_speed baudrate;
};

/**
 * @brief initialize SOFT USART as RS232 port
 *
 * @param a_speed speed (e_soft_serial_speed enumeration can be used for standard BAUD rates)
 *
 * @return always success
 */
e_return soft_serial_init(volatile struct soft_serial *a_bus);


/**
 * @brief install soft_serial handlers so we can use printf-like functions with soft_serial console
 */
void soft_serial_install_stdio();


/**
 * @brief check how many bytes are available, pending to be read 
 *
 * @return number of bytes received and waiting
 */
unsigned char soft_serial_available();


/**
 * @brief check if there is any data available pending 
 *
 * @param a_data buffer to place any data in
 * @param a_size size of the buffer
 *
 * @return number of bytes available
 */
unsigned char soft_serial_peek(void *a_data, unsigned char a_size);


/**
 * @brief receive data
 *
 * @param a_data buffer for the data
 * @param a_size buffer size
 * @param a_waitall 1 - block until a data block of requested size is assembled
 *
 * @return number of bytes read
 */
unsigned int soft_serial_recv(void *a_data, unsigned int a_size, unsigned char a_waitall);


/**
 * @brief read a single char from the port
 *
 * @param a_data data read from the port
 *
 * @return 1 if character available, 0 if not
 */
unsigned char soft_serial_getc(unsigned char *a_data);


/**
 * @brief send data using interrupts
 *
 * @param a_data data to be send
 * @param a_size size of data
 * @param a_waitall block until everything is sent
 *
 * @return number of characters send
 */
unsigned char soft_serial_send(void *a_data, unsigned int a_size, unsigned char a_waitall);


/**
 * @brief send a character using interrupts
 *
 * @param a_data character to be sent
 *
 * @return 1 if sent
 */
unsigned char soft_serial_sendc(unsigned char a_data);


/**
 * @brief return soft_serial buffer context for RX (information and statistics about soft_serial port)
 *
 * @return soft_serial buffer info
 */
volatile t_ssbuffer* soft_serial_get_rx_state();


/**
 * @brief return soft_serial buffer context for TX (information and statistics about soft_serial port)
 *
 * @return soft_serial buffer info
 */
volatile t_ssbuffer* soft_serial_get_tx_state();


#endif /* __SOFT_SERIAL_H__ */
