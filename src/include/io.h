/*
 * This file is part of the libopencm3 project.
 *
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser General Public License
 * (LGPL) version 2.1 which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/lgpl-2.1.html
 *
 * Contributors:
 *     Tuan PM <tuanpm@live.com>
 * File: io.h
 * Created: 8:51:17 AM Nov 25, 2015 GMT+7
 */
#ifndef SRC_INCLUDE_IO_H_
#define SRC_INCLUDE_IO_H_



typedef enum
{
	PORT_ON = 0,
	PORT_OFF,
	PORT_STOP,
	PORT_MANUAL
} port_io_state;

typedef struct
{
	uint32_t port;
	uint32_t pin;
	uint32_t on_time;
	uint32_t off_time;
	uint32_t repeat;
	uint32_t repeat_cnt;
	uint32_t tick;
	uint32_t on_value;
	port_io_state state;

}port_io;



void io_init(void);
void io_port_init(port_io *pio, uint32_t port, uint32_t pin, uint32_t on_value);
void io_port_blink(port_io *pio, uint32_t on_time_ms, uint32_t off_time_ms, uint32_t repeat);
void io_port_on(port_io *pio);
void io_port_off(port_io *pio);

void io_port_blink_idx(uint32_t idx, uint32_t on_time_ms, uint32_t off_time_ms, uint32_t repeat);
void io_port_on_idx(uint32_t idx);
void io_port_off_idx(uint32_t idx);

void io_manage(void);
void io_port_on_timeout(port_io *pio, uint32_t timeout_ms);
void io_stop(void);
void io_resume(void);
#endif /* SRC_INCLUDE_IO_H_ */
