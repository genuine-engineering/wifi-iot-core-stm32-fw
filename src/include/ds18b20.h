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
 * File: ds18b20.h
 * Created: 4:58:39 PM Nov 17, 2015 GMT+7
 */
#ifndef SRC_INCLUDE_DS18B20_H_
#define SRC_INCLUDE_DS18B20_H_

//commands
#define DS18B20_CMD_CONVERTTEMP 0x44
#define DS18B20_CMD_RSCRATCHPAD 0xbe
#define DS18B20_CMD_WSCRATCHPAD 0x4e
#define DS18B20_CMD_CPYSCRATCHPAD 0x48
#define DS18B20_CMD_RECEEPROM 0xb8
#define DS18B20_CMD_RPWRSUPPLY 0xb4
#define DS18B20_CMD_SEARCHROM 0xf0
#define DS18B20_CMD_READROM 0x33
#define DS18B20_CMD_MATCHROM 0x55
#define DS18B20_CMD_SKIPROM 0xcc
#define DS18B20_CMD_ALARMSEARCH 0xec

#ifndef DS_READ_INTERVAL_MS
#define DS_READ_INTERVAL_MS 2000
#endif

typedef enum
{
	DS_STATE_IDLE = 0,
	DS_STATE_READ,
	DS_STATE_STOP
} ds_state;
typedef struct
{
	uint32_t port;
	uint32_t pin;
	double last_temp;
	int32_t valid;
	uint32_t tick;
	ds_state state;
} ds18b20;

void ds_init(uint32_t gpioport, uint32_t gpiopin);

void ds_manage(void);
uint8_t ds_crc(uint8_t *crc, uint8_t data);
void ds_stop(void);
void ds_resume(void);
ds18b20 *ds_get(void);
#endif /* SRC_INCLUDE_DS18B20_H_ */
