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
 * File: dht.h
 * Created: 3:28:18 PM Nov 18, 2015 GMT+7
 */
#ifndef SRC_INCLUDE_DHT_H_
#define SRC_INCLUDE_DHT_H_


#define DHT_11					0
#define DHT_22					1
#ifndef DHT_READ_INTERVAL
#define DHT_READ_INTERVAL	2000	//milisec
#endif

typedef enum
{
	DHT_STATE_IDLE = 0,
	DHT_STATE_START,
	DHT_STATE_WAIT_START,
	DHT_STATE_READ,
	DHT_STATE_WAIT_READ,
	DHT_STATE_STOP
} dht_state;
typedef struct
{
	uint32_t port;
	uint32_t pin;
	uint32_t type;
	double last_temp;
	double last_humidity;
	int32_t valid;
	uint32_t tick;
	dht_state state;
	uint8_t data[5];
} dht_device;

void dht_manage(void);
void dht_init(uint32_t type, uint32_t port, uint32_t pin);
void dht_stop(void);
void dht_resume(void);
dht_device *dht_get(void);
#define dht_read()	gpio_get(dht->port, dht->pin)

#define dht_reread_if(x, y) if(x){ asm volatile ("cpsie i");dht_input(); dht->state = DHT_STATE_IDLE; INFO(y);break;}


#endif /* SRC_INCLUDE_DHT_H_ */
