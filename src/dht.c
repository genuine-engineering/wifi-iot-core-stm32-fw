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
 * File: dht.c
 * Created: 3:28:05 PM Nov 18, 2015 GMT+7
 */
#include "config.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <stdint.h>
#include <stdio.h>
#include "tick.h"
#include "dht.h"
#include "debug.h"

#define dht_output() gpio_clear(dht->port, dht->pin);\
  gpio_set_mode(dht->port, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, dht->pin)
#define dht_input() gpio_set_mode(dht->port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, dht->pin)

static dht_device g_dht;
void dht_init(uint32_t type, uint32_t port, uint32_t pin)
{
  dht_device *dht = &g_dht;
  dht->type = type;
  dht->port = port;
  dht->pin = pin;
  dht->last_temp = 0;
  dht->last_humidity = 0;
  dht->state = DHT_STATE_IDLE;
  dht->tick = tick_get();
  dht_input();
}

void dht_stop(void)
{
  dht_device *dht = &g_dht;
  if (dht->state != DHT_STATE_STOP)
    dht->state = DHT_STATE_STOP;
}

void dht_resume(void)
{
  dht_device *dht = &g_dht;
  if (dht->state == DHT_STATE_STOP)
    dht->state = DHT_STATE_IDLE;
}
void dht_manage(void)
{
  dht_device *dht = &g_dht;
  switch (dht->state)
  {
    case DHT_STATE_IDLE:
      if (tick_expire_ms(&dht->tick, DHT_READ_INTERVAL))
      {
        dht->state = DHT_STATE_START;
      }
      break;
    case DHT_STATE_START:
      dht->valid = -1;

      dht_output(); /*begin output low >18ms*/
      tick_wait_ms(40);

      asm volatile ("cpsid i");

      dht_input();
      uint32_t timeout_tick = tick_get_tick();

      while (dht_read() != 0 && tick_expire_us(&timeout_tick, 40) == 0); /* pull-up max 40us, dht response next 80us low */

      dht_reread_if(dht_read() != 0, "Error wait start pulse\r\n");

      timeout_tick = tick_get_tick();


      while (dht_read() == 0 && tick_expire_us(&timeout_tick, 90) == 0); /*wait dht pull up, timeout 100us*/
      dht_reread_if(dht_read() == 0, "Error wait start high pulse\r\n"); /*timeout*/

      timeout_tick = tick_get_tick();
      while (dht_read() != 0 && tick_expire_us(&timeout_tick, 90) == 0); /* wait dht pull down, timeout 100us*/

      dht_reread_if(dht_read() != 0, "Error wait start low pulse\r\n"); /*timeout*/

      uint8_t dht_idx = 0;

      uint32_t delta_t, t1, t2;

      t2 = tick_get_tick();
      timeout_tick = tick_get_tick();
      while (dht_idx < 40)
      {
        uint8_t dht_byte = dht_idx / 8;

        while (dht_read() == 0 && tick_expire_us(&timeout_tick, 60) == 0);
        timeout_tick = tick_get_tick();
        t1 = tick_get_tick();
        delta_t = tick_delta_time_tick(t1, t2) / 74;
        dht_reread_if(delta_t < 40 || delta_t > 60, "DHT: Invalid bit start\n");

        while (dht_read() != 0 && tick_expire_us(&timeout_tick, 10) == 0);

        t2 = tick_get_tick();
        timeout_tick = tick_get_tick();
        delta_t = tick_delta_time_tick(t2, t1) / 72;
        dht_reread_if(delta_t < 20 || delta_t > 100, "DHT: Invalid bit, re read\n");
        if (delta_t > 30)
        {

          dht->data[dht_byte] <<= 1;
          dht->data[dht_byte] |= 1;

        }
        else
        {
          dht->data[dht_byte] <<= 1;
        }
        dht_idx ++;

      }

      /* checksum */
      uint8_t cksum = 0;
      for (dht_idx = 0; dht_idx < 4; dht_idx++)
      {
        cksum += dht->data[dht_idx];
      }
      if (cksum == dht->data[4])
      {
        dht->last_temp = (double)dht->data[2];
        dht->last_humidity =  (double)dht->data[0];
        dht->valid = 1;
        INFO("DHT: = %f *C, %f \%RH\n", dht->last_temp, dht->last_humidity);
      }


      asm volatile ("cpsie i");
      dht->state = DHT_STATE_IDLE;
      dht->tick = tick_get();

  }
}
dht_device * dht_get(void)
{
  return &g_dht;
}
