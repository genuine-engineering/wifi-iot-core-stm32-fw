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
 * File: ds18b20.c
 * Created: 4:50:53 PM Nov 17, 2015 GMT+7
 */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <stdint.h>
#include "tick.h"
#include "ds18b20.h"

static void ds18b20_writebit(ds18b20* dev, uint8_t bit);
static uint8_t ds18b20_readbit(ds18b20* dev);
static void ds18b20_writebyte(ds18b20* dev, uint8_t byte);
static uint8_t ds18b20_readbyte(ds18b20* dev);
uint8_t ds18b20_reset(ds18b20* dev);

#define _delay_us(x) tick_wait_us(x)
#define ds_output() gpio_clear(dev->port, dev->pin);\
  gpio_set_mode(dev->port, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, dev->pin)
#define ds_input() gpio_set_mode(dev->port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, dev->pin)

ds18b20 g_ds;

ds18b20 *ds_get(void)
{
  return &g_ds;
}

void ds_init(uint32_t gpioport, uint32_t gpiopin)
{
  ds18b20* dev = &g_ds;
  dev->port = gpioport;
  dev->pin = gpiopin;
  dev->last_temp = 0;
  dev->tick = tick_get();
  dev->valid = -1;
  dev->state = DS_STATE_IDLE;
}
void ds_stop(void)
{
  ds18b20* dev = &g_ds;
  if (dev->state != DS_STATE_STOP)
    dev->state = DS_STATE_STOP;
}
void ds_resume(void)
{
  ds18b20* dev = &g_ds;
  if (dev->state == DS_STATE_STOP)
    dev->state = DS_STATE_IDLE;
}

/*
 * get temperature
 */
static double ds_gettemp(ds18b20* dev)
{
  uint8_t temp_h, temp_l, crc = 0;
  double retd = 0;


  ds18b20_reset(dev); //reset
  ds18b20_writebyte(dev, DS18B20_CMD_SKIPROM); //skip ROM
  ds18b20_writebyte(dev, DS18B20_CMD_RSCRATCHPAD); //read scratchpad

  //read 2 byte from scratchpad
  temp_l = ds18b20_readbyte(dev);
  ds_crc(&crc, temp_l);
  temp_h = ds18b20_readbyte(dev);
  ds_crc(&crc, temp_h);
  ds_crc(&crc, ds18b20_readbyte(dev));
  ds_crc(&crc, ds18b20_readbyte(dev));
  ds_crc(&crc, ds18b20_readbyte(dev));
  ds_crc(&crc, ds18b20_readbyte(dev));
  ds_crc(&crc, ds18b20_readbyte(dev));
  ds_crc(&crc, ds18b20_readbyte(dev));

  if (crc != ds18b20_readbyte(dev))
  {
    dev->valid = -1;
    return 0;
  }

  //printf("temph: %X, templ: %X\r\n", temperature_h, temperature_l);
  //convert the 12 bit value obtained
  retd = ((temp_h << 8) + temp_l) * 0.0625;
  dev->last_temp = retd;
  dev->valid = 0;
  return retd;
}

void ds_manage(void)
{
  ds18b20* dev = &g_ds;
  switch (dev->state)
  {
    case DS_STATE_READ:
      if (tick_expire_ms(&dev->tick, 750))
      {

        //tick_wait_ms(10);
        asm volatile("cpsid i");
        ds_gettemp(dev);
        asm volatile("cpsie i");
        dev->state = DS_STATE_IDLE;
      }
      break;
    case DS_STATE_IDLE:
      if (tick_expire_ms(&dev->tick, DS_READ_INTERVAL_MS))
      {
        dev->state = DS_STATE_READ;
        //asm volatile("cpsid i");
        ds18b20_reset(dev); //reset
        ds18b20_writebyte(dev, DS18B20_CMD_SKIPROM); //skip ROM
        ds18b20_writebyte(dev, DS18B20_CMD_CONVERTTEMP); //start temperature conversion
        //asm volatile("cpsie i");

      }

      break;
  }


}

uint8_t ds_crc(uint8_t *crc, uint8_t data)
{
  uint8_t i = (data ^ *crc) & 0xff;
  *crc = 0;
  if (i & 0x01) *crc ^= 0x5e;
  if (i & 0x02) *crc ^= 0xbc;
  if (i & 0x04) *crc ^= 0x61;
  if (i & 0x08) *crc ^= 0xc2;
  if (i & 0x10) *crc ^= 0x9d;
  if (i & 0x20) *crc ^= 0x23;
  if (i & 0x40) *crc ^= 0x46;
  if (i & 0x80) *crc ^= 0x8c;
  return *crc;
}

uint8_t ds18b20_reset(ds18b20* dev)
{
  uint8_t pressence, short_circuit;

  //low for 480us
  //low

  ds_output();

  _delay_us(480);

  //release line and wait for 60uS
  ds_input();
  _delay_us(64);

  pressence = gpio_get(dev->port, dev->pin);
  _delay_us(480 - 64);
  short_circuit = gpio_get(dev->port, dev->pin);
  //get value and wait 420us

  return pressence | (short_circuit >> 1);
}

/*
 * write one bit
 */
static void ds18b20_writebit(ds18b20* dev, uint8_t bit)
{
  //low for 1uS
  //low
  ds_output();
  _delay_us(1);

  //if we want to write 1, release the line (if not will keep low)
  if (bit)
    ds_input();

  //wait 60uS and release the line
  _delay_us(58);
  ds_input();
}

/*
 * read one bit
 */
static uint8_t ds18b20_readbit(ds18b20* dev)
{
  uint8_t bit = 0;

  //low for 1uS
  //low
  ds_output();
  _delay_us(1);

  //release line and wait for 14uS
  ds_input();
  _delay_us(12);

  //read the value
  if (gpio_get(dev->port, dev->pin))
    bit = 1;

  //wait 45uS and return read value
  _delay_us(43);
  return bit;
}

/*
 * write one byte
 */
static void ds18b20_writebyte(ds18b20* dev, uint8_t byte)
{
  uint8_t i = 8;

  while (i--)
  {
    ds18b20_writebit(dev, byte & 1);
    byte >>= 1;
  }

}

/*
 * read one byte
 */
static uint8_t ds18b20_readbyte(ds18b20* dev)
{
  uint8_t i = 8, n = 0;

  while (i--)
  {
    n >>= 1;
    n |= (ds18b20_readbit(dev) << 7);
  }

  return n;
}
