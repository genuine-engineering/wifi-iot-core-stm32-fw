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
 * File: io.c
 * Created: 8:51:07 AM Nov 25, 2015 GMT+7
 */
#include "config.h"
#include <libopencm3/stm32/gpio.h>
#include "esp.h"
#include "io.h"
#include "tick.h"
#include "debug.h"


port_io ports[MAX_PORT_OUTPUT];

void io_init(void)
{
  io_port_init(&ports[PORT_LED_STATUS_IDX], GPIOB, GPIO4, 0);
  AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST; /*special for GPIOB_4*/

  io_port_init(&ports[PORT_DOSING_PUMP1_IDX], GPIOA, GPIO10, 1);
  io_port_init(&ports[PORT_DOSING_PUMP2_IDX], GPIOA, GPIO9, 1);
  io_port_init(&ports[PORT_REFLUX_PUMP_IDX], GPIOB, GPIO6, 1);
  io_port_init(&ports[PORT_PRESSURE_PUMP_IDX], GPIOC, GPIO13, 1);
  io_port_init(&ports[PORT_VALVE_INPUT_IDX], GPIOB, GPIO7, 1);
  io_port_init(&ports[PORT_VALVE_OUTPUT_IDX], GPIOB, GPIO8, 1);
  io_port_init(&ports[PORT_LED_PHOTOSYNTHESIS_IDX], GPIOB, GPIO9, 1);
  io_port_init(&ports[PORT_WATER_SENSOR_VOLT_IDX], GPIOB, GPIO12, 1);

  //io_port_on(&ports[PORT_WATER_SENSOR_VOLT_IDX]);
  //io_port_on(&ports[PORT_LED_STATUS_IDX]);
  //io_port_off(&ports[PORT_LED_STATUS_IDX]);
  //tick_wait_ms(1000);

  //io_port_blink(&ports[PORT_LED_STATUS_IDX], 500, 500, 0);
}

void io_port_blink(port_io *pio, uint32_t on_time_ms, uint32_t off_time_ms,
                   uint32_t repeat)
{
  pio->on_time = on_time_ms;
  pio->off_time = off_time_ms;
  pio->repeat = repeat;
  pio->repeat_cnt = 0;
  pio->tick = tick_get();
}

void io_port_on_timeout(port_io *pio, uint32_t timeout_ms)
{
  io_port_blink(pio, timeout_ms, 0, 1);
}
void io_port_init(port_io *pio, uint32_t port, uint32_t pin, uint32_t on_value)
{
  pio->port = port;
  pio->pin = pin;
  pio->on_time = 0;
  pio->off_time = 0;
  pio->repeat = 0;
  pio->repeat_cnt = 0;
  pio->tick = tick_get();
  pio->state = PORT_MANUAL;
  pio->on_value = on_value;
  gpio_set_mode(pio->port, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                pio->pin);
  //io_port_off(pio);
}

void io_port_on(port_io *pio)
{
  if (pio->on_value)
    gpio_set(pio->port, pio->pin);
  else
    gpio_clear(pio->port, pio->pin);

}

void io_port_off(port_io *pio)
{
  if (pio->on_value)
    gpio_clear(pio->port, pio->pin);
  else
    gpio_set(pio->port, pio->pin);

}

void io_port_blink_idx(uint32_t idx, uint32_t on_time_ms, uint32_t off_time_ms, uint32_t repeat)
{
  io_port_blink(&ports[idx], on_time_ms, off_time_ms, repeat);
}
void io_port_on_idx(uint32_t idx)
{

  io_port_on(&ports[idx]);

}
void io_port_off_idx(uint32_t idx)
{
  io_port_off(&ports[idx]);

}
static void io_port_stop(port_io *pio)
{
  if (pio->state < PORT_STOP)
    pio->state = PORT_STOP;
}
static void io_port_resume(port_io *pio)
{
  if (pio->state == PORT_STOP)
    pio->state = PORT_ON;
}
void io_stop(void)
{
  uint8_t i;
  for (i = 0; i < MAX_PORT_OUTPUT; i++)
  {
    io_port_stop(&ports[i]);
  }

}
void io_resume(void)
{
  uint8_t i;
  for (i = 0; i < MAX_PORT_OUTPUT; i++)
  {
    io_port_resume(&ports[i]);
  }

}
static void io_port_manage(port_io *pio)
{
  switch (pio->state)
  {
    case PORT_ON:
      if (pio->repeat == 0
          || (pio->repeat > 0 && pio->repeat_cnt < pio->repeat))
      {
        if (tick_expire_ms(&pio->tick, pio->off_time))
        {
          pio->state = PORT_OFF;
          io_port_on(pio);
          //INFO("OFF\r\n");
        }
      }
      break;
    case PORT_OFF:
      if (pio->repeat == 0
          || (pio->repeat > 0 && pio->repeat_cnt < pio->repeat))
      {
        if (tick_expire_ms(&pio->tick, pio->on_time))
        {
          pio->state = PORT_ON;
          io_port_off(pio);
          if (pio->repeat > 0)
            pio->repeat_cnt++;
          //INFO("ON\r\n");
        }
      }
  }
}
void io_manage(void)
{
  uint8_t i;
  for (i = 0; i < MAX_PORT_OUTPUT; i++)
  {
    io_port_manage(&ports[i]);
  }


}
