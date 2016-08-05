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
 * File: main.c
 * Created: 8:15:47 AM Nov 17, 2015 GMT+7
 */

#include "config.h"
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include "cdcacm.h"
#include "esp.h"
#include "tick.h"
#include "rtc.h"
#include "ds18b20.h"
#include "analog.h"
#include "dht.h"
#include "io.h"
#include "debug.h"
#include "light.h"

void clock_init(void);

int main(void)
{
  clock_init();
  tick_init();
  io_init();
  cdcacm_init();
  esp_init();
  rtc_init();
  dht_init(DHT_11, GPIOB, GPIO13);
  ds_init(GPIOC, GPIO6);

  analog_init();
  light_init();

  int32_t main_tick = tick_get();


  while (1)
  {
    cdcacm_manage();
    esp_manage();
    analog_manage();
    ds_manage();
    io_manage();
    dht_manage();
    light_manage();

    if (esp_is_prog_mode() == 0)
    {

      analog_resume();
      ds_resume();
      io_resume();
      dht_resume();

    }
    else
    {
      analog_stop();
      ds_stop();
      io_stop();
      dht_stop();
    }

  }

}

void clock_init(void)
{
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_GPIOD);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_AFIO);
}

/* Output for printf */
int _write(int file, char *ptr, int len);
int _write(int file, char *ptr, int len)
{
  int i;
  if (esp_is_prog_mode())
    return len;
  if (file == STDOUT_FILENO || file == STDERR_FILENO)
  {
    for (i = 0; i < len; i++)
    {
      if (ptr[i] == '\n')
      {
        cdcacm_input('\r');
      }
      cdcacm_input(ptr[i]);
    }
    return i;
  }
  errno = EIO;
  return -1;
}
