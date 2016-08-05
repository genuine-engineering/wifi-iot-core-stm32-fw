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
 * File: rtc.c
 * Created: 2:40:52 PM Nov 17, 2015 GMT+7
 */
#include "config.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/cm3/nvic.h>
#include "cdcacm.h"
#include "rtc.h"
#include <stdio.h>

void rtc_init(void)
{
  rtc_auto_awake(RCC_LSE, 0x7fff);
  //rtc_interrupt_enable(RTC_SEC);
  //nvic_enable_irq(NVIC_RTC_IRQ);
  //nvic_set_priority(NVIC_RTC_IRQ, 1);
}


void rtc_isr(void)
{
  volatile uint32_t j = 0, c = 0;

  /* The interrupt flag isn't cleared by hardware, we have to do it. */
  rtc_clear_flag(RTC_SEC);

  c = rtc_get_counter_val();

  /* Display the current counter value in binary via USART1.
  for (j = 0; j < 32; j++) {
    if ((c & (0x80000000 >> j)) != 0)
      cdcacm_input('1');
    else
      cdcacm_input('0');
  }
  cdcacm_input('\r');
  cdcacm_input('\n');
  */
  //printf("Current counter: %d \r\n", c);
}
