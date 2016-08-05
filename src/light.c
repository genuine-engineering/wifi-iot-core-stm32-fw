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
 * File: light.c
 * Created: 2:57:19 PM Nov 25, 2015 GMT+7
 */
#include "config.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include "tick.h"
#include "light.h"
#include "debug.h"

static light_dev light;
void light_init(void)
{
  /* Enable clocks for I2C2 and AFIO. */
  rcc_periph_clock_enable(RCC_I2C2);
  rcc_periph_clock_enable(RCC_AFIO);

  /* Set alternate functions for the SCL and SDA pins of I2C2. */
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                GPIO_I2C2_SCL | GPIO_I2C2_SDA);
  /* Disable the I2C before changing any configuration. */
  i2c_peripheral_disable(I2C2);

  /* APB1 is running at 36MHz. */
  i2c_set_clock_frequency(I2C2, I2C_CR2_FREQ_36MHZ);

  /* 400KHz - I2C Fast Mode */
  i2c_set_fast_mode(I2C2);

  /*
   * fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
   * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
   * Datasheet suggests 0x1e.
   */
  i2c_set_ccr(I2C2, 0x1e);

  /*
   * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
   * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
   * Incremented by 1 -> 11.
   */
  i2c_set_trise(I2C2, 0x0b);

  /*
   * This is our slave address - needed only if we want to receive from
   * other masters.
   */
  i2c_set_own_7bit_slave_address(I2C2, 0x32);

  /* If everything is configured -> enable the peripheral. */
  i2c_peripheral_enable(I2C2);
  light.tick = tick_get();
  light.valid = -1;
}

static void light_start_mesure(uint32_t i2c)
{
  uint32_t reg32 __attribute__((unused));

  /* Send START condition. */
  i2c_send_start(i2c);
  light.timeout_tick = tick_get_tick();
  /* Waiting for START is send and switched to master mode. */
  while (!((I2C_SR1(i2c) & I2C_SR1_SB)
           & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)))
         && tick_expire_us(&light.timeout_tick, 1000) == 0)
    ;

  /* Send destination address. */
  i2c_send_7bit_address(i2c, 0x23, I2C_WRITE);

  /* Waiting for address is transferred. */
  light.timeout_tick = tick_get_tick();
  while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)
         && tick_expire_us(&light.timeout_tick, 1000) == 0)
    ;

  /* Cleaning ADDR condition sequence. */
  reg32 = I2C_SR2(i2c);

  /* Sending the data. */
  i2c_send_data(i2c, 0x10);

  /* After the last byte we have to wait for TxE too. */
  light.timeout_tick = tick_get_tick();
  while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE))
         && tick_expire_us(&light.timeout_tick, 1000) == 0)
    ;

  /* Send STOP condition. */
  i2c_send_stop(i2c);
}

static uint16_t light_read(uint32_t i2c)
{
  uint32_t reg32 __attribute__((unused));
  uint16_t ret_data = 0;
  /* Send START condition. */
  i2c_send_start(i2c);
  light.timeout_tick = tick_get_tick();
  while (!((I2C_SR1(i2c) & I2C_SR1_SB)
           & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)))
         && tick_expire_us(&light.timeout_tick, 1000) == 0)
    ;

  i2c_send_7bit_address(i2c, 0x23, I2C_READ);

  /* 2-byte receive is a special case. See datasheet POS bit. */
  I2C_CR1(i2c) |= (I2C_CR1_POS | I2C_CR1_ACK);

  /* Waiting for address is transferred. */
  light.timeout_tick = tick_get_tick();
  while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)
         && tick_expire_us(&light.timeout_tick, 1000) == 0)
    ;

  /* Cleaning ADDR condition sequence. */
  reg32 = I2C_SR2(i2c);

  /* Cleaning I2C_SR1_ACK. */
  I2C_CR1(i2c) &= ~I2C_CR1_ACK;

  /* Now the slave should begin to send us the first byte. Await BTF. */
  light.timeout_tick = tick_get_tick();
  while (!(I2C_SR1(i2c) & I2C_SR1_BTF)
         && tick_expire_us(&light.timeout_tick, 1000) == 0)
    ;
  ret_data = (uint16_t) (I2C_DR(i2c) << 8); /* MSB */

  /*
   * Yes they mean it: we have to generate the STOP condition before
   * saving the 1st byte.
   */
  I2C_CR1(i2c) |= I2C_CR1_STOP;

  ret_data |= I2C_DR(i2c); /* LSB */

  /* Original state. */
  I2C_CR1(i2c) &= ~I2C_CR1_POS;
  light.valid = 0;

  return ret_data;

}

void light_manage(void)
{
  if (tick_expire_ms(&light.tick, 1000))
  {
    light.valid = -1;
    light_start_mesure(I2C2);
    tick_wait_ms(200);
    light.last_lux = light_read(I2C2);
    INFO("Light: %d\r\n", light.last_lux);
  }
}

light_dev *light_get(void)
{
  return &light;
}
