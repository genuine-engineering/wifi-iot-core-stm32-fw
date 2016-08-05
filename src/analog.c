/*
 * This file is part of the libopencm3 project.
 * *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser General Public License
 * (LGPL) version 2.1 which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/lgpl-2.1.html
 *
 * Contributors:
 *     Tuan PM <tuanpm@live.com>
 * File: analog.c
 * Created: 5:31:04 PM Nov 18, 2015 GMT+7
 */
#include "config.h"
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include "analog.h"
#include "tick.h"
#include "debug.h"

typedef struct
{
  uint32_t running;
  uint32_t is_ready;
  uint32_t tick;
  uint16_t buf[ADC_CHANNEL * ADC_SAMPLE];
} analog_dev;

static analog_dev analog;

double analog_get(uint8_t index)
{
  uint16_t i, min = 5000, max = 0;
  double retval = 0;
  for (i = index; i < ADC_SAMPLE * ADC_CHANNEL; i += ADC_CHANNEL)
  {
    retval += analog.buf[i];
  }
  retval /= ADC_SAMPLE;
  return retval;
}

#define ANALOG_TIMER          TIM4
#define ANALOG_RCC_TIMER      RCC_TIM4
#define ANALOG_TIMER_IRQ      NVIC_TIM4_IRQ
#define ANALOG_TIMER_ISR      tim4_isr

static void analog_init_timer(void)
{

  rcc_periph_clock_enable(ANALOG_RCC_TIMER);

  timer_reset(ANALOG_TIMER);
  timer_set_mode(ANALOG_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_set_period(ANALOG_TIMER, SENSOR_TICK);
  timer_set_prescaler(ANALOG_TIMER, 71);
  timer_set_clock_division(ANALOG_TIMER, 0x0);

  /* Generate TRGO on every update. */
  timer_set_master_mode(ANALOG_TIMER, TIM_CR2_MMS_UPDATE);
  timer_enable_counter(ANALOG_TIMER);

  timer_enable_irq(ANALOG_TIMER, TIM_DIER_UIE);

  nvic_enable_irq(ANALOG_TIMER_IRQ);
}

void ANALOG_TIMER_ISR()
{

  if (timer_get_flag(ANALOG_TIMER, TIM_SR_UIF))
  {

    timer_clear_flag(ANALOG_TIMER, TIM_SR_UIF);
    adc_start_conversion_regular(ADC1);
  }
}

static void analog_init_dma(void)
{
  /* Reset DMA channel*/
  rcc_periph_clock_enable(RCC_DMA1);
  dma_channel_reset(DMA1, DMA_CHANNEL1);

  dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t) &ADC1_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t) analog.buf);
  dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_SAMPLE * ADC_CHANNEL);
  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
  dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
  dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);

  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);

  dma_enable_channel(DMA1, DMA_CHANNEL1);

  //adc_enable_eoc_interrupt(ADC1);

  nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 5);
  nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
}

void dma1_channel1_isr(void)
{
  if ((DMA1_ISR & DMA_ISR_TCIF1) != 0)
  {
    DMA1_IFCR |= DMA_IFCR_CTCIF1;

    analog.is_ready = 1;
  }

}

static void analog_init_adc(void)
{
  uint8_t channel_array[ADC_CHANNEL];
  rcc_periph_clock_enable(RCC_ADC1);

  /* Make sure the ADC doesn't run during config. */
  adc_off(ADC1);

  /* We configure everything for one single timer triggered injected conversion with interrupt generation. */
  /* While not needed for a single channel, try out scan mode which does all channels in one sweep and
   * generates the interrupt/EOC/JEOC flags set at the end of all channels, not each one.
   */
  adc_enable_scan_mode(ADC1);
  adc_set_single_conversion_mode(ADC1);
  /* We want to start the injected conversion with the TIM2 TRGO */
  adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
  /* Generate the ADC1_2_IRQ */
  //adc_enable_eoc_interrupt(ADC1);
  adc_set_right_aligned(ADC1);
  /* We want to read the temperature sensor, so we have to enable it. */
  adc_enable_temperature_sensor(ADC1);
  adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_7DOT5CYC);

  /* Select the channels we want to convert.
   * 16=temperature_sensor, 17=Vrefint, 13=ADC1, 10=ADC2
   */

  channel_array[CH_DOSING_PUMP1_IDX] = CH_DOSING_PUMP1;
  channel_array[CH_DOSING_PUMP2_IDX] = CH_DOSING_PUMP2;
  channel_array[CH_REFLUX_PUMP_IDX] = CH_REFLUX_PUMP;
  channel_array[CH_VALVE_INPUT_IDX] = CH_VALVE_IN;
  channel_array[CH_VALVE_OUTPUT_IDX] = CH_VALVE_OUT;
  channel_array[CH_LED_PHOTOSYNT_IDX] = CH_LED_PHOTOSYNT;
  channel_array[CH_PRESSURE_PUMP_IDX] = CH_PRESSURE_PUMP;
  channel_array[CH_WATER_LEVEL_HIGH_IDX] = CH_WATER_LEVEL_HIGH;
  channel_array[CH_WATER_LEVEL_MID_IDX] = CH_WATER_LEVEL_MID;
  channel_array[CH_WATER_LEVEL_LOW_IDX] = CH_WATER_LEVEL_LOW;
  channel_array[CH_EC_SENSOR_IDX] = CH_EC_SENSOR;
  channel_array[CH_INTERNAL_TEMP_IDX] = CH_INTERNAL_TEMP;
  channel_array[CH_INTERVAL_VREF_IDX] = CH_INTERVAL_VREF;

  adc_set_regular_sequence(ADC1, ADC_CHANNEL, channel_array);

  adc_power_on(ADC1);
  /* Wait for ADC starting up. */
  int i;
  for (i = 0; i < 800000; i++) /* Wait a bit. */
    __asm__("nop");

  adc_reset_calibration(ADC1);
  while ((ADC_CR2(ADC1) & ADC_CR2_RSTCAL) != 0)
  {
  }
  //added this check
  adc_calibration(ADC1);
  while ((ADC_CR2(ADC1) & ADC_CR2_CAL) != 0)
  {
  }

  adc_enable_dma(ADC1);

}

void analog_stop(void)
{
  if (analog.running == 1)
  {
    analog.running = 0;

    timer_enable_counter(ANALOG_TIMER);
  }

}

void analog_resume(void)
{
  if (analog.running == 0)
  {
    analog.running = 1;
    timer_disable_counter(ANALOG_TIMER);
  }
}

void analog_init()
{

  /* Setup  GPIO */
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO7 | /* Valve input current*/
                GPIO6 | /*  LED Photosynthesis*/
                GPIO5 | /*  Pressure pump */
                GPIO0 | /* Water level high */
                GPIO1 | /* Water level middle */
                GPIO4); /* EC */
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1 | /* dosing pump 2*/
                GPIO0); /*  Valve out */

  gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO5 | /* dosing pump 1*/
                GPIO4 | /*  reflux pump 1*/
                GPIO0); /* Water level low*/
  analog_init_adc();
  analog_init_dma();
  analog_init_timer();

  analog.is_ready = 0;
  analog.running = 1;
  analog.tick = tick_get();
}

void analog_manage(void)
{
  if (tick_expire_ms(&analog.tick, 5000) && analog.is_ready && analog.running)
  {
    analog.is_ready = 0;
    //INFO("AN16: %d, AN17: %d \r\n", analog_get(CH_INTERNAL_TEMP_IDX),
    //analog_get(CH_INTERVAL_VREF_IDX));
  }

}

void adc1_2_isr(void)
{
  /* Clear Injected End Of Conversion (JEOC) */
  ADC_SR(ADC1) &= ~ADC_SR_JEOC;
  //printf(".");
}
