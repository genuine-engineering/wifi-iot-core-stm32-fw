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
 * File: esp.c
 * Created: 8:15:47 AM Nov 17, 2015 GMT+7
 */
#include "config.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rtc.h>

#include <stdio.h>
#include <string.h>
#include "esp.h"
#include "cdcacm.h"
#include "tick.h"
#include "proto.h"
#include "ringbuf.h"
#include "crc16.h"
#include "debug.h"
#include "analog.h"
#include "io.h"
#include "light.h"
#include "ds18b20.h"
#include "dht.h"

esp_dev esp;

static void dma_write(int size)
{
  /*
   * Using channel 2 for USART2_TX
   */

  dma_set_number_of_data(DMA1, DMA_CHANNEL7, size);

  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL7);

  dma_enable_channel(DMA1, DMA_CHANNEL7);

  usart_enable_tx_dma(USART2);
}

void dma1_channel7_isr(void)
{
  if ((DMA1_ISR & DMA_ISR_TCIF7) != 0)
  {
    DMA1_IFCR |= DMA_IFCR_CTCIF7;

    //transfered = 1;
    INFO("DMA Send done\n");
  }

  dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL7);

  usart_disable_tx_dma(USART2);

  dma_disable_channel(DMA1, DMA_CHANNEL7);
}

static void proto_callback(void *parser)
{
  proto_parser *p = (proto_parser*)parser;
  device_control *dev_ctrl = (device_control*)p->buf;
  int32_t *port_ptr = &dev_ctrl->led_status_value, temp_port_value;
  uint32_t *port_ptr_neg = &dev_ctrl->led_status_value;
  uint8_t i;

  //Calculate checksum except begin 4 bytes contain checksum calculated
  uint16_t checksum = crc16_calc(p->buf + 4, p->data_len - 4, 0);

  if (dev_ctrl->checksum == checksum)
  {
    INFO("VALID Packet, process control command\r\n");
    memcpy(&esp.dev_ctrl, dev_ctrl, sizeof(device_control));
    for (i = 0; i < MAX_PORT_OUTPUT; i++)
    {
      temp_port_value = *(port_ptr + i);
      if (temp_port_value < 0 && temp_port_value < -1)
      {
        /* blink value*/
      }
      else if (temp_port_value == 0)
      {
        io_port_off_idx(i);
      }
      else if (temp_port_value > 0)
      {
        io_port_on_idx(i);
      }
      else
      {
        /* no change */
      }
    }
  }
  else
  {
    INFO("MCU: Invalid packet from ESP\r\n");
  }

}
void esp_init(void)
{
  nvic_disable_irq(NVIC_USART2_IRQ);
  /* Setup GPIO pin GPIO_USART2_RE_TX on GPIO port A for transmit. */

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

  /* Setup GPIO pin GPIO_USART2_RE_RX on GPIO port A for receive. */
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

  gpio_set_mode(ESP_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, ESP_RST | ESP_PROG | ESP_PD);

  /* Setup UART parameters. */
  usart_set_baudrate(USART2, 115200);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX);

  /* Enable USART2 Receive interrupt. */
  USART_CR1(USART2) |= USART_CR1_RXNEIE;
  USART_CR1(USART2) &= ~USART_CR1_TXEIE;


  /* Reset DMA channel*/
  /*
  dma_channel_reset(DMA1, DMA_CHANNEL7);

  dma_set_peripheral_address(DMA1, DMA_CHANNEL7, (uint32_t)&USART2_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL7, (uint32_t)esp.tx_buf);

  dma_set_read_from_memory(DMA1, DMA_CHANNEL7);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL7);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL7, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL7, DMA_CCR_MSIZE_8BIT);
  dma_set_priority(DMA1, DMA_CHANNEL7, DMA_CCR_PL_VERY_HIGH);
  nvic_set_priority(NVIC_DMA1_CHANNEL7_IRQ, 1);
  nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);
  */
  /* Finally enable the USART. */
  usart_enable(USART2);
  nvic_enable_irq(NVIC_USART2_IRQ);

  /* Init proto parser and ringbuf for handle data from ESP */
  ringbuf_init(&esp.rb, esp.rb_buf, sizeof esp.rb_buf);
  proto_init(&esp.parser, proto_callback, esp.proto_buf, sizeof esp.proto_buf);
  esp.queue_restart_time = 0;
  esp.prog_mode = 0;
  esp.tick = tick_get();

  memset(&esp.dev_status, 0xFF, sizeof(device_status));
  esp_restart();
}

void usart2_isr(void)
{
  uint8_t data;

  /* Check if we were called because of RXNE. */
  if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
      ((USART_SR(USART2) & USART_SR_RXNE) != 0))
  {


    /* Retrieve the data from the peripheral. */
    data = usart_recv(USART2);
    ringbuf_put(&esp.rb, data);
    /* Enable transmit interrupt so it sends back the data. */
    //USART_CR1(USART2) |= USART_CR1_TXEIE;
  }

  /* Check if we were called because of TXE. */
  if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
      ((USART_SR(USART2) & USART_SR_TXE) != 0))
  {


    /* Disable the TXE interrupt as we don't need it anymore. */
    USART_CR1(USART2) &= ~USART_CR1_TXEIE;
  }
}

void esp_enter_prog(void)
{
  uint32_t wait = 100000;

  gpio_clear(ESP_PORT, ESP_RST | ESP_PROG | ESP_PD);
  while (wait--);
  gpio_set(ESP_PORT, ESP_RST | ESP_PD);
  esp.prog_mode = 1;
}
void esp_leave_prog(void)
{
  esp.prog_mode = 0;
}
uint32_t esp_is_prog_mode(void)
{
  return esp.prog_mode;
}

void esp_restart(void)
{
  uint32_t wait = 100000;
  gpio_clear(ESP_PORT, ESP_RST | ESP_PD);
  gpio_set(ESP_PORT, ESP_PROG);
  while (wait--);
  gpio_set(ESP_PORT, ESP_RST | ESP_PD);

}

void esp_restart_timeout(uint32_t time_ms)
{
  esp.queue_restart_time = time_ms;
  esp.tick = tick_get();
  esp.prog_mode = 0;
}

void esp_clear_restart(void)
{
  esp.queue_restart_time = 0;
}



uint32_t esp_check_sync_frame(uint8_t data)
{
  static uint8_t sync_count = 0;
  if (data == 0x55)
  {
    sync_count ++;
    if (sync_count >= 32)
    {
      sync_count = 0;
      return 1;
    }
  }
  else
  {
    sync_count = 0;
  }
  return 0;
}

uint32_t esp_check_end_frame(uint8_t data)
{
  const uint8_t end_pkt[] = {0x00, 0x04, 0x04, 0, 0, 0, 0, 0, 0x01, 0, 0, 0, 0};
  static uint8_t end_count = 0;
  if (data == end_pkt[end_count])
  {
    end_count ++;
    if (end_count >= 12)
    {
      return 1;
    }
  }
  else
  {
    end_count = 0;
  }
  return 0;
}

void esp_write(uint8_t *data, uint32_t len)
{
  uint32_t i, send_len = 0;
  uint8_t temp;
  uint32_t *checksum = (uint32_t*)data;

  if (len < 4)
    return;
  *checksum = crc16_calc((uint8_t*) (data + 4), len - 4, 0);


  usart_send_blocking(USART2, 0x7E);
  //esp.tx_buf[send_len++] = 0x7E;
  for (i = 0; i < len; i++)
  {
    temp = data[i];
    switch (temp)
    {
      case 0x7D:
      case 0x7E:
      case 0x7F:

        //esp.tx_buf[send_len++] = 0x7D;
        //esp.tx_buf[send_len++] = temp ^ 0x20;
        usart_send_blocking(USART2, 0x7D);
        usart_send_blocking(USART2, temp ^ 0x20);

        break;
      default:
        usart_send_blocking(USART2, temp);

        //esp.tx_buf[send_len++] = temp;
        break;
    }

  }

  //esp.tx_buf[send_len++] = 0x7F;
  //dma_write(send_len);
  usart_send_blocking(USART2, 0x7F);

}


void esp_manage(void)
{
  uint8_t buf_data;

  if (tick_expire_ms(&esp.tick, ESP_REPORT_INTERVAL))
  {
    double vref = analog_get(CH_INTERVAL_VREF_IDX);
    esp.dev_status.device_utc_time = rtc_get_counter_val();
    esp.dev_status.sensor[SENSOR_EC_IDX].sensor_status = 0;
    esp.dev_status.sensor[SENSOR_EC_IDX].sensor_value = (double)analog_get(CH_EC_SENSOR_IDX);

    double water_level_high = (analog_get(CH_WATER_LEVEL_HIGH_IDX) * 1.2 / vref) * 100;
    double water_level_middle = (analog_get(CH_WATER_LEVEL_MID_IDX) * 1.2 / vref) * 100;
    double water_level_low = (analog_get(CH_WATER_LEVEL_LOW_IDX) * 1.2 / vref) * 100;
    double water_level_percent = 0;
    if (water_level_low > WATER_LEVEL_THRESHOLD)
      water_level_percent += 20;
    if (water_level_middle > WATER_LEVEL_THRESHOLD)
      water_level_percent += 30;
    if (water_level_high > WATER_LEVEL_THRESHOLD)
      water_level_percent += 40;

    esp.dev_status.sensor[SENSOR_WATER_LEVEL_IDX].sensor_status = 0;
    esp.dev_status.sensor[SENSOR_WATER_LEVEL_IDX].sensor_value = water_level_percent;

    light_dev *light = light_get();


    esp.dev_status.sensor[SENSOR_LIGHT_IDX].sensor_status = light->valid;
    esp.dev_status.sensor[SENSOR_LIGHT_IDX].sensor_value = light->last_lux;


    dht_device *dht = dht_get();

    esp.dev_status.sensor[SENSOR_DHT_TEMP_IDX].sensor_status = dht->valid;
    esp.dev_status.sensor[SENSOR_DHT_TEMP_IDX].sensor_value = dht->last_temp;

    esp.dev_status.sensor[SENSOR_DHT_HUMIDITY_IDX].sensor_status = dht->valid;
    esp.dev_status.sensor[SENSOR_DHT_HUMIDITY_IDX].sensor_value = dht->last_humidity;

    ds18b20 *ds = ds_get();;

    esp.dev_status.sensor[SENSOR_WATER_TEMP_IDX].sensor_status = ds->valid;
    esp.dev_status.sensor[SENSOR_WATER_TEMP_IDX].sensor_value = ds->last_temp;


    esp.dev_status.sensor[SENSOR_DOSING_PUMP1_IDX].sensor_status = 0;
    esp.dev_status.sensor[SENSOR_DOSING_PUMP1_IDX].sensor_value = analog_get(CH_DOSING_PUMP1_IDX);

    esp.dev_status.sensor[SENSOR_DOSING_PUMP2_IDX].sensor_status = 0;
    esp.dev_status.sensor[SENSOR_DOSING_PUMP2_IDX].sensor_value = analog_get(CH_DOSING_PUMP2_IDX);

    esp.dev_status.sensor[SENSOR_REFLUX_PUMP_IDX].sensor_status = 0;
    esp.dev_status.sensor[SENSOR_REFLUX_PUMP_IDX].sensor_value = analog_get(CH_REFLUX_PUMP_IDX);

    esp.dev_status.sensor[SENSOR_VALVE_INPUT_IDX].sensor_status = 0;
    esp.dev_status.sensor[SENSOR_VALVE_INPUT_IDX].sensor_value = analog_get(CH_VALVE_INPUT_IDX);

    esp.dev_status.sensor[SENSOR_VALVE_OUTPUT_IDX].sensor_status = 0;
    esp.dev_status.sensor[SENSOR_VALVE_OUTPUT_IDX].sensor_value = analog_get(CH_VALVE_OUTPUT_IDX);

    esp.dev_status.sensor[SENSOR_PHOTOSYNTHERSIS_IDX].sensor_status = 0;
    esp.dev_status.sensor[SENSOR_PHOTOSYNTHERSIS_IDX].sensor_value = analog_get(CH_REFLUX_PUMP_IDX);

    esp.dev_status.sensor[SENSOR_PRESSURE_IDX].sensor_status = 0;
    esp.dev_status.sensor[SENSOR_PRESSURE_IDX].sensor_value = analog_get(CH_PRESSURE_PUMP_IDX);


    INFO("MCU: Sending data to ESP...%d\r\n", sizeof(device_status));
    if (esp.prog_mode == 0)
      esp_write((uint8_t*)&esp.dev_status, sizeof(device_status));

  }

  if (ringbuf_get(&esp.rb, &buf_data))
  {
    cdcacm_input(buf_data);
    proto_parse_byte(&esp.parser, buf_data);
  }
}
