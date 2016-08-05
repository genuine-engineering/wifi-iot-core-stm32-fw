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
 * File: esp.h
 * Created: 8:15:47 AM Nov 17, 2015 GMT+7
 */




#ifndef INCLUDE_ESP_H_
#define INCLUDE_ESP_H_

#include "ringbuf.h"
#include "proto.h"

#ifndef ESP_REPORT_INTERVAL
#define ESP_REPORT_INTERVAL 1000
#endif

#define ESP_PORT		GPIOC
#define ESP_RST			GPIO1
#define ESP_PD			GPIO2
#define ESP_PROG		GPIO3

typedef enum
{
	DEVICE_INVALID = 0,
	DEVICE_RUNNING,
	DEVICE_STOPING
} DEVICE_STATUS;




#define	PACKET_CTRL			0x01
#define PACKET_STATUS		0x81


#define MAX_PORT_OUTPUT					9

#define PORT_LED_STATUS_IDX			0
#define PORT_DOSING_PUMP1_IDX		1
#define PORT_DOSING_PUMP2_IDX		2
#define PORT_REFLUX_PUMP_IDX		3
#define PORT_PRESSURE_PUMP_IDX	4
#define PORT_VALVE_INPUT_IDX		5
#define PORT_VALVE_OUTPUT_IDX		6
#define PORT_LED_PHOTOSYNTHESIS_IDX	7
#define PORT_WATER_SENSOR_VOLT_IDX	8

/*
 * ALL OFF VALUE = 0
 * DIGITAL OUT ON VALUE = OTHER < 32768
 * PWM OUT VALUE = OTHER < 32768
 * NO CHANGE = -1
 * BLINK VALUE > 32768, 2bytes high value is blink on is milisec, 2bytes low value is blink off
 */
typedef struct  __attribute__((__packed__))
{
	uint32_t	checksum;	/* from esp_uid to end of data */
	uint32_t	packet_type;
	uint32_t	esp_uid;
	uint32_t	esp_utc_time; 			/* 0 (zero) if do not update MCU RTC timer*/
	uint32_t	esp_version; 				/* or build time*/
	uint32_t	extra_data_len;			/* for features, when upgrate more data */

	int32_t		led_status_value;
	int32_t		dosing_pump1_value; 	/* off = 0, on = other, no change = -1*/				//index 1
	int32_t		dosing_pump2_value; 	/* off = 0, on = other, no change = -1*/				//index 2
	int32_t		reflux_pump_value; 		/* off = 0, on = other, no change = -1*/				//index 3
	int32_t		pressure_pump_value; 	/* off = 0, on = other, no change = -1*/				//index 4
	int32_t		valve_input_value; 		/* off = 0, on = other, no change = -1*/				//index 5
	int32_t		valve_output_value; 	/* off = 0, on = other, no change = -1*/				//index 6
	int32_t		led_photosynthesis_value; 		/* off = 0, on pwm = other, no change = -1*///index 7
	int32_t 	water_sensor_volt_value;			/* off = 0; on = other; no change = -1, default on*/ //index 8
} device_control;

/* device status report
 * - 1 if invalid, 0 = valid
 */

#define MAX_SENSOR							13

#define SENSOR_EC_IDX						0
#define SENSOR_WATER_LEVEL_IDX	1
#define SENSOR_LIGHT_IDX 				2
#define SENSOR_DHT_TEMP_IDX 		3
#define SENSOR_DHT_HUMIDITY_IDX	4
#define SENSOR_WATER_TEMP_IDX		5
#define SENSOR_DOSING_PUMP1_IDX	6
#define SENSOR_DOSING_PUMP2_IDX	7
#define SENSOR_REFLUX_PUMP_IDX	8
#define SENSOR_VALVE_INPUT_IDX	9
#define SENSOR_VALVE_OUTPUT_IDX 10
#define SENSOR_PHOTOSYNTHERSIS_IDX 11
#define SENSOR_PRESSURE_IDX 12

typedef struct __attribute__((__packed__))
{
	float 		sensor_value;
	int32_t		sensor_status;
} sensor_property;

typedef struct  __attribute__((__packed__))
{
	uint32_t	checksum;	/* from device_uid to end of data */
	uint32_t	packet_type;
	uint32_t	device_uid;
	uint32_t	device_utc_time;
	uint32_t	device_version; /* or build time*/
	uint32_t	extra_data_len;	/* for features, when upgrate more data */

	sensor_property sensor[MAX_SENSOR];
} device_status;

typedef struct
{
	uint32_t queue_restart_time;
	uint32_t tick;
	uint32_t prog_mode;
	proto_parser parser;
	ringbuf rb;
	uint8_t proto_buf[256];
	uint8_t rb_buf[256];
	uint8_t tx_buf[512];
	device_control dev_ctrl;
	device_status dev_status;
}esp_dev;

void esp_init(void);
void esp_enter_prog(void);
void esp_restart(void);
void esp_restart_timeout(uint32_t time_ms);
void esp_manage(void);
uint32_t esp_check_sync_frame(uint8_t data);
uint32_t esp_check_end_frame(uint8_t data);
uint32_t esp_is_prog_mode(void);
void esp_clear_restart(void);
void esp_leave_prog(void);
void esp_write(uint8_t *data, uint32_t len);

#endif /* INCLUDE_ESP_H_ */
