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
 * File: analog.h
 * Created: 5:31:16 PM Nov 18, 2015 GMT+7
 */
#ifndef SRC_INCLUDE_ANALOG_H_
#define SRC_INCLUDE_ANALOG_H_

#define SENSOR_PWM				2000
#define SENSOR_SAMPLE			20
#define SENSOR_TICK				((1000000/SENSOR_PWM)/SENSOR_SAMPLE)


#define ADC_SAMPLE 				SENSOR_SAMPLE
#define ADC_SAMPLE_MEAN		10
#define ADC_CHANNEL				13

#define	CH_DOSING_PUMP1_IDX			0
#define CH_DOSING_PUMP2_IDX					1
#define CH_REFLUX_PUMP_IDX					2
#define CH_VALVE_INPUT_IDX							3
#define CH_VALVE_OUTPUT_IDX						4
#define CH_LED_PHOTOSYNT_IDX				5
#define CH_PRESSURE_PUMP_IDX				6
#define CH_WATER_LEVEL_HIGH_IDX			7
#define CH_WATER_LEVEL_MID_IDX			8
#define CH_WATER_LEVEL_LOW_IDX			9
#define CH_EC_SENSOR_IDX						10
#define CH_INTERNAL_TEMP_IDX					11
#define CH_INTERVAL_VREF_IDX						12

#define CH_DOSING_PUMP1					15
#define CH_DOSING_PUMP2					9
#define CH_REFLUX_PUMP					14
#define CH_VALVE_IN							7
#define CH_VALVE_OUT						8
#define CH_LED_PHOTOSYNT				16
#define CH_PRESSURE_PUMP				5
#define CH_WATER_LEVEL_HIGH			0
#define CH_WATER_LEVEL_MID			1
#define CH_WATER_LEVEL_LOW			10
#define CH_EC_SENSOR						4
#define CH_INTERNAL_TEMP						16
#define CH_INTERVAL_VREF						17

void analog_init(void);
void analog_manage(void);
void analog_stop(void);
void analog_resume(void);
double analog_get(uint8_t index);

#endif /* SRC_INCLUDE_ANALOG_H_ */
