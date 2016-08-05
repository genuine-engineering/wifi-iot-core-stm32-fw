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
 * File: light.h
 * Created: 3:00:48 PM Nov 25, 2015 GMT+7
 */
#ifndef SRC_LIGHT_H_
#define SRC_LIGHT_H_
typedef struct
{
	uint32_t tick;
	uint32_t timeout_tick;
	int32_t valid;
	uint32_t last_lux;
}light_dev;
void light_init(void);
void light_manage(void);
light_dev *light_get(void);

#endif /* SRC_LIGHT_H_ */
