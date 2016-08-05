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
 * File: proto.h
 * Created: 2:02:01 PM Nov 17, 2015 GMT+7
 */
#ifndef INCLUDE_PROTO_H_
#define INCLUDE_PROTO_H_

typedef void(proto_parse_callback)(void *parser);

typedef struct{
	uint8_t *buf;
	uint16_t buf_size;
	uint16_t data_len;
	uint8_t is_esc;
	proto_parse_callback* callback;
}proto_parser;

int8_t proto_init(proto_parser *parser, proto_parse_callback *cb, uint8_t *buf, uint32_t buf_size);
int8_t proto_parse_byte(proto_parser *parser, uint8_t value);



#endif /* INCLUDE_PROTO_H_ */
