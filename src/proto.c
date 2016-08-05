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
 * File: proto.c
 * Created: 1:59:54 PM Nov 17, 2015 GMT+7
 */
#include <stdint.h>
#include <stdlib.h>
#include "proto.h"


int8_t proto_init(proto_parser *parser, proto_parse_callback *cb, uint8_t *buf, uint32_t buf_size)
{
  parser->buf = buf;
  parser->buf_size = buf_size;
  parser->data_len = 0;
  parser->callback = cb;
  parser->is_esc = 0;
  return 0;
}
int8_t proto_parse_byte(proto_parser *parser, uint8_t value)
{
  switch (value)
  {
    case 0x7D:
      parser->is_esc = 1;
      break;

    case 0x7E:
      parser->data_len = 0;
      parser->is_esc = 0;
      break;

    case 0x7F:
      if (parser->callback != NULL)
        parser->callback(parser);
      break;

    default:
      if (parser->is_esc)
      {
        parser->buf[parser->data_len++] = value ^ 0x20;
        parser->is_esc = 0;
      }
      else
        parser->buf[parser->data_len++] = value;
      if (parser->data_len >= parser->buf_size)parser->data_len = 0;
      break;
  }
  return 0;
}
