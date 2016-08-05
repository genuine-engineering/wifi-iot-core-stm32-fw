#include <stdint.h>
#include "crc16.h"
/* CITT CRC16 polynomial ^16 + ^12 + ^5 + 1 */
/*---------------------------------------------------------------------------*/
uint16_t crc16_add(uint8_t b, uint16_t acc)
{

  acc ^= b;
  acc  = (acc >> 8) | (acc << 8);
  acc ^= (acc & 0xff00) << 4;
  acc ^= (acc >> 8) >> 4;
  acc ^= (acc & 0xff00) >> 5;
  return acc;
}
/*---------------------------------------------------------------------------*/
uint16_t crc16_calc(uint8_t *data, uint32_t datalen, uint16_t acc)
{
  uint32_t i;

  for (i = 0; i < datalen; ++i)
  {
    acc = crc16_add(*data, acc);
    ++data;
  }
  return acc;
}

