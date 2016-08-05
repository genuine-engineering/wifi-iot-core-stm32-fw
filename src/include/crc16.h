
#ifndef _CRC16_H_
#define _CRC16_H_

uint16_t crc16_add(uint8_t b, uint16_t acc);
uint16_t crc16_calc(uint8_t *data, uint32_t datalen, uint16_t acc);

#endif /* _CRC16_H_ */
