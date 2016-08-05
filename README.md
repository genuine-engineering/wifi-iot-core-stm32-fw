# Thông tin chung về dự án iot-core

Phần cứng IoT core kết hợp STM32 + ESP8266:

- Chip ESP8266 có kết nối Wifi tuy nhiên lại có nhiều hạn chế về phần cứng như thiết các giao tiếp ngoại vi và USB, thiếu module đọc Analog chính xác, thiếu IO ... Trong khi việc phát triển Ứng dụng trên ESP8266 bắt buộc cần giao tiếp USB-Serial, chính vì vậy nhiều module trên thế giới xuất hiện thêm con chip USB-TTL, ngoài giá trị cho develop thì ít có giá trị trong thực tế. Thiết kế này sẽ sử dụng STM32 giá rẻ, bổ sung giao tiếp USB, hỗ trợ cho develop, và có các module thiếu hụt của ESP8266.

- Dự án IoT core bao gồm các dự án con:
    + [iot-core-hw](https://github.com/genuine-engineering/iot-core-hw) Phần cứng vẽ bằng Kicad
    + [iot-core-stm32-fw](https://github.com/genuine-engineering/iot-core-stm32-fw) Firmware cho STM32 base trên [libopencm3](https://github.com/libopencm3/libopencm3)
    + [iot-core-esp8266-fw](https://github.com/genuine-engineering/iot-core-esp8266-fw) Firmware cho ESP8266 dựa trên SDK 2.0


# iot-core-stm32-fw

LƯU Ý: Dự án đang triển khai, và mã nguồn chưa sẵn sàng

### Packet format: SLIP Protocol

```c
[0x7E - byte1 - byte2 - byten - ... - 0x7F]

if byten = 0x7E or 0x7D or 0x7F then
	byten = [0x7D byten^0x20]
endif
```

see: `src/proto.c`

### CRC : CRC16

```c
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

  for(i = 0; i < datalen; ++i) {
    acc = crc16_add(*data, acc);
    ++data;
  }
  return acc;
}

```

## Distribution

Please install sublime editorconfig plugin and sublime AStyleFormater, see project settings and: http://astyle.sourceforge.net/astyle.html[http://astyle.sourceforge.net/astyle.html]
