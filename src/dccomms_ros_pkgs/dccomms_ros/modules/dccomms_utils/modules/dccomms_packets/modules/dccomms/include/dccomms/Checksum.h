/*
 * Checksum.h
 *
 *  Created on: Feb 11, 2015
 *      Author: diego
 */

#ifndef DCCOMMS_CHECKSUM_H_
#define DCCOMMS_CHECKSUM_H_

#include <cstdint>
#include <cstdio>

namespace dccomms {

class Checksum {
public:
  Checksum();
  virtual ~Checksum();

  static uint16_t crc16(const void *buf, size_t size);
  static uint16_t crc16(const void *buf, size_t size, uint16_t crc);

  static uint32_t crc32_2dfd2d88(const void *buf, size_t size);
  static uint32_t crc32_2dfd2d88(const void *buf, size_t size, uint32_t crc);

  static uint32_t crc32_cbf43926(const void *buf, size_t size);
  static uint32_t crc32_cbf43926(const void *buf, size_t size, uint32_t crc);

  static uint32_t crc32(const void *buf, size_t size);
  static uint32_t crc32(const void *buf, size_t size, uint32_t crc);

private:
  static uint16_t crc_xmodem_update(uint16_t crc, uint8_t data);
  static uint32_t crc32_tab[258];
  static uint32_t crc32_tab_2[258];
};

} /* namespace radiotransmission */

#endif /* DCCOMMS_CHECKSUM_H_ */
