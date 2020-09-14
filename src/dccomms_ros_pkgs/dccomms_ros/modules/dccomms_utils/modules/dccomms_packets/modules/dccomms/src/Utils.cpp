/*
 * Utils.cpp
 *
 *  Created on: 05/03/2015
 *      Author: diego
 */

#include <cstdint>
#include <dccomms/Utils.h>
#include <openssl/md5.h>
#include <thread>

namespace dccomms {

Utils::Utils() {
  // TODO Auto-generated constructor stub
}

Utils::~Utils() {
  // TODO Auto-generated destructor stub
}

bool Utils::IsBigEndian() {
  // https://es.wikipedia.org/wiki/Endianness
  uint32_t word = 0x1;
  uint8_t *byte = (uint8_t *)&word;
  return *byte != 0x1;
}

void Utils::Switch4Bytes(void *dst, void *src) {
  uint8_t *buf = (uint8_t *)dst;
  *buf = *((uint8_t *)src + 3);
  *(buf + 1) = *((uint8_t *)src + 2);
  *(buf + 2) = *((uint8_t *)src + 1);
  *(buf + 3) = *((uint8_t *)src);
}

void Utils::Switch8Bytes(void *dst, void *src) {
  uint8_t *buf = (uint8_t *)dst;
  *buf = *((uint8_t *)src + 7);
  *(buf + 1) = *((uint8_t *)src + 6);
  *(buf + 2) = *((uint8_t *)src + 5);
  *(buf + 3) = *((uint8_t *)src + 4);
  *(buf + 4) = *((uint8_t *)src + 3);
  *(buf + 5) = *((uint8_t *)src + 2);
  *(buf + 6) = *((uint8_t *)src + 1);
  *(buf + 7) = *((uint8_t *)src);
}

void Utils::IntSwitchEndian(void *b, uint32_t entero) {
  uint8_t *buf = (uint8_t *)b;
  *buf = (uint8_t)((entero >> 24) & 0xff);
  *(buf + 1) = (uint8_t)((entero >> 16) & 0xff);
  *(buf + 2) = (uint8_t)((entero >> 8) & 0xff);
  *(buf + 3) = (uint8_t)(entero & 0xff);
}

void Utils::IntSwitchEndian(void *b, uint16_t integer) {
  *(uint8_t *)b = (uint8_t)(integer >> 8);
  *(((uint8_t *)b) + 1) = (uint8_t)(integer & 0xff);
}
/*
void Utils::SaveInt16AsBigEndian(void * b, uint16_t integer)
{
      if(IsBigEndian())
      {
              *(uint16_t*) b = integer;
      }
      else
      {
              Int16SwitchEndian(b, integer);
      }
}
*/

std::string Utils::BuildString(std::initializer_list<std::string> list) {
  std::string res = "";
  for (auto elem : list) {
    res += elem;
  }
  return res;
}

void Utils::Sleep(int millis) {
  std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

void Utils::md5(void *data, unsigned int length, void *md5) {
  MD5((uint8_t *)data, length, (uint8_t *)md5);
}

void Utils::Debug(std::ostream &o, std::string &msg) {
#ifdef DEBUG
  o << msg << std::endl;
#endif
}

} /* namespace radiotransmission */
