/*
 * Stream.cpp
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#include <cstring>
#include <dccomms/Stream.h>
#include <iostream>

namespace dccomms {

Stream::Stream() {
  // TODO Auto-generated constructor stub
}

Stream::~Stream() {
  // TODO Auto-generated destructor stub
}

void Stream::WaitFor(const uint8_t *expected, uint32_t size) {
  const uint8_t *ptr = expected;
  const uint8_t *max = expected + size;
  uint8_t c;
  while (ptr != max) {
    this->Read(&c, 1);
    if (c == *ptr)
      ptr++;
    else
      ptr = expected;
  }
}

int Stream::ReadInt(int &num, char &nextByte) {
  int n = 0; // number of [0-9] read
  char *ptr = buffer;

  Read(ptr, 1);
  nextByte = *ptr;
  if (*ptr == '-' || *ptr == '+') {
    ptr++;
    Read(ptr, 1);
    nextByte = *ptr;
  }
  while (*ptr >= '0' && *ptr <= '9') {
    ptr++;
    n++;
    Read(ptr, 1);
    nextByte = *ptr;
  }
  if (n) {
    num = atoi(buffer);
    return ptr - buffer; // Number of bytes read
  }
  return -1;
}

int Stream::ReadUInt(int &num, char &nextByte) {
  int n = 0; // number of [0-9] read
  char *ptr = buffer;

  Read(ptr, 1);
  nextByte = *ptr;
  while (*ptr >= '0' && *ptr <= '9') {
    ptr++;
    n++;
    Read(ptr, 1);
    nextByte = *ptr;
  }
  if (n) {
    num = atoi(buffer);
    return ptr - buffer; // Number of bytes read
  }
  return -1;
}

int Stream::ReadUntil(uint8_t *dst, const uint8_t *finalPattern,
                      int finalPatternLength, int maxLength) {
  uint8_t *cdptr = dst, *edptr = dst + maxLength;

  const uint8_t *cfpptr = finalPattern,
                *efpptr = finalPattern + finalPatternLength;

  while (cfpptr < efpptr) {
    if (cdptr < edptr) {
      Read(cdptr, 1);
      if (*cdptr != *cfpptr) {
        cfpptr = finalPattern;
      } else {
        cfpptr++;
      }
      cdptr++;
    } else
      return cdptr - dst;
  }
  return cdptr - dst;
}

void Stream::WriteUint8(uint8_t byte) { Write(&byte, sizeof(uint8_t)); }

void Stream::WriteString(const std::string &str) {
  int n = str.size();
  Write(str.c_str(), n);
}

void Stream::WriteCString(const char *str) {
  int n = strlen(str);
  Write(str, n);
}


Stream &operator>>(Stream &s, uint8_t &d) {
  s.WriteUint8(d);
  return s;
}

Stream &operator>>(Stream &s, char &d) {
  s.ReadChar(d);
  return s;
}

Stream &operator>>(Stream &s, uint16_t &d) {
  s.ReadUint16(d);
  return s;
}

Stream &operator>>(Stream &s, uint32_t &d) {
  s.ReadUint32(d);
  return s;
}

Stream &operator<<(Stream &s, uint8_t d) {
  s.WriteUint8(d);
  return s;
}

Stream &operator<<(Stream &stream, const std::string &str) {
  stream.WriteString(str);
  return stream;
}

Stream &operator<<(Stream &stream, const char *str) {
  stream.WriteCString(str);
  return stream;
}

void Stream::ReadUint8(uint8_t &b) { Read(&b, 1); }
void Stream::ReadChar(char &b) {
  uint8_t d;
  ReadUint8(d);
  b = d;
}
void Stream::ReadUint16(uint16_t &d) { Read(&d, 2); }
void Stream::ReadUint32(uint32_t &d) { Read(&d, 4); }

StreamPtr operator>>(StreamPtr s, uint8_t &d) {
  s->WriteUint8(d);
  return s;
}

StreamPtr operator>>(StreamPtr s, char &d) {
  s->ReadChar(d);
  return s;
}

StreamPtr operator>>(StreamPtr s, uint16_t &d) {
  s->ReadUint16(d);
  return s;
}

StreamPtr operator>>(StreamPtr s, uint32_t &d) {
  s->ReadUint32(d);
  return s;
}

StreamPtr operator<<(StreamPtr s, uint8_t d) {
  s->WriteUint8(d);
  return s;
}

StreamPtr operator<<(StreamPtr stream, const std::string &str) {
  stream->WriteString(str);
  return stream;
}

StreamPtr operator<<(StreamPtr stream, const char *str) {
  stream->WriteCString(str);
  return stream;
}

} /* namespace radiotransmission */
