/*
 * Stream.h
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#ifndef DCCOMMS_STREAM_H_
#define DCCOMMS_STREAM_H_

#include <cstdint>
#include <memory>
#include <string>

namespace dccomms {

class Stream;
typedef std::shared_ptr<Stream> StreamPtr;
class Stream {
public:
  Stream();
  virtual ~Stream();

  virtual int Read(void *, uint32_t, unsigned long msTimeout = 0) = 0;
  virtual int Write(const void *, uint32_t, uint32_t msTimeout = 0) = 0;

  virtual void ReadUint8(uint8_t &);
  virtual void ReadChar(char &);
  virtual void ReadUint16(uint16_t &);
  virtual void ReadUint32(uint32_t &);
  virtual void WriteUint8(uint8_t);

  virtual void WriteCString(const char *str);
  virtual void WriteString(const std::string &str);

  friend Stream &operator>>(Stream &, uint8_t &);
  friend Stream &operator>>(Stream &, char &);
  friend Stream &operator>>(Stream &, uint16_t &);
  friend Stream &operator>>(Stream &, uint32_t &);
  friend Stream &operator<<(Stream &, uint8_t);

  friend Stream &operator<<(Stream &, const char *str);
  friend Stream &operator<<(Stream &, const std::string &str);

  friend StreamPtr operator>>(StreamPtr , uint8_t &);
  friend StreamPtr operator>>(StreamPtr , char &);
  friend StreamPtr operator>>(StreamPtr , uint16_t &);
  friend StreamPtr operator>>(StreamPtr , uint32_t &);
  friend StreamPtr operator<<(StreamPtr , uint8_t);

  friend StreamPtr operator<<(StreamPtr , const char *str);
  friend StreamPtr operator<<(StreamPtr , const std::string &str);

  virtual int Available() = 0;

  virtual bool IsOpen() = 0;
  // virtual void TimeoutMode(bool) = 0;

  virtual void FlushInput() = 0;
  virtual void FlushOutput() = 0;
  virtual void FlushIO() = 0;

  void WaitFor(const uint8_t *expected, uint32_t size);
  int ReadInt(int &num, char &nextByte);
  int ReadUInt(int &num, char &nextByte);

  int ReadUntil(uint8_t *dst, const uint8_t *finalPattern,
                int finalPatternLength, int maxLength);

private:
  char buffer[1024];
};

} /* namespace radiotransmission */

#endif /* DCCOMMS_STREAM_H_ */
