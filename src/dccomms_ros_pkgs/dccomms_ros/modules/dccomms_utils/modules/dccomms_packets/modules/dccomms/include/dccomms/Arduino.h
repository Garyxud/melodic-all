/*
 * Arduino.h
 *
 *  Created on: Feb 21, 2015
 *      Author: diego
 */

#ifndef DCCOMMS_ARDUINO_H_
#define DCCOMMS_ARDUINO_H_

#include <dccomms/SerialPortStream.h>

namespace dccomms {

class Arduino : public SerialPortStream {
public:
  enum BaudRate {
    BAUD_50 = SerialPortStream::BAUD_50,
    BAUD_75 = SerialPortStream::BAUD_75,
    BAUD_110 = SerialPortStream::BAUD_110,
    BAUD_134 = SerialPortStream::BAUD_134,
    BAUD_150 = SerialPortStream::BAUD_150,
    BAUD_200 = SerialPortStream::BAUD_200,
    BAUD_300 = SerialPortStream::BAUD_300,
    BAUD_600 = SerialPortStream::BAUD_600,
    BAUD_1200 = SerialPortStream::BAUD_1200,
    BAUD_1800 = SerialPortStream::BAUD_1800,
    BAUD_2400 = SerialPortStream::BAUD_2400,
    BAUD_4800 = SerialPortStream::BAUD_4800,
    BAUD_9600 = SerialPortStream::BAUD_9600,
    BAUD_19200 = SerialPortStream::BAUD_19200,
    BAUD_38400 = SerialPortStream::BAUD_38400,
    BAUD_57600 = SerialPortStream::BAUD_57600,
    BAUD_115200 = SerialPortStream::BAUD_115200
  };

  Arduino();
  Arduino(const char *p, SerialPortStream::BaudRate baud);
  Arduino(SerialPortStream s);
  Arduino(SerialPortStream, const char *port, Arduino::BaudRate,
          const char *hello, const char *validReply);
  bool TryReconnect();
  virtual ~Arduino();

  static Arduino FindArduino(Arduino::BaudRate, const char *hello,
                             const char *validReply);

  std::string hello;
  std::string validReply;
  Arduino::BaudRate baud;

private:
  static bool _checkDevice(Stream *s, const char *h, const char *r,
                           unsigned long long m = 0);
};

} /* namespace radiotransmission */

#endif /* DCCOMMS_ARDUINO_H_ */
