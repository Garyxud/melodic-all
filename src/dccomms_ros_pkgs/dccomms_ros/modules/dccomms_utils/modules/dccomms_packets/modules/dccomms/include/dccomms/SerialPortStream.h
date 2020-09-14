/*
 * SerialPortInterface.h
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#ifndef DCCOMMS_SERIALPORTINTERFACE_H_
#define DCCOMMS_SERIALPORTINTERFACE_H_

#include <dccomms/StreamCommsDevice.h>
#include <stdio.h> /* Standard input/output definitions */

#include <string>

#include <errno.h>   /* Error number definitions */
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <sys/ioctl.h>
#include <sys/time.h> /*para timeout*/

namespace dccomms {

class SerialPortStream : public StreamCommsDevice {
public:
  enum BaudRate {
    BAUD_50 = B50,
    BAUD_75 = B75,
    BAUD_110 = B110,
    BAUD_134 = B134,
    BAUD_150 = B150,
    BAUD_200 = B200,
    BAUD_300 = B300,
    BAUD_600 = B600,
    BAUD_1200 = B1200,
    BAUD_1800 = B1800,
    BAUD_2400 = B2400,
    BAUD_4800 = B4800,
    BAUD_9600 = B9600,
    BAUD_19200 = B19200,
    BAUD_38400 = B38400,
    BAUD_57600 = B57600,
    BAUD_115200 = B115200
  };

  static BaudRate BaudRateFromUInt(const uint32_t baudrate) {
    BaudRate baud;
    switch (baudrate) {
    case 50:
      baud = BAUD_50;
      break;
    case 75:
      baud = BAUD_75;
      break;
    case 110:
      baud = BAUD_110;
      break;
    case 134:
      baud = BAUD_134;
      break;
    case 150:
      baud = BAUD_150;
      break;
    case 200:
      baud = BAUD_200;
      break;
    case 300:
      baud = BAUD_300;
      break;
    case 600:
      baud = BAUD_600;
      break;
    case 1200:
      baud = BAUD_1200;
      break;
    case 1800:
      baud = BAUD_1800;
      break;
    case 2400:
      baud = BAUD_2400;
      break;
    case 4800:
      baud = BAUD_4800;
      break;
    case 9600:
      baud = BAUD_9600;
      break;
    case 19200:
      baud = BAUD_19200;
      break;
    case 38400:
      baud = BAUD_38400;
      break;
    case 57600:
      baud = BAUD_57600;
      break;
    case 115200:
      baud = BAUD_115200;
      break;
    default:
      baud = BAUD_9600;
      break;
    }
    return baud;
  }
  enum Parity { EVEN, ODD, NOPARITY };

  enum StopBits { SB2, SB1 };

  enum CharacterSize { CHAR5 = CS5, CHAR6 = CS6, CHAR7 = CS7, CHAR8 = CS8 };

  struct PortSettings {
    BaudRate baudrate;
    Parity parity;
    StopBits stopBits;
    CharacterSize dataBits;

    PortSettings() {
      // Arduino default configuration
      baudrate = SerialPortStream::BAUD_9600;
      parity = SerialPortStream::NOPARITY;
      stopBits = SerialPortStream::SB1;
      dataBits = SerialPortStream::CHAR8;
    }
  };

  SerialPortStream();
  SerialPortStream(const std::string &);
  SerialPortStream(const std::string &, const uint32_t &baudrate);
  SerialPortStream(const std::string &, SerialPortStream::BaudRate);
  SerialPortStream(const std::string &, SerialPortStream::PortSettings);

  virtual void SetHwFlowControl(bool v);

  bool Open();
  bool Open(const std::string &p, SerialPortStream::BaudRate);
  bool Open(const std::string &p, SerialPortStream::PortSettings);
  void Close();

  int Read(void *, uint32_t, unsigned long msTimeout = 0);
  int Write(const void *, uint32_t, uint32_t msTimeout = 0);

  void ReadUint8(uint8_t &);
  void ReadChar(char &);
  void ReadUint16(uint16_t &);
  void ReadUint32(uint32_t &);
  int Available();

  bool IsOpen();
  // void TimeoutMode(bool);
  void FlushInput();
  void FlushOutput();
  void FlushIO();
  virtual bool BusyTransmitting();

  void SetTimeout(unsigned long ms);

protected:
  bool Connected();
  bool Ready();
  PortSettings portSettings;
  int fd;
  std::string port;
  bool isOpen = false;
  bool hwFlowControl = false;
};

} /* namespace radiotransmission */

#endif /* DCCOMMS_SERIALPORTINTERFACE_H_ */
