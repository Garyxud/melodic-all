/*
 * SerialPortInterface.h
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#ifndef DCCOMMS_NAMEDPIPESTREAM_H_
#define DCCOMMS_NAMEDPIPESTREAM_H_

#include <stdio.h> /* Standard input/output definitions */

#include <string>

#include <dccomms/StreamCommsDevice.h>
#include <errno.h>   /* Error number definitions */
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <sys/ioctl.h>
#include <sys/time.h> /*para timeout*/

namespace dccomms {

#define DLS_INBUFFER_SIZE 10000
#define DLS_INBUFFER_SIZE_FLUSH 200000

class NamedPipeStream : public StreamCommsDevice {
public:
  struct PortSettings {
    std::string file;

    PortSettings() { file = "/tmp/radiorx"; }

    PortSettings(std::string f) { file = f; }
  };

  NamedPipeStream();
  NamedPipeStream(const char *);
  NamedPipeStream(NamedPipeStream::PortSettings);
  bool Open();
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

  int GetBufferSize();
  void SetBufferSize(int);

protected:
  PortSettings portSettings;
  int fd;
  bool _open = false;
  char *port;

private:
  int bufferSize = 0;
  char tmp[DLS_INBUFFER_SIZE_FLUSH];
};

} /* namespace radiotransmission */

#endif /* DCCOMMS_NAMEDPIPESTREAM_H_ */
