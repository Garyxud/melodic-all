/*
 * SerialPortInterface.cpp
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#include <dccomms/CommsException.h>
#include <string>

#include <cstring>   /* String function definitions */
#include <errno.h>   /* Error number definitions */
#include <fcntl.h>   /* File control definitions */
#include <stdio.h>   /* Standard input/output definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>  /* UNIX standard function definitions */

#include <dccomms/SerialPortStream.h>
#include <sys/ioctl.h>
#include <sys/time.h> /*para timeout*/

#include <iostream>

/* According to POSIX.1-2001, POSIX.1-2008 */
#include <sys/select.h>

/* According to earlier standards */
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

namespace dccomms {

SerialPortStream::SerialPortStream() {}
SerialPortStream::SerialPortStream(const std::string &p) { port = p; }

SerialPortStream::SerialPortStream(const std::string &p,
                                   SerialPortStream::BaudRate baud) {
  port = p;
  portSettings.baudrate = baud;
}

SerialPortStream::SerialPortStream(const std::string &p, const uint32_t &baud) {
  port = p;
  portSettings.baudrate = BaudRateFromUInt(baud);
}

SerialPortStream::SerialPortStream(const std::string &p,
                                   SerialPortStream::PortSettings ps) {
  port = p;
  portSettings = ps;
}

bool SerialPortStream::BusyTransmitting() { return false; }

bool SerialPortStream::Open(const std::string &p,
                            SerialPortStream::BaudRate baud) {
  port = p;
  portSettings.baudrate = baud;
  return Open();
}

bool SerialPortStream::Open(const std::string &p,
                            SerialPortStream::PortSettings ps) {
  port = p;
  portSettings = ps;
  return Open();
}

bool SerialPortStream::Open() {
  struct termios options;

  fd = open(port.c_str(), O_RDWR);
  if (fd != -1) {
    fcntl(fd, F_SETFL, FNDELAY);
    SetTimeout(_timeout);
    /*
               * Get the current options for the port...
               */
    tcgetattr(fd, &options);
    /*
               * Set the baud rates to 19200...
               */
    cfsetispeed(&options, portSettings.baudrate);
    cfsetospeed(&options, portSettings.baudrate);
    /*
               * Enable the receiver and set local mode...
               */
    options.c_cflag |= (CLOCAL | CREAD);
    /*
               * Set the new options for the port...
               */
    switch (portSettings.parity) {
    case NOPARITY:
      options.c_cflag &= ~PARENB;
      break;
    case EVEN:
      options.c_cflag |= PARENB;
      options.c_cflag &= ~PARODD;
      break;
    case ODD:
      options.c_cflag |= PARENB;
      options.c_cflag |= PARODD;
      break;
    }
    switch (portSettings.stopBits) {
    case SB2:
      options.c_cflag |= CSTOPB;
      break;
    case SB1:
      options.c_cflag &= ~CSTOPB;
      break;
    }

    options.c_cflag &= ~CSIZE;
    options.c_cflag |= portSettings.dataBits;

    // set hardware flow control:
    if (hwFlowControl)
      options.c_cflag |= CRTSCTS;
    else
      options.c_cflag &= ~CRTSCTS;

    // disable software flow controls:
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    //...
    options.c_iflag &= ~ICRNL;

    // Choosing Raw Input
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Choosing Raw Output
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSAFLUSH, &options);
    isOpen = true;

    SetTimeout(0);
    return true;
  }
  isOpen = false;
  throw CommsException("Error trying to connect with the serial port",
                       COMMS_EXCEPTION_PHYLAYER_ERROR);
  return false;
}

void SerialPortStream::SetHwFlowControl(bool v) {
  hwFlowControl = v;
  if (isOpen) {
    struct termios options;
    tcgetattr(fd, &options);
    if (hwFlowControl)
      options.c_cflag |= CRTSCTS;
    else
      options.c_cflag &= ~CRTSCTS;
    tcsetattr(fd, TCSAFLUSH, &options);
  }
}

void SerialPortStream::Close() {

  close(fd);
  isOpen = false;
}

bool SerialPortStream::Connected() {
  // http://stackoverflow.com/questions/34170350/detecting-if-a-character-device-has-disconnected-in-linux-in-with-termios-api-c
  bool connected =
      !(Ready() &&
        Available() == 0); // But does not work with the virtual serial port...
  return connected;
}

bool SerialPortStream::Ready() {
  fd_set fds;
  struct timeval t;
  t.tv_sec = 0;
  t.tv_usec = 0;

  FD_ZERO(&fds);
  FD_SET(fd, &fds);
  int r = select(fd + 1, &fds, NULL, NULL, &t);
  if (-1 == r) {
    throw CommsException("Error when reading from descriptor",
                         COMMS_EXCEPTION_LINEDOWN);
    return 0;
  }
  return r;
}

int SerialPortStream::Read(void *buf, uint32_t size, unsigned long ms) {
  struct timeval time0, time1;
  gettimeofday(&time0, NULL);
  unsigned long long t0 = time0.tv_sec * 1000 + time0.tv_usec / 1000;
  unsigned long long t1 = t0;

  unsigned char *ptr = (uint8_t *)buf;
  unsigned char *max = (uint8_t *)buf + size;

  int n = 0;
  int bytesLeft = size - n;

  unsigned long m = ms ? ms : _timeout;

  if (m == 0) {
    // Bloqueado hasta coger m bytes
    while (true) {
      int res = read(fd, ptr, bytesLeft);
      if (res > 0) {
        n += res;
        ptr = (uint8_t *)buf + n;
        if (ptr == max)
          return n; // == size
        bytesLeft = size - n;
      } else if (!Connected()) // TODO: CHECK THE CONNECTION STATUS AND RAISE
                               // EXCEPTION IF DOWN
      {
        throw CommsException("Problem happened when reading socket",
                             COMMS_EXCEPTION_LINEDOWN);
      }
    }
  }
  while (t1 - t0 < m) {
    if (Available() > 0) {
      n += read(fd, ptr, bytesLeft);
      ptr = (uint8_t *)buf + n;
      if (ptr == max)
        return n; // == size
      bytesLeft = size - n;
    }
    gettimeofday(&time1, NULL);
    t1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
  }

  // Si se llega hasta este punto, es que ha transcurrido el timeout
  char sig = '-'; // Un byte aleatorio...
  int res = write(fd, &sig, 1);
  if (res < 0) {
    close(fd);
    throw CommsException("Fallo de comunicacion al leer",
                         COMMS_EXCEPTION_LINEDOWN);
  }

  throw CommsException("Read Timeout", COMMS_EXCEPTION_TIMEOUT);
}

void SerialPortStream::SetTimeout(unsigned long ms) {

  _timeout = ms >= 0 ? ms : 0;
  if (ms)
    fcntl(fd, F_SETFL, FNDELAY);
  else
    fcntl(fd, F_SETFL, 0);
}

void SerialPortStream::FlushInput() { tcflush(fd, TCIFLUSH); }

void SerialPortStream::FlushIO() { tcflush(fd, TCIOFLUSH); }

void SerialPortStream::FlushOutput() { tcflush(fd, TCOFLUSH); }
int SerialPortStream::Write(const void *buf, uint32_t size, uint32_t to) {

#ifdef BADWRITE
  int w = 0;
  for (int i = 0; i < size; i++) {
    w = write(fd, ((const uint8_t *)buf) + i, 1);
    if (w < 0) {
      close(fd);
      throw CommsException("Fallo de comunicacion al escribir",
                           COMMS_EXCEPTION_LINEDOWN);
    }
  }
  return w;
#elif BADWRITE2
  int w = 0;
  int increment = BADWRITE2;
  int left = size % increment;
  std::cerr << "Size: " << size << std::endl;
  std::cerr << "Left: " << left << std::endl;
  std::cerr << "Increment: " << increment << std::endl;
  int its = size / increment;
  std::cerr << "Its: " << its << std::endl;
  uint8_t *ptr = (uint8_t *)buf;
  for (int i = 0; i < its; i++) {
    w = write(fd, ptr, increment);

    ptr += increment;
    if (w < 0) {
      close(fd);
      throw CommsException("Fallo de comunicacion al escribir",
                           COMMS_EXCEPTION_LINEDOWN);
    }
  }
  if (left) {
    w = write(fd, ptr, left);

    if (w < 0) {
      close(fd);
      throw CommsException("Fallo de comunicacion al escribir",
                           COMMS_EXCEPTION_LINEDOWN);
    }
  }
  return w;

#elif GOODWRITE
  int tbw = 0, bw;
  for (int i = 0; i < size; i += bw) {
    bw = write(fd, ((const uint8_t *)buf) + tbw, size - tbw);
    std::cerr << "Bytes written: " << bw << std::endl;
    tbw += bw;
    if (bw < 0) {
      close(fd);
      throw CommsException("Fallo de comunicacion al escribir",
                           COMMS_EXCEPTION_LINEDOWN);
    }
  }
  return tbw;
#else
  int w = write(fd, (const uint8_t *)buf, size);
  if (w < 0) {
    close(fd);
    throw CommsException("Fallo de comunicacion al escribir",
                         COMMS_EXCEPTION_LINEDOWN);
  }

  return w;
#endif
}

int SerialPortStream::Available() {
  int n = 0;
  if (ioctl(fd, FIONREAD, &n) < 0)
    throw CommsException("Some error happened when trying to read",
                         COMMS_EXCEPTION_LINEDOWN);
  return n;
}

bool SerialPortStream::IsOpen() { return isOpen; }

void SerialPortStream::ReadUint8(uint8_t &byte) {
  fcntl(fd, F_SETFL, 0);
  read(fd, &byte, sizeof(uint8_t));
  fcntl(fd, F_SETFL, FNDELAY);
}

void SerialPortStream::ReadChar(char &byte) {
  fcntl(fd, F_SETFL, 0);
  read(fd, &byte, sizeof(uint8_t));
  fcntl(fd, F_SETFL, FNDELAY);
}

void SerialPortStream::ReadUint16(uint16_t &data16) {
  fcntl(fd, F_SETFL, 0);
  read(fd, &data16, sizeof(uint16_t));
  fcntl(fd, F_SETFL, FNDELAY);
}

void SerialPortStream::ReadUint32(uint32_t &data32) {
  fcntl(fd, F_SETFL, 0);
  read(fd, &data32, sizeof(uint32_t));
  fcntl(fd, F_SETFL, FNDELAY);
}

} /* namespace radiotransmission */
