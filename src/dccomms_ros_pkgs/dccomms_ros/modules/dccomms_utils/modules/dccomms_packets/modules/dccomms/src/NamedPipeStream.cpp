/*
 * DataLinkStream.cpp
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#include <dccomms/CommsException.h>
#include <string>

#include <cstring> /* String function definitions */
#include <dccomms/NamedPipeStream.h>
#include <errno.h>   /* Error number definitions */
#include <fcntl.h>   /* File control definitions */
#include <stdio.h>   /* Standard input/output definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>  /* UNIX standard function definitions */

#include <iostream>
#include <sys/ioctl.h>
#include <sys/time.h> /*para timeout*/

namespace dccomms {

NamedPipeStream::NamedPipeStream() {}
NamedPipeStream::NamedPipeStream(const char *p) {
  int s = strlen(p);
  port = new char[s + 1];
  strcpy(port, p);
}
NamedPipeStream::NamedPipeStream(NamedPipeStream::PortSettings ps) {
  int s = ps.file.size();
  port = new char[s + 1];
  strcpy(port, ps.file.c_str());
  portSettings = ps;
}

bool NamedPipeStream::BusyTransmitting() { return false; }

bool NamedPipeStream::Open() {
  // struct termios options;

  fd = open(port, O_RDWR);
  if (fd != -1) {
    SetBufferSize(DLS_INBUFFER_SIZE);
    bufferSize = GetBufferSize();
    return true;
  }
  _open = false;
  return false;
}

void NamedPipeStream::Close() {

  close(fd);
  _open = false;
}

int NamedPipeStream::Read(void *buf, uint32_t size, unsigned long ms) {
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
#ifndef SERIAL_DISCONNECT_TEST
      if (Available() > 0) {
        n += read(fd, ptr, bytesLeft);
        ptr = (uint8_t *)buf + n;
        if (ptr == max)
          return n; // == size
        bytesLeft = size - n;
      }
#else
      int res = Available();
      if (res > 0) {
        n += read(fd, ptr, bytesLeft);
        ptr = (uint8_t *)buf + n;
        if (ptr == max)
          return n; // == size
        bytesLeft = size - n;
      } else {
        char sig = '-'; // Un byte aleatorio...
        res = write(fd, &sig, 1);
        if (res < 0) {
          close(fd);
          throw CommsException("Fallo de comunicacion al leer",
                               COMMS_EXCEPTION_LINEDOWN);
        }
      }
#endif
    }
  }

#ifndef CHECK_TIMEOUTEXCEPTION
  while (t1 - t0 < m) {
    if (Available() > 0) {

#ifdef PRINTSERIAL
      int SZ = Available();
      std::cout << std::endl << SZ << std::endl;
#endif
      n += read(fd, ptr, bytesLeft);
      ptr = (uint8_t *)buf + n;
      if (ptr == max)
        return n; // == size
      bytesLeft = size - n;
    }
    gettimeofday(&time1, NULL);
    t1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
  }
#endif

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

void NamedPipeStream::SetTimeout(unsigned long ms) {
  _timeout = ms >= 0 ? ms : 0;
  if (ms)
    fcntl(fd, F_SETFL, FNDELAY);
  else
    fcntl(fd, F_SETFL, 0);
}

int NamedPipeStream::GetBufferSize() { return fcntl(fd, F_GETPIPE_SZ); }

void NamedPipeStream::SetBufferSize(int bs) {
  fcntl(fd, F_SETPIPE_SZ, bs);
  bufferSize = GetBufferSize();
}

void NamedPipeStream::FlushInput() {
  int n;
  n = read(fd, tmp, DLS_INBUFFER_SIZE_FLUSH);

  std::cerr << "N: " << n << " Buff. Size: " << bufferSize << std::endl;

  /*
      int flags=fcntl(fd, F_GETFL,0);
      int flags2 = flags;
      flags2 |= O_NONBLOCK;
      fcntl(fd, F_SETFL, flags2);
      */

  // while((n = read(fd, tmp, DLS_INBUFFER_SIZE_FLUSH))>0)
  //{
  //	std::cerr << "N: " << n << " Buff. Size: " << bufferSize << std::endl;
  //}
  // std::cerr << "N: " << n << " Buff. Size: " << bufferSize << std::endl;
  // fcntl(fd, F_SETFL, flags); /* set the new flagts */
}

void NamedPipeStream::FlushIO() {
  // tcflush(fd, TCIOFLUSH);
}

void NamedPipeStream::FlushOutput() {
  // tcflush(fd, TCOFLUSH);
}

int NamedPipeStream::Write(const void *buf, uint32_t size, uint32_t to) {
  int w = write(fd, (const uint8_t *)buf, size);
  if (w < 0) {
    close(fd);
    throw CommsException("Fallo de comunicacion al escribir",
                         COMMS_EXCEPTION_LINEDOWN);
  }

  return w;
}

int NamedPipeStream::Available() {
  int n;
  if (ioctl(fd, FIONREAD, &n) < 0)
    return -1;
  return n;
}

bool NamedPipeStream::IsOpen() { return _open; }

void NamedPipeStream::ReadUint8(uint8_t &byte) {
  read(fd, &byte, sizeof(uint8_t));
}

void NamedPipeStream::ReadChar(char &byte) { read(fd, &byte, sizeof(uint8_t)); }

void NamedPipeStream::ReadUint16(uint16_t &data16) {
  read(fd, &data16, sizeof(uint16_t));
}

void NamedPipeStream::ReadUint32(uint32_t &data32) {
  read(fd, &data32, sizeof(uint32_t));
}
} /* namespace radiotransmission */
