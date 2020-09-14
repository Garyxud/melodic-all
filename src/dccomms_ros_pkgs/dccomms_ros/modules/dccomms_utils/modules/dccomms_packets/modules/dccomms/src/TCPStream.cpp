/*
 * TCPStream.cpp
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#include <boost/algorithm/string.hpp>
#include <dccomms/CommsException.h>
#include <dccomms/TCPStream.h>
#include <sys/ioctl.h>
#include <sys/time.h> /*para timeout*/

/* According to POSIX.1-2001, POSIX.1-2008 */
#include <sys/select.h>

/* According to earlier standards */
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

namespace dccomms {

TCPStream::TCPStream() {
  portno = 8090;
  ip = "localhost";
}

TCPStream::TCPStream(std::string address) {
  // TODO Auto-generated constructor stub
  SetServerAddr(address);
}

TCPStream::~TCPStream() {
  // TODO Auto-generated destructor stub
  CloseConnection();
}

void TCPStream::SetServerAddr(std::string address) {
  std::vector<std::string> strs;
  boost::split(strs, address, boost::is_any_of(":"));
  portno = std::stoi(strs[1]);
  ip = strs[0];
}

void TCPStream::CloseConnection() { close(sockfd); }

void TCPStream::OpenConnection() {
  device = gethostbyname(ip.c_str());
  if (device == NULL) {
    throw CommsException("TCP ERROR: No such host",
                         COMMS_EXCEPTION_PHYLAYER_ERROR);
  }
  bzero((char *)&device_addr, sizeof(device_addr));
  device_addr.sin_family = AF_INET;
  bcopy((char *)device->h_addr, (char *)&device_addr.sin_addr.s_addr,
        device->h_length);
  device_addr.sin_port = htons(portno);

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    throw CommsException("TCP ERROR: Creating a TCP socket",
                         COMMS_EXCEPTION_PHYLAYER_ERROR);

  int keepalive = 1;
  socklen_t optlen = sizeof(keepalive);
  int res = setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, optlen);
  if (res < 0) {
    throw CommsException("Error when setting the keepalive to the socket",
                         COMMS_EXCEPTION_PHYLAYER_ERROR);
  }
  keepalive = 0;
  /* Check the status again */
  if (getsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, &optlen) < 0) {
    perror("getsockopt()");
    close(sockfd);
    throw CommsException("Error when setting the keepalive to the socket",
                         COMMS_EXCEPTION_PHYLAYER_ERROR);
  }
  // printf("SO_KEEPALIVE is %s\n", (keepalive ? "ON" : "OFF"));

  if (connect(sockfd, (struct sockaddr *)&device_addr, sizeof(device_addr)) < 0)
    throw CommsException("TCP ERROR: Connection to device",
                         COMMS_EXCEPTION_PHYLAYER_ERROR);
}

void TCPStream::Close() { CloseConnection(); }

bool TCPStream::Open() {
  OpenConnection();
  return true;
}

int TCPStream::Write(const void *buf, uint32_t size, uint32_t to) {
  int w = write(sockfd, (const uint8_t *)buf, size);
  if (w < 0) {
    close(sockfd);
    throw CommsException("Fallo de comunicacion al escribir",
                         COMMS_EXCEPTION_LINEDOWN);
  }
  return w;
}

int TCPStream::Recv(unsigned char *ptr, int bytesLeft, bool block) {
  int res = 0;
  if (block)
    res = recv(sockfd, ptr, bytesLeft, MSG_WAITALL);
  else
    res = recv(sockfd, ptr, bytesLeft, MSG_DONTWAIT);
  if (res < 0) {
    switch (errno) {
    case EAGAIN:
      return 0;
      break;
    default:
      close(sockfd);
      throw CommsException("Problem happened when reading socket",
                           COMMS_EXCEPTION_LINEDOWN);
    }
  } else if (res == 0) {
    close(sockfd);
    throw CommsException("The client closed the connection",
                         COMMS_EXCEPTION_LINEDOWN);
  }
  return res;
}

int TCPStream::Read(void *buf, uint32_t size, unsigned long ms) {
  struct timeval time0, time1;
  gettimeofday(&time0, NULL);
  unsigned long long t0 = time0.tv_sec * 1000 + time0.tv_usec / 1000;
  unsigned long long t1 = t0;

  unsigned char *ptr = (uint8_t *)buf;
  unsigned char *max = (uint8_t *)buf + size;

  int n = 0;
  int bytesLeft = size - n;

  unsigned long m = ms ? ms : _timeout;
  int res;
  if (m == 0) {
    // Bloqueado hasta coger m bytes
    while (true) {
      res = Recv(ptr, bytesLeft);
      if (res > 0) {
        n += res;
        ptr = (uint8_t *)buf + n;
        if (ptr == max)
          return n; // == size
        bytesLeft = size - n;
      } else {
        if (!Connected()) {
          close(sockfd);
          throw CommsException("Problem happened when reading socket",
                               COMMS_EXCEPTION_LINEDOWN);
        }
      }
    }
  }
  while (t1 - t0 < m) {
    res = Recv(ptr, bytesLeft, false);
    if (res > 0) {
      n += res;
      ptr = (uint8_t *)buf + n;
      if (ptr == max)
        return n; // == size
      bytesLeft = size - n;
    }
    gettimeofday(&time1, NULL);
    t1 = time1.tv_sec * 1000 + time1.tv_usec / 1000;
  }
  /*
      //Si se llega hasta este punto, es que ha transcurrido el timeout
      char sig = '-'; //Un byte aleatorio...
      res = write(sockfd, &sig, 1);
      if(res < 0)
      {
              close(sockfd);
              throw CommsException("Fallo de comunicacion al leer", LINEDOWN);
      }
      */

  throw CommsException("Read Timeout", COMMS_EXCEPTION_TIMEOUT);
}

void TCPStream::ThrowExceptionIfErrorOnSocket() {
  // http://stackoverflow.com/questions/4142012/how-to-find-the-socket-connection-state-in-c
  int error = 0;
  socklen_t len = sizeof(error);
  int retval = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &error, &len);
  if (retval != 0) {
    /* there was a problem getting the error code */
    close(sockfd);
    throw CommsException("error getting socket error code: %s\n" +
                             std::string(strerror(retval)),
                         COMMS_EXCEPTION_LINEDOWN);
  }

  if (error != 0) {
    /* socket has a non zero error status */
    close(sockfd);
    throw CommsException("socket error: %s\n" + std::string(strerror(error)),
                         COMMS_EXCEPTION_LINEDOWN);
  }
}

bool TCPStream::Connected() {
  // http://stackoverflow.com/questions/283375/detecting-tcp-client-disconnect
  // http://blog.stephencleary.com/2009/05/detection-of-half-open-dropped.html
  return !(Ready() && Available() == 0);
}

bool TCPStream::Ready() {
  fd_set fds;
  struct timeval t;
  t.tv_sec = 0;
  t.tv_usec = 0;

  FD_ZERO(&fds);
  FD_SET(sockfd, &fds);
  int r = select(sockfd + 1, &fds, NULL, NULL, &t);
  if (-1 == r) {
    close(sockfd);
    throw CommsException("Error when reading from descriptor",
                         COMMS_EXCEPTION_LINEDOWN);
    return 0;
  }
  return r;
}

int TCPStream::Available() {
  int n;
  if (ioctl(sockfd, FIONREAD, &n) < 0)
    throw CommsException("Some error happened when trying to read",
                         COMMS_EXCEPTION_LINEDOWN);
  return n;
}

void TCPStream::FlushInput() {
  throw CommsException("void TCPStream::FlushInput() Not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}

void TCPStream::FlushIO() {
  throw CommsException("void TCPStream::FlushIO() Not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}

void TCPStream::FlushOutput() {
  throw CommsException("void TCPStream::FlushOutput() Not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}

bool TCPStream::IsOpen() {
  throw CommsException("bool TCPStream::IsOpen() Not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}
}
