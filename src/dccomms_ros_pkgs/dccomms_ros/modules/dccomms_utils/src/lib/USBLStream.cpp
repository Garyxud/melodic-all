/*
 * USBLStream.cpp
 *
 *  Created on: 16 nov. 2016
 *      Author: centelld
 */

#include <cstring>
#include <dccomms/DataLinkFrame.h>
#include <dccomms_utils/USBLStream.h>
#include <sys/time.h>

namespace dccomms_utils {

USBLStream::USBLStream() {
  // TODO Auto-generated constructor stub
  init();
}

USBLStream::USBLStream(std::string addr) : TCPStream(addr) {
  // TODO Auto-generated constructor stub
  init();
}

USBLStream::~USBLStream() {
  // TODO Auto-generated destructor stub
}

void USBLStream::init() {
  //+++AT*SENDPBM,10,2,1234567890
  pbmHeader = "+++AT*SENDPBM,";
  pbmHeaderLength = pbmHeader.length();
}

void USBLStream::WritePacket(const PacketPtr &pkt) {
  auto dlf = DataLinkFrame::BuildDataLinkFrame(DataLinkFrame::fcsType::crc16);
  dlf->GetInfoFromBufferWithPreamble(pkt->GetBuffer());
  // Example of pbm command:
  //+++AT*SENDPBM,10,2,1234567890
  Write(pbmHeader.c_str(), pbmHeaderLength);
  std::string plength = std::to_string(dlf->GetPayloadSize());
  Write(plength.c_str(), plength.length());
  std::string dstdir = std::to_string(dlf->GetDesDir());
  Write(",", 1);
  Write(dstdir.c_str(), dstdir.length());
  Write(",", 1);
  Write(dlf->GetPayloadBuffer(), dlf->GetPayloadSize());
  Write("\n", 1);
}

int USBLStream::_Recv(void *dbuf, int n, bool block) {
  return Recv((unsigned char *)dbuf, n, block); // blocking call
}

int USBLStream::Read(void *buf, uint32_t size, unsigned long ms) {
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
      res = ReadData(ptr, bytesLeft);
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
    res = ReadData(ptr, bytesLeft, false);
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

} /* namespace merbots */
