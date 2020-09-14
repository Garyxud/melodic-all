/*
 * USBLStream.h
 *
 *  Created on: 16 nov. 2016
 *      Author: centelld
 */

#ifndef INCLUDE_MERBOTS_USBLSTREAM_H_
#define INCLUDE_MERBOTS_USBLSTREAM_H_

#include <dccomms/TCPStream.h>
#include <dccomms_utils/Constants.h>
#include <dccomms_utils/EvologicsStream.h>
#include <string>

namespace dccomms_utils {

using namespace dccomms;
using namespace cpplogging;

// USBL will send all using Piggybag messages
class USBLStream : public TCPStream, public EvologicsStream {
public:
  USBLStream();
  USBLStream(std::string addr);
  virtual ~USBLStream();
  virtual void WritePacket(const PacketPtr &dlf);
  virtual int Read(void *, uint32_t, unsigned long msTimeout = 0);

private:
  virtual int _Recv(void *, int n, bool block = true);
  void init();
  std::string pbmHeader;
  int pbmHeaderLength;

  int ReadFromBuffer(void *dbuf, int n);
};

} /* namespace merbots */

#endif /* INCLUDE_MERBOTS_USBLSTREAM_H_ */
