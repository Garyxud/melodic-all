/*
 * GironaStream.h
 *
 *  Created on: 16 nov. 2016
 *      Author: centelld
 */

#ifndef INCLUDE_MERBOTS_GIRONASTREAM_H_
#define INCLUDE_MERBOTS_GIRONASTREAM_H_

#include <dccomms/SerialPortStream.h>
#include <dccomms_utils/Constants.h>
#include <dccomms_utils/WFSStream.h>
#include <regex>
#include <string>

namespace dccomms_utils {

using namespace dccomms;

class S100Stream : public SerialPortStream, public WFSStream {
public:
  S100Stream(std::string serialportname = "/dev/ttyUSB0",
             BaudRate = BaudRate::BAUD_19200, int maxBaudrate = 2400);
  virtual ~S100Stream();

  virtual void WritePacket(const PacketPtr &dlf);

  virtual void LogConfig();
  virtual void SetHwFlowControl(bool v);

private:
  void init();
  virtual int _Recv(void *dbuf, int n, bool block = true);

  char *notificationPayload;
  char notification[MAX_NOTIFICATION_LENGTH];
  int _maxBaudrate;
  uint64_t _byteTransmissionTimeNanos;
  int _maxTrunkSize;
  std::function<void(const PacketPtr &)> _WritePacket;
  void _WritePacketHwFlowControl(const PacketPtr &dlf);
  void _WritePacketManualFlowControl(const PacketPtr &dlf);
  uint8_t _endOfPacket[2] = {0xd, 0xa};
};

} /* namespace merbots */

#endif /* INCLUDE_MERBOTS_GIRONASTREAM_H_ */
