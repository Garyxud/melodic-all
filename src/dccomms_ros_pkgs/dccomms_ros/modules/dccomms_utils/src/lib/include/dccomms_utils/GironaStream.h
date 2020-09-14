/*
 * GironaStream.h
 *
 *  Created on: 16 nov. 2016
 *      Author: centelld
 */

#ifndef INCLUDE_MERBOTS_GIRONASTREAM_H_
#define INCLUDE_MERBOTS_GIRONASTREAM_H_

#include <dccomms/DataLinkFrame.h>
#include <dccomms/SerialPortStream.h>
#include <dccomms_utils/Constants.h>
#include <dccomms_utils/EvologicsStream.h>
#include <regex>
#include <string>

namespace dccomms_utils {

using namespace dccomms;

// Girona will receive all from Piggybag messages
class GironaStream : public SerialPortStream, public EvologicsStream {
public:
  GironaStream(std::string serialportname = "/dev/ttyUSB0",
               BaudRate = BaudRate::BAUD_19200);
  virtual ~GironaStream();
  virtual void ReadPacket(const PacketPtr &pkt);

private:
  void init();
  virtual int _Recv(void *dbuf, int n, bool block = true);
  std::string recPbmHeader;
  int recPbmHeaderLength;
  char *notificationPayload;
  char notification[MAX_NOTIFICATION_LENGTH];
  uint8_t *pbmData; // (but 64 is the maximum...)

  DataLinkFramePtr dlf;
  // Example of pbm notification:
  //+++AT:47:RECVPBM,10,1,2,311411,-17,536,0.0307,1234567890
  std::regex pbmregex;
  // std::regex
  // pbmregex("RECVPBM,(\\d+),(\\d+),(\\d+),(\\d+(?:\\.\\d+)?),(-(?:\\d+(?:\\.\\d+)?)),(\\d+(?:\\.\\d+)?),(.+)");
};

} /* namespace merbots */

#endif /* INCLUDE_MERBOTS_GIRONASTREAM_H_ */
