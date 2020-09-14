/*
 * DeviceInterface.h
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#ifndef DCCOMMS_ISTREAMCOMMSDEVICE_H_
#define DCCOMMS_ISTREAMCOMMSDEVICE_H_

#include <dccomms/CommsDevice.h>
#include <dccomms/Stream.h>

namespace dccomms {

class StreamCommsDevice : public Stream, public CommsDevice {
public:
  StreamCommsDevice();
  virtual ~StreamCommsDevice();

  virtual void ReadPacket(const PacketPtr &);
  virtual void WritePacket(const PacketPtr &);
};

} /* namespace dccomms */

#endif /* DCCOMMS_ISTREAMCOMMSDEVICE_H_ */
