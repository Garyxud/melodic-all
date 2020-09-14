/*
 * DeviceInterface.cpp
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#include <dccomms/CommsException.h>
#include <dccomms/StreamCommsDevice.h>

namespace dccomms {

StreamCommsDevice::StreamCommsDevice() {
  // TODO Auto-generated constructor stub
}

StreamCommsDevice::~StreamCommsDevice() {
  // TODO Auto-generated destructor stub
}

void StreamCommsDevice::ReadPacket(const PacketPtr &pkt) { pkt->Read(this); }

void StreamCommsDevice::WritePacket(const PacketPtr &pkt) { pkt->Write(this); }
}
