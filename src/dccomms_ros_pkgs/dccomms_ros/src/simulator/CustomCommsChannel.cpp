#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <ns3/core-module.h>
#include <ns3/simulator.h>

using namespace ns3;

namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(CustomCommsChannel);

ns3::TypeId CustomCommsChannel::GetTypeId(void) {
  static ns3::TypeId tid =
      ns3::TypeId("dccomms_ros::CustomCommsChannel").SetParent<CommsChannel>();
  return tid;
}

CustomCommsChannel::CustomCommsChannel(uint32_t id) {
  _rosChannelId = id;
  SetLogLevel(debug);
  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("[%T.%F] %v"));
}

void CustomCommsChannel::SetMinPrTime(double prTime) {
  _minPrTime = prTime * 1000000;
} // from millis to nanos

void CustomCommsChannel::SetPrTimeInc(double inc) {
  _prTimeIncPerMeter = inc * 1000000;
} // from millis to nanos

void CustomCommsChannel::AddDevice(CustomROSCommsDeviceNs3Ptr dev) {
  _devices.push_back(dev);
}

double CustomCommsChannel::GetPropSpeed() {
  return (1. / _prTimeIncPerMeter) // meters/nano
         * 1e9;                    // meters/second
}
void CustomCommsChannel::SendPacket(CustomROSCommsDeviceNs3Ptr dev,
                                    const OutcomingPacketPtr & pkt) {
  Debug("CustomCommsChannel: SendPacket");
  auto txpos = dev->GetPosition();
  // auto minErrRate = dev->GetMinPktErrorRate();
  // auto errRateInc = dev->GetPktErrorRateInc();

  for (CustomROSCommsDeviceNs3Ptr dst : _devices) {
    if (dst != dev) {
      auto rxpos = dst->GetPosition();
      auto distance = txpos.distance(rxpos);
      auto maxdm = dev->GetMaxDistance();
      auto mindm = dev->GetMinDistance();
      if (distance <= maxdm && distance >= mindm) { // dst is in range
        auto delay = _minPrTime + _prTimeIncPerMeter * distance;
        auto totalTime = static_cast<uint64_t>(round(delay));
        // auto errRate = minErrRate + errRateInc * distance;
        auto cpkt = pkt->packet->Copy();
        NetsimHeader header;
        cpkt->RemoveHeader(header);
        auto propagationError = dev->ErrOnPkt(distance, pkt->packetSize);

        cpkt->AddHeader(header);
        Debug("CustomCommsChannel: distance({} m) ; totalTime({} secs)",
              distance, totalTime / 1e9);
        ns3::Simulator::ScheduleWithContext(
            dev->GetMac(), NanoSeconds(totalTime),
            &CustomROSCommsDevice::AddNewPacket, dst, cpkt, propagationError);
      }
    }
  }
}
} // namespace dccomms_ros
