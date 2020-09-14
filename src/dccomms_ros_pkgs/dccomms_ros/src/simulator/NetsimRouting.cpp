#include <dccomms_ros/simulator/CustomROSCommsDevice.h>
#include <dccomms_ros/simulator/NetsimRouting.h>

NS_LOG_COMPONENT_DEFINE(
    "NetsimRouting"); // NS3 LOG DOES NOT WORK HERE (TODO: FIX IT)
namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(NetsimRouting);

ns3::TypeId NetsimRouting::GetTypeId(void) {
  static ns3::TypeId tid =
      ns3::TypeId("dccomms_ros::NetsimRouting").SetParent<Object>();
  return tid;
}

NetsimRouting::NetsimRouting(CustomROSCommsDeviceNs3Ptr dev) { _dev = dev; }

bool NetsimRouting::Recv(ns3::Ptr<ns3::Packet> pkt, const ns3::Address &dest,
                         uint16_t protocolNumber) {
  AquaSimHeader ash;
  pkt->PeekHeader(ash);
  if (ash.GetDirection() == AquaSimHeader::DOWN) {
    SendDown(pkt, ash.GetDAddr(), ns3::Seconds(0));
  } else {
    auto header = NetsimHeader::Build(pkt);
    pkt->RemoveHeader(ash);
    pkt->AddHeader(header);
    _dev->ReceiveFrame(pkt);
  }
  return true;
}
} // namespace dccomms_ros
