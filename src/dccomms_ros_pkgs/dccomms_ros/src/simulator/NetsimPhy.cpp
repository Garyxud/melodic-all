#include <dccomms_ros/simulator/CustomROSCommsDevice.h>
#include <dccomms_ros/simulator/NetsimPhy.h>

NS_LOG_COMPONENT_DEFINE(
    "NetsimPhy"); // NS3 LOG DOES NOT WORK HERE (TODO: FIX IT)
namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(NetsimPhy);

ns3::TypeId NetsimPhy::GetTypeId(void) {
  static ns3::TypeId tid =
      ns3::TypeId("dccomms_ros::NetsimPhy").SetParent<Object>();
  return tid;
}

NetsimPhy::NetsimPhy(CustomROSCommsDeviceNs3Ptr dev) { _dev = dev; }

bool NetsimPhy::Recv(ns3::Ptr<ns3::Packet> pkt) {
  AquaSimPacketStamp apsh;
  pkt->RemoveHeader(apsh);
  auto header = NetsimHeader::Build(pkt);
  pkt->AddHeader(header);
  _dev->PhySend(pkt);
  return true;
}
ns3::Time NetsimPhy::CalcTxTime(uint32_t pktSize, std::string *modName) {
  ns3::Time res = ns3::NanoSeconds(pktSize * _dev->GetNanosPerByte() +
                                   _dev->GetIntrinsicDelay() * 1e6 +
                                   _dev->GetFixedIPGNanos());
  return res;
}

double NetsimPhy::CalcPktSize(double txTime, std::string *modName) {
  txTime = txTime - _dev->GetIntrinsicDelay() / 1000.;
  double res = (txTime * 1e9) / _dev->GetNanosPerByte();
  return res;
}
} // namespace dccomms_ros
