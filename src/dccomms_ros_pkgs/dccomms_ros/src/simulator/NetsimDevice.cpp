#include <dccomms_ros/simulator/CustomROSCommsDevice.h>
#include <dccomms_ros/simulator/NetsimDevice.h>

NS_LOG_COMPONENT_DEFINE(
    "NetsimDevice"); // NS3 LOG DOES NOT WORK HERE (TODO: FIX IT)

using namespace dccomms_ros;
namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(NetsimDevice);

ns3::TypeId NetsimDevice::GetTypeId(void) {
  static ns3::TypeId tid =
      ns3::TypeId("dccomms_ros::NetsimDevice").SetParent<AquaSimNetDevice>();
  return tid;
}

NetsimDevice::NetsimDevice(ns3::Ptr<CustomROSCommsDevice> dev) {
  _propSpeed = 1500;
  _dev = dev;
}

NetsimDevice::~NetsimDevice() {}

TransStatus NetsimDevice::GetTransmissionStatus() {
  TransStatus res;
  if (_dev->HalfDuplex()) {
    if (_dev->Receiving())
      res = TransStatus::RECV;
    else if (_dev->Transmitting())
      res = TransStatus::SEND;
    else {
      res = TransStatus::NIDLE;
    }
  } else if (_dev->Transmitting()) {
    res = TransStatus::SEND;
  } else {
    res = TransStatus::NIDLE;
  }
  return res;
}

void NetsimDevice::SetTransmissionStatus(TransStatus status) {
  // Nothing to do. The status is set by the custom comms device
}

void NetsimDevice::SetPropSpeed(const double &speed) { _propSpeed = speed; }

double NetsimDevice::GetPropSpeed() { return _propSpeed; }
} // namespace dccomms_ros
