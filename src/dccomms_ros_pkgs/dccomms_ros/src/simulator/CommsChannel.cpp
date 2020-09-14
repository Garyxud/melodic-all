#include <dccomms_ros/simulator/CommsChannel.h>

using namespace ns3;

namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(CommsChannel);

ns3::TypeId CommsChannel::GetTypeId(void) {
  static ns3::TypeId tid = ns3::TypeId("dccomms_ros::CommsChannel").SetParent<Object>();
  return tid;
}
}
