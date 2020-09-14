#include <dccomms_ros/simulator/AcousticCommsChannel.h>

#include <ns3/aqua-sim-channel.h>
#include <ns3/channel-list.h>
#include <ns3/pointer.h>

namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(AcousticCommsChannel);

ns3::TypeId AcousticCommsChannel::GetTypeId(void) {
  static ns3::TypeId tid = ns3::TypeId("dccomms_ros::AcousticCommsChannel")
                               .SetParent<CommsChannel>();
  return tid;
}

AcousticCommsChannel::AcousticCommsChannel(uint32_t id) {
  _rosChannelId = id;
  _ns3ChannelId = ns3::ChannelList::GetNChannels();
  _channelHelper = ns3::AquaSimChannelHelper::Default();
  _channelHelper.SetPropagation("ns3::AquaSimRangePropagation");
  _aquaSimChannel = _channelHelper.Create();

  // https://www.nsnam.org/docs/manual/html/attributes.html#changing-values
  ns3::PointerValue tmp;
  _aquaSimChannel->GetAttribute("SetProp", tmp);
  _prop = tmp.GetObject()->GetObject<ns3::AquaSimPropagation>();
  SetBandwidth(4096);
  SetTemperature(25);
  SetSalinity(35);
  SetNoiseLevel(0);
}

void AcousticCommsChannel::SetBandwidth(double value) {
  _prop->SetAttribute("Bandwidth", ns3::DoubleValue(value));
}
void AcousticCommsChannel::SetTemperature(double value) {
  _prop->SetAttribute("Temperature", ns3::DoubleValue(value));
}
void AcousticCommsChannel::SetSalinity(double value) {
  _prop->SetAttribute("Salinty", ns3::DoubleValue(value));
}
void AcousticCommsChannel::SetNoiseLevel(double value) {
  _prop->SetAttribute("NoiseLvl", ns3::DoubleValue(value));
}

double AcousticCommsChannel::GetBandwidth() {
  ns3::DoubleValue value;
  _prop->GetAttribute("Bandwidth", value);
  return value.Get();
}
double AcousticCommsChannel::GetTemperature() {
  ns3::DoubleValue value;
  _prop->GetAttribute("Temperature", value);
  return value.Get();
}
double AcousticCommsChannel::GetSalinity() {
  ns3::DoubleValue value;
  _prop->GetAttribute("Salinty", value);
  return value.Get();
}
double AcousticCommsChannel::GetNoiseLevel() {
  ns3::DoubleValue value;
  _prop->GetAttribute("NoiseLvl", value);
  return value.Get();
}
double AcousticCommsChannel::GetPropSpeed() {
  return ns3::SOUND_SPEED_IN_WATER;
}
} // namespace dccomms_ros
