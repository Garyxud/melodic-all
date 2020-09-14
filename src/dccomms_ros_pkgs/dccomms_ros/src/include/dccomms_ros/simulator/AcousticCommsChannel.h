#ifndef DCCOMMS_ROS_ACOUSTIC_COMMS_CHANNEL_H_
#define DCCOMMS_ROS_ACOUSTIC_COMMS_CHANNEL_H_

#include <condition_variable>
#include <dccomms_ros/simulator/CommsChannel.h>
#include <dccomms_ros_msgs/types.h>
#include <list>
#include <memory>
#include <ns3/aqua-sim-ng-module.h>
#include <random>

using namespace std;
namespace dccomms_ros {

typedef ns3::Ptr<ns3::AquaSimChannel> AquaSimChannelPtr;
typedef std::unordered_map<int, AquaSimChannelPtr> AquaSimChannelSet;

class AcousticCommsChannel : public CommsChannel {
public:
  AcousticCommsChannel(uint32_t id);
  uint32_t GetId() { return _rosChannelId; }
  CHANNEL_TYPE GetType() { return ACOUSTIC_UNDERWATER_CHANNEL; }
  AquaSimChannelPtr GetAquaSimChannel() { return _aquaSimChannel; }
  void SetBandwidth(double value);
  void SetTemperature(double value);
  void SetSalinity(double value);
  void SetNoiseLevel(double value);

  double GetBandwidth();
  double GetTemperature();
  double GetSalinity();
  double GetNoiseLevel();
  virtual double GetPropSpeed() override;

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static ns3::TypeId GetTypeId(void);

private:
  int _rosChannelId;
  int _ns3ChannelId;
  AquaSimChannelPtr _aquaSimChannel;
  static AquaSimChannelSet _aquaSimChannels;
  ns3::AquaSimChannelHelper _channelHelper;
  ns3::Ptr<ns3::AquaSimPropagation> _prop;
};

typedef ns3::Ptr<AcousticCommsChannel> AcousticCommsChannelNs3Ptr;
} // namespace dccomms_ros

#endif // COMMSCHANNELPROPERTIES_H
