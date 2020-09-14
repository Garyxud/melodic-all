#ifndef DCCOMMS_ROS_VIRTUALDEVICELINK_H_
#define DCCOMMS_ROS_VIRTUALDEVICELINK_H_

#include <condition_variable>
#include <dccomms_ros/simulator/CommsChannel.h>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <list>
#include <memory>
#include <random>

using namespace std;

namespace dccomms_ros {

class VirtualDeviceLink;
typedef std::shared_ptr<VirtualDeviceLink> VirtualDeviceLinkPtr;

class VirtualDeviceLink {

public:
  VirtualDeviceLink(ROSCommsDevicePtr dev0, CommsChannelPtr channel);
  bool LinkOk() { return _linkOk; }
  void LinkOk(bool v) { _linkOk = v; }
  CommsChannelPtr GetChannel() { return _channel; }

  ROSCommsDevicePtr GetDevice0() { return _dev0; }
  ROSCommsDevicePtr GetDevice1() { return _dev1; }

private:
  CommsChannelPtr _channel;
  ROSCommsDevicePtr _dev0, _dev1;
  uint32_t _distance;
  bool _linkOk;
};

typedef std::list<VirtualDeviceLinkPtr> VirtualDevicesLinks;
}

#endif
