#ifndef DCCOMMS_ROS_ROSCOMMSSIMULATOR_H_
#define DCCOMMS_ROS_ROSCOMMSSIMULATOR_H_

#include <cpplogging/Loggable.h>
#include <dccomms/dccomms.h>
#include <dccomms_ros/simulator/AcousticROSCommsDevice.h>
#include <dccomms_ros/simulator/CommsChannel.h>
#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <dccomms_ros/simulator/CustomROSCommsDevice.h>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <functional>
#include <memory>
#include <random>
#include <unordered_map>

// ROS
#include <cctype>
#include <dccomms_ros_msgs/AddAcousticChannel.h>
#include <dccomms_ros_msgs/AddAcousticDevice.h>
#include <dccomms_ros_msgs/AddCustomChannel.h>
#include <dccomms_ros_msgs/AddCustomDevice.h>
#include <dccomms_ros_msgs/CheckChannel.h>
#include <dccomms_ros_msgs/CheckDevice.h>
#include <dccomms_ros_msgs/LinkDeviceToChannel.h>
#include <dccomms_ros_msgs/RemoveDevice.h>
#include <dccomms_ros_msgs/StartSimulation.h>
#include <ros/ros.h>
// end ROS

// ns3
#include <ns3/core-module.h>
// end ns3

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros {

static std::string GetMACPType(const string &input) {
  std::string typestr, name;
  name = input;

  std::transform(name.begin(), name.end(), name.begin(),
                 [](unsigned char c) { return std::toupper(c); });

  if (name == "FAMA")
    typestr = "ns3::AquaSimFama";
  else if (name == "SLOTTED-FAMA" || name == "SFAMA" || name == "S-FAMA")
    typestr = "ns3::AquaSimSFama";
  else if (name == "UWAN-MAC")
    typestr = "ns3::AquaSimUwan";
  else if (name == "COPE-MAC")
    typestr = "ns3::AquaSimCopeMac";
  else if (name == "ALOHA")
    typestr = "ns3::AquaSimAloha";
  else if (name == "COPE-MAC")
    typestr = "ns3::AquaSimCopeMac";
  else if (name == "R-MAC" || name == "RMAC")
    typestr = "ns3::AquaSimRMac";
  else if (name == "GOAL")
    typestr = "ns3::AquaSimGoal";
  else if (name == "BROADCAST MAC" || name == "BMAC" || name == "B-MAC")
    typestr = "ns3::AquaSimBroadcastMac";
  else if (name == "T-MAC" || name == "TMAC")
    typestr = "ns3::AquaSimTMac";
  else
    typestr = "";
  return typestr;
}

enum PACKET_TYPE { TX_PACKET, RX_PACKET };

struct DevicePacketBuilder {
  PacketBuilderPtr txpb, rxpb;
};
typedef std::unordered_map<std::string, DevicePacketBuilder> PacketBuilderMap;
typedef std::unordered_map<uint32_t, ROSCommsDevicePtr> Mac2DevMap;
typedef std::shared_ptr<Mac2DevMap> Mac2DevMapPtr;
typedef std::unordered_map<uint32_t, Mac2DevMapPtr> Type2DevMapMap;

typedef std::unordered_map<std::string, ROSCommsDevicePtr> DccommsDevMap;

typedef std::unordered_map<uint32_t, CommsChannelPtr> Id2ChannelMap;
typedef std::shared_ptr<Id2ChannelMap> Id2ChannelMapPtr;
typedef std::unordered_map<uint32_t, Id2ChannelMapPtr> Type2ChannelMapMap;

class ROSCommsSimulator;

// typedef dccomms::Ptr<ROSCommsSimulator> ROSCommsSimulatorPtr;
typedef ROSCommsSimulator *ROSCommsSimulatorPtr;

class ROSCommsSimulator : public virtual Logger, public ns3::Object {
public:
  ROSCommsSimulator();
  ~ROSCommsSimulator();
  void StartROSInterface();

  void SetTransmitPDUCb(
      std::function<void(ROSCommsDevice *txdev, dccomms::PacketPtr)> cb);
  void SetReceivePDUCb(
      std::function<void(ROSCommsDevice *rxdev, dccomms::PacketPtr)> cb);
  void SetPositionUpdatedCb(
      std::function<void(ROSCommsDeviceNs3Ptr dev, tf::Vector3)> cb,
      double cbMinPeriod,
      uint32_t positionUpdateRate =
          10); // callback period = ms ; update rate = Hz

  void GetSimTime(std::string &datetime, double &secsFromStart);

  PacketBuilderPtr GetPacketBuilder(const std::string &dccommsId,
                                    PACKET_TYPE type);
  void SetPacketBuilder(const std::string &dccommsId, PACKET_TYPE type,
                        PacketBuilderPtr pb);
  void SetPacketBuilder(const std::string &dccommsId, PACKET_TYPE type,
                        const std::string &libName,
                        const std::string &className);
  void SetDefaultPacketBuilder(const std::string &lib,
                               const std::string &className);
  void SetDefaultPacketBuilder(PacketBuilderPtr pb) {
    _defaultPacketBuilder = pb;
  }
  PacketBuilderPtr GetDefaultPacketBuilder() { return _defaultPacketBuilder; }
  bool Ready(DEV_TYPE);

  bool AddAcousticDevice(dccomms_ros_msgs::AddAcousticDevice::Request &req);
  bool LinkDevToChannel(dccomms_ros_msgs::LinkDeviceToChannel::Request &req);
  bool AddAcousticChannel(dccomms_ros_msgs::AddAcousticChannel::Request &req);
  bool AddCustomChannel(dccomms_ros_msgs::AddCustomChannel::Request &req);
  bool AddCustomDevice(dccomms_ros_msgs::AddCustomDevice::Request &req);
  bool StartSimulation();
  void _StartLinkUpdaterWork();

  static ns3::TypeId GetTypeId(void);
  void Stop();

  friend class ROSCommsDevice;

  std::vector<ROSCommsDeviceNs3Ptr> GetDevices() { return _devices; }

private:
  const char _timeFormat[100] = "%Y-%m-%d %H:%M:%S";
  int _publish_rate;
  void _Run();
  void _Init();

  std::chrono::high_resolution_clock::time_point _startPoint;
  void _SetSimulationStartDateTime();

  std::function<void(ROSCommsDevice *dev, dccomms::PacketPtr)> TransmitPDUCb,
      ReceivePDUCb;
  std::function<void(ROSCommsDeviceNs3Ptr dev, tf::Vector3)> PositionUpdatedCb;

  bool _AddAcousticDevice(dccomms_ros_msgs::AddAcousticDevice::Request &req,
                          dccomms_ros_msgs::AddAcousticDevice::Response &res);
  bool _CheckDevice(dccomms_ros_msgs::CheckDevice::Request &req,
                    dccomms_ros_msgs::CheckDevice::Response &res);
  bool _CheckChannel(dccomms_ros_msgs::CheckChannel::Request &req,
                     dccomms_ros_msgs::CheckChannel::Response &res);
  bool _RemoveDevice(dccomms_ros_msgs::RemoveDevice::Request &req,
                     dccomms_ros_msgs::RemoveDevice::Response &res);
  bool _LinkDevToChannel(dccomms_ros_msgs::LinkDeviceToChannel::Request &req,
                         dccomms_ros_msgs::LinkDeviceToChannel::Response &res);
  bool _AddAcousticChannel(dccomms_ros_msgs::AddAcousticChannel::Request &req,
                           dccomms_ros_msgs::AddAcousticChannel::Response &res);
  bool _AddCustomChannel(dccomms_ros_msgs::AddCustomChannel::Request &req,
                         dccomms_ros_msgs::AddCustomChannel::Response &res);
  bool _AddCustomDevice(dccomms_ros_msgs::AddCustomDevice::Request &req,
                        dccomms_ros_msgs::AddCustomDevice::Response &res);

  void _AddDeviceToSet(std::string iddev, ROSCommsDeviceNs3Ptr dev);
  bool _DeviceExists(std::string iddev);
  bool _ChannelExists(uint32_t id);
  void _RemoveDeviceFromSet(std::string iddev);

  bool _StartSimulation(dccomms_ros_msgs::StartSimulation::Request &req,
                        dccomms_ros_msgs::StartSimulation::Response &res);

  bool _CommonPreAddDev(const std::string &dccommsId, DEV_TYPE deviceType,
                        uint32_t mac);
  ROSCommsDeviceNs3Ptr _GetDevice(std::string iddev);

  CommsChannelNs3Ptr _GetChannel(int id);

  ros::ServiceServer _addDevService, _checkDevService, _addChannelService,
      _removeDevService, _linkDeviceToChannelService, _startSimulationService,
      _addCustomDeviceService, _addCustomChannelService, _checkChannelService;
  ros::NodeHandle _rosNode;

  std::mutex _devLinksMutex, _idDevMapMutex, _channelsMutex;

  PacketBuilderPtr _defaultPacketBuilder;
  ServiceThread<ROSCommsSimulator> _linkUpdaterWorker;
  void _LinkUpdaterWork();
  void _IsAliveWork();

  ////////////////////
  Type2DevMapMap _type2DevMap;
  DccommsDevMap _dccommsDevMap;
  // Type2ChannelMapMap _type2ChannelMapMap;
  Id2ChannelMap _channelMap;
  //////////
  ROSCommsSimulatorPtr _this;
  tf::TransformListener listener;
  dccomms::Timer _showLinkUpdaterLogTimer;
  bool _started, _callPositionUpdatedCb;
  double _positionUpdatedCbMinPeriod;
  uint32_t _updatePositionRate;
  ros::Rate _linkUpdaterLoopRate;

  PacketBuilderMap _packetBuilderMap;
  std::vector<ROSCommsDeviceNs3Ptr> _devices;
  std::vector<CustomROSCommsDeviceNs3Ptr> _customDevices;
  std::vector<AcousticROSCommsDeviceNs3Ptr> _acousticDevices;
  std::vector<CustomCommsChannelNs3Ptr> _customChannels;
  std::vector<AcousticCommsChannelNs3Ptr> _acousticChannels;
  std::vector<CommsChannelNs3Ptr> _channels;

  template <typename T> void _InsertDeviceAsc(std::vector<T> &devices, T dev);
  template <typename T>
  void _InsertChannelAsc(std::vector<T> &channels, T channel);
};

template <typename T>
void ROSCommsSimulator::_InsertDeviceAsc(std::vector<T> &devices, T dev) {

  if (devices.size() > 0) {
    uint32_t i = 0;
    for (i = 0; i < devices.size(); i++) {
      auto cdev = devices[i];
      if (cdev->GetMac() > dev->GetMac()) {
        devices.push_back(cdev);
        devices[i] = dev;
        break;
      }
    }
    if (i >= devices.size())
      devices.push_back(dev);
  } else
    devices.push_back(dev);
}

template <typename T>
void ROSCommsSimulator::_InsertChannelAsc(std::vector<T> &channels, T chn) {

  if (channels.size() > 0) {
    uint32_t i = 0;
    for (i = 0; i < channels.size(); i++) {
      auto cchn = channels[i];
      if (cchn->GetId() > chn->GetId()) {
        channels.push_back(cchn);
        channels[i] = chn;
        break;
      }
    }
    if (i >= channels.size())
      channels.push_back(chn);
  } else
    channels.push_back(chn);
}
} // namespace dccomms_ros
#endif // WHROVSIMULATOR_H
