#ifndef DCCOMMS_ROS_ROSCOMMSDEVICE_H_
#define DCCOMMS_ROS_ROSCOMMSDEVICE_H_

#include <cpplogging/Loggable.h>
#include <dccomms/CommsDeviceService.h>
#include <dccomms/Utils.h>
#include <dccomms_ros/simulator/CommsChannel.h>
#include <dccomms_ros/simulator/NetsimPacket.h>
#include <ns3/config-store-module.h>
#include <ns3/core-module.h>
#include <ns3/packet.h>
#include <tf/transform_listener.h>
#include <ns3/aqua-sim-helper.h>
#include <ns3/aqua-sim-net-device.h>
#include <ns3/mobility-helper.h>

using namespace dccomms;
using namespace cpplogging;
namespace dccomms_ros {

enum PacketErrorType { PE_PROP, PE_COL };
class ROSCommsDevice;
typedef ns3::Ptr<ROSCommsDevice> ROSCommsDeviceNs3Ptr;
// typedef dccomms::Ptr<ROSCommsDevice> ROSCommsDevicePtr;
typedef ROSCommsDevice *ROSCommsDevicePtr;
typedef std::unordered_map<uint32_t, uint64_t> macToCurrentSeqMap;

class ROSCommsSimulator;
// typedef dccomms::Ptr<ROSCommsSimulator> ROSCommsSimulatorPtr;
typedef ROSCommsSimulator *ROSCommsSimulatorPtr;

typedef ns3::Ptr<ns3::Packet> ns3PacketPtr;
typedef ns3::Ptr<const ns3::Packet> ns3ConstPacketPtr;
// why inheritance for std::enable_shared_from_this??:
//  https://stackoverflow.com/questions/11711034/stdshared-ptr-of-this
//  https://stackoverflow.com/questions/16082785/use-of-enable-shared-from-this-with-multiple-inheritance
class ROSCommsDevice : public virtual Logger,
                       public ns3::Object,
                       public std::enable_shared_from_this<ROSCommsDevice> {
public:
  ROSCommsDevice(ROSCommsSimulatorPtr, PacketBuilderPtr txpb,
                 PacketBuilderPtr rxpb);
  ~ROSCommsDevice();

  CommsDeviceServicePtr GetService();
  void ReceiveFrame(ns3PacketPtr);
  std::string GetDccommsId();
  void SetDccommsId(const std::string name);

  void SetBitRate(uint32_t bps);
  uint64_t GetNanosPerByte() { return _nanosPerByte; }

  void SetPosition(const tf::Vector3 &position);
  void SetMaxTxFifoSize(uint32_t size);
  uint32_t GetMaxTxFifoSize();

  virtual void SetLogName(std::string name);
  virtual void SetLogLevel(cpplogging::LogLevel);
  virtual void FlushLog();
  virtual void FlushLogOn(cpplogging::LogLevel);
  virtual void LogToConsole(bool);
  virtual void LogToFile(const std::string &filename);

  void SetMac(uint32_t mac);
  void SetTfFrameId(const std::string &);

  uint32_t GetMac();
  std::string GetTfFrameId();

  std::string ToString();

  void Start();

  void LinkToChannel(CommsChannelNs3Ptr channel,
                     CHANNEL_LINK_TYPE linkType = CHANNEL_TXRX);
  CommsChannelNs3Ptr GetLinkedTxChannel();
  CommsChannelNs3Ptr GetLinkedRxChannel();

  tf::Vector3 GetPosition();

  virtual DEV_TYPE GetDevType() = 0;
  bool Started();
  void Stop();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static ns3::TypeId GetTypeId(void);

  typedef void (*PacketReceivedCallback)(std::string path, ROSCommsDevice *,
                                         ns3::Ptr<const ns3::Packet>);
  typedef void (*PacketTransmittingCallback)(std::string path, ROSCommsDevice *,
                                             ns3::Ptr<const ns3::Packet>);
  typedef void (*PacketErrorCallback)(std::string path, ROSCommsDevice *,
                                      ns3::Ptr<const ns3::Packet>, bool, bool);
  typedef void (*CourseChangeCallback)(std::string path, ROSCommsDevice *,
                                       const tf::Vector3 &);

  void InitTracedValues();
  void StartTracedValues();
  //  void StopTracedValues();
  void Send(const PacketPtr &pkt);
  void SetMacMaxTransmitDistance(double v);

protected:
  virtual std::string DoToString() = 0;
  virtual void DoSetMac(uint32_t mac) = 0;
  virtual void DoSend(ns3PacketPtr dlf) = 0;
  virtual void DoLinkToChannel(CommsChannelNs3Ptr channel,
                               CHANNEL_LINK_TYPE linkType = CHANNEL_TXRX) = 0;
  virtual void DoStart() = 0;
  virtual void DoSetPosition(const tf::Vector3 &position) = 0;
  virtual bool DoStarted() = 0;
  virtual void DoSetMaxTxFifoSize(uint32_t size) = 0;
  virtual void DoSetMacMaxTransmitDistance(double v);

  ROSCommsSimulatorPtr _sim;
  PacketBuilderPtr _txpb, _rxpb;
  void _BuildMac2SeqMap();

  ns3::TracedCallback<ROSCommsDevice *, ns3ConstPacketPtr> _rxCbTrace;
  ns3::TracedCallback<ROSCommsDevice *, ns3ConstPacketPtr> _txCbTrace;
  ns3::TracedCallback<ROSCommsDevice *, ns3ConstPacketPtr, bool, bool>
      _pktErrorCbTrace;
  ns3::TracedCallback<ROSCommsDevice *, const tf::Vector3 &>
      _courseChangesCbTrace;
  ns3::TracedCallback<ROSCommsDevice *, ns3ConstPacketPtr> _routingTxCbTrace, _routingRxCbTrace,
      _macTxCbTrace, _macRxCbTrace;

  void _RoutingTxCb(string context, ns3ConstPacketPtr pkt);
  void _RoutingRxCb(string context, ns3ConstPacketPtr pkt);
  void _MacTxCb(string context, ns3ConstPacketPtr pkt);
  void _MacRxCb(string context, ns3ConstPacketPtr pkt);

private:
  void _StartDeviceService();
  void _StartNodeWorker();
  void _TxWork();
  void _SetPosition(const tf::Vector3 &position);

protected:
  std::mutex _receiveFrameMutex;
  CommsDeviceServicePtr _device;
  CommsChannelNs3Ptr _txChannel, _rxChannel;
  ServiceThread<ROSCommsDevice> _txserv;
  PacketPtr _txdlf;
  std::string _name, _tfFrameId;
  uint32_t _mac;
  uint32_t _bitRate;
  uint64_t _nanosPerByte;
  tf::Vector3 _position;
  macToCurrentSeqMap _macToSeq;
  uint32_t _maxTxFifoSize;
  TracedValue<uint32_t> _currentTxFifoSize, _txPacketDrops;
  uint32_t _currentNumberOfPacketsInTxFifo;

  ns3::Ptr<ns3::AquaSimMac> _macLayer;
  ns3::Ptr<ns3::AquaSimRouting> _routingLayer;
  ns3::Ptr<ns3::Node> _node;
  ns3::Ptr<ns3::MobilityModel> _mobility;
  ns3::AquaSimHelper _asHelper;
  ns3::AquaSimAddress _aquaSimAddr;
  ns3::MobilityHelper _mobh;
  uint32_t _nodeListIndex;

  double _macMaxTransmitDistance;

  // ROSCommsDevicePtr _ownPtr;

  bool _commonStarted;
};
} // namespace dccomms_ros
#endif // COMMSNODE_H
