#include <cstdio>
#include <dccomms_ros/simulator/NetsimLogFormatter.h>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <dccomms_ros_msgs/types.h>
#include <ns3/aqua-sim-header.h>
#include <ns3/core-module.h>
#include <ns3/node-list.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE(
    "ROSCommsDevice"); // NS3 LOG DOES NOT WORK HERE (TODO: FIX IT)
namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(ROSCommsDevice);

ns3::TypeId ROSCommsDevice::GetTypeId(void) {
  static ns3::TypeId tid =
      ns3::TypeId("dccomms_ros::ROSCommsDevice")
          .SetParent<AquaSimNetDevice>()
          .AddTraceSource(
              "PacketReceived",
              "Trace source indicating a packet has been "
              "delivered to the upper layer.",
              ns3::MakeTraceSourceAccessor(&ROSCommsDevice::_rxCbTrace),
              "dccomms_ros::ROSCommsDevice::PacketReceivedCallback")
          .AddTraceSource(
              "PacketTransmitting",
              "Trace source indicating a packet has been "
              "delivered to the lower layer.",
              ns3::MakeTraceSourceAccessor(&ROSCommsDevice::_txCbTrace),
              "dccomms_ros::ROSCommsDevice::PacketTransmittingCallback")
          .AddTraceSource(
              "PacketError",
              "Trace source indicating a packet has been "
              "corrupted.",
              ns3::MakeTraceSourceAccessor(&ROSCommsDevice::_pktErrorCbTrace),
              "dccomms_ros::ROSCommsDevice::PacketPropagationErrorCallback")
          .AddTraceSource(
              "PhyRxError",
              "Trace source indicating a packet has been "
              "corrupted.",
              ns3::MakeTraceSourceAccessor(&ROSCommsDevice::_pktErrorCbTrace),
              "dccomms_ros::ROSCommsDevice::PacketPropagationErrorCallback")
          .AddTraceSource(
              "MacTx",
              "Trace source indicating a packet has been "
              "delivered to the Phy Layer",
              ns3::MakeTraceSourceAccessor(&ROSCommsDevice::_macTxCbTrace),
              "dccomms_ros::ROSCommsDevice::PacketTransmittingCallback")
          .AddTraceSource(
              "MacRx",
              "Trace source indicating a packet has been "
              "delivered to the Mac layer.",
              ns3::MakeTraceSourceAccessor(&ROSCommsDevice::_macRxCbTrace),
              "dccomms_ros::ROSCommsDevice::PacketReceivedCallback")
          .AddTraceSource(
              "CourseChange", "Device's position updated.",
              MakeTraceSourceAccessor(&ROSCommsDevice::_courseChangesCbTrace),
              "dccomms_ros::ROSCommsDevice::CourseChangeCallback")
          .AddTraceSource(
              "TxFifoSize",
              "Current number of bytes in device's transmission fifo",
              MakeTraceSourceAccessor(&ROSCommsDevice::_currentTxFifoSize),
              "ns3::TracedValueCallback::Uint32")
          .AddTraceSource(
              "TxPacketDrops",
              "Number of transmitted packet drops due to the TxFifo is full",
              MakeTraceSourceAccessor(&ROSCommsDevice::_txPacketDrops),
              "ns3::TracedValueCallback::Uint32");
  return tid;
}

ROSCommsDevice::ROSCommsDevice(ROSCommsSimulatorPtr s, PacketBuilderPtr txpb,
                               PacketBuilderPtr rxpb)
    : _sim(s), _txserv(this) {

  _nodeListIndex = ns3::NodeList::GetNNodes();
  _node = ns3::CreateObject<ns3::Node>();
  _mobh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  _mobh.Install(_node);
  _rxpb = rxpb;
  _txpb = txpb;
  _device = CommsDeviceService::BuildCommsDeviceService(
      _txpb, CommsDeviceService::IPHY_TYPE_PHY);
  _device->SetBlockingTransmission(false);
  _device->SetLogLevel(cpplogging::LogLevel::info);
  SetLogLevel(cpplogging::info);
  _txserv.SetWork(&ROSCommsDevice::_TxWork);
  _commonStarted = false;
  _position = tf::Vector3(0, 0, 0);
  LogComponentEnable("ROSCommsDevice",
                     LOG_LEVEL_ALL); // NS3 DOES NOT WORK (TODO: FIX IT)
  SetLogLevel(debug);
  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("[%T.%F] %v"));
  FlushLogOn(off);
  InitTracedValues();
}

ROSCommsDevice::~ROSCommsDevice() {}

void ROSCommsDevice::InitTracedValues() {
  _txPacketDrops = UINT32_MAX;
  _currentNumberOfPacketsInTxFifo = UINT32_MAX;
  _currentTxFifoSize = UINT32_MAX;
}

void ROSCommsDevice::StartTracedValues() {

  _txPacketDrops = 0;
  _currentNumberOfPacketsInTxFifo = 0;
  _currentTxFifoSize = 0;
}

void ROSCommsDevice::SetMacMaxTransmitDistance(double v) {
  _macMaxTransmitDistance = v;
  DoSetMacMaxTransmitDistance(v);
}

void ROSCommsDevice::DoSetMacMaxTransmitDistance(double v) {}

void ROSCommsDevice::_RoutingRxCb(string context, ns3ConstPacketPtr pkt) {
  _routingRxCbTrace(this, pkt);
}

void ROSCommsDevice::_RoutingTxCb(string context, ns3ConstPacketPtr pkt) {
  _routingTxCbTrace(this, pkt);
}

void ROSCommsDevice::_MacTxCb(string context, ns3ConstPacketPtr pkt) {
  _macTxCbTrace(this, pkt);
}

void ROSCommsDevice::_MacRxCb(string context, ns3ConstPacketPtr pkt) {
  _macRxCbTrace(this, pkt);
}

// void ROSCommsDevice::StopTracedValues() { InitTracedValues(); }

void ROSCommsDevice::_BuildMac2SeqMap() {
  auto devs = _sim->GetDevices();
  for (auto dev : devs) {
    if (dev->GetDevType() == GetDevType() && dev->GetMac() != GetMac()) {
      _macToSeq[dev->GetMac()] = 0;
    }
  }
}

void ROSCommsDevice::_StartDeviceService() {
  auto startWorker = std::thread([this]() {
    _device->Start();
    _BuildMac2SeqMap();
    _commonStarted = true;
    _device->SetPhyLayerState(CommsDeviceService::READY);
    std::this_thread::sleep_for(chrono::seconds(4));
    // flush input
    while (_device->GetRxFifoSize() > 0) {
      _device >> _txdlf;
      std::this_thread::sleep_for(chrono::milliseconds(200));
    }
  });
  startWorker.detach();

  _StartNodeWorker();
}

void ROSCommsDevice::Stop() {
  auto level = Log->level();
  // StopTracedValues();
  Log->set_level(spdlog::level::info);
  Info("Stopping comms service...");
  _device->Stop();
  Info("Stopping packet sender service...");
  _txserv.Stop();
  FlushLog();
  Info("Device stopped");
  Log->set_level(level);
}

void ROSCommsDevice::Start() {
  ns3::Simulator::ScheduleWithContext(GetMac(), ns3::Seconds(0),
                                      &ROSCommsDevice::StartTracedValues, this);
  //  ns3::Simulator::ScheduleDestroy(&ROSCommsDevice::StopTracedValues, this);
  _StartDeviceService();
  DoStart();
}

bool ROSCommsDevice::Started() { return _commonStarted && DoStarted(); }

void ROSCommsDevice::_StartNodeWorker() { _txserv.Start(); }

void ROSCommsDevice::ReceiveFrame(ns3PacketPtr packet) {
  Debug("ROSCommsDevice: Frame received");
  _rxCbTrace(this, packet);
  if (Started()) {
    char ser[5000];
    NetsimHeader header;
    packet->RemoveHeader(header);
    auto size = packet->GetSize();
    packet->CopyData((uint8_t *)ser, size);
    auto dccommsPacket = _rxpb->CreateFromBuffer(ser);
    _receiveFrameMutex.lock();
    _device << dccommsPacket;
    _receiveFrameMutex.unlock();
  } else {
    Warn("Device's service has not been been started when packet reception");
  }
}

void ROSCommsDevice::SetBitRate(uint32_t v) {
  _bitRate = v; // bps
  _nanosPerByte = static_cast<uint64_t>(std::round(8 * 1e9 / _bitRate));
}

void ROSCommsDevice::SetDccommsId(const std::string name) {
  _name = name;
  _txdlf = _txpb->Create();
  _device->SetCommsDeviceId(_name);
  SetLogName(_name);
  _device->SetLogName(_name + ":Service");
}

std::string ROSCommsDevice::GetDccommsId() { return _name; }

void ROSCommsDevice::SetMaxTxFifoSize(uint32_t size) {
  DoSetMaxTxFifoSize(size);
}

uint32_t ROSCommsDevice::GetMaxTxFifoSize() { return _maxTxFifoSize; }

void ROSCommsDevice::SetMac(uint32_t mac) {
  _mac = mac;
  DoSetMac(_mac);
}

void ROSCommsDevice::LinkToChannel(CommsChannelNs3Ptr channel,
                                   CHANNEL_LINK_TYPE linkType) {
  if (channel->GetType() == CHANNEL_TYPE::ACOUSTIC_UNDERWATER_CHANNEL) {
    linkType = CHANNEL_LINK_TYPE::CHANNEL_TXRX;
  }
  switch (linkType) {
  case CHANNEL_LINK_TYPE::CHANNEL_TXRX: {
    _rxChannel = channel;
    _txChannel = channel;
    break;
  }
  case CHANNEL_LINK_TYPE::CHANNEL_TX: {
    _txChannel = channel;
    break;
  }
  case CHANNEL_LINK_TYPE::CHANNEL_RX: {
    _rxChannel = channel;
    break;
  }
  }
  DoLinkToChannel(channel, linkType);
}

CommsChannelNs3Ptr ROSCommsDevice::GetLinkedTxChannel() { return _txChannel; }

uint32_t ROSCommsDevice::GetMac() { return _mac; }

void ROSCommsDevice::_TxWork() {
  NS_LOG_FUNCTION(this);
  try {
    _device->WaitForFramesFromRxFifo();
    _device->SetPhyLayerState(CommsDeviceService::BUSY);
    unsigned int txFifoSize;
    do {
      _device >> _txdlf;
      txFifoSize = _device->GetRxFifoSize();
      PacketPtr txdlf = _txpb->CreateFromBuffer(_txdlf->GetBuffer());
      if (txdlf->IsOk()) {
        ns3::Simulator::ScheduleWithContext(GetMac(), ns3::Seconds(0),
                                            &ROSCommsDevice::Send, this, txdlf);
      } else {
        Log->critical("packet received with errors from the upper layer!");
      }
    } while (txFifoSize > 0);

    _device->SetPhyLayerState(CommsDeviceService::READY);
  } catch (CommsException e) {
    if (e.code != COMMS_EXCEPTION_STOPPED)
      Warn("CommsException in the packet sender service: {}", e.what());
  } catch (exception e) {
    Error("Something failed in the packet sender service: {}", e.what());
  } catch (int e) {
    Error("Something failed in the packet sender service. Code: {}", e);
  }
}

void ROSCommsDevice::Send(const PacketPtr &txdlf) {
  txdlf->SetVirtualSrcAddr(GetMac());
  auto header = NetsimHeader::Build(txdlf);
  auto pkt =
      ns3::Create<ns3::Packet>(txdlf->GetBuffer(), txdlf->GetBufferSize());
  pkt->AddHeader(header);
  _txCbTrace(this, pkt);
  NS_LOG_DEBUG("Send packet");
  Debug("ROSCommsDevice: Send frame");
  DoSend(pkt);
}

void ROSCommsDevice::SetLogName(std::string name) {
  Loggable::SetLogName(name);
  _device->SetLogName(name + ":CommsDeviceService");
}

void ROSCommsDevice::SetLogLevel(cpplogging::LogLevel _level) {
  Loggable::SetLogLevel(_level);
  // _device->SetLogLevel(_level);
}

void ROSCommsDevice::LogToConsole(bool c) {
  Loggable::LogToConsole(c);
  //_device->LogToConsole(c);
}

void ROSCommsDevice::LogToFile(const string &filename) {
  Loggable::LogToFile(filename);
  _device->LogToFile(filename);
}

void ROSCommsDevice::FlushLog() {
  Loggable::FlushLog();
  _device->FlushLog();
}

void ROSCommsDevice::FlushLogOn(cpplogging::LogLevel level) {
  Loggable::FlushLogOn(level);
  _device->FlushLogOn(level);
}

void ROSCommsDevice::SetTfFrameId(const string &id) { _tfFrameId = id; }

void ROSCommsDevice::_SetPosition(const tf::Vector3 &position) {
  _position = position;
  _courseChangesCbTrace(this, _position);
  DoSetPosition(position);
}
void ROSCommsDevice::SetPosition(const tf::Vector3 &position) {
  ns3::Simulator::ScheduleWithContext(
      GetMac(), ns3::Seconds(0), &ROSCommsDevice::_SetPosition, this, position);
}

tf::Vector3 ROSCommsDevice::GetPosition() { return _position; }

std::string ROSCommsDevice::GetTfFrameId() { return _tfFrameId; }

std::string ROSCommsDevice::ToString() { return DoToString(); }
} // namespace dccomms_ros
