#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <dccomms_ros/simulator/CustomROSCommsDevice.h>
#include <dccomms_ros/simulator/NetsimDevice.h>
#include <dccomms_ros/simulator/NetsimPhy.h>
#include <dccomms_ros/simulator/NetsimRouting.h>
#include <dccomms_ros/simulator/NetsimTime.h>
#include <ns3/aqua-sim-mac-aloha.h>
#include <ns3/core-module.h>
#include <ns3/simulator.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE(
    "CustomROSCommsDevice"); // NS3 LOG DOES NOT WORK HERE (TODO: FIX IT)
namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(CustomROSCommsDevice);

TypeId CustomROSCommsDevice::GetTypeId(void) {
  static TypeId tid =
      TypeId("dccomms_ros::CustomROSCommsDevice").SetParent<ROSCommsDevice>();

  return tid;
}

CustomROSCommsDevice::CustomROSCommsDevice(ROSCommsSimulatorPtr sim,
                                           PacketBuilderPtr txpb,
                                           PacketBuilderPtr rxpb)
    : ROSCommsDevice(sim, txpb, rxpb), _erDist(0.0, 1.0) {
  Transmitting(false);
  Receiving(false);
  LogComponentEnable("CustomROSCommsDevice",
                     LOG_LEVEL_ALL); // NS3 LOG DOES NOT WORK (TODO: FIX IT)
  _maxTxFifoSize = 2048;
  _nextPacketReceptionTime = 0;
  _neg = false;
  _started = false;
  _phy = ns3::CreateObject<NetsimPhy>(this);
  _routingLayer = ns3::CreateObject<NetsimRouting>(this);
  _dev = ns3::CreateObject<NetsimDevice>(this);
  _dev->SetPhy(_phy);
  _dev->SetRouting(_routingLayer);
  _routingLayer->SetNetDevice(_dev);
  _node->AddDevice(_dev);
}

DEV_TYPE CustomROSCommsDevice::GetDevType() { return DEV_TYPE::CUSTOM_DEV; }

void CustomROSCommsDevice::SetJitter(double tx, double rx) { // jitter in
                                                             // milliseconds
  _txJitterDist = UniformIntDist(0, static_cast<int>(tx * 1000));
  _txJitter = tx;

  _rxJitterDist = UniformIntDist(0, static_cast<int>(rx * 1000));
  _rxJitter = rx;

  _rxJitterSd = 2;
  double rx1 = std::round(rx * 1000 / 2);
  double rx2 = std::round((rx + _rxJitterSd) * 1000 / 2);

  _rxJitterBernoulliDist = std::bernoulli_distribution(0.5);

  _rxJitterBase = static_cast<int64_t>(rx2 * 1000);
  _rxJitterNormalDist =
      NormalRealDist(static_cast<int64_t>(rx1), _rxJitterSd * 1000);
}

void CustomROSCommsDevice::GetBitRate(double &bitrate) { bitrate = _bitRate; }

void CustomROSCommsDevice::SetMinPktErrorRate(double minPktErrorRate) {
  _minPktErrorRate = minPktErrorRate;
}

double CustomROSCommsDevice::GetMinPktErrorRate() { return _minPktErrorRate; }

void CustomROSCommsDevice::SetPktErrorRateInc(double inc) {
  _pktErrorRateIncPerMeter = inc;
}

double CustomROSCommsDevice::GetPktErrorRateInc() {
  return _pktErrorRateIncPerMeter;
}

void SimpleVarExprEval::CompileExpr(const string &expr,
                                    const std::string &var) {
  _sexpr = expr;
  _symbol_table.add_variable(var, _var);
  _symbol_table.add_constants();
  _expression.register_symbol_table(_symbol_table);
  _parser.compile(_sexpr, _expression);
}

SimpleVarExprEval::SimpleVarExprEval() {}
double SimpleVarExprEval::ComputeVal(double var) {
  _var = var;
  return _expression.value();
}

double CustomROSCommsDevice::_GetErrorRate(double meters) {
  return _mExprEval.ComputeVal(meters);
}
void CustomROSCommsDevice::GetRateErrorModel(std::string &expr,
                                             std::string &unit) {
  RateErrorModel::ErrorUnit eunit = _rem->GetUnit();
  switch (eunit) {
  case RateErrorModel::ERROR_UNIT_BIT:
    unit = "ERROR_UNIT_BIT";
    break;
  case RateErrorModel::ERROR_UNIT_BYTE:
    unit = "ERROR_UNIT_BYTE";
    break;
  default:
    unit = "ERROR_UNIT_PACKET";
  }
  expr = _eexpr;
}

void CustomROSCommsDevice::SetRateErrorModel(const std::string &expr,
                                             const std::string &unit) {
  ns3::Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
  // Set this variable to a specific stream
  uv->SetStream(50);

  _rem = CreateObject<RateErrorModel>();
  _rem->SetRandomVariable(uv);
  if (unit == "bit")
    _rem->SetUnit(RateErrorModel::ERROR_UNIT_BIT);
  else if (unit == "byte")
    _rem->SetUnit(RateErrorModel::ERROR_UNIT_BYTE);
  else
    _rem->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);

  _rem->Enable();
  if (expr == "")
    _eexpr = "0.01*m";
  else
    _eexpr = expr;

  Debug("SetRateErrorModel: expression = {}", _eexpr);
  _mExprEval.CompileExpr(_eexpr, "m");
}

bool CustomROSCommsDevice::ErrOnPkt(double range, const uint32_t &size) {
  double rate = _GetErrorRate(range);
  _rem->SetRate(rate);
  auto pkt = ns3::Create<ns3::Packet>(size);
  Debug("ErrOnPkt: {} ; {} --> {}", pkt->GetSize(), range, rate);
  return _rem->IsCorrupt(pkt);
}

void CustomROSCommsDevice::SetMaxDistance(double d) { _maxDistance = d; }

double CustomROSCommsDevice::GetMaxDistance() { return _maxDistance; }

void CustomROSCommsDevice::SetMinDistance(double d) { _minDistance = d; }

double CustomROSCommsDevice::GetMinDistance() { return _minDistance; }

void CustomROSCommsDevice::SetIntrinsicDelay(double d) { _intrinsicDelay = d; }

double CustomROSCommsDevice::GetIntrinsicDelay() { return _intrinsicDelay; }

inline void
CustomROSCommsDevice::EnqueueTxPacket(const OutcomingPacketPtr &opkt) {
  uint32_t leftSpace = _maxTxFifoSize - _currentTxFifoSize;
  if (leftSpace >= opkt->packetSize) {
    _outcomingPackets.push_back(opkt);
    _currentTxFifoSize += opkt->packetSize;
  } else {
    // Drop packet
    _txPacketDrops += 1;
    Warn("{} Outcoming packet dropped! Tx Fifo size: {}. Packet Drops: {}",
         GetDccommsId(), _currentTxFifoSize, _txPacketDrops);
  }
}
OutcomingPacketPtr CustomROSCommsDevice::PopTxPacket() {
  auto opkt = _outcomingPackets.front();
  _outcomingPackets.pop_front();
  _currentTxFifoSize -= opkt->packetSize;
  return opkt;
}
inline bool CustomROSCommsDevice::TxFifoEmpty() {
  return _outcomingPackets.empty();
}

void CustomROSCommsDevice::MarkIncommingPacketsAsCollisioned() {
  // Mark all packets invalid (collisioned)
  Debug("CustomROSCommsDevice({}): MarkIncommingPacketsAsCollisioned",
        GetDccommsId());
  for (auto ipkt : _incomingPackets) {
    ipkt->collisionError = true;
  }
}

void CustomROSCommsDevice::ReceiveOldestPacketAfterJitter() {
  auto ipkt = _rxJitteredPackets.front();
  _rxJitteredPackets.pop_front();
  if (ipkt->Error()) {
    Debug("Packet received with errors");
    _pktErrorCbTrace(this, ipkt->packet, ipkt->propagationError,
                     ipkt->collisionError);
  } else {
    ReceiveFrame(ipkt->packet);
  }
}

void CustomROSCommsDevice::ReceivePacketAfterJitter(
    const IncomingPacketPtr &ipkt) {
  if (ipkt->Error()) {
    Debug("Packet received with errors");
    _pktErrorCbTrace(this, ipkt->packet, ipkt->propagationError,
                     ipkt->collisionError);
  } else {
    if (_enableMacLayer) {
      auto pkt = ipkt->packet;
      NetsimHeader header;
      pkt->RemoveHeader(header);
      AquaSimHeader ash;
      pkt->RemoveHeader(ash);
      ash.SetDirection(AquaSimHeader::UP);
      pkt->AddHeader(ash);
      _macRxCbTrace(this, pkt);
      _macLayer->RecvProcess(pkt);

    } else {
      ReceiveFrame(ipkt->packet);
    }
  }
}

uint64_t CustomROSCommsDevice::GetNextPacketReceptionTime() {
  return _nextPacketReceptionTime;
}

void CustomROSCommsDevice::SetNextPacketReceptionTime(uint64_t nanos) {
  _nextPacketReceptionTime = nanos;
}

uint64_t CustomROSCommsDevice::GetCurrentSimTime() {
  return NetsimTime::GetNanos();
}

void CustomROSCommsDevice::CheckOrderOfReceptions() {
  //    std::size_t cont = _rxJitteredPackets.size();
  //    for(std::size_t i = 0; i < cont; i++)
  //    {
  //        auto asd = _rxJitteredPackets[0];
  //    }
}

void CustomROSCommsDevice::HandleNextIncomingPacket() {
  NS_LOG_FUNCTION(this);
  Debug("CustomROSCommsDevice({}): HandleNextIncommingPacket", GetDccommsId());
  if (!_incomingPackets.empty()) {
    IncomingPacketPtr ptr = _incomingPackets.front();
    _incomingPackets.pop_front();
    //    uint64_t jitter = 0; //GetNextRxNormalJitter();                   //
    //    nanos uint64_t nextPacketReception = GetNextPacketReceptionTime(); //
    //    nanos uint64_t currentNanos = GetCurrentSimTime();                 //
    //    nanos uint64_t jitteredReception = currentNanos + jitter; if
    //    (nextPacketReception > currentNanos &&
    //        jitteredReception <= nextPacketReception) {
    //      jitteredReception = nextPacketReception + 1000000;
    //      jitter = jitteredReception - currentNanos;
    //    }

    //    SetNextPacketReceptionTime(jitteredReception);

    //    ns3::EventImpl *event = ns3::MakeEvent(
    //        &CustomROSCommsDevice::ReceivePacketAfterJitter, this, ptr);

    //    ns3::Simulator::ScheduleWithContext(
    //        GetMac(), ns3::NanoSeconds(static_cast<uint64_t>(jitter)), event);

    if (_incomingPackets.empty())
      Receiving(false);

    ReceivePacketAfterJitter(ptr);
  } else {
    Critical("internal error: incomming packets queue empty when "
             "HandleNextIncommingPacket!");
  }
}

void CustomROSCommsDevice::AddNewPacket(ns3PacketPtr pkt,
                                        bool propagationError) {
  NS_LOG_FUNCTION(this);
  Debug("CustomROSCommsDevice({}): AddNewPacket", GetDccommsId());
  IncomingPacketPtr ipkt = dccomms::CreateObject<IncomingPacket>();
  ipkt->propagationError = propagationError;
  NetsimHeader header;
  pkt->PeekHeader(header);
  ipkt->packet = pkt;
  _incomingPackets.push_back(ipkt);
  // TODO: check if propagation error and increase traced value
  if (Receiving() || HalfDuplex() && Transmitting()) {
    // TODO: increase colission errors traced value
    MarkIncommingPacketsAsCollisioned(); // Should be a maximum of 1 packet in
                                         // the _incommingPackets queue
  }
  Receiving(true);
  auto pktSize = header.GetPacketSize();
  auto byteTrt = header.GetNanosPerByte();
  auto trTime = pktSize * byteTrt;
  Debug("CustomROSCommsDevice({}): Receiving packet: size({} bytes) ; "
        "rcTime({} "
        "secs)",
        GetDccommsId(), pktSize, trTime / 1e9);
  ns3::Simulator::ScheduleWithContext(
      GetMac(), ns3::NanoSeconds(trTime),
      &CustomROSCommsDevice::HandleNextIncomingPacket, this);
}

void CustomROSCommsDevice::DoSetMaxTxFifoSize(uint32_t size) {
  _maxTxFifoSize = size;
}

bool CustomROSCommsDevice::Transmitting() { return _transmitting; }

void CustomROSCommsDevice::Transmitting(bool transmitting) {
  Debug("CustomROSCommsDevice({}): Setting transmitting status: {}",
        GetDccommsId(), transmitting);
  _transmitting = transmitting;
  if (!_transmitting && !TxFifoEmpty()) {
    TransmitEnqueuedPacket();
  }
}

void CustomROSCommsDevice::_TransmitEnqueuedPacket() {
  Debug("Transmit next packet in fifo");
  auto pkt = PopTxPacket();
  StartPacketTransmission(pkt);
}

void CustomROSCommsDevice::TransmitEnqueuedPacket() {
  Transmitting(true);
  ns3::Simulator::ScheduleWithContext(
      GetMac(), ns3::MicroSeconds(GetFixedIPGMicros()),
      &CustomROSCommsDevice::_TransmitEnqueuedPacket, this);
}

bool CustomROSCommsDevice::Receiving() { return _receiving; }

void CustomROSCommsDevice::Receiving(bool receiving) {
  _receiving = receiving;
  NS_LOG_DEBUG("Receiving: " << receiving ? "TRUE" : "FALSE");
  Debug("CustomROSCommsDevice({}): Setting receiving state to {}",
        GetDccommsId(), receiving);
  if (!_receiving && !_transmitting && !TxFifoEmpty())
    TransmitEnqueuedPacket();
}

void CustomROSCommsDevice::PropagatePacket(const OutcomingPacketPtr &pkt) {
  static_cast<CustomCommsChannel *>(ns3::PeekPointer(_txChannel))
      ->SendPacket(this, pkt);
}

void CustomROSCommsDevice::TransmitPacket() {
  NS_LOG_FUNCTION(this);
  Debug("CustomROSCommsDevice: Transmit packet");
  auto opkt = _txJitteredPackets.front();
  _txJitteredPackets.pop_front();
  StartPacketTransmission(opkt);
}

uint64_t CustomROSCommsDevice::GetNextTxJitter() {
  int tmp = _txJitterDist(_txJitterGenerator);
  return static_cast<uint64_t>(tmp) * 1000;
}
uint64_t CustomROSCommsDevice::GetNextRxJitter() {
  int tmp = _rxJitterDist(_rxJitterGenerator);
  return static_cast<uint64_t>(tmp) * 1000;
}

uint64_t CustomROSCommsDevice::GetNextRxNormalJitter() {
  double tmp = _rxJitterNormalDist(_rxJitterGenerator) * 1000;
  // bool positive = _rxJitterBernoulliDist(_rxJitterGenerator2);
  int64_t jitter;
  if (_neg)
    jitter = static_cast<int64_t>(tmp);
  else
    jitter = -1 * static_cast<int64_t>(tmp);
  auto res = _rxJitterBase + jitter;
  if (res < 0)
    res = 0;
  _neg = !_neg;
  return static_cast<uint64_t>(res);
}

void CustomROSCommsDevice::StartPacketTransmission(
    const OutcomingPacketPtr &opkt) {

  auto byteTrt = GetNanosPerByte();
  auto trTime = opkt->packetSize * byteTrt;
  Debug("CustomROSCommsDevice({}): Transmitting packet: size({} bytes) ; "
        "trTime({} secs)",
        GetDccommsId(), opkt->packetSize, trTime / 1e9);
  ns3::Simulator::ScheduleWithContext(GetMac(), ns3::NanoSeconds(trTime),
                                      &CustomROSCommsDevice::SetTransmitting,
                                      this, false);
  PropagatePacket(opkt);
}

void CustomROSCommsDevice::PhySend(ns3PacketPtr dlf) {
  if (!Transmitting() &&
      (_rxChannel != _txChannel || _rxChannel == _txChannel && !Receiving())) {
    Transmitting(true);
    ns3::Simulator::ScheduleWithContext(
        GetMac(), ns3::NanoSeconds(_intrinsicDelay * 1e6),
        &CustomROSCommsDevice::SchedulePacketTransmissionAfterJitter, this,
        dlf);
  } else {
    Debug("CustomROSCommsDevice({}): Enqueue packet", GetDccommsId());
    OutcomingPacketPtr opkt = dccomms::CreateObject<OutcomingPacket>();
    NetsimHeader header;
    dlf->RemoveHeader(header);
    header.SetNanosPerByte(GetNanosPerByte());
    dlf->AddHeader(header);
    opkt->packet = dlf;
    opkt->packetSize = header.GetPacketSize();
    EnqueueTxPacket(opkt);
  }
}

void CustomROSCommsDevice::DoSetMac(uint32_t mac) { _mac = mac; }
void CustomROSCommsDevice::DoSend(ns3PacketPtr dlf) {
  while (!_started) {
    this_thread::sleep_for(chrono::milliseconds(500));
  }
  if (_enableMacLayer) {
    NetsimHeader header;
    dlf->RemoveHeader(header);
    auto pktSize = header.GetPacketSize();
    auto saddr = AquaSimAddress::ConvertFrom(_dev->GetAddress());
    auto daddr = AquaSimAddress(static_cast<uint16_t>(header.GetDst()));
    uint32_t seq = static_cast<uint32_t>(header.GetSeqNum());

    ns3::AquaSimHeader ash;
    ash.SetSAddr(saddr);
    ash.SetDAddr(daddr);
    ash.SetSize(static_cast<uint16_t>(pktSize));
    ash.SetSeqNum(seq);
    ash.SetDirection(AquaSimHeader::DOWN);
    dlf->AddHeader(ash);

    ns3::Simulator::ScheduleWithContext(GetMac(), ns3::NanoSeconds(0),
                                        &ns3::AquaSimNetDevice::SendWithHeader,
                                        _dev, dlf, 0);
  } else {
    PhySend(dlf);
  }
}

void CustomROSCommsDevice::SchedulePacketTransmissionAfterJitter(
    const ns3PacketPtr &pkt) {
  NetsimHeader header;
  pkt->RemoveHeader(header);
  header.SetNanosPerByte(GetNanosPerByte());
  pkt->AddHeader(header);
  auto pktSize = header.GetPacketSize();
  uint64_t jitter = 0; // GetNextTxJitter();
  OutcomingPacketPtr opkt = dccomms::CreateObject<OutcomingPacket>();
  opkt->packet = pkt;
  opkt->packetSize = pktSize;
  _txJitteredPackets.push_back(opkt);
  ns3::Simulator::ScheduleWithContext(GetMac(), ns3::NanoSeconds(jitter),
                                      &CustomROSCommsDevice::TransmitPacket,
                                      this);
}

void CustomROSCommsDevice::DoLinkToChannel(CommsChannelNs3Ptr channel,
                                           CHANNEL_LINK_TYPE linkType) {
  if (channel->GetType() == CHANNEL_TYPE::CUSTOM_CHANNEL) {
    //_channel = static_pointer_cast<CustomCommsChannel>(channel);
    if (linkType == CHANNEL_TX) {
      _txChannel = channel;
      double propSpeed = _txChannel->GetPropSpeed();
      _dev->SetPropSpeed(propSpeed);
    } else if (linkType == CHANNEL_RX) {
      _rxChannel = channel;
      static_cast<CustomCommsChannel *>(ns3::PeekPointer(_rxChannel))
          ->AddDevice(this);
    } else if (linkType == CHANNEL_TXRX) {
      _txChannel = channel;
      _rxChannel = channel;
      double propSpeed = _txChannel->GetPropSpeed();
      _dev->SetPropSpeed(propSpeed);
      static_cast<CustomCommsChannel *>(ns3::PeekPointer(_rxChannel))
          ->AddDevice(this);
    }

  } else {
    Log->critical(
        "internal error: attempted to link device to a wrong channel type");
  }
}

void CustomROSCommsDevice::SetMacLayer(ns3::Ptr<AquaSimMac> mac) {
  _macLayer = mac;
  _macLayer->SetDevice(_dev);
  _routingLayer->SetMac(_macLayer);
  _dev->SetMac(_macLayer);
}

void CustomROSCommsDevice::EnableMac(bool v) { _enableMacLayer = v; }

void CustomROSCommsDevice::DoSetMacMaxTransmitDistance(double v) {
  _phy->SetTransRange(v);
}
void CustomROSCommsDevice::DoStart() {
  auto aqmac = ns3::AquaSimAddress(static_cast<uint16_t>(_mac));

  _phy->SetTransRange(GetMaxDistance());

  if (_enableMacLayer) {
    _dev->SetAddress(aqmac);
    _dev->MacEnabled(_enableMacLayer);
    _phy->SetTransRange(_macMaxTransmitDistance);

    ns3::Config::Connect(
        "/NodeList/" + std::to_string(_nodeListIndex) +
            "/DeviceList/0/Mac/RoutingRx",
        MakeCallback(&CustomROSCommsDevice::_RoutingRxCb, this));

    ns3::Config::Connect("/NodeList/" + std::to_string(_nodeListIndex) +
                             "/DeviceList/0/Mac/MacTx",
                         MakeCallback(&CustomROSCommsDevice::_MacTxCb, this));
  }
  _started = true;
}
bool CustomROSCommsDevice::DoStarted() { return _started; }
void CustomROSCommsDevice::DoSetPosition(const tf::Vector3 &position) {
  _position = position;
}

std::string CustomROSCommsDevice::DoToString() {
  int maxBuffSize = 2048;
  char buff[maxBuffSize];
  string txChannelLinked;
  if (_txChannel)
    txChannelLinked = "Type: " + ChannelType2String(_txChannel->GetType()) +
                      " ; Id: " + to_string(_txChannel->GetId());
  else
    txChannelLinked = "not linked";

  string rxChannelLinked;
  if (_rxChannel)
    rxChannelLinked = "Type: " + ChannelType2String(_rxChannel->GetType()) +
                      " ; Id: " + to_string(_rxChannel->GetId());
  else
    rxChannelLinked = "not linked";

  double bitrateD;
  GetBitRate(bitrateD);
  uint32_t bitrate = (uint32_t)bitrateD;
  std::string expr, eunit;
  GetRateErrorModel(expr, eunit);

  int n;
  n = snprintf(buff, maxBuffSize,
               "\tdccomms ID: ............... '%s'\n"
               "\tMAC ....................... %d\n"
               "\tDevice type ............... %s\n"
               "\tFrame ID: ................. '%s'\n"
               "\tTX channel: ............... '%s'\n"
               "\tRX channel: ............... '%s'\n"
               "\tMax. distance: ............ %.2f m\n"
               "\tMin. distance: ............ %.2f m\n"
               "\tBitrate: .................. %d bps\n"
               "\tIntrinsic delay (ms)....... %.3f\n"
               "\tJitter\n"
               "\t\ttx (ms): ................ %.3f\n"
               "\t\trx (ms): ................ %.3f\n"
               "\tTx Fifo Size: ............. %d bytes\n"
               "\tError Expression: ......... %s\n"
               "\tError Unit: ............... %s",
               _name.c_str(), _mac, DevType2String(GetDevType()).c_str(),
               _tfFrameId.c_str(), txChannelLinked.c_str(),
               rxChannelLinked.c_str(), _maxDistance, _minDistance, bitrate,
               _intrinsicDelay, _txJitter, _rxJitter, GetMaxTxFifoSize(),
               expr.c_str(), eunit.c_str());

  return std::string(buff);
}
} // namespace dccomms_ros
