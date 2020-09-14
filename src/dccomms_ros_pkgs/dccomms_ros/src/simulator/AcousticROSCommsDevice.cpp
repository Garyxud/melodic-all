#include <dccomms_ros/simulator/AcousticCommsChannel.h>
#include <dccomms_ros/simulator/AcousticROSCommsDevice.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>

using namespace ns3;

namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(AcousticROSCommsDevice);

TypeId AcousticROSCommsDevice::GetTypeId(void) {
  static TypeId tid =
      TypeId("dccomms_ros::AcousticROSCommsDevice").SetParent<ROSCommsDevice>();

  return tid;
}

AcousticROSCommsDevice::AcousticROSCommsDevice(ROSCommsSimulatorPtr s,
                                               PacketBuilderPtr txpb,
                                               PacketBuilderPtr rxpb)
    : ROSCommsDevice(s, txpb, rxpb) {
  _started = false;

  _asHelper = ns3::AquaSimHelper::Default();
  _routingType = AQS_NOROUTING;
  _device = ns3::CreateObject<ns3::AquaSimNetDevice>();
  _pT = 0.2818;
  _freq = 25;
  _L = 0;
  _K = 2.0;
  _turnOnEnergy = 0;
  _turnOffEnergy = 0;
  _preamble = 0;
  _pTConsume = 0.660;
  _pRConsume = 0.395;
  _pIdle = 0.0;
}

void AcousticROSCommsDevice::_SendTrace(string context,
                                        ns3::Ptr<const ns3::Packet> pkt) {
  std::string datetime;
  double secs;
  _sim->GetSimTime(datetime, secs);

  AquaSimHeader ash;
  pkt->PeekHeader(ash);
  auto saddr = ash.GetSAddr().GetAsInt();
  auto daddr = ash.GetDAddr().GetAsInt();
  auto nhaddr = ash.GetNextHop().GetAsInt();

  Debug(
      "({} secs; {}) {}: (Addr: {}) Transmitting packet to {}. Next hop: {} ; "
      "{} bytes",
      secs, datetime, context, saddr, daddr, nhaddr, pkt->GetSize());
  FlushLog();
}

void AcousticROSCommsDevice::_Recv(std::string context,
                                   ns3::Ptr<const ns3::Packet> pkt) {
  std::string datetime;
  double secs;
  _sim->GetSimTime(datetime, secs);

  auto packet = pkt->Copy();
  ns3::AquaSimHeader ash;

  packet->RemoveHeader(ash);
  auto saddr = ash.GetSAddr().GetAsInt();
  auto daddr = ash.GetDAddr().GetAsInt();
  auto psize = packet->GetSize();

  switch (_routingType) {
  case AQS_NOROUTING: {
    Debug("Packet received ({} bytes ; {} bytes ; {} bytes) , S:{} ; D:{}",
          psize, packet->GetSize(), ash.GetSize(), saddr, daddr);
    Debug(
        "({} secs; {}) {}: (Own Addr: {} Dest. Addr: {}) Received packet from "
        "{} ({} bytes)",
        secs, datetime, context, GetMac(), daddr, saddr, psize);
    FlushLog();
    break;
  }
  case AQS_ROUTING_DUMMY: {
    auto numForwards = ash.GetNumForwards();
    while (ash.GetNumForwards() > 0) {
      packet->RemoveHeader(ash);
    }
    Debug(
        "({} secs; {}) {}: (Own Addr: {} Dest. Addr: {}) Received packet from "
        "{} ({} forwards) ({} bytes)",
        secs, datetime, context, GetMac(), daddr, saddr, numForwards, psize);
    FlushLog();
    break;
  }
  case AQS_ROUTING_VBF: {
    break;
  default:
    break;
  }
  }
  NetsimHeader header;
  header.SetPacketError(ash.GetErrorFlag());
  header.SetPacketSize(psize);
  header.SetSrc(saddr);
  header.SetDst(daddr);
  header.SetSeqNum(0);
  packet->AddHeader(header);
  ReceiveFrame(packet);
}

void AcousticROSCommsDevice::_RxError(std::string context,
                                      ns3::Ptr<const ns3::Packet> pkt) {
  Warn("Packet received with errors!");
  // TODO: send the packet to de upper layer despite the errors
}

DEV_TYPE AcousticROSCommsDevice::GetDevType() {
  return DEV_TYPE::ACOUSTIC_UNDERWATER_DEV;
}

void AcousticROSCommsDevice::DoSetMac(uint32_t mac) {
  _aquaSimAddr = ns3::AquaSimAddress(static_cast<uint16_t>(mac));
}

void AcousticROSCommsDevice::DoSend(ns3PacketPtr ns3pkt) {
  while (!_started) {
    this_thread::sleep_for(chrono::milliseconds(500));
  }

  NetsimHeader header;
  ns3pkt->RemoveHeader(header);
  uint16_t daddr = header.GetDst();
  switch (_routingType) {
  case AQS_NOROUTING: {
    ns3::Simulator::ScheduleWithContext(GetMac(), Seconds(0),
                                        &ns3::AquaSimNetDevice::Send, _device,
                                        ns3pkt, AquaSimAddress(daddr), 0);
    break;
  }
  case AQS_ROUTING_DUMMY: {
    //    ns3::AquaSimHeader ash;
    //    ash.SetSize(pkt->GetPacketSize());
    //    ash.SetNumForwards(0);
    //    ns3pkt->AddHeader(ash);
    //    ns3::Simulator::ScheduleWithContext(GetMac(), Seconds(0),
    //                                        &ns3::AquaSimNetDevice::Send,
    //                                        _device,
    //                                        ns3pkt, AquaSimAddress(daddr), 0);
    break;
  }
  case AQS_ROUTING_VBF: {
    break;
  default:
    break;
  }
  }
}

void AcousticROSCommsDevice::DoLinkToChannel(CommsChannelNs3Ptr channel,
                                             CHANNEL_LINK_TYPE linkType) {
  if (channel->GetType() == CHANNEL_TYPE::ACOUSTIC_UNDERWATER_CHANNEL) {

    AcousticCommsChannel *acChannel =
        static_cast<AcousticCommsChannel *>(ns3::PeekPointer(channel));
    _channel = acChannel->GetAquaSimChannel();
  } else {
    Log->critical(
        "internal error: attempted to link device to a wrong channel type");
  }
}

void AcousticROSCommsDevice::DoSetPosition(const tf::Vector3 &position) {
  if (_started) {
    double x = position.getX(), y = position.getY(), z = position.getZ();
    ns3::Simulator::ScheduleWithContext(GetMac(), Seconds(0),
                                        &ns3::MobilityModel::SetPosition,
                                        _mobility, ns3::Vector3D(x, y, z));
  }
}

void AcousticROSCommsDevice::_PositionUpdated(
    std::string context, ns3::Ptr<const MobilityModel> model) {
  Vector position = model->GetPosition();

  std::string datetime;
  double secs;
  _sim->GetSimTime(datetime, secs);

  Info("({} secs; {}) {}: [x,y,z] = [{},{},{}]", secs, datetime, context,
       position.x, position.y, position.z);
}

void AcousticROSCommsDevice::SetMACProtocol(const std::string &name) {
  _macP = GetMACPType(name);
}
void AcousticROSCommsDevice::SetRange(double value) { _range = value; }
void AcousticROSCommsDevice::SetPT(double value) { _pT = value; }
void AcousticROSCommsDevice::SetFreq(double value) { _freq = value; }
void AcousticROSCommsDevice::SetL(double value) { _L = value; }
void AcousticROSCommsDevice::SetK(double value) { _K = value; }
void AcousticROSCommsDevice::SetTurnOnEnergy(double value) {
  _turnOnEnergy = value;
}
void AcousticROSCommsDevice::SetTurnOffEnergy(double value) {
  _turnOffEnergy = value;
}
void AcousticROSCommsDevice::SetPreamble(double value) { _preamble = value; }
void AcousticROSCommsDevice::SetTxPower(double value) { _pTConsume = value; }
void AcousticROSCommsDevice::SetRxPower(double value) { _pRConsume = value; }
void AcousticROSCommsDevice::SetIdlePower(double value) { _pIdle = value; }

void AcousticROSCommsDevice::SetSymbolsPerSecond(uint32_t value) {
  _symbPerSec = value;
}
void AcousticROSCommsDevice::SetCodingEff(double value) { _codingEff = value; }
void AcousticROSCommsDevice::SetBitErrorRate(uint32_t value) {
  _bitErrorRate = value;
}

void AcousticROSCommsDevice::SetInitialEnergy(double value) {
  _initialEnergy = value;
}

void AcousticROSCommsDevice::DoSetMaxTxFifoSize(uint32_t size) {}

void AcousticROSCommsDevice::DoStart() {

  if (_macP != "")
    _asHelper.SetMac(_macP);
  _asHelper.SetChannel(_channel);

  if (_routingType == AQS_NOROUTING)
    _asHelper.CreateWithoutRouting(_node, _device);
  else
    _asHelper.Create(_node, _device);
  _device->SetAddress(_aquaSimAddr);
  _macLayer = _device->GetMac();
  _mobility = _node->GetObject<ns3::MobilityModel>();

  auto phy = _device->GetPhy();
  phy->SetTransRange(_range);
  phy->SetAttribute("PT", ns3::DoubleValue(_pT));
  phy->SetAttribute("Frequency", ns3::DoubleValue(_freq));
  phy->SetAttribute("L", ns3::DoubleValue(_L));
  phy->SetAttribute("K", ns3::DoubleValue(_K));
  phy->SetAttribute("TurnOnEnergy", ns3::DoubleValue(_turnOnEnergy));
  phy->SetAttribute("TurnOffEnergy", ns3::DoubleValue(_turnOffEnergy));
  phy->SetAttribute("Preamble", ns3::DoubleValue(_preamble));
  auto em = phy->EM();
  em->SetAttribute("InitialEnergy", ns3::DoubleValue(_initialEnergy));
  em->SetAttribute("RxPower", ns3::DoubleValue(_pRConsume));
  em->SetAttribute("TxPower", ns3::DoubleValue(_pTConsume));
  em->SetAttribute("IdlePower", ns3::DoubleValue(_pIdle));
  ns3::Ptr<AquaSimModulation> modulation = phy->Modulation(NULL);
  modulation->SetAttribute("CodingEff", ns3::DoubleValue(_codingEff));
  modulation->SetAttribute("SPS", ns3::UintegerValue(_symbPerSec));
  modulation->SetAttribute("BER", ns3::DoubleValue(_bitErrorRate));

  _mobility->SetPosition(Vector3D(10 * _nodeListIndex, 0, 0));
  if (_routingType != AQS_NOROUTING) {
    _routingLayer = _device->GetRouting();
    ns3::Config::Connect("/NodeList/" + std::to_string(_nodeListIndex) +
                             "/DeviceList/0/Routing/PacketReceived",
                         MakeCallback(&AcousticROSCommsDevice::_Recv, this));
    ns3::Config::Connect(
        "/NodeList/" + std::to_string(_nodeListIndex) +
            "/DeviceList/0/Routing/PacketTransmitting",
        MakeCallback(&AcousticROSCommsDevice::_SendTrace, this));
  } else {
    if (_macP != "") {
      ns3::Config::Connect("/NodeList/" + std::to_string(_nodeListIndex) +
                               "/DeviceList/0/Mac/RoutingRx",
                           MakeCallback(&AcousticROSCommsDevice::_Recv, this));
      ns3::Config::Connect(
          "/NodeList/" + std::to_string(_nodeListIndex) +
              "/DeviceList/0/Mac/MacTx",
          MakeCallback(&AcousticROSCommsDevice::_SendTrace, this));
    } else {
      _device->MacEnabled(false);
      ns3::Config::Connect("/NodeList/" + std::to_string(_nodeListIndex) +
                               "/DeviceList/0/Phy/MacRx",
                           MakeCallback(&AcousticROSCommsDevice::_Recv, this));
      ns3::Config::Connect(
          "/NodeList/" + std::to_string(_nodeListIndex) +
              "/DeviceList/0/Phy/MacTx",
          MakeCallback(&AcousticROSCommsDevice::_SendTrace, this));
    }
  }
  _started = true;
}

bool AcousticROSCommsDevice::DoStarted() { return _started; }

std::string AcousticROSCommsDevice::DoToString() {

  AcousticCommsChannel *acousticChannel =
      static_cast<AcousticCommsChannel *>(ns3::PeekPointer(_txChannel));

  int maxBuffSize = 2048;
  char buff[maxBuffSize];
  char *txChannelLinked = buff;
  int n;
  if (_txChannel) {
    n = snprintf(txChannelLinked, maxBuffSize,
                 "Id: %d:\n"
                 "\t  Bandwidth: .............. %.01f Hz\n"
                 "\t  Temperature: ............ %.01f ºC\n"
                 "\t  Salinity: ............... %.02f ppt\n"
                 "\t  Noise Level: ............ %.01f dB",
                 acousticChannel->GetId(), acousticChannel->GetBandwidth(),
                 acousticChannel->GetTemperature(),
                 acousticChannel->GetSalinity(),
                 acousticChannel->GetNoiseLevel());

  } else {
    n = snprintf(txChannelLinked, maxBuffSize, "not linked");
  }

  char *ptr = txChannelLinked + n + 1;
  n = snprintf(ptr, maxBuffSize, "\tdccomms ID: ............... '%s'\n"
                                 "\tMAC ....................... %d\n"
                                 "\tDevice type ............... %s\n"
                                 "\tFrame ID: ................. '%s'\n"
                                 "\tChannel: .................. %s\n"
                                 "\tTx Fifo Size: ............. %d bytes\n"
                                 "\tMAC protocol: ............. %s\n"
                                 "\tMax. Range: ............... %.02f m\n"
                                 "\tPT: ....................... %.02f W\n"
                                 "\tFreq: ..................... %.02f KHz\n"
                                 "\tL: ........................ %.02f\n"
                                 "\tK: ........................ %.03f\n"
                                 "\tInitial energy: ........... %.02f J\n"
                                 "\tTurnOnEnergy: ............. %.02f J\n"
                                 "\tTurnOffEnergy: ............ %.02f J\n"
                                 "\tPreamble: ................. %.02f\n"
                                 "\tPTConsume: ................ %.02f W\n"
                                 "\tPRConsume: ................ %.02f W\n"
                                 "\tPIdle: .................... %.02f W\n"
                                 "\tSymbols per second: ....... %d symb/s\n"
                                 "\tBit error rate: ........... %.2f\n"
                                 "\tCoding efficiency: ........ %.1f\n",
               _name.c_str(), _mac, DevType2String(GetDevType()).c_str(),
               _tfFrameId.c_str(), txChannelLinked, GetMaxTxFifoSize(),
               _macP.c_str(), _range, _pT, _freq, _L, _K, _initialEnergy,
               _turnOnEnergy, _turnOffEnergy, _preamble, _pTConsume, _pRConsume,
               _pIdle, _symbPerSec, _bitErrorRate, _codingEff);

  std::string repr = (const char *)ptr;
  return repr;
}
}
