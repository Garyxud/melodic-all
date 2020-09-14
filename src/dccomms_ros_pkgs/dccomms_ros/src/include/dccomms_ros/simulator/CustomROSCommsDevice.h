#ifndef DCCOMMS_ROS_CUSTOMROSCOMMSDEVICE_H_
#define DCCOMMS_ROS_CUSTOMROSCOMMSDEVICE_H_

#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <exprtk.hpp>
#include <ns3/aqua-sim-mac.h>
#include <ns3/error-model.h>
#include <random>

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros {

// enum DEV_STATUS { RECV, SEND, IDLE };
class NetsimDevice;
class NetsimPhy;
class NetsimRouting;
class CustomROSCommsDevice;

typedef ns3::Ptr<CustomROSCommsDevice> CustomROSCommsDeviceNs3Ptr;
// typedef dccomms::Ptr<CustomROSCommsDevice> CustomROSCommsDevicePtr;
typedef CustomROSCommsDevice *CustomROSCommsDevicePtr;

class IncomingPacket;
typedef dccomms::Ptr<IncomingPacket> IncomingPacketPtr;
class IncomingPacket {
public:
  bool propagationError;
  bool collisionError;
  ns3PacketPtr packet;
  ns3::EventImpl *event;
  uint64_t jitter, receptionTime;
  IncomingPacket() {
    propagationError = false;
    collisionError = false;
    event = NULL;
    jitter = 0;
    receptionTime = 0;
    packet = NULL;
  }
  bool Error() { return propagationError || collisionError; }
};

class OutcomingPacket;
typedef dccomms::Ptr<OutcomingPacket> OutcomingPacketPtr;
class OutcomingPacket {
public:
  uint32_t packetSize;
  ns3PacketPtr packet;
  OutcomingPacket() { packet = NULL; }
};

class SimpleVarExprEval {
public:
  typedef exprtk::symbol_table<double> symbol_table_t;
  typedef exprtk::expression<double> expression_t;
  typedef exprtk::parser<double> parser_t;

  SimpleVarExprEval();
  void CompileExpr(const std::string &expr, const std::string &var);
  double ComputeVal(double var);

private:
  std::string _sexpr;
  symbol_table_t _symbol_table;
  expression_t _expression;
  double _var;
  parser_t _parser;
};

class CustomROSCommsDevice : public ROSCommsDevice {
public:
  CustomROSCommsDevice(ROSCommsSimulatorPtr, PacketBuilderPtr txpb,
                       PacketBuilderPtr rxpb);

  static CustomROSCommsDeviceNs3Ptr Build(ROSCommsSimulatorPtr sim,
                                          PacketBuilderPtr txpb,
                                          PacketBuilderPtr rxpb) {
    auto dev = ns3::CreateObject<CustomROSCommsDevice>(sim, txpb, rxpb);
    // auto dev = dccomms::CreateObject<CustomROSCommsDevice>(sim, txpb, rxpb);
    return dev;
    // return new CustomROSCommsDevice(sim, txpb, rxpb);
  }
  void SetJitter(double tx, double rx);
  void SetMinPktErrorRate(double minPktErrorRate);
  void SetPktErrorRateInc(double pktErrorRateInc);
  void SetMaxDistance(double d);
  void SetMinDistance(double d);
  void SetIntrinsicDelay(double d);

  void GetBitRate(double &bps); // as bps
  double GetMinPktErrorRate();
  double GetPktErrorRateInc();
  double GetMaxDistance();
  double GetMinDistance();
  double GetIntrinsicDelay();
  uint64_t GetNextPacketReceptionTime();
  void SetNextPacketReceptionTime(uint64_t nanos);
  uint64_t GetCurrentSimTime();

  void PropagatePacket(const OutcomingPacketPtr & pkt);
  void TransmitPacket();
  void TransmitEnqueuedPacket();
  void _TransmitEnqueuedPacket();
  void StartPacketTransmission(const OutcomingPacketPtr &opkt);

  inline bool HalfDuplex() { return _txChannel == _rxChannel; }

  inline void SetTransmitting(bool v) { Transmitting(v); }
  bool ErrOnPkt(double range, const uint32_t & size);
  uint64_t GetNextTxJitter();
  uint64_t GetNextRxJitter();
  uint64_t GetNextRxNormalJitter();
  void SetRateErrorModel(const std::string &expr, const std::string &unit);
  void GetRateErrorModel(std::string &expr, std::string &unit);
  //  inline DEV_STATUS GetStatus() { return _status; }
  //  inline void SetStatus(DEV_STATUS status);

  virtual DEV_TYPE GetDevType();

  typedef std::normal_distribution<double> NormalRealDist;
  typedef std::normal_distribution<int> NormalIntDist;
  typedef std::uniform_int_distribution<int> UniformIntDist;
  typedef std::default_random_engine RandEngGen;
  typedef std::uniform_real_distribution<double> UniformRealDist;

  inline void EnqueueTxPacket(const OutcomingPacketPtr &opkt);
  inline bool TxFifoEmpty();
  OutcomingPacketPtr PopTxPacket();
  bool _neg;

  void AddNewPacket(ns3PacketPtr pkt, bool propagationError);
  void HandleNextIncomingPacket();

  void MarkIncommingPacketsAsCollisioned();
  bool Transmitting();
  void Transmitting(bool);

  bool Receiving();
  void Receiving(bool);

  static ns3::TypeId GetTypeId(void);
  void SchedulePacketTransmissionAfterJitter(const ns3PacketPtr &pkt);
  void ReceiveOldestPacketAfterJitter();
  void ReceivePacketAfterJitter(const IncomingPacketPtr &pkt);
  void CheckOrderOfReceptions();
  void PhySend(ns3PacketPtr dlf);
  void EnableMac(bool v);
  void SetMacLayer(ns3::Ptr<ns3::AquaSimMac> mac);

  inline uint32_t GetFixedIPGMicros() { return _fixedIPGmicros; }
  inline uint32_t GetFixedIPGNanos() { return _fixedIPGNanos; }

protected:
  virtual void DoSetMac(uint32_t mac);
  virtual void DoSend(ns3PacketPtr dlf);
  virtual void DoLinkToChannel(CommsChannelNs3Ptr channel,
                               CHANNEL_LINK_TYPE linkType);
  virtual void DoStart();
  virtual void DoSetPosition(const tf::Vector3 &position);
  virtual bool DoStarted();
  virtual void DoSetMaxTxFifoSize(uint32_t size);
  virtual void DoSetMacMaxTransmitDistance(double v);
  std::string DoToString();

private:
  uint32_t _fixedIPGmicros = 100;
  uint32_t _fixedIPGNanos = 100000;
  uint32_t _mac;
  double _minPktErrorRate, _pktErrorRateIncPerMeter;
  double _intrinsicDelay; // ms

  double _maxDistance, _minDistance; // in meters
  tf::Vector3 _position;
  // CustomROSCommsDevicePtr _ownPtr;

  NormalRealDist _ttDist;
  NormalRealDist _rxJitterNormalDist;
  UniformRealDist _erDist;
  double _rxJitterSd;
  UniformIntDist _txJitterDist, _rxJitterDist;

  std::bernoulli_distribution _rxJitterBernoulliDist;
  RandEngGen _ttGenerator, _erGenerator, _txJitterGenerator, _rxJitterGenerator,
      _rxJitterGenerator2;

  std::list<IncomingPacketPtr> _incomingPackets, _rxJitteredPackets;
  std::list<OutcomingPacketPtr> _outcomingPackets, _txJitteredPackets;

  CommsChannelNs3Ptr _txChannel, _rxChannel;
  // DEV_STATUS _status;
  bool _transmitting, _receiving;
  ns3::Ptr<ns3::RateErrorModel> _rem;

  double _GetErrorRate(double meters);
  std::string _eexpr;
  SimpleVarExprEval _mExprEval;
  double _txJitter, _rxJitter;
  int64_t _rxJitterBase;
  uint64_t _nextPacketReceptionTime;
  ns3::Ptr<NetsimDevice> _dev;
  ns3::Ptr<NetsimPhy> _phy;
  ns3::Ptr<ns3::AquaSimMac> _macLayer;
  ns3::Ptr<NetsimRouting> _routingLayer;
  bool _enableMacLayer;
  bool _started;
};
} // namespace dccomms_ros
#endif // COMMSNODE_H
