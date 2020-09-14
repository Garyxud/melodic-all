#ifndef DCCOMMS_ROS_NETSIMPHY_H_
#define DCCOMMS_ROS_NETSIMPHY_H_

#include <cpplogging/Logger.h>
#include <ns3/aqua-sim-header.h>
#include <ns3/aqua-sim-net-device.h>
#include <ns3/aqua-sim-phy-cmn.h>
#include <ns3/core-module.h>
#include <ns3/node-list.h>
using namespace cpplogging;
namespace dccomms_ros {

class CustomROSCommsDevice;

class NetsimPhy : public virtual Logger, public ns3::AquaSimPhyCmn {
public:
  NetsimPhy(ns3::Ptr<CustomROSCommsDevice>);

  virtual bool Recv(ns3::Ptr<ns3::Packet> pkt) override;
  virtual ns3::Time CalcTxTime(uint32_t pktSize, std::string * modName) override;
  double CalcPktSize (double txTime, std::string * modName) override;
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static ns3::TypeId GetTypeId(void);

private:
  ns3::Ptr<CustomROSCommsDevice> _dev;
protected:
};
} // namespace ns3
#endif // DCCOMMS_ROS_NETSIMDEVICE_H_
