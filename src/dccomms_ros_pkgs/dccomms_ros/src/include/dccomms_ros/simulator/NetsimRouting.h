#ifndef DCCOMMS_ROS_NETSIMROUTING_H_
#define DCCOMMS_ROS_NETSIMROUTING_H_

#include <cpplogging/Logger.h>
#include <ns3/aqua-sim-header.h>
#include <ns3/aqua-sim-net-device.h>
#include <ns3/aqua-sim-phy-cmn.h>
#include <ns3/core-module.h>
#include <ns3/node-list.h>
using namespace cpplogging;
namespace dccomms_ros {

class CustomROSCommsDevice;

class NetsimRouting : public virtual Logger, public ns3::AquaSimRouting {
public:
  NetsimRouting(ns3::Ptr<CustomROSCommsDevice>);

  virtual bool Recv(ns3::Ptr<ns3::Packet> pkt, const ns3::Address &dest, uint16_t protocolNumber) override;
  virtual int64_t AssignStreams(int64_t stream) override { return 0; };
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static ns3::TypeId GetTypeId(void);

private:
  ns3::Ptr<CustomROSCommsDevice> _dev;

protected:
};
} // namespace dccomms_ros
#endif // DCCOMMS_ROS_NETSIMROUTING_H_
