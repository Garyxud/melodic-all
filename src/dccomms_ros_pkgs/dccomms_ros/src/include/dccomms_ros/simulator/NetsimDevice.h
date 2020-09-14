#ifndef DCCOMMS_ROS_NETSIMDEVICE_H_
#define DCCOMMS_ROS_NETSIMDEVICE_H_

#include <cpplogging/Logger.h>
#include <ns3/aqua-sim-header.h>
#include <ns3/aqua-sim-net-device.h>
#include <ns3/core-module.h>
#include <ns3/node-list.h>

using namespace cpplogging;
using namespace ns3;

namespace dccomms_ros {

class CustomROSCommsDevice;

class NetsimDevice : public virtual Logger, public ns3::AquaSimNetDevice {
public:
  NetsimDevice(ns3::Ptr<CustomROSCommsDevice>);
  ~NetsimDevice();
  virtual TransStatus GetTransmissionStatus() override;
  virtual void SetTransmissionStatus(TransStatus status) override;
  virtual double GetPropSpeed(void) override;
  virtual void SetPropSpeed(const double &speed);

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static ns3::TypeId GetTypeId(void);

private:
  ns3::Ptr<CustomROSCommsDevice> _dev;
  double _propSpeed;
protected:
};
} // namespace ns3
#endif // DCCOMMS_ROS_NETSIMDEVICE_H_
