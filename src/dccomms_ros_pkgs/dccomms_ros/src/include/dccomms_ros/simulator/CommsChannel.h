#ifndef DCCOMMS_ROS_COMMS_CHANNEL_H_
#define DCCOMMS_ROS_COMMS_CHANNEL_H_

#include <dccomms/dccomms.h>
#include <dccomms_ros_msgs/types.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

namespace dccomms_ros {

class CommsChannel : public virtual cpplogging::Logger, public ns3::Object {
public:
  virtual uint32_t GetId() = 0;
  virtual CHANNEL_TYPE GetType() = 0;
  virtual double GetPropSpeed() = 0;
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static ns3::TypeId GetTypeId(void);
};

typedef ns3::Ptr<CommsChannel> CommsChannelNs3Ptr;
typedef CommsChannel *CommsChannelPtr;
}
#endif // COMMSCHANNELPROPERTIES_H
