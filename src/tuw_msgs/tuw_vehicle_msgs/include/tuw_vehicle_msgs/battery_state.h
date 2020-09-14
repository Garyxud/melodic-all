#ifndef TUWR_MSGS_BATTERY_STATE_H
#define TUWR_MSGS_BATTERY_STATE_H

#include <tuw_vehicle_msgs/BatteryState.h>

namespace tuw
{
namespace ros_msgs
{
class BatteryState : public tuw_vehicle_msgs::BatteryState
{
public:
  BatteryState();
  double GetLowestCellVoltage();
  double GetTotalVoltage();
};
};
};
#endif  // TUWR_MSGS_BATTERY_STATE_H
