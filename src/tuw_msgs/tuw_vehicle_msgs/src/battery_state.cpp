#include <tuw_vehicle_msgs/battery_state.h>

using namespace tuw::ros_msgs;

BatteryState::BatteryState(){

};

double BatteryState::GetLowestCellVoltage()
{
  bool lowestFound = false;
  double lowest = 0;
  for (double const &voltage : this->cell_voltages)
  {
    if (false == lowestFound || voltage < lowest)
    {
      lowest = voltage;
    }
  }
  return lowest;
}

double BatteryState::GetTotalVoltage()
{
  double sum = 0;
  for (double const &voltage : this->cell_voltages)
  {
    sum += voltage;
  }
  return sum;
}
