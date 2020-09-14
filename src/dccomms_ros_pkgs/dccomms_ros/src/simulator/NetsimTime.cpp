#include <dccomms_ros/simulator/NetsimTime.h>

using namespace std::chrono;
namespace dccomms_ros {

NetsimTime::netsim_time NetsimTime::_startSimTime;
}

/*
uint64_t GetMillisSinceEpoch();
uint64_t GetMicrosSinceEpoch();
uint64_t GetNanosSinceEpoch();
*/

/*
template <typename T> uint64_t GetTimeUnitsSinceEpoch() {
  time_point<std::chrono::system_clock, T> ts =
      time_point_cast<T>(system_clock::now());
  return ts.time_since_epoch().count();
}

uint64_t GetMicrosSinceEpoch() {
  return GetTimeUnitsSinceEpoch<microseconds>();
}

uint64_t GetNanosSinceEpoch() { return GetTimeUnitsSinceEpoch<nanoseconds>(); }

uint64_t GetMillisSinceEpoch() {
  return GetTimeUnitsSinceEpoch<milliseconds>();
}
*/
