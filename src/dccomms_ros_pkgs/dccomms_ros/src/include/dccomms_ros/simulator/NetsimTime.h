#ifndef NETSIMTIME_H
#define NETSIMTIME_H

#include <chrono>

namespace dccomms_ros {

using namespace std::chrono;

class NetsimTime {
public:
  typedef steady_clock clock;
  typedef time_point<clock, nanoseconds> netsim_time;
  static inline void Reset();
  static inline netsim_time GetTime();
  static inline uint64_t GetSeconds();
  static inline uint64_t GetMillis();
  static inline uint64_t GetMicros();
  static inline uint64_t GetNanos();

private:
  static netsim_time _startSimTime;

  template <typename T> static uint64_t GetTimeUnitsSinceEpoch() {
    auto ts = duration_cast<T>(clock::now() - _startSimTime);
    return ts.count();
  }
};

void NetsimTime::Reset() { _startSimTime = clock::now(); }

NetsimTime::netsim_time NetsimTime::GetTime() { return clock::now(); }
uint64_t NetsimTime::GetSeconds() { return GetTimeUnitsSinceEpoch<seconds>(); }
uint64_t NetsimTime::GetMillis() {
  return GetTimeUnitsSinceEpoch<milliseconds>();
}
uint64_t NetsimTime::GetMicros() {
  return GetTimeUnitsSinceEpoch<microseconds>();
}
uint64_t NetsimTime::GetNanos() {
  return GetTimeUnitsSinceEpoch<nanoseconds>();
}

}
#endif
