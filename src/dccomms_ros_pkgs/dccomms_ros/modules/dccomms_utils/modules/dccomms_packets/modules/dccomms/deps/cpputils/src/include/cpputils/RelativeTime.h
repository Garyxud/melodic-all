#include <chrono>
#include <iostream>

using namespace std;
using namespace std::chrono;

namespace cpputils {
class RelativeTime {
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

RelativeTime::netsim_time RelativeTime::_startSimTime;

void RelativeTime::Reset() { _startSimTime = clock::now(); }

RelativeTime::netsim_time RelativeTime::GetTime() { return clock::now(); }
uint64_t RelativeTime::GetSeconds() { return GetTimeUnitsSinceEpoch<seconds>(); }
uint64_t RelativeTime::GetMillis() { return GetTimeUnitsSinceEpoch<milliseconds>(); }
uint64_t RelativeTime::GetMicros() { return GetTimeUnitsSinceEpoch<microseconds>(); }
uint64_t RelativeTime::GetNanos() { return GetTimeUnitsSinceEpoch<nanoseconds>(); }

}
