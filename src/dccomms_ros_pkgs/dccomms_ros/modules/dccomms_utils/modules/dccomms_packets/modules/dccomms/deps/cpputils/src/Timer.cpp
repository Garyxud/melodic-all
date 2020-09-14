#include <cpputils/Timer.h>

namespace cpputils {
//TODO: make template
Timer::Timer() : beg_(clock_::now()) {}
void Timer::Reset() { beg_ = clock_::now(); }
unsigned int Timer::Elapsed() const {
  return std::chrono::duration_cast<milis_>(clock_::now() - beg_).count();
}

TimerNanos::TimerNanos() : beg_(clock_::now()) {}
void TimerNanos::Reset() { beg_ = clock_::now(); }

long TimerNanos::Last() const {
  auto epoch = beg_.time_since_epoch();
  auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch);
  return value.count();
}

long TimerNanos::Now() const {
  auto epoch = clock_::now().time_since_epoch();
  auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch);
  return value.count();
}
unsigned long TimerNanos::Elapsed() const {
  return std::chrono::duration_cast<nanos_>(clock_::now() - beg_).count();
}
} // namespace cpputils
