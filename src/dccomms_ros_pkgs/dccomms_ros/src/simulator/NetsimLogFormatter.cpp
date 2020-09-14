#include <chrono>
#include <dccomms_ros/simulator/NetsimLogFormatter.h>
#include <dccomms_ros/simulator/NetsimTime.h>

namespace dccomms_ros {

NetsimLogFormatter::NetsimLogFormatter(const std::string &pattern)
    : _formatter(pattern) {}

void NetsimLogFormatter::format(details::log_msg &msg) {
  msg.formatted.write("[{:.9f}] ", NetsimTime::GetNanos() / 1e9);
  // Getting the simulation time aproximation of ns3 seems not to work well...
  // msg.formatted.write("[{:.9f}] ", ns3::Simulator::Now().GetDouble() / 1e9);
  _formatter.format(msg);
}
}
