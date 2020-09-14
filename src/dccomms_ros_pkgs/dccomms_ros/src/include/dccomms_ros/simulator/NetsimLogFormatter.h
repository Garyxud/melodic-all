#ifndef NETSIMLOGFORMATTER_H
#define NETSIMLOGFORMATTER_H

#include <ns3/simulator.h>
#include <spdlog/spdlog.h>

namespace dccomms_ros {

using namespace ns3;
using namespace spdlog;

class NetsimLogFormatter : public spdlog::formatter {
public:
  NetsimLogFormatter(const std::string& pattern);
  void format(details::log_msg &msg) override;
private:
  pattern_formatter _formatter;
};
}

#endif
