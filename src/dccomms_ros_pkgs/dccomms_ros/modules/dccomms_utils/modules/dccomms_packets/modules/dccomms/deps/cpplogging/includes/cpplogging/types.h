#ifndef CPPLOGGING_TYPES_H
#define CPPLOGGING_TYPES_H

#include <iostream>

namespace cpplogging {

enum LogLevel { critical, debug, err, info, off, trace, warn };

static LogLevel GetLevelFromString(const std::string &level) {
  // critical, debug, err, info, off, trace, warn
  if (level == "info") {
    return LogLevel::info;
  } else if (level == "critical") {
    return LogLevel::critical;
  } else if (level == "debug") {
    return LogLevel::debug;
  } else if (level == "err") {
    return LogLevel::err;
  } else if (level == "off") {
    return LogLevel::off;
  } else if (level == "trace") {
    return LogLevel::trace;
  } else {
    return LogLevel::warn;
  }
}

}

#endif // CPPLOGGING_TYPES_H
