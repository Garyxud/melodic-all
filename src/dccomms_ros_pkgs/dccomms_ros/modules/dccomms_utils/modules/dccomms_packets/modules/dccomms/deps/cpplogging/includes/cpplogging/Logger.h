#ifndef CPPLOGGING_LOGGER_H
#define CPPLOGGING_LOGGER_H

#include <cpplogging/Loggable.h>

namespace cpplogging {
class Logger;
typedef std::shared_ptr<cpplogging::Logger> LoggerPtr;

class Logger : public Loggable {
public:
  Logger();
  // http://en.cppreference.com/w/cpp/language/parameter_pack
  template <typename... Targs> inline void Debug(Targs... Fargs) {
    Log->debug(Fargs...);
  }
  template <typename... Targs> inline void Error(Targs... Fargs) {
    Log->error(Fargs...);
  }
  template <typename... Targs> inline void Info(Targs... Fargs) {
    Log->info(Fargs...);
  }
  template <typename... Targs> inline void Trace(Targs... Fargs) {
    Log->trace(Fargs...);
  }
  template <typename... Targs> inline void Warn(Targs... Fargs) {
    Log->warn(Fargs...);
  }
  template <typename... Targs> inline void Critical(Targs... Fargs) {
    Log->critical(Fargs...);
  }
};
static LoggerPtr CreateLogger(std::string logname) {
  LoggerPtr logger(new Logger());
  logger->SetLogName(logname);
  return logger;
}
}

#endif // CPPLOGGING_LOGGER_H_
