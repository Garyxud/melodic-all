/*
 * Logable.h
 *
 *  Created on: 9 nov. 2016
 *      Author: diego
 */

#ifndef CPPLOGGING_LOGGABLE_H_
#define CPPLOGGING_LOGGABLE_H_

#include <cpplogging/types.h>
#include <iostream>
#include <memory>
#include <spdlog/sinks/ansicolor_sink.h>
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/spdlog.h>
#include <string>

namespace cpplogging {

namespace spd = spdlog;

class Loggable {
public:
  Loggable(std::string logname = "log");
  virtual ~Loggable();

  virtual void SetLogName(std::string name);
  virtual void LogToFile(const std::string &filename);
  virtual void LogToConsole(bool);
  virtual void SetLogLevel(LogLevel);
  virtual std::string GetLogName() { return LogName; }
  virtual void FlushLogOn(LogLevel);
  virtual void FlushLog();
  virtual void SetLogFormatter(spdlog::formatter_ptr);
  virtual void SetAsyncMode(uint32_t qsize = 8192);
  virtual void SetSyncMode();

protected:
  std::shared_ptr<spd::sinks::dist_sink_mt> dist_sink;
  // std::shared_ptr<spd::sinks::stdout_sink_mt> console_sink;
  std::shared_ptr<spdlog::sinks::ansicolor_sink> console_sink;

  spdlog::level::level_enum GetSpdLevel(LogLevel);
  std::string LogName;
  LogLevel Level, flushLevel;
  std::shared_ptr<spd::logger> Log;
  spdlog::formatter_ptr _formatter;
  bool logToConsole;
  bool _async;
  uint32_t _async_qsize;
};

} /* namespace cpplogging */

#endif /* CPPLOGGING_LOGGABLE_H_ */
