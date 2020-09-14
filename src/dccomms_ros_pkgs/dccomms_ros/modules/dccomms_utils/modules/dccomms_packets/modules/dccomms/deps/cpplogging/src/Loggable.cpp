/*
 * Logable.cpp
 *
 *  Created on: 9 nov. 2016
 *      Author: diego
 */

#include <cpplogging/Loggable.h>
#include <spdlog/sinks/file_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

namespace cpplogging {

namespace spd = spdlog;

Loggable::Loggable(std::string logname) {
  Level = LogLevel::debug;
  auto stdout_sink = spdlog::sinks::stdout_sink_mt::instance();
  console_sink = std::make_shared<spdlog::sinks::ansicolor_sink>(stdout_sink);
  dist_sink = std::make_shared<spdlog::sinks::dist_sink_mt>();
  dist_sink->add_sink(console_sink);
  logToConsole = true;
  flushLevel = LogLevel::err;
  _async = false;
  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("%+"));
  SetLogName(logname);
}

Loggable::~Loggable() { spd::drop(LogName); }

void Loggable::LogToConsole(bool _logtoconsole) {
  if (logToConsole) {
    if (!_logtoconsole) {
      dist_sink->remove_sink(console_sink);
      logToConsole = false; // == _logtoconsole
    }
  } else {
    if (_logtoconsole) {
      dist_sink->add_sink(console_sink);
      logToConsole = true;
    }
  }
}

void Loggable::LogToFile(const std::string &filename) {
  auto fileSink = std::make_shared<spd::sinks::simple_file_sink_mt>(filename);
  dist_sink->add_sink(fileSink);
}

void Loggable::FlushLog() { Log->flush(); }

void Loggable::SetAsyncMode(uint32_t qsize) {
  _async_qsize = qsize;
  spdlog::set_async_mode(_async_qsize);
  _async = true;
}

void Loggable::SetSyncMode() {
  spdlog::set_sync_mode();
  _async = false;
}

spdlog::level::level_enum Loggable::GetSpdLevel(LogLevel _level) {
  switch (_level) {
  case critical:
    return spd::level::critical;
    break;
  case debug:
    return spd::level::debug;
    break;
  case err:
    return spd::level::err;
    break;
  case info:
    return spd::level::info;
    break;
  case off:
    return spd::level::off;
    break;
  case trace:
    return spd::level::trace;
    break;
  case warn:
    return spd::level::warn;
    break;
  }
  return spd::level::info;
}

void Loggable::FlushLogOn(LogLevel level) {
  flushLevel = level;
  Log->flush_on(GetSpdLevel(flushLevel));
}

void Loggable::SetLogFormatter(spdlog::formatter_ptr formatter) {
  _formatter = formatter;
  if (Log)
    Log->set_formatter(formatter);
}

void Loggable::SetLogName(std::string newname) {
  if (newname != LogName) {
    if (Log) {
      spd::drop(LogName);
    }
    LogName = newname;

    Log = spd::get(LogName);
    if (!Log) {
      Log = std::make_shared<spdlog::logger>(LogName, dist_sink);
      Log->set_formatter(_formatter);
      FlushLogOn(flushLevel);
      SetLogLevel(Level);
      if (_async)
        SetAsyncMode(_async_qsize);
      else
        SetSyncMode();
    }
  }
}

void Loggable::SetLogLevel(LogLevel _level) {
  Level = _level;
  if (Log) {
    switch (_level) {
    case critical:
      Log->set_level(spd::level::critical);
      break;
    case debug:
      Log->set_level(spd::level::debug);
      break;
    case err:
      Log->set_level(spd::level::err);
      break;
    case info:
      Log->set_level(spd::level::info);
      break;
    case off:
      Log->set_level(spd::level::off);
      break;
    case trace:
      Log->set_level(spd::level::trace);
      break;
    case warn:
      Log->set_level(spd::level::warn);
      break;
    }
  }
}

} /* namespace cpplogging */
