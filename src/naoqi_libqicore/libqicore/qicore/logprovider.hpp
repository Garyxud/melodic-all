/*
** Author(s):
**  - Herve Cuche <hcuche@aldebaran-robotics.com>
**  - Matthieu Nottale <mnottale@aldebaran-robotics.com>
**
** Copyright (C) 2013 Aldebaran Robotics
*/

#ifndef LOGPROVIDER_HPP_
#define LOGPROVIDER_HPP_

#include <string>
#include <utility> // std::pair
#include <vector>

#include <qi/log.hpp>

#include <qicore/api.hpp>
#include <qicore/logmessage.hpp>

#include <qi/anyobject.hpp>
#include <qi/session.hpp>

namespace qi
{
class LogManager;
using LogManagerPtr = qi::Object<LogManager>;

/** Registers to a local or remote Logger service
* Sends local logger message to it
* Honors commands from it to configure local logger verbosity.
* @threadSafe
*/
class QICORE_API LogProvider
{
protected:
  LogProvider() = default;

public:
  virtual ~LogProvider() = default;

  virtual void setCategoryPrefix(const std::string& categoryPrefix) = 0;
  virtual void setLevel(qi::LogLevel level) = 0;
  virtual void addFilter(const std::string& filter, qi::LogLevel level) = 0;
  virtual void setFilters(const std::vector<std::pair<std::string, qi::LogLevel> >& filters) = 0;
  virtual void setLogger(LogManagerPtr logger) = 0;
};

using LogProviderPtr = qi::Object<LogProvider>;

QICORE_API LogProviderPtr makeLogProvider(LogManagerPtr logger);
QICORE_API LogProviderPtr makeLogProvider();

QICORE_API qi::FutureSync<qi::LogProviderPtr> initializeLogging(SessionPtr session,
                                                                const std::string& categoryPrefix = "");
} // !qi

#endif // !LOGPROVIDER_HPP_
