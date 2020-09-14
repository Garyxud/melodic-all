/*
** Author(s):
**  - Herve Cuche <hcuche@aldebaran-robotics.com>
**  - Matthieu Nottale <mnottale@aldebaran-robotics.com>
**
** Copyright (C) 2013 Aldebaran Robotics
*/

#ifndef LOGLISTENER_HPP_
#define LOGLISTENER_HPP_

#include <string>

#include <qi/log.hpp>

#include <qicore/api.hpp>
#include <qicore/logmanager.hpp>
#include <qicore/logmessage.hpp>

#include <qi/signal.hpp>
#include <qi/property.hpp>
#include <qi/anyobject.hpp>

namespace qi
{
class QICORE_API LogListener
{
protected:
  template< class... T >
  using PropertyType = qi::UnsafeProperty<T...>;

  LogListener() = default;
  LogListener(PropertyType<qi::LogLevel>::Getter get,
              PropertyType<qi::LogLevel>::Setter set,
              boost::function<Future<void>(bool)> func = {})
    : logLevel(get, set)
    , onLogMessagesWithBacklog(func)
  {
  }

public:
  virtual ~LogListener() = default;

  virtual void setLevel(qi::LogLevel level) = 0;
  virtual void addFilter(const std::string& filter, qi::LogLevel level) = 0;

  /// Remove all filters set by addFilter
  virtual void clearFilters() = 0;

public:
  PropertyType<qi::LogLevel> logLevel;
  qi::Signal<qi::LogMessage> onLogMessage;
  qi::Signal<std::vector<qi::LogMessage> > onLogMessages;
  qi::Signal<std::vector<qi::LogMessage> > onLogMessagesWithBacklog;
};

using LogListenerPtr = qi::Object<LogListener>;
} // !qi

namespace qi
{
namespace detail
{
  template <>
  struct QICORE_API ForceProxyInclusion<qi::LogListener>
  {
    bool dummyCall();
  };
}
}

#endif // !LOGLISTENER_HPP_
