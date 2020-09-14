/*
** Author(s):
**  - Herve Cuche <hcuche@aldebaran-robotics.com>
**  - Matthieu Nottale <mnottale@aldebaran-robotics.com>
**
** Copyright (C) 2013 Aldebaran Robotics
*/

#ifndef LOGMANAGER_HPP_
#define LOGMANAGER_HPP_

#include <qi/macro.hpp>
#include <qicore/api.hpp>
#include <qicore/logmessage.hpp>

#include <qi/anyobject.hpp>

namespace qi
{
class LogListener;
using LogListenerPtr = qi::Object<LogListener>;
class LogProvider;
using LogProviderPtr = qi::Object<LogProvider>;
class QICORE_API LogManager
{
protected:
  LogManager() = default;

public:
  virtual ~LogManager() = default;
  virtual void log(const std::vector<LogMessage>& msgs) = 0;

  virtual LogListenerPtr createListener() = 0;
  /**
   * \deprecated since 2.3 use createListener() instead
   */
  virtual QI_API_DEPRECATED_MSG(Use 'createListener' instead) LogListenerPtr getListener() = 0;
  virtual int addProvider(LogProviderPtr provider) = 0;
  virtual void removeProvider(int idProvider) = 0;
};

using LogManagerPtr = qi::Object<LogManager>;
} // !qi

namespace qi
{
namespace detail
{
  template <>
  struct QICORE_API ForceProxyInclusion<qi::LogManager>
  {
    bool dummyCall();
  };
}
}

#endif // !LOGMANAGER_HPP_
