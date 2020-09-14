#include <string>

#include <qi/types.hpp>

#include <qi/anyobject.hpp>

#include <qicore/logmessage.hpp>
#include <qicore/logmanager.hpp>
#include <qicore/loglistener.hpp>
#include <qi/detail/warn_push_ignore_deprecated.hpp>

bool qi::detail::ForceProxyInclusion<qi::LogManager>::dummyCall()
{
  return true;
}

namespace qi
{
class LogManagerProxy : public qi::Proxy, public LogManager
{
public:
  explicit LogManagerProxy(qi::AnyObject obj)
    : qi::Proxy(std::move(obj))
  {
  }

  void log(const std::vector<LogMessage>& p0)
  {
    _obj.call<void>("log", p0);
  }

  LogListenerPtr createListener()
  {
    return _obj.call<LogListenerPtr>("createListener");
  }

  LogListenerPtr getListener()
  {
    return _obj.call<LogListenerPtr>("getListener");
  }

  int addProvider(Object<LogProvider> p0)
  {
    return _obj.call<int>("addProvider", p0);
  }

  void removeProvider(int p0)
  {
    _obj.call<void>("removeProvider", p0);
  }
};

QI_REGISTER_PROXY_INTERFACE(LogManagerProxy, LogManager);
} // !qi

#include <qi/detail/warn_push_ignore_deprecated.hpp>
