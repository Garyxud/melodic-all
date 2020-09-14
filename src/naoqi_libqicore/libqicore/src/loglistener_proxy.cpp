#include <string>

#include <qi/types.hpp>

#include <qi/anyobject.hpp>
#include <qi/signal.hpp>
#include <qi/property.hpp>
#include <qi/type/proxysignal.hpp>
#include <qi/type/proxyproperty.hpp>

#include <qicore/loglistener.hpp>
#include <qicore/logmessage.hpp>

bool qi::detail::ForceProxyInclusion<qi::LogListener>::dummyCall()
{
  return true;
}

namespace qi
{
class LogListenerProxy : public qi::Proxy, public LogListener
{
public:
  explicit LogListenerProxy(qi::AnyObject obj)
    : qi::Proxy(std::move(obj))
    , qi::LogListener()
  {
    qi::makeProxySignal(onLogMessage, obj, "onLogMessage");
    qi::makeProxySignal(onLogMessages, obj, "onLogMessages");
    qi::makeProxySignal(onLogMessagesWithBacklog, obj, "onLogMessagesWithBacklog");
    qi::makeProxyProperty(logLevel, obj, "logLevel");
  }

  void setLevel(qi::LogLevel p0)
  {
    _obj.call<void>("setLevel", p0);
  }

  void addFilter(const std::string& p0, qi::LogLevel p1)
  {
    _obj.call<void>("addFilter", p0, p1);
  }

  void clearFilters()
  {
    _obj.call<void>("clearFilters");
  }
};

QI_REGISTER_PROXY_INTERFACE(LogListenerProxy, LogListener);
} // !qi
