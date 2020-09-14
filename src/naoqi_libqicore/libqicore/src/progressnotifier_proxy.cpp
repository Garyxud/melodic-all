#include <qicore/file.hpp>
#include <qi/anymodule.hpp>
#include <qi/detail/warn_push_ignore_deprecated.hpp>

namespace qi
{
class ProgressNotifierProxy : public ProgressNotifier, public qi::Proxy
{
public:
  explicit ProgressNotifierProxy(qi::AnyObject obj)
    : qi::Proxy(std::move(obj))
  {
  }

  void reset() override
  {
    _obj.call<void>("reset");
  }

  void notifyRunning() override
  {
    _obj.call<void>("notifyRunning");
  }

  void notifyFinished() override
  {
    _obj.call<void>("notifyFinished");
  }

  void notifyCanceled() override
  {
    _obj.call<void>("notifyCanceled");
  }

  void notifyFailed() override
  {
    _obj.call<void>("notifyFailed");
  }

  void notifyProgressed(double newProgress) override
  {
    _obj.call<void>("notifyProgressed", newProgress);
  }

  bool isRunning() const override
  {
    return _obj.call<bool>("isRunning");
  }

  Future<void> waitForFinished() override
  {
    return _obj.async<void>("waitForFinished");
  }

  // Deprecated members:
  void _reset() override
  {
    _obj.call<void>("_reset");
  }

  void _notifyRunning() override
  {
    _obj.call<void>("_notifyRunning");
  }

  void _notifyFinished() override
  {
    _obj.call<void>("_notifyFinished");
  }

  void _notifyCanceled() override
  {
    _obj.call<void>("_notifyCanceled");
  }

  void _notifyFailed() override
  {
    _obj.call<void>("_notifyFailed");
  }

  void _notifyProgressed(double newProgress) override
  {
    _obj.call<void>("_notifyProgressed", newProgress);
  }

};

void _qiregisterProgressNotifierProxy()
{
  ::qi::registerProxyInterface<ProgressNotifierProxy, ProgressNotifier>();
}
}

#include <qi/detail/warn_pop_ignore_deprecated.hpp>
