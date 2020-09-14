#include <qicore/file.hpp>
#include <qi/anymodule.hpp>
#include <qi/detail/warn_push_ignore_deprecated.hpp>

namespace qi
{
class FileProxy : public File, public qi::Proxy
{
public:
  explicit FileProxy(qi::AnyObject obj)
    : qi::Proxy(std::move(obj))
  {
  }

  ~FileProxy() = default;

  Buffer read(std::streamsize countBytesToRead) override
  {
    return _obj.call<Buffer>("read", countBytesToRead);
  }

  Buffer read(std::streamoff beginOffset, std::streamsize countBytesToRead) override
  {
    return _obj.call<Buffer>("read", beginOffset, countBytesToRead);
  }

  bool seek(std::streamoff offsetFromBegin) override
  {
    return _obj.call<bool>("seek", offsetFromBegin);
  }

  void close() override
  {
    return _obj.call<void>("close");
  }

  std::streamsize size() const override
  {
    return _obj.call<std::streamsize>("size");
  }

  bool isOpen() const override
  {
    return _obj.call<bool>("isOpen");
  }

  bool isRemote() const override
  {
    return true;
  }

  ProgressNotifierPtr operationProgress() const override
  {
    return _obj.call<ProgressNotifierPtr>("operationProgress");
  }

  // Deprecated members
  Buffer _read(std::streamsize countBytesToRead) override
  {
    return _obj.call<Buffer>("_read", countBytesToRead);
  }

  Buffer _read(std::streamoff beginOffset, std::streamsize countBytesToRead) override
  {
    return _obj.call<Buffer>("_read", beginOffset, countBytesToRead);
  }

  bool _seek(std::streamoff offsetFromBegin) override
  {
    return _obj.call<bool>("_seek", offsetFromBegin);
  }

  void _close() override
  {
    return _obj.call<void>("_close");
  }
};

void _qiregisterFileProxy()
{
  ::qi::registerProxyInterface<FileProxy, File>();
}
}

#include <qi/detail/warn_pop_ignore_deprecated.hpp>
