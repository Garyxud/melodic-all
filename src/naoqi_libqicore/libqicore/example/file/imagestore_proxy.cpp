#include "imagestore.hpp"

qiLogCategory("test.qi.fileExample.imageStoreProxy");

namespace alice
{
class ImageStoreProxy : public ImageStore, public qi::Proxy
{
public:
  explicit ImageStoreProxy(qi::AnyObject obj)
    : qi::Proxy(obj)
  {
  }

  void storeImage(qi::FilePtr imageFile, std::string name) override
  {
    return _obj.call<void>("storeImage", imageFile, name);
  }

  qi::FilePtr getImage(std::string name) override
  {
    return _obj.call<qi::FilePtr>("getImage", name);
  }
};
QI_REGISTER_PROXY_INTERFACE(ImageStoreProxy, ImageStore);
}
