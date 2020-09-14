#include "imagestore.hpp"

qiLogCategory("test.qi.fileExample.imageStore");

#include <map>

#include <boost/filesystem/operations.hpp>

QI_TYPE_INTERFACE(alice::ImageStore);

namespace alice
{
class ImageStoreImpl : public ImageStore
{
public:
  ImageStoreImpl() = default;

  ~ImageStoreImpl() override
  {
    for (const auto& slot : _fileRegistry)
    {
      boost::filesystem::remove(slot.second);
    }
  }

  void storeImage(qi::FilePtr imageFile, std::string name) override
  {
    // Note that ideally this would be implemented in an asynchronous way,
    // but for simplicity we will do it synchronously.

    // First, make a local copy in a temporary files directory:
    const auto tempFilePath = generateTemporaryFilePath();

    // This call will block until the end because it returns a FutureSync.
    qi::copyToLocal(imageFile, tempFilePath);

    // We now have a local copy of the remote file,
    // so we don't need the remote access anymore.
    imageFile.reset();

    // Now we can work with the local file.
    storeFileInDatabase(name, tempFilePath);
  }

  qi::FilePtr getImage(std::string name) override
  {
    const auto fileLocation = findFileLocation(name);

    // Now we can open it and provide it to the user for reading.
    return qi::openLocalFile(fileLocation);
  }

private:
  using FileRegistry = std::map<std::string, qi::Path>;
  FileRegistry _fileRegistry;

  qi::Path generateTemporaryFilePath()
  {
    static int idx = 0;
    static const std::string PATH_PREFIX = "./temp_image_file_";
    static const std::string PATH_SUFFIX = ".data";

    std::stringstream newPath;
    newPath << PATH_PREFIX << idx << PATH_SUFFIX;
    ++idx;
    return newPath.str();
  }

  void storeFileInDatabase(const std::string& name, const qi::Path& path)
  {
    //...fake it
    _fileRegistry[name] = path;
  }

  qi::Path findFileLocation(const std::string& name)
  {
    return _fileRegistry[name];
  }
};
QI_REGISTER_OBJECT(ImageStore, storeImage, getImage);
QI_REGISTER_IMPLEMENTATION(ImageStore, ImageStoreImpl);

ImageStorePtr getImageStore()
{
  static ImageStorePtr imageStore = boost::make_shared<ImageStoreImpl>();
  return imageStore;
}
}
