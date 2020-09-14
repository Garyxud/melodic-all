#ifndef ALICE_SERVICE_IMAGESTORE_HPP
#define ALICE_SERVICE_IMAGESTORE_HPP
#pragma once

#include <qicore/file.hpp>

#include "api.hpp"

namespace alice
{
class ALICE_SERVICE_API ImageStore
{
public:
  virtual ~ImageStore() = default;

  // Store a copy of the image file and associate it with the provided name.
  virtual void storeImage(qi::FilePtr imageFile, std::string name) = 0;

  // Provide access to an image file associated with the provided name.
  virtual qi::FilePtr getImage(std::string name) = 0;

protected:
  ImageStore() = default;
};

using ImageStorePtr = qi::Object<ImageStore>;

ALICE_SERVICE_API ImageStorePtr getImageStore();
}
#endif
