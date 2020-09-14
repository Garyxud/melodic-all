#include <dccomms_ros/simulator/PacketBuilderLoader.h>

namespace dccomms_ros {

ClassLoaderMap PacketBuilderLoader::loaders;

PacketBuilderLoaderException::PacketBuilderLoaderException(std::string msg,
                                                           int cod) {
  message = msg;
  code = cod;
}

dccomms::PacketBuilderPtr
PacketBuilderLoader::LoadPacketBuilder(const std::string &libName,
                                       const std::string className) {

  auto loader = loaders[libName];
  if (!loader) {
    loader = ClassLoaderPtr(new class_loader::ClassLoader(libName));
    loaders[libName] = loader;
  }
  dccomms::IPacketBuilder *pb;
  std::vector<std::string> classes =
      loader->getAvailableClasses<dccomms::IPacketBuilder>();
  for (unsigned int c = 0; c < classes.size(); ++c) {
    if (classes[c] == className) {
      pb = loader->createUnmanagedInstance<dccomms::IPacketBuilder>(classes[c]);
      break;
    }
  }
  if (!pb)
    throw PacketBuilderLoaderException("no '" + className + "' class found",
                                       PB_LOADER_EXCEPTION_NOCLASSFOUND);

  return dccomms::PacketBuilderPtr(pb);
}
} // namespace dccomms_ros
