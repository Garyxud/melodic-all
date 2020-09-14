#ifndef DCCOMMSROS_PACKETBUILDERFACTORYLOADER_H
#define DCCOMMSROS_PACKETBUILDERFACTORYLOADER_H

#include <class_loader/multi_library_class_loader.hpp>
#include <dccomms/IPacketBuilder.h>
#include <dccomms_ros/simulator/PacketBuilderLoader.h>
#include <exception>
#include <string>
#include <unordered_map>

namespace dccomms_ros {

#define PB_LOADER_EXCEPTION_NOCLASSFOUND 0

typedef std::shared_ptr<class_loader::ClassLoader> ClassLoaderPtr;
typedef std::unordered_map<std::string, ClassLoaderPtr> ClassLoaderMap;

class PacketBuilderLoaderException : public std::exception {
public:
  PacketBuilderLoaderException(std::string msg, int cod);
  virtual const char *what() const throw() { return message.c_str(); }
  int code;

private:
  std::string message;
};

class PacketBuilderLoader {
public:
  static dccomms::PacketBuilderPtr
  LoadPacketBuilder(const std::string &libName, const std::string className);

private:
  static ClassLoaderMap loaders;
};
} // namespace dccomms_ros
#endif
