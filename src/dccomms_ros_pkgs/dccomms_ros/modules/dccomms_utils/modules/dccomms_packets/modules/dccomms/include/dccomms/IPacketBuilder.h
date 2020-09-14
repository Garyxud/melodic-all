#ifndef DCCOMMS_IPACKETBUILDER_H
#define DCCOMMS_IPACKETBUILDER_H

#include <dccomms/Packet.h>

namespace dccomms {

class IPacketBuilder;
typedef std::shared_ptr<IPacketBuilder> PacketBuilderPtr;

class IPacketBuilder {
public:
  virtual dccomms::PacketPtr CreateFromBuffer(void *) = 0;
  virtual dccomms::PacketPtr Create() = 0;
  virtual std::string GetName() { return ""; }
};
}
#endif // DCCOMMS_PACKET_H
