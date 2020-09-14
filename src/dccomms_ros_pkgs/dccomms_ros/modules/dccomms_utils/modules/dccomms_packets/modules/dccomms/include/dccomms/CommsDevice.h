#ifndef DCCOMMS_ICOMMSDEVICE_H
#define DCCOMMS_ICOMMSDEVICE_H

#include <cpplogging/cpplogging.h>
#include <dccomms/Packet.h>
#include <cpputils/Object.h>

namespace dccomms {

using namespace cpplogging;
using namespace cpputils;

class CommsDevice;

class CommsDevice : public virtual Logger {
public:
  CommsDevice();

  friend CommsDevice &operator>>(CommsDevice &, const PacketPtr &dlf);
  friend CommsDevice &operator<<(CommsDevice &, const PacketPtr &dlf);

  friend Ptr<CommsDevice> operator>>(Ptr<CommsDevice>, const PacketPtr &dlf);
  friend Ptr<CommsDevice> operator<<(Ptr<CommsDevice>, const PacketPtr &dlf);

  virtual void ReadPacket(const PacketPtr &) = 0;
  virtual void WritePacket(const PacketPtr &) = 0;

  virtual bool BusyTransmitting();
  virtual void SetTimeout(unsigned long);
  virtual unsigned long GetTimeout();

  virtual bool Open();
  virtual void Close();

protected:
  unsigned long _timeout = 0;
};

} /* namespace dccomms */

#endif // DCCOMMS_ICOMMSDEVICE_H
