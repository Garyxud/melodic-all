/*
 * DLinkInterface.h
 *
 *  Created on: 22 oct. 2016
 *      Author: centelld
 */

#ifndef DCCOMMS_COMMSBRIDGE_H_
#define DCCOMMS_COMMSBRIDGE_H_

#include <cpplogging/Loggable.h>
#include <dccomms/CommsDevice.h>
#include <dccomms/CommsDeviceService.h>
#include <dccomms/Utils.h>
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <string>

namespace dccomms {

#define MAX_ODATA_BUF 4096

using namespace dccomms;
using namespace cpplogging;

class CommsBridge : public virtual Loggable {
public:
  CommsBridge(CommsDevice *, PacketBuilderPtr txPacketBuilder,
              PacketBuilderPtr rxPacketBuilder, int _baudrate = 0);
  virtual ~CommsBridge();
  virtual void Start();
  virtual void Stop();

  inline PacketPtr GetTxPacket() { return txpkt; }
  inline PacketPtr GetRxPacket() { return rxpkt; }

  void SetTransmitingPacketCb(std::function<void(const PacketPtr &)> cb);
  void
  SetReceivedPacketWithoutErrorsCb(std::function<void(const PacketPtr &)> cb);
  void SetReceivedPacketWithErrorsCb(std::function<void(const PacketPtr &)> cb);

  // Two instances of CommsBridge for the same purpose in the same machine (for
  // debug reasons) must have different namespaces
  // This method must be called before Start
  void SetCommsDeviceId(std::string nspace);
  virtual void SetLogName(std::string name);
  virtual void SetLogLevel(cpplogging::LogLevel);
  virtual void FlushLog();
  virtual void FlushLogOn(cpplogging::LogLevel);
  virtual void LogToConsole(bool);
  virtual void LogToFile(const std::string &filename);

private:
  std::function<void(const PacketPtr &)> _PacketReceivedWithoutErrorsCb,
      _PacketReceivedWithErrorsCb, _TransmittingPacketCb;

protected:
  virtual void TxWork();
  virtual void RxWork();
  virtual void _TransmitPacket();
  virtual bool _ReceivePacket();
  virtual bool TryToConnect();
  virtual bool TryToReconnect();

  Timer timer;
  unsigned int _frameTransmissionTime; // milis
  double _byteTransmissionTime;        // milis

  std::string serv_namespace;
  CommsDeviceService phyService;
  PacketPtr txpkt;
  PacketPtr rxpkt;

  uint8_t obuf[MAX_ODATA_BUF];

  std::mutex devicemutex;
  bool connected;
  bool transcurridoTiempoEnvio;
  int baudrate;
  CommsDevice *device;

  ServiceThread<CommsBridge> txserv, rxserv;
  PacketBuilderPtr _txPacketBuilder;
  PacketBuilderPtr _rxPacketBuilder;
};
} /* namespace dcent */

#endif /* DCCOMMS_COMMSBRIDGE_H_ */
