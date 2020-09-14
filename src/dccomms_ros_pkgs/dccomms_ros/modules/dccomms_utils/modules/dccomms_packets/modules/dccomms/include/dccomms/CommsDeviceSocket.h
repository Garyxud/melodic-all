/*
 * Radio.h
 *
 *  Created on: Feb 11, 2015
 *      Author: diego
 */

#ifndef DCCOMMS_COMMSDEVICESOCKET_H_
#define DCCOMMS_COMMSDEVICESOCKET_H_

#include <cpplogging/Loggable.h>
#include <dccomms/StreamCommsDevice.h>
#include <dccomms/DataLinkFrame.h>
#include <mutex>
#include <condition_variable>

namespace dccomms {

class CommsDeviceSocket;
typedef std::shared_ptr<CommsDeviceSocket> CommsDeviceSocketPtr;

class CommsDeviceSocket : public cpplogging::Loggable, public Stream {
public:
  ~CommsDeviceSocket();
  CommsDeviceSocket(uint32_t addr, uint32_t maxRxBufferSize = 5000);

  void SetStreamCommsDevice(Ptr<StreamCommsDevice> dev);
  void SetPacketBuilder(PacketBuilderPtr pb);
  void Send(const void *, uint32_t size);
  uint32_t Recv(void *, uint32_t size, unsigned long ms = 0);
  inline void SetDestAddr(uint32_t dst) { _defaultDestAddr = dst; }
  inline void SetPayloadSize(uint32_t dst) { _packetSize = dst; }
  inline void EnableWaitForDeviceReady(bool v) { _waitForDevice = v;}

  //void Start();
  // Stream methods to implement
  int Read(void *, uint32_t, unsigned long msTimeout = 0);
  int Write(const void *, uint32_t, uint32_t msTimeout = 0);
  int Available();
  bool IsOpen();
  void FlushInput();
  void FlushOutput();
  void FlushIO();

  int TotalErrors = 0;

private:
  PacketPtr _BuildPacket(uint8_t *buffer, uint32_t dataSize, uint32_t _addr,
                         uint32_t dst);

  Ptr<StreamCommsDevice> _device;
  uint32_t _addr, _packetSize;
  PacketBuilderPtr _packetBuilder;

  uint32_t _maxRxBufferSize;
  uint8_t *_rxBuffer;
  uint32_t _rxBufferFirstPos;
  uint32_t _rxBufferLastPos;
  uint32_t _bytesInBuffer = 0;
  uint32_t _defaultDestAddr;
  bool _waitForDevice = false;
  bool _started;

  //std::thread _worker;
  //std::mutex _rxFifoMutex;
  //std::condition_variable _dataAvailableCond;

  void _DecreaseBytesInBuffer();
  void _IncreaseBytesInBuffer();

  void _ReadNextPacket();
  void _GetNextPayloadBytes();
  PacketPtr _packet;
};
}

#endif /* DCCOMMS_RADIO_H_ */
