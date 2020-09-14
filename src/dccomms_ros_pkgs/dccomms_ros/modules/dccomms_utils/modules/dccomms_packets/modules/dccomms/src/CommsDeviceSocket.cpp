/*
 * Radio.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: diego
 */

#include <chrono>
#include <dccomms/Arduino.h>
#include <dccomms/CommsDeviceSocket.h>
#include <dccomms/CommsException.h>
#include <dccomms/DataLinkFrame.h>
#include <iostream>
#include <thread> // std::this_thread::sleep_for

namespace dccomms {

CommsDeviceSocket::CommsDeviceSocket(uint32_t d, uint32_t maxRxBufferSize)
    : _addr(d) {
  _maxRxBufferSize = maxRxBufferSize;
  _rxBuffer = new uint8_t[_maxRxBufferSize];
  _bytesInBuffer = 0;
  _rxBufferLastPos = 0;
  _rxBufferFirstPos = 0;
  TotalErrors = 0;

  // FCSType = (DataLinkFrame::fcsType) fcst;
  SetLogName("CommsDeviceSocket");
  SetLogLevel(cpplogging::off);
  SetPayloadSize(1000);
  EnableWaitForDeviceReady(false);
}

CommsDeviceSocket::~CommsDeviceSocket() {
  if (_rxBuffer != NULL)
    delete _rxBuffer;
}

void CommsDeviceSocket::SetStreamCommsDevice(Ptr<StreamCommsDevice> dev) {
  _device = dev;
}

void CommsDeviceSocket::SetPacketBuilder(PacketBuilderPtr pb) {
  _packetBuilder = pb;
  _packet = pb->Create();
}

PacketPtr CommsDeviceSocket::_BuildPacket(uint8_t *buffer, uint32_t dataSize,
                                          uint32_t addr, uint32_t dst) {
  auto packet = _packetBuilder->Create();
  packet->SetPayload(buffer, dataSize);
  packet->SetSrc(addr);
  packet->SetDst(dst);
  return packet;
}

void CommsDeviceSocket::Send(const void *buf, uint32_t size) {
  uint8_t *buffer = (uint8_t *)buf;
  uint32_t numPackets = size / _packetSize;
  uint32_t np;
  for (np = 1; np < numPackets; np++) {
    while (_waitForDevice && _device->BusyTransmitting())
      ;

    PacketPtr dlfPtr =
        _BuildPacket(buffer, _packetSize, _addr, _defaultDestAddr);

    Log->debug("Sending packet...");

    _device << dlfPtr;
    buffer += _packetSize;
  }
  if (numPackets > 0) {
    while (_waitForDevice && _device->BusyTransmitting())
      ;
    PacketPtr dlfPtr =
        _BuildPacket(buffer, _packetSize, _addr, _defaultDestAddr);

    Log->debug("Sending packet...");

    _device << dlfPtr;
    buffer += _packetSize;
  }

  uint32_t bytesLeft = size % _packetSize;
  if (bytesLeft) {
    while (_waitForDevice && _device->BusyTransmitting())
      ;
    PacketPtr dlfPtr = _BuildPacket(buffer, bytesLeft, _addr, _defaultDestAddr);

    Log->debug("Sending packet...");

    _device << dlfPtr;
  }
}

void CommsDeviceSocket::_IncreaseBytesInBuffer() {
  if (_bytesInBuffer < _maxRxBufferSize) {
    _rxBufferLastPos = (_rxBufferLastPos + 1) % _maxRxBufferSize;
    _bytesInBuffer++;
  }
}

void CommsDeviceSocket::_DecreaseBytesInBuffer() {
  if (_bytesInBuffer) {
    _rxBufferFirstPos = (_rxBufferFirstPos + 1) % _maxRxBufferSize;
    _bytesInBuffer--;
  }
}

uint32_t CommsDeviceSocket::Recv(void *buf, uint32_t size, unsigned long ms) {
  uint8_t *buffer = (uint8_t *)buf;
  uint32_t bytes = 0;
  uint16_t i;

  if (_bytesInBuffer) // If bytes in buffer: read them
  {
    while (bytes < size && _bytesInBuffer) {
      *buffer = _rxBuffer[_rxBufferFirstPos];
      _DecreaseBytesInBuffer();
      buffer++;
      bytes++;
    }
  }
  try {
    _device->SetTimeout(ms);
    while (bytes < size &&
           (_device->Available() > 0 || ms == 0)) // If more bytes needed, wait for them
    {
      _device >> _packet;
      if (_packet->IsOk()) {
        Log->debug("Frame received without errors!");
        uint32_t bytesToRead = (bytes + _packet->GetPayloadSize()) <= size
                                   ? _packet->GetPayloadSize()
                                   : size - bytes;

        auto payloadBuffer = _packet->GetPayloadBuffer();
        for (i = 0; i < bytesToRead; i++) {
          *buffer = payloadBuffer[i];
          buffer++;
        }
        bytes += _packet->GetPayloadSize();
      } else {
        TotalErrors += 1;
        Log->error("Error in packet (Total Errors: {})", TotalErrors);
      }
    }
  } catch (CommsException &e) {
    Log->debug("Timeout when waiting for the next packet");
  }
  if (bytes > size) // If we have received more bytes save them
  {
    uint32_t bytesLeft = bytes - size;
    uint8_t *ptr = _rxBuffer;
    _bytesInBuffer =
        bytesLeft <= _maxRxBufferSize ? bytesLeft : _maxRxBufferSize;
    uint8_t *maxPtr = _rxBuffer + _bytesInBuffer;
    auto payloadBuffer = _packet->GetPayloadBuffer();
    while (ptr != maxPtr) {
      *ptr = payloadBuffer[i++];
      ptr++;
    }
    _rxBufferLastPos = 0;
    _rxBufferFirstPos = 0;
  }

  return bytes;
}

int CommsDeviceSocket::Read(void *buff, uint32_t size,
                            unsigned long msTimeout) {
  Recv(buff, size, msTimeout);
  return size;
}
int CommsDeviceSocket::Write(const void *buff, uint32_t size,
                             uint32_t msTimeout) {
  Send(buff, size);
  return size;
}

void CommsDeviceSocket::_GetNextPayloadBytes() {
  _device >> _packet;
  if (_packet->IsOk()) {
    Log->debug("Frame received without errors!");
    uint32_t bytesToRead = _packet->GetPayloadSize();
    auto payloadBuffer = _packet->GetPayloadBuffer();
    for (int i = 0; i < bytesToRead; i++) {
      _rxBuffer[_rxBufferLastPos] = payloadBuffer[i];
      _IncreaseBytesInBuffer();
    }
  } else {
    TotalErrors += 1;
    Log->error("Error in packet (Total Errors: {})", TotalErrors);
  }
}

int CommsDeviceSocket::Available() {
  while (_device->Available()) {
    _GetNextPayloadBytes();
  }
  return _bytesInBuffer;
}
bool CommsDeviceSocket::IsOpen() { return _device->IsOpen(); }
void CommsDeviceSocket::FlushInput() {
  throw CommsException("void CommsDeviceSocket::FlushInput() Not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}
void CommsDeviceSocket::FlushOutput() {
  throw CommsException("void CommsDeviceSocket::FlushOutput() Not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}
void CommsDeviceSocket::FlushIO() {
  throw CommsException("void CommsDeviceSocket::FlushIO() Not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}
}
