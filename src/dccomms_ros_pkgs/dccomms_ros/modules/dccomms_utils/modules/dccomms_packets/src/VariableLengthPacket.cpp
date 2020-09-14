#include <class_loader/multi_library_class_loader.hpp>
#include <dccomms_packets/VariableLengthPacket.h>

namespace dccomms_packets {
VariableLengthPacket::VariableLengthPacket() {
  _overheadSize = PRE_SIZE + PAYLOAD_SIZE_FIELD + FCS_SIZE;
  _maxPacketSize = _overheadSize + MAX_PAYLOAD_SIZE;
  _AllocBuffer(_maxPacketSize);
  _Init();
}

void VariableLengthPacket::_Init() {
  _pre = GetBuffer();
  *_pre = 0x55;
  _payloadSize = _pre + 1;
  *_payloadSize = 0;
  _payload = _payloadSize + 1;
  _fcs = _payload + *_payloadSize;
}

void VariableLengthPacket::DoCopyFromRawBuffer(void *buffer) {
  uint8_t payloadSize = *((uint8_t *)buffer + PRE_SIZE);
  memcpy(GetBuffer(), buffer, payloadSize + _overheadSize);
}

inline uint8_t *VariableLengthPacket::GetPayloadBuffer() { return _payload; }

inline uint32_t VariableLengthPacket::GetPayloadSize() { return *_payloadSize; }

inline int VariableLengthPacket::GetPacketSize() {
  return _overheadSize + *_payloadSize;
}

void VariableLengthPacket::Read(Stream *stream) {
  stream->WaitFor(_pre, PRE_SIZE);
  stream->Read(_payloadSize, 1);
  stream->Read(_payload, *_payloadSize + FCS_SIZE);
}

void VariableLengthPacket::PayloadUpdated(uint32_t payloadSize) {
  *_payloadSize = payloadSize;
  _fcs = _payload + *_payloadSize;
  UpdateFCS();
}

void VariableLengthPacket::GetPayload(void *copy, int size) {
  auto copySize = *_payloadSize < size ? *_payloadSize : size;
  memcpy(copy, _payload, copySize);
}

uint32_t VariableLengthPacket::SetPayload(uint8_t *data, uint32_t size) {
  auto copySize = MAX_PAYLOAD_SIZE < size ? MAX_PAYLOAD_SIZE : size;
  *_payloadSize = size;
  memcpy(_payload, data, copySize);
  _fcs = _payload + *_payloadSize;
  return copySize;
}

void VariableLengthPacket::UpdateFCS() {
  uint16_t crc = Checksum::crc16(_payload, *_payloadSize);
  *_fcs = (uint8_t)(crc >> 8);
  *(_fcs + 1) = (uint8_t)(crc & 0xff);
}

bool VariableLengthPacket::_CheckFCS() {
  uint16_t crc = Checksum::crc16(_payload, *_payloadSize + FCS_SIZE);
  return crc == 0;
}
bool VariableLengthPacket::IsOk() { return _CheckFCS(); }
PacketPtr VariableLengthPacket::Create(){
    return CreateObject<VariableLengthPacket>();
}
CLASS_LOADER_REGISTER_CLASS(VariableLengthPacketBuilder, IPacketBuilder)
}
