#include <class_loader/multi_library_class_loader.hpp>
#include <dccomms_packets/VariableLength2BPacket.h>

namespace dccomms_packets {
VariableLength2BPacket::VariableLength2BPacket() {
  _overheadSize = PRE_SIZE + PAYLOAD_SIZE_FIELD + FCS_SIZE;
  _maxPacketSize = _overheadSize + MAX_PAYLOAD_SIZE;
  _AllocBuffer(_maxPacketSize);
  _Init();
}

void VariableLength2BPacket::_Init() {
  _pre = GetBuffer();
  *_pre = 0x55;
  _payloadSize = (uint16_t *)(_pre + 1);
  _SetPayloadSizeField(0);
  _payload = (uint8_t *)(_payloadSize + 1);
  _fcs = _payload + _GetPayloadSize();
}

void VariableLength2BPacket::DoCopyFromRawBuffer(void *buffer) {
  uint8_t *ownbuf = GetBuffer();
  uint8_t *curptr = (uint8_t*) buffer;
  uint32_t headSize = PRE_SIZE + PAYLOAD_SIZE_FIELD;
  memcpy(ownbuf, curptr, headSize);
  curptr += headSize;
  uint16_t payloadSize = _GetPayloadSize();
  memcpy(_payload, curptr, payloadSize + FCS_SIZE);
}

inline uint8_t *VariableLength2BPacket::GetPayloadBuffer() { return _payload; }

inline uint32_t VariableLength2BPacket::GetPayloadSize() {
  return _GetPayloadSize();
}

inline int VariableLength2BPacket::GetPacketSize() {
  return _overheadSize + _GetPayloadSize();
}

void VariableLength2BPacket::Read(Stream *stream) {
  stream->WaitFor(_pre, PRE_SIZE);
  stream->Read(_payloadSize, PAYLOAD_SIZE_FIELD);
  auto psize = _GetPayloadSize();
  stream->Read(_payload, psize + FCS_SIZE);
}
void VariableLength2BPacket::_SetPayloadSizeField(uint16_t payloadSize) {
  if (_bigEndian)
    *_payloadSize = payloadSize;
  else {
    *(uint8_t *)_payloadSize = (uint8_t)(payloadSize >> 8);
    *(((uint8_t *)_payloadSize) + 1) = (uint8_t)(payloadSize & 0xff);
  }
}

uint16_t VariableLength2BPacket::_GetPayloadSize() {
  uint16_t result;
  if (_bigEndian) {
    result = *_payloadSize;
  } else {
    result = ((*_payloadSize) << 8) | ((*_payloadSize) >> 8);
  }
  result = result <= MAX_PAYLOAD_SIZE ? result : 0; // if psize > MAX_PAYLOAD_SIZE ==> error on packet
  return result;
}

void VariableLength2BPacket::PayloadUpdated(uint32_t payloadSize) {
  payloadSize = payloadSize <= MAX_PAYLOAD_SIZE ? payloadSize : MAX_PAYLOAD_SIZE;
  _SetPayloadSizeField(payloadSize);
  _fcs = _payload + payloadSize;
  UpdateFCS();
}

void VariableLength2BPacket::GetPayload(void *copy, int size) {
  uint32_t psize = _GetPayloadSize();
  auto copySize = psize < size ? psize : size;
  memcpy(copy, _payload, copySize);
}

uint32_t VariableLength2BPacket::SetPayload(uint8_t *data, uint32_t size) {
  auto copySize = size <= MAX_PAYLOAD_SIZE ? size : MAX_PAYLOAD_SIZE;
  _SetPayloadSizeField(copySize);
  memcpy(_payload, data, copySize);
  _fcs = _payload + copySize;
  return copySize;
}

void VariableLength2BPacket::UpdateFCS() {
  uint32_t psize = _GetPayloadSize();
  uint16_t crc = Checksum::crc16(_payload, psize);
  *_fcs = (uint8_t)(crc >> 8);
  *(_fcs + 1) = (uint8_t)(crc & 0xff);
}

bool VariableLength2BPacket::_CheckFCS() {
  uint32_t psize = _GetPayloadSize();
  uint16_t crc = Checksum::crc16(_payload, psize + FCS_SIZE);
  return crc == 0;
}
bool VariableLength2BPacket::IsOk() { return _CheckFCS(); }
PacketPtr VariableLength2BPacket::Create(){
    return CreateObject<VariableLength2BPacket>();
}
CLASS_LOADER_REGISTER_CLASS(VariableLength2BPacketBuilder, IPacketBuilder)
}
