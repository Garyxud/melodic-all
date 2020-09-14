#include <cstring>
#include <dccomms/Packet.h>
#include <stdint.h>
#include <stdlib.h>

namespace dccomms {

Packet::Packet() {
  _buffer = NULL;
  _ownBuffer = true;
  uint32_t word = 0x1;
  uint8_t *byte = (uint8_t *)&word;
  _bigEndian = *byte != 0x1;
}

Packet::~Packet() { _FreeBuffer(); }

void Packet::_FreeBuffer() {
  if (_ownBuffer && _buffer) {
    delete _buffer;
    _buffer = NULL;
  }
}

void Packet::CopyFromRawBuffer(void *buffer) {
  memcpy(_va, reinterpret_cast<uint8_t *>(buffer) + _maxRealAreaSize,
         VIRTUAL_AREA_SIZE);
  DoCopyFromRawBuffer(buffer);
}

void Packet::_AllocBuffer(int size) {
  _FreeBuffer();
  _maxRealAreaSize = static_cast<uint32_t>(size);
  _bufferSize = _maxRealAreaSize + VIRTUAL_AREA_SIZE;
  _buffer = new uint8_t[_bufferSize];
  _va = _buffer + _maxRealAreaSize;
  _virtual_seq = reinterpret_cast<uint32_t *>(_va);
  _virtual_src = _virtual_seq + 1;
  _virtual_dst = _virtual_src + 1;
  _ownBuffer = true;
}

uint32_t Packet::GetBufferSize() { return _bufferSize; }

uint32_t Packet::SetPayload(uint8_t *data) {
  uint32_t psize = GetPayloadSize();
  uint8_t *pb = GetPayloadBuffer();
  memcpy(pb, data, psize);
  PayloadUpdated(psize);
  return psize;
}

uint32_t Packet::SetPayload(const char *data) {
  uint32_t psize = strlen(data);
  uint8_t *pb = GetPayloadBuffer();
  memcpy(pb, data, psize);
  PayloadUpdated(psize);
  return psize;
}

uint32_t Packet::SetPayload(const std::string &data) {
  uint32_t psize = data.size();
  uint8_t *pb = GetPayloadBuffer();
  memcpy(pb, data.c_str(), psize);
  PayloadUpdated(psize);
  return psize;
}

void Packet::Write(Stream *comms) {
  comms->Write(GetBuffer(), GetPacketSize());
}

void Packet::_SetBuffer(void *buffer) {
  _FreeBuffer();
  _buffer = (uint8_t *)buffer;
  _ownBuffer = false;
}

bool Packet::IsOk() { return true; }

PacketPtr Packet::CreateCopy() {
  PacketPtr pkt = Create();
  pkt->CopyFromRawBuffer(GetBuffer());
  return pkt;
}
} // namespace dccomms
