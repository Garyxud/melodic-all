#ifndef DCCOMMS_PACKET_H
#define DCCOMMS_PACKET_H

#include <dccomms/Stream.h>
#include <iostream>
#include <memory>

namespace dccomms {

class Packet;
typedef std::shared_ptr<Packet> PacketPtr;

class Packet {
public:
  Packet();
  virtual ~Packet();
  inline uint8_t *GetBuffer() const { return _buffer; }

  void CopyFromRawBuffer(void *buffer);
  virtual void DoCopyFromRawBuffer(void *buffer) = 0;
  virtual uint8_t *GetPayloadBuffer() = 0;
  virtual uint32_t GetPayloadSize() = 0;
  virtual int GetPacketSize() = 0;
  virtual void Read(Stream *comms) = 0;
  virtual PacketPtr Create() = 0;
  virtual PacketPtr CreateCopy();

  // This method could update the FCS or attributes of the subclass
  virtual void PayloadUpdated(uint32_t payloadSize = 0) = 0;

  // This method calls SetPayload(data, GetPayloadSize()) and
  // PayloadUpdated(GetPayloadSize())
  // Then returns the number of bytes written
  virtual uint32_t SetPayload(uint8_t *data);
  virtual uint32_t SetPayload(const char * data);
  virtual uint32_t SetPayload(const std::string & data);

  // It tries to set datasize bytes in the payload buffer and returns the number
  // of bytes written
  virtual uint32_t SetPayload(uint8_t *data, uint32_t datasize) = 0;

  virtual bool IsOk();
  virtual void Write(Stream *comms);
  virtual bool IsBroadcast() { return true; }
  virtual uint32_t GetSeq() { return GetVirtualSeq(); }
  virtual uint32_t GetDst() { return GetVirtualDestAddr(); }
  virtual uint32_t GetSrc() { return GetVirtualSrcAddr(); }

  virtual uint32_t GetVirtualSeq() { return *_virtual_seq; }
  virtual uint32_t GetVirtualDestAddr() { return *_virtual_dst; }
  virtual uint32_t GetVirtualSrcAddr() { return *_virtual_src; }

  virtual void SetSeq(const uint32_t &seq) { SetVirtualSeq(seq); }
  virtual void SetDst(const uint32_t &ddir) { SetVirtualDestAddr(ddir); }
  virtual void SetSrc(const uint32_t &sdir) { SetVirtualSrcAddr(sdir); }

  virtual void SetVirtualSeq(const uint32_t &seq) { *_virtual_seq = seq; }
  virtual void SetVirtualDestAddr(const uint32_t &ddir) {
    *_virtual_dst = ddir;
  }
  virtual void SetVirtualSrcAddr(const uint32_t &sdir) { *_virtual_src = sdir; }

  uint32_t GetBufferSize();

protected:
  void _AllocBuffer(int size);
  virtual void _SetBuffer(void *buffer);
  bool _bigEndian;
  uint8_t *_va;
  uint32_t *_virtual_seq, *_virtual_src, *_virtual_dst;
  static const uint32_t VIRTUAL_AREA_SIZE = 4 + 4 + 4;

private:
  uint8_t *_buffer;
  bool _ownBuffer;
  uint32_t _maxRealAreaSize;
  uint32_t _bufferSize;
  void _FreeBuffer();
}; // namespace dccomms
} // namespace dccomms
#endif // DCCOMMS_PACKET_H
