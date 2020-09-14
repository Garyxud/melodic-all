/*
 * This packet is a copy of wireless_ardusub::SimplePacket (wireless_ardusub
 * project)
 */

#ifndef DCCOMMS_PACKETS_SIMPLEPACKET_H_
#define DCCOMMS_PACKETS_SIMPLEPACKET_H_

#include <dccomms/dccomms.h>
#include <dccomms_packets/types.h>

using namespace dccomms;
namespace dccomms_packets {

class SimplePacket : public Packet {
public:
  SimplePacket(int payloadSize, FCS fcs = CRC16);
  void DoCopyFromRawBuffer(void *buffer);
  uint8_t *GetPayloadBuffer();
  uint32_t GetPayloadSize();
  int GetPacketSize();
  void Read(Stream *comms);
  void PayloadUpdated(uint32_t payloadSize);
  void SetSeq(const uint32_t &seq);
  uint32_t GetSeq();
  void SetDst(const uint32_t &ddir);
  uint32_t GetDst();
  void SetSrc(const uint32_t &sdir);
  uint32_t GetSrc();


  bool IsOk();

  void GetPayload(void *copy, int size);
  uint32_t SetPayload(uint8_t *data, uint32_t size);

  void UpdateFCS();
  PacketPtr Create();

private:
  int PAYLOAD_SIZE, FCS_SIZE;
  static const int PRE_SIZE = 1;

  uint8_t *_pre;
  uint8_t *_payload;
  uint8_t *_seqByte0, *_seqByte1;
  uint8_t *_fcs;
  int _packetSize;
  void _Init();
  bool _CheckFCS();
};

class SimplePacketBuilder : public IPacketBuilder {
public:
  SimplePacketBuilder(int payloadSize, FCS fcs = CRC16) {
    _payloadSize = payloadSize;
    _fcs = fcs;
  }
  PacketPtr CreateFromBuffer(void *buffer) {
    auto pkt = CreateObject<SimplePacket>(_payloadSize, _fcs);
    pkt->CopyFromRawBuffer(buffer);
    return pkt;
  }
  PacketPtr Create() { return CreateObject<SimplePacket>(_payloadSize, _fcs); }

private:
  int _payloadSize;
  FCS _fcs;
};

class SimplePacketBuilder20crc16 : public SimplePacketBuilder {
public:
  SimplePacketBuilder20crc16() : SimplePacketBuilder(20, FCS::CRC16) {}
};

// Used by Operator in wirelress_ardusub
class SimplePacketBuilder9crc16 : public SimplePacketBuilder {
public:
  SimplePacketBuilder9crc16() : SimplePacketBuilder(9, FCS::CRC16) {}
};

// Used by ROV in wirelress_ardusub
class SimplePacketBuilder109crc16 : public SimplePacketBuilder {
public:
  SimplePacketBuilder109crc16() : SimplePacketBuilder(109, FCS::CRC16) {}
};
} // namespace dccomms_packets

#endif // DCCOMMS_PACKETS_SIMPLEPACKET_H_
