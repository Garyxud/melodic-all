/*
 * DataLinkFrame.h
 *
 *  Created on: Feb 12, 2015
 *      Author: diego
 */

#ifndef DCCOMMS_DATALINKFRAME_H_
#define DCCOMMS_DATALINKFRAME_H_

#include <boost/shared_ptr.hpp>
#include <cstring>
#include <dccomms/CommsException.h>
#include <dccomms/DataLinkFrame.h>
#include <dccomms/IPacketBuilder.h>
#include <dccomms/Packet.h>
#include <ostream>

namespace dccomms {

#define DLNK_PREAMBLE_SIZE 2
#define DLNK_DIR_SIZE 1
#define DLNK_DSIZE_SIZE 2
#define DLNK_MAX_PAYLOAD_SIZE 2048

class DataLinkFrame;

typedef std::shared_ptr<DataLinkFrame> DataLinkFramePtr;

class DataLinkFrame : public Packet {
  friend class IStreamCommsDevice;

public:
  enum fcsType { crc16, crc32, nofcs };

  static DataLinkFramePtr BuildDataLinkFrame(fcsType fcst);
  static DataLinkFramePtr BuildDataLinkFrame(uint8_t,   // destination dir
                                             uint8_t,   // source dir
                                             uint16_t,  // data size
                                             uint8_t *, // data
                                             fcsType    // fcstype
                                             );

  DataLinkFrame(fcsType fcst);
  DataLinkFrame(uint8_t,   // destination dir
                uint8_t,   // source dir
                uint16_t,  // data size
                uint8_t *, // data
                fcsType    // fcstype
                );

  static DataLinkFramePtr Copy(DataLinkFramePtr src);

  ~DataLinkFrame();
  inline uint8_t GetDesDir() { return *_ddir; }
  inline uint8_t GetSrcDir() { return *_sdir; }

  int GetFrameSize() const { return _frameSize; }
  uint8_t *GetFrameBuffer() const { return GetBuffer(); }
  fcsType GetFcsType() const { return _fcstype; }

  virtual void SetDst(uint32_t ddir) { SetDesDir(ddir); }
  virtual void SetSrc(uint32_t sdir) { SetSrcDir(sdir); }
  void SetDesDir(uint8_t _ddir);
  void SetSrcDir(uint8_t _sdir);

  void UpdateFrame(uint8_t,  // destination dir
                   uint8_t,  // source dir
                   uint16_t, // data size
                   uint8_t * // data
                   );

  void PayloadUpdated(unsigned int datasize);

  void GetInfoFromBuffer(void *);
  void GetInfoFromBufferWithPreamble(void *o);

  uint8_t *GetFrameBits(void *dst);

  void printFrame(std::ostream &);
  bool checkFrame();

  static bool IsBigEndian();
  static const unsigned char *manchesterPre;

  void DoCopyFromRawBuffer(void *buffer);
  inline uint8_t *GetPayloadBuffer() { return _payload; }
  inline uint32_t GetPayloadSize() { return _payloadSize; }
  inline int GetPacketSize() { return _frameSize; }
  void Read(Stream *comms);
  inline bool IsOk() { return checkFrame(); }

  inline uint32_t GetDst() { return GetDesDir(); }
  inline uint32_t GetSrc() { return GetSrcDir(); }
  inline bool IsBroadcast() { return GetDst() == 255; }

  uint32_t SetPayload(uint8_t *data, uint32_t psize);
  PacketPtr Create();

private:
  void Init(DataLinkFrame::fcsType fcst);
  void _SetFcsType(fcsType fcst);

  uint8_t *_pre, *_ddir, *_sdir, *_fcs;
  uint16_t *_dsize;

  uint16_t _overheadSize = 0;

  fcsType _fcstype;
  uint32_t _fcsSize = 0;

  int _frameSize = 0;

  uint16_t _payloadSize = 0;
  uint8_t *_payload = NULL;

  uint8_t _totalInfoSize = 0;

  void _calculateCRC();
  void _SetPayloadSizeInBuffer(unsigned int datasize);
  bool _BigEndian;
  bool _dataIn = false;
};

class DataLinkFramePacketBuilder : public IPacketBuilder {
public:
  DataLinkFramePacketBuilder(DataLinkFrame::fcsType fcstype);
  PacketPtr CreateFromBuffer(void *buffer);
  PacketPtr Create();

private:
  DataLinkFrame::fcsType _fcsType;
};

class DataLinkFrameBuilderCRC16 : public DataLinkFramePacketBuilder {
public:
  DataLinkFrameBuilderCRC16()
      : DataLinkFramePacketBuilder(DataLinkFrame::fcsType::crc16) {}
  std::string GetName() { return "DataLinkFrameBuilderCRC16"; }
};
} /* namespace radiotransmission */

#endif /* DCCOMMS_DATALINKFRAME_H_ */
