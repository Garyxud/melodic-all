/*
 * DataLinkFrame.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: diego
 */

#include <class_loader/multi_library_class_loader.hpp>
#include <cstdlib>
#include <cstring>
#include <dccomms/Checksum.h>
#include <dccomms/DataLinkFrame.h>

namespace dccomms {

static const unsigned char _manchesterPre[DLNK_PREAMBLE_SIZE] = {0x55, 0x55};

const unsigned char *DataLinkFrame::manchesterPre = _manchesterPre;

static void ThrowDLinkLayerException(std::string msg) {
  throw CommsException("DLINK EXCEPTION: " + msg,
                       COMMS_EXCEPTION_DLNKLAYER_ERROR);
}

void DataLinkFrame::_SetFcsType(fcsType fcst) {
  _fcstype = fcst;
  switch (_fcstype) {
  case crc16:
    _fcsSize = 2;
    break;

  case crc32:
    _fcsSize = 4;
    break;

  case nofcs:
    _fcsSize = 0;
    break;
  }
}
void DataLinkFrame::Init(DataLinkFrame::fcsType fcst) {
  _overheadSize = DLNK_PREAMBLE_SIZE + DLNK_DIR_SIZE * 2 + DLNK_DSIZE_SIZE;

  _BigEndian = DataLinkFrame::IsBigEndian();

  _SetFcsType(fcst);

  _overheadSize += _fcsSize;
  _AllocBuffer(_overheadSize + DLNK_MAX_PAYLOAD_SIZE);
  _pre = GetBuffer();
  _ddir = _pre + DLNK_PREAMBLE_SIZE;
  _sdir = _ddir + DLNK_DIR_SIZE;
  _dsize = (uint16_t *)(_sdir + DLNK_DIR_SIZE);
  _payload = ((uint8_t *)_dsize) + DLNK_DSIZE_SIZE;

  memcpy(_pre, DataLinkFrame::manchesterPre, DLNK_PREAMBLE_SIZE);
  _SetPayloadSizeInBuffer(0);
  _totalInfoSize = DLNK_DIR_SIZE * 2 + DLNK_DSIZE_SIZE;
  _dataIn = false;
}

DataLinkFrame::DataLinkFrame(DataLinkFrame::fcsType fcst) { Init(fcst); }

DataLinkFrame::DataLinkFrame(uint8_t desdir, uint8_t srcdir, uint16_t datasize,
                             uint8_t *data, fcsType fcst) {
  Init(fcst);

  *_ddir = desdir;
  *_sdir = srcdir;
  _SetPayloadSizeInBuffer(datasize);
  _fcs = ((uint8_t *)_payload) + _payloadSize;
  memcpy(_payload, data, _payloadSize);

  _calculateCRC();
  _dataIn = true;
}

static uint8_t *getBits(void *data, int length, void *bits) {

  unsigned char *sptr, *dptr, *maxsptr;

  for (sptr = (unsigned char *)data, dptr = (unsigned char *)bits,
      maxsptr = sptr + length;
       sptr < maxsptr; sptr++) {
    *dptr++ = *sptr & 1;
    *dptr++ = (*sptr >> 1) & 1;
    *dptr++ = (*sptr >> 2) & 1;
    *dptr++ = (*sptr >> 3) & 1;
    *dptr++ = (*sptr >> 4) & 1;
    *dptr++ = (*sptr >> 5) & 1;
    *dptr++ = (*sptr >> 6) & 1;
    *dptr++ = (*sptr >> 7) & 1;
  }
  return dptr;
}

uint8_t *DataLinkFrame::GetFrameBits(void *dst) {
  uint8_t *ptr = (uint8_t *)dst;
  ptr = getBits(_pre, _frameSize, ptr);
  return ptr;
}

DataLinkFrame::~DataLinkFrame() {
  //_buffer is deleted in parent destructor (Packet)
}

void DataLinkFrame::_calculateCRC() {
  switch (_fcstype) {
  case crc16:
    uint16_t crc1;
    crc1 = Checksum::crc16(_ddir, _totalInfoSize);
    crc1 = Checksum::crc16(_payload, _payloadSize, crc1);

    *_fcs = (uint8_t)(crc1 >> 8);
    *(_fcs + 1) = (uint8_t)(crc1 & 0x00ff);
    break;

  case crc32:
    uint32_t crc2;
    crc2 = Checksum::crc32(_ddir, _totalInfoSize);
    crc2 = Checksum::crc32(_payload, _payloadSize, crc2);

    *(_fcs) = (uint8_t)((crc2 >> 24) & 0x000000ff);
    *(_fcs + 1) = (uint8_t)((crc2 >> 16) & 0x000000ff);
    *(_fcs + 2) = (uint8_t)((crc2 >> 8) & 0x000000ff);
    *(_fcs + 3) = (uint8_t)(crc2 & 0x000000ff);
    break;
  //*(uint32_t*)fcs = crc2;
  case nofcs:
    break;
  }
}

bool DataLinkFrame::checkFrame() {
  switch (_fcstype) {
  case crc16:
    uint16_t crc1;
    crc1 = Checksum::crc16(_ddir, _totalInfoSize);
    crc1 = Checksum::crc16(_payload, _payloadSize, crc1);
    crc1 = Checksum::crc16(_fcs, _fcsSize, crc1);
    return crc1 == 0;
    break;

  case crc32:
    uint32_t crc2;
    crc2 = Checksum::crc32(_ddir, _totalInfoSize);
    crc2 = Checksum::crc32(_payload, _payloadSize, crc2);
    crc2 = Checksum::crc32(_fcs, _fcsSize, crc2);
    return crc2 == 0;
    break;

  case nofcs:
    return true;
    break;
  }
  return true;
}

void DataLinkFrame::_SetPayloadSizeInBuffer(unsigned int datasize) {
  if (datasize <= DLNK_MAX_PAYLOAD_SIZE) {
    _payloadSize = datasize;
    _frameSize = _overheadSize + _payloadSize;
    _fcs = ((uint8_t *)_payload) + _payloadSize;
    if (_BigEndian)
      *_dsize = _payloadSize;
    else {
      *(uint8_t *)_dsize = (uint8_t)(_payloadSize >> 8);
      *(((uint8_t *)_dsize) + 1) = (uint8_t)(_payloadSize & 0xff);
    }
  } else
    ThrowDLinkLayerException(
        std::string("El tamano del payload no puede ser mayor que ") +
        std::to_string(DLNK_MAX_PAYLOAD_SIZE));
}

void DataLinkFrame::PayloadUpdated(unsigned int datasize) {
  _SetPayloadSizeInBuffer(datasize);
  _calculateCRC();
}

uint32_t DataLinkFrame::SetPayload(uint8_t *data, uint32_t psize) {
  uint32_t bytesWritten =
      psize <= DLNK_MAX_PAYLOAD_SIZE ? psize : DLNK_MAX_PAYLOAD_SIZE;
  memcpy(_payload, data, bytesWritten);
  PayloadUpdated(bytesWritten);
  return bytesWritten;
}

void DataLinkFrame::UpdateFrame(uint8_t ddir,   // destination dir
                                uint8_t sdir,   // source dir
                                uint16_t dsize, // data size
                                uint8_t *data   // data
) {
  *_ddir = ddir;
  *_sdir = sdir;
  SetPayload(data, dsize);
}

void DataLinkFrame::SetSrcDir(uint8_t sdir) {
  *_sdir = sdir;
  _calculateCRC();
}

void DataLinkFrame::SetDesDir(uint8_t ddir) {
  *_ddir = ddir;
  _calculateCRC();
}

void DataLinkFrame::GetInfoFromBufferWithPreamble(void *o) {
  GetInfoFromBuffer((uint8_t *)o + DLNK_PREAMBLE_SIZE);
}

void DataLinkFrame::GetInfoFromBuffer(void *o) {
  uint8_t *optr = (uint8_t *)o;

  memcpy(this->_ddir, optr, DLNK_DIR_SIZE);
  optr += DLNK_DIR_SIZE;

  memcpy(this->_sdir, optr, DLNK_DIR_SIZE);
  optr += DLNK_DIR_SIZE;

  memcpy((uint8_t *)this->_dsize, optr, DLNK_DSIZE_SIZE);
  optr += DLNK_DSIZE_SIZE;

  if (this->_BigEndian) {
    this->_payloadSize = *this->_dsize;
  } else {
    this->_payloadSize = ((*this->_dsize) << 8) | ((*this->_dsize) >> 8);
  }

  if (this->_payloadSize > DLNK_MAX_PAYLOAD_SIZE) {
    ThrowDLinkLayerException(
        std::string("El tamano del payload no puede ser mayor que ") +
        std::to_string(DLNK_MAX_PAYLOAD_SIZE));
  }

  memcpy(this->_payload, optr, this->_payloadSize);
  optr += this->_payloadSize;

  _fcs = ((uint8_t *)_payload) + _payloadSize;
  memcpy(this->_fcs, optr, this->_fcsSize);
  optr += this->_fcsSize;

  this->_frameSize = this->_overheadSize + this->_payloadSize;
  _dataIn = true;
}

void DataLinkFrame::printFrame(std::ostream &o) {
  o << std::hex;
  o << "Preamble: 0x";

  uint8_t *p = _pre;
  for (int i = 0; i < DLNK_PREAMBLE_SIZE; i++) {
    o.width(2);
    o.fill('0');
    o << (int)*p;
    p++;
  }
  o << std::endl;
  o << "Dest. Dir: 0x";
  p = _ddir;
  for (int i = 0; i < DLNK_DIR_SIZE; i++) {
    o.width(2);
    o.fill('0');
    o << (uint32_t)*p;
    p++;
  }
  o << std::endl;
  o << "Source. Dir: 0x";
  p = _sdir;
  for (int i = 0; i < DLNK_DIR_SIZE; i++) {
    o.width(2);
    o.fill('0');
    o << (uint32_t)*p;
    p++;
  }

  o << std::endl;

  if (_dataIn) {
    o << "Data size: 0x";
    p = (uint8_t *)_dsize;
    for (int i = 0; i < DLNK_DSIZE_SIZE; i++) {
      o.width(2);
      o.fill('0');
      o << (int)*p;
      p++;
    }

    uint16_t psize;
    if (_BigEndian) {
      psize = *_dsize;
    } else {
      psize = ((*_dsize) << 8) | ((*_dsize) >> 8);
    }
    o << std::endl << std::dec << "Data (";
    o << psize << " bytes): 0x" << std::hex;
    p = _payload;
    for (int i = 0; i < psize; i++) {
      o.width(2);
      o.fill('0');
      o << (int)*p;
      p++;
    }

    o << std::endl << "FCS: 0x";
    p = _fcs;
    for (unsigned int i = 0; i < _fcsSize; i++) {
      o.width(2);
      o.fill('0');
      o << (int)*p;
      p++;
    }
    o << " (0x";
    o.width(_fcsSize * 2);
    if (_fcsSize == 2)
      o << *(uint16_t *)_fcs << ")" << std::endl;
    else if (_fcsSize == 4)
      o << *(uint32_t *)_fcs << ")" << std::endl;

    o << std::dec;
  } else {
    o << "No data in frame!" << std::endl;
  }
}

bool DataLinkFrame::IsBigEndian() {
  uint32_t word = 0x1;
  uint8_t *byte = (uint8_t *)&word;
  return *byte != 0x1;
}

DataLinkFramePtr DataLinkFrame::BuildDataLinkFrame(fcsType fcst) {
  return DataLinkFramePtr(new DataLinkFrame(fcst));
}

DataLinkFramePtr DataLinkFrame::BuildDataLinkFrame(uint8_t desdir,
                                                   uint8_t srcdir,
                                                   uint16_t datasize,
                                                   uint8_t *data,
                                                   fcsType fcst) {
  return DataLinkFramePtr(
      new DataLinkFrame(desdir, srcdir, datasize, data, fcst));
}

DataLinkFramePtr DataLinkFrame::Copy(DataLinkFramePtr src) {
  auto dlf = BuildDataLinkFrame(src->GetFcsType());
  dlf->GetInfoFromBufferWithPreamble(src->GetFrameBuffer());
  return dlf;
}

void DataLinkFrame::DoCopyFromRawBuffer(void *buffer) {
  GetInfoFromBufferWithPreamble(buffer);
}

void DataLinkFrame::Read(Stream *comms) {
  comms->WaitFor((const uint8_t *)_pre, DLNK_PREAMBLE_SIZE);

  comms->Read(_ddir, DLNK_DIR_SIZE);
  comms->Read(_sdir, DLNK_DIR_SIZE);

  comms->Read((uint8_t *)_dsize, DLNK_DSIZE_SIZE);

  if (_BigEndian) {
    _payloadSize = *_dsize;
  } else {
    _payloadSize = ((*_dsize) << 8) | ((*_dsize) >> 8);
  }

  if (_payloadSize > DLNK_MAX_PAYLOAD_SIZE) {
    throw CommsException(
        std::string(
            "DLNKLAYER_ERROR: El tamano del payload no puede ser mayor que ") +
            std::to_string(DLNK_MAX_PAYLOAD_SIZE),
        COMMS_EXCEPTION_DLNKLAYER_ERROR);
  }

  comms->Read(_payload, _payloadSize);

  _fcs = ((uint8_t *)_payload) + _payloadSize;
  comms->Read(_fcs, _fcsSize);

  _frameSize = _overheadSize + _payloadSize;
}
PacketPtr DataLinkFrame::Create() {
  return DataLinkFrame::BuildDataLinkFrame(_fcstype);
}
DataLinkFramePacketBuilder::DataLinkFramePacketBuilder(
    DataLinkFrame::fcsType fcstype)
    : _fcsType(fcstype){};

PacketPtr DataLinkFramePacketBuilder::CreateFromBuffer(void *buffer) {
  auto dlf = DataLinkFrame::BuildDataLinkFrame(_fcsType);
  dlf->GetInfoFromBufferWithPreamble(buffer);
  return dlf;
}

PacketPtr DataLinkFramePacketBuilder::Create() {
  return DataLinkFrame::BuildDataLinkFrame(_fcsType);
}

CLASS_LOADER_REGISTER_CLASS(DataLinkFrameBuilderCRC16,
                            DataLinkFramePacketBuilder)

} // namespace dccomms
