/*
 * Radio.h
 *
 *  Created on: Feb 11, 2015
 *      Author: diego
 */

#ifndef DCCOMMS_RADIO_H_
#define DCCOMMS_RADIO_H_

#include <cpplogging/Loggable.h>
#include <dccomms/CommsDevice.h>
#include <dccomms/DataLinkFrame.h>

namespace dccomms {
class Radio : public cpplogging::Loggable {
public:
  enum fcsType {
    crc16 = DataLinkFrame::crc16,
    crc32 = DataLinkFrame::crc32,
    nofcs = DataLinkFrame::nofcs
  };

  ~Radio();
  Radio(unsigned char dir, CommsDevice &, fcsType fcst = fcsType::crc32,
        uint32_t maxRxBufferSize = 5000);
  void SendBytes(const void *, uint32_t size, uint8_t dirTo = 255,
                 uint32_t packetSize = 1000, unsigned long ms = 0);
  void ReceiveBytes(void *, uint32_t size, uint8_t dirFrom = 255,
                    unsigned long ms = 10000);
  bool BusyTransmitting();

  int TotalErrors = 0;

private:
  CommsDevice &serial;
  unsigned char dir;

  uint32_t _maxRxBufferSize;
  uint8_t *_rxBuffer;
  uint32_t _rxBufferFirstPos;
  uint32_t _rxBufferLastPos;
  uint32_t _bytesInBuffer = 0;

  DataLinkFrame::fcsType FCSType;

  void _DecreaseBytesInBuffer();
  void _IncreaseBytesInBuffer();
};
}

#endif /* DCCOMMS_RADIO_H_ */
