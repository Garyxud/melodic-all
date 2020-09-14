/*
 * BlockRadioTransmitter.h
 *
 *  Created on: 01/03/2015
 *      Author: diego
 */

#ifndef DCCOMMS_BLOCKRADIOTRANSMITTER_H_
#define DCCOMMS_BLOCKRADIOTRANSMITTER_H_
#include <dccomms/Radio.h>
namespace dccomms {

class BlockRadioTransmitter {
public:
  BlockRadioTransmitter(Radio &);
  void Send(const char *identifier, void *buffer, uint32_t size, uint8_t dest,
            uint32_t packetSize = 0, unsigned long ms = 50);
  uint32_t Receive(const char *identifier, void *buffer, unsigned long ms = 0);
  virtual ~BlockRadioTransmitter();
  bool BusyTransmitting();

private:
  Radio &radio;
  bool _bigEndian;
};
}
#endif /* BLOCKRADIOTRANSMITTER_H_ */
