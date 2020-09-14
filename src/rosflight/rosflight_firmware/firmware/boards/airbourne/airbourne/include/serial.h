/*
 * Copyright (c) 2017, James Jackson
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SERIAL_CLASS_H
#define SERIAL_CLASS_H

#include <stdint.h>
#include <functional>
#include "gpio.h"

#define RX_BUFFER_SIZE 512
#define TX_BUFFER_SIZE 512


class Serial
{
public:
  enum
  {
    POLLING = 0x00,
    INTERRUPT = 0x01,
    DMA_TX = 0x02,
    DMA_RX = 0x04
  };

  enum
  {
    UART = 0,
    VCP = 1
  };
  Serial(){}
  virtual void write(const uint8_t*ch, uint8_t len) = 0;
  virtual uint32_t rx_bytes_waiting() = 0;
  virtual uint32_t tx_bytes_free() = 0;
  virtual uint8_t read_byte() = 0;
  virtual bool tx_buffer_empty() = 0;
  virtual void put_byte(uint8_t ch) = 0;
  virtual bool flush() = 0;
  virtual void register_rx_callback(void (*cb)(uint8_t data) ) = 0;
  virtual void unregister_rx_callback() = 0;

protected:
  GPIO tx_pin_;
  GPIO rx_pin_;

  uint8_t mode_;

  std::function<void(uint8_t)> receive_CB_ = nullptr;

};

#endif // SERIAL CLASS_H
