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

#ifndef UART_H
#define UART_H

// from serial.h
#include <functional>

#include "system.h"

#include "serial.h"
#include "gpio.h"

class UART : public Serial
{
public:

  typedef enum{
    MODE_8N1,
    MODE_8E2
  } uart_mode_t;

  UART();
  void init(const uart_hardware_struct_t *conf, uint32_t baudrate_, uart_mode_t mode=MODE_8N1);

  void write(const uint8_t*ch, uint8_t len) override;
  uint32_t rx_bytes_waiting() override;
  uint32_t tx_bytes_free() override;
  uint8_t read_byte() override;
  bool set_mode(uint32_t baud, uart_mode_t mode);
  bool tx_buffer_empty() override;
  void put_byte(uint8_t ch) override;
  bool flush() override;
  void register_rx_callback(void (*cb)(uint8_t data) ) override;
  void unregister_rx_callback() override;

  void DMA_Tx_IRQ_callback();
  void DMA_Rx_IRQ_callback();
  void USART_IRQ_callback();

private:
  void init_UART(uint32_t baudrate_, uart_mode_t mode = MODE_8N1);
  void init_DMA();
  void init_NVIC();
  void startDMA();

  const uart_hardware_struct_t* c_; //contains config information

  uint32_t baudrate_; //the baudrate for the connection
  uint8_t rx_buffer_[RX_BUFFER_SIZE]; //the buffer for incoming data
  uint8_t tx_buffer_[TX_BUFFER_SIZE]; //the buffer for outgoing data
  uint16_t rx_buffer_head_;
  uint16_t rx_buffer_tail_;
  uint16_t tx_buffer_head_;
  uint16_t tx_buffer_tail_;
  GPIO rx_pin_; //The pin used for incoming data
  GPIO tx_pin_; //The pin used for outgoing data
};

#endif // UART_H
