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


#include "rc_sbus.h"
#include <functional>

RC_SBUS* rc_ptr;
void rx_callback(uint8_t byte)
{
    rc_ptr->read_cb(byte);
}

void RC_SBUS::init(GPIO* inv_pin, UART *uart)
{
  rc_ptr = this;
  uart_ = uart;
  inv_pin_ = inv_pin;

  uart_->set_mode(100000, UART::MODE_8E2);
  uart_->register_rx_callback(rx_callback);

  // turn on the serial inverter
  inv_pin_->write(GPIO::HIGH);

  decode_buffer();
}

float RC_SBUS::read(uint8_t channel)
{
  return (static_cast<float>(raw_[channel]) - 172.0)/1639.0;
}

bool RC_SBUS::lost()
{
  return millis() > frame_start_ms_ + 100 || failsafe_status_ != SBUS_SIGNAL_OK;
}

void RC_SBUS::decode_buffer()
{
  frame_start_ms_ = millis();

  // process actual sbus data, use union to decode
  raw_[0]  = sbus_union_.frame.chan0;
  raw_[1]  = sbus_union_.frame.chan1;
  raw_[2]  = sbus_union_.frame.chan2;
  raw_[3]  = sbus_union_.frame.chan3;
  raw_[4]  = sbus_union_.frame.chan4;
  raw_[5]  = sbus_union_.frame.chan5;
  raw_[6]  = sbus_union_.frame.chan6;
  raw_[7]  = sbus_union_.frame.chan7;
  raw_[8]  = sbus_union_.frame.chan8;
  raw_[9]  = sbus_union_.frame.chan9;
  raw_[10] = sbus_union_.frame.chan10;
  raw_[11] = sbus_union_.frame.chan11;
  raw_[12] = sbus_union_.frame.chan12;
  raw_[13] = sbus_union_.frame.chan13;
  raw_[14] = sbus_union_.frame.chan14;
  raw_[15] = sbus_union_.frame.chan15;

  // Digital Channel 1
  if (sbus_union_.frame.digichannels & (1<<0))
    raw_[16] = 1811;
  else
    raw_[16] = 172;

  // Digital Channel 2
  if (sbus_union_.frame.digichannels & (1<<1))
    raw_[17] = 1811;
  else
    raw_[17] = 172;

  // Failsafe
  failsafe_status_ = SBUS_SIGNAL_OK;
//  if (sbus_union_.frame.digichannels & (1<<2))
//    failsafe_status_ = SBUS_SIGNAL_LOST;
  if (sbus_union_.frame.digichannels & (1<<3))
    failsafe_status_ = SBUS_SIGNAL_FAILSAFE;
}

void RC_SBUS::read_cb(uint8_t byte)
{
  if (byte == START_BYTE && prev_byte_ == END_BYTE)
  {
    buffer_pos_ = 0;
    frame_started_ = true;
    frame_start_ms_ = millis();
  }

  if (frame_started_)
  {
    sbus_union_.data[buffer_pos_++] = byte;

    if (buffer_pos_ == 25)
    {
      // If we have a complete packet, decode
      if (sbus_union_.frame.endByte == END_BYTE && sbus_union_.frame.startByte == START_BYTE)
        decode_buffer();
      else
        errors_++;
      frame_started_ = false;
    }
  }
  prev_byte_ = byte;
}

