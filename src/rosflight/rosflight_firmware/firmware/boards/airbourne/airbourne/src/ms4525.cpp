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

#include "ms4525.h"

MS4525* as_ptr;

static void cb(uint8_t result)
{
  as_ptr->read_cb(result);
}

MS4525::MS4525(){}

bool MS4525::init(I2C *_i2c)
{
  as_ptr = this;
  i2c_ = _i2c;
  sensor_present_ = false;
  uint8_t buf[1];
  if (i2c_->read(ADDR, 0xFF, buf) == I2C::RESULT_SUCCESS)
    sensor_present_ = true;
  else
    sensor_present_ = false;
  next_update_ms_ = 0;    
  return sensor_present_;
}

bool MS4525::present()
{
  if (sensor_present_ && millis() > last_update_ms_ + 200)
    sensor_present_ = false;
  return sensor_present_;
}

void MS4525::update()
{
  if (millis() > next_update_ms_)
  {
    if (i2c_->read(ADDR, 0xFF, 4, buf_, &cb) == I2C::RESULT_SUCCESS)
      next_update_ms_ += 100;
  }
}

void MS4525::read_cb(uint8_t result)
{
  if (result == I2C::RESULT_SUCCESS)
  {
    new_data_ = true;
    sensor_present_ = true;
  }
  next_update_ms_ = millis() + 20;
  last_update_ms_ = millis();
}

void MS4525::read(float* differential_pressure, float* temp)
{
  if (new_data_)
  {
    uint8_t status = (buf_[0] & 0xC0) >> 6;
    if(status == 0x00) // good data packet
    {
      int16_t raw_diff_pressure = 0x3FFF & ((buf_[0] << 8) + buf_[1]);
      int16_t raw_temp = ( 0xFFE0 & ((buf_[2] << 8) + buf_[3])) >> 5;
      // Convert to Pa and K
      diff_press_ = -((static_cast<float>(raw_diff_pressure) - 1638.3f) / 6553.2f - 1.0f) * 6894.757f;
      temp_ = ((200.0f * raw_temp) / 2047.0) - 50 ; // K
    }
    new_data_ = false;
  }
  (*differential_pressure) = diff_press_;
  (*temp) = temp_;
}

