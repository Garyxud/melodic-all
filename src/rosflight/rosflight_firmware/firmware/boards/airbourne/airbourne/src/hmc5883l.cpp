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


#include "hmc5883l.h"

static HMC5883L* mag_ptr;
static void read_cb(uint8_t result);

bool HMC5883L::init(I2C *i2c_drv)
{
  mag_ptr = this;
  mag_present_ = false;
  i2c_ = i2c_drv;
  
  // Wait for the chip to power up
  
  last_update_ms_ = millis();
  next_update_ms_ = millis();

  // Detect Magnetometer
  uint8_t byte = 0;
  if (i2c_->read(HMC58X3_ADDR, 0xFF, &byte) != I2C::RESULT_SUCCESS)
  {
    mag_present_ = false;
    return false;
  }
  else
  {
    bool result = true;
    // Configure HMC5833L
    result &= i2c_->write(HMC58X3_ADDR, HMC58X3_CRA, HMC58X3_CRA_DO_75 | HMC58X3_CRA_NO_AVG | HMC58X3_CRA_MEAS_MODE_NORMAL ); // 75 Hz Measurement, no bias, no averaging
    result &= i2c_->write(HMC58X3_ADDR, HMC58X3_CRB, HMC58X3_CRB_GN_390); // 390 LSB/Gauss
    result &= i2c_->write(HMC58X3_ADDR, HMC58X3_MODE, HMC58X3_MODE_CONTINUOUS); // Continuous Measurement Mode
    mag_present_ = true;
    return result;
  }
}

bool HMC5883L::present()
{
  if (mag_present_ && millis() > last_update_ms_ + 200)
    mag_present_ = false;
  return mag_present_;
}

void HMC5883L::update()
{
  if ( millis() > next_update_ms_)
  {
    if (i2c_->read(HMC58X3_ADDR, HMC58X3_DATA, 6, i2c_buf_, &read_cb) == I2C::RESULT_SUCCESS)
      next_update_ms_ += 10;
  }
}

void HMC5883L::cb(uint8_t result)
{
  if (result == I2C::RESULT_SUCCESS)
    mag_present_ = true;
  last_update_ms_ = millis();
  data_[0] = static_cast<float>(static_cast<int16_t>((i2c_buf_[0] << 8) | i2c_buf_[1]));
  data_[1] = static_cast<float>(static_cast<int16_t>((i2c_buf_[2] << 8) | i2c_buf_[3]));
  data_[2] = static_cast<float>(static_cast<int16_t>((i2c_buf_[4] << 8) | i2c_buf_[5]));
}

void read_cb(uint8_t result)
{
  mag_ptr->cb(result);
}

bool HMC5883L::read(float mag_data[3])
{
  mag_data[0] = data_[0];
  mag_data[1] = data_[1];
  mag_data[2] = data_[2];

  //if the mag's ADC over or underflows, then the data register is given the value of -4096
  //the data register can also be assigned -4096 if there's a math overflow during bias calculation
  return mag_data[0] != -4096 && mag_data[1] != -4096 && mag_data[2] != -4096;
}
