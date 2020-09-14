/*
 * Copyright (c) 2017, James Jackson amd Trey Henrichsen
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

#include "mb1242.h"

I2CSonar* sonarPtr;

void _I2C_Sonar_start_read_cb(uint8_t result)
{
  (void)result;
  sonarPtr->cb_start_read(result);
}

void _I2C_Sonar_finished_read_cb(uint8_t result)
{
  (void)result;
  sonarPtr->cb_finished_read(result);
}

I2CSonar::I2CSonar()
{
  sonarPtr = this;
}

void I2CSonar::init(I2C *_i2c)
{
  i2c_ = _i2c;
  new_data_ = false;
  value_ = 0;
  last_update_ms_ = millis();
  ready_to_ping_ = true;
  if (i2c_->write(MB1242_DEFAULT_ADDRESS, MB1242_DEFAULT_REGISTER, MB1242_PING_COMMAND) == I2C::RESULT_SUCCESS)
  {
    sensor_present_ = true;
    last_callback_ms_ = millis();
  }
  else
  {
    sensor_present_ = false;
    last_callback_ms_ = 0;
  }
}

bool I2CSonar::present()
{
  if (sensor_present_ && millis() > last_callback_ms_ + 500)
  {
    sensor_present_ = false;
  }
  return sensor_present_;
}

// Tries to either start a measurement, or read it from the sensor
// Does nothing if it has done something in the last UPDATE_WAIT_MILLIS ms
// Feel free to call it more often, though.
void I2CSonar::update()
{
  uint64_t now=millis();
  if (now > (last_update_ms_ + MB1242_UPDATE_WAIT_MILLIS))
  {
    last_update_ms_ = now;
    if (ready_to_ping_)
      i2c_->write(MB1242_DEFAULT_ADDRESS, MB1242_DEFAULT_REGISTER, MB1242_PING_COMMAND, &_I2C_Sonar_start_read_cb);
    else
      i2c_->read(MB1242_DEFAULT_ADDRESS, MB1242_DEFAULT_REGISTER, 2, buffer_, &_I2C_Sonar_finished_read_cb);
  }
}

//Returns the most recent reading
//It is during this method that the reading is converted to meters, as well
//If there has not yet been a successful reading, returns 0
float I2CSonar::read()
{
  if (new_data_)
  {
    uint16_t centimeters = buffer_[0] << 8 | buffer_[1];//Convert to a single number
#ifdef MB1242_RAW
    value=(float)centimeters * 0.01;
#else
    //Calibration from BreezySTM32 by Simon D. Levy
    value_=(1.071 * static_cast<float>(centimeters) + 3.103) / 100.0;
#endif
    new_data_=false;
  }
  return value_;
}

//callback after the measure command has been sent to the sensor
void I2CSonar::cb_start_read(uint8_t result)
{
  if (result == I2C::RESULT_SUCCESS)
  {
    sensor_present_ = true;
    last_callback_ms_ = millis();
    ready_to_ping_ = false;
  }
}

//callback after reading from the sensor has finished
void I2CSonar::cb_finished_read(uint8_t result)
{
  if (result == I2C::RESULT_SUCCESS)
  {
    sensor_present_ = true;
    new_data_ = true;
    last_callback_ms_ = millis();
    ready_to_ping_ = true;
  }
}
