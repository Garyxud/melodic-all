/*
   drv_ms4525.c : driver for MS4525 differential pressure sensor

   This file is part of BreezySTM32.

   BreezySTM32 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BreezySTM32 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <breezystm32.h>

static const uint8_t ADDR = 0x28;
static uint8_t buf_[4];
static float diff_press_;
static float temp_;
static uint32_t next_update_ms_;
static uint32_t last_update_ms_;
static bool new_data_;
static bool sensor_present_;

static void cb(uint8_t result);


static inline float sign(float x)
{
  return (x > 0) - (x < 0);
}

bool ms4525_init()
{
  sensor_present_ = false;
  if (i2cWrite(ADDR, 0xFF, 0) == true)
    sensor_present_ = true;
  else
    sensor_present_ = false;
  next_update_ms_ = millis();    
  last_update_ms_ = millis();
  return sensor_present_;
}


void ms4525_async_update()
{
  if (millis() > next_update_ms_)
  {
    i2c_queue_job(READ, ADDR, 0xFF, buf_, 4, NULL, &cb);
    next_update_ms_ += 100;
  }
}

bool ms4525_present()
{
  if (sensor_present_ && millis() > last_update_ms_ + 1000)
    sensor_present_ = false;
  return sensor_present_;
}

void cb(uint8_t result)
{
  if (result != I2C_JOB_ERROR)
  {
    new_data_ = true;
    sensor_present_ = true;
  }
  next_update_ms_ += 20;
  last_update_ms_ = millis();
}

void ms4525_async_read(float* differential_pressure, float* temp)
{
  if (new_data_)
  {
    uint8_t status = (buf_[0] & 0xC0) >> 6;
    if(status == 0x00) // good data packet
    {
      int16_t raw_diff_pressure = 0x3FFF & ((buf_[0] << 8) + buf_[1]);
      int16_t raw_temp = ( 0xFFE0 & ((buf_[2] << 8) + buf_[3])) >> 5;
      // Convert to Pa and K
      diff_press_ = -(((float)(raw_diff_pressure) - 1638.3f) / 6553.2f - 1.0f) * 6894.757f;
      temp_ = ((200.0f * raw_temp) / 2047.0) - 50 ; // K
    }
    new_data_ = false;
  }
  (*differential_pressure) = diff_press_;
  (*temp) = temp_;
}

