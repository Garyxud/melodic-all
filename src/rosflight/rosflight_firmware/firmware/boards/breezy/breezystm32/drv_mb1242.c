/*
   drv_mb1242.c : driver for MaxBotix MB1242 sonar

   Copyright (C) 2016 Simon D. Levy

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

#include <stdbool.h>
#include <stdint.h>

#include "drv_i2c.h"

#define MB1242_DEFAULT_ADDRESS 0x70

static const uint8_t START_MEASUREMENT_CMD = 0x51;

static bool sensor_present_;
static uint32_t last_update_time_ms_;
static uint32_t next_update_time_ms_;
static uint8_t buf_[2];
static uint32_t distance_cm_;
static float distance_;
static bool new_data_ = false;
static uint8_t cmd_;
typedef enum
{
  START_MEAS,
  READ_DATA
} sonar_state_t;
static sonar_state_t state_;
typedef enum 
{
  START_MEAS_CB,
  READ_DATA_CB
} callack_type_t;
static callack_type_t callback_type_;

static void cb(uint8_t result);
static void convert();

bool mb1242_init()
{
  last_update_time_ms_ = millis();
  next_update_time_ms_ = millis();
  if (i2cWrite(MB1242_DEFAULT_ADDRESS, 0xFF, START_MEASUREMENT_CMD))
    sensor_present_ = true;
  else
    sensor_present_ = false;
  state_ = START_MEAS;
  return sensor_present_;
}

bool mb1242_present()
{
  if (sensor_present_ && millis() > last_update_time_ms_ + 1000)
    sensor_present_ = false;
  return sensor_present_;
}

void mb1242_async_update()
{
  uint32_t now_ms = millis();
  if (now_ms > next_update_time_ms_)
  {
    next_update_time_ms_ = now_ms +25;
    switch (state_)
    {
    case START_MEAS:
      callback_type_ = START_MEAS_CB;
      cmd_ = START_MEASUREMENT_CMD;
      i2c_queue_job(WRITE, MB1242_DEFAULT_ADDRESS, 0xFF, &cmd_, 1, NULL, &cb);
      break;
    case READ_DATA:
      callback_type_ = READ_DATA_CB;
      i2c_queue_job(READ, MB1242_DEFAULT_ADDRESS, 0xFF, buf_, 2, NULL, &cb);
      break;
    }
  }
  if (new_data_)
    convert();
}

float mb1242_async_read()
{
  return distance_;
}


void cb(uint8_t result)
{
  if (result != I2C_JOB_ERROR)
  {
    sensor_present_ = true;
    last_update_time_ms_ = millis();
    
    switch (callback_type_)
    {
      case START_MEAS_CB:
        state_ = READ_DATA;
        break;
      case READ_DATA_CB:
        state_ = START_MEAS;
        new_data_ = true;
        break;
    }
  }
}

void convert()
{
  distance_cm_ = (buf_[0] << 8) + buf_[1];
  if (distance_cm_ > 850)
  {
      distance_cm_ = 0;
  }
  distance_ = (1.071 * (float)distance_cm_ + 3.103)/100.0;
}

