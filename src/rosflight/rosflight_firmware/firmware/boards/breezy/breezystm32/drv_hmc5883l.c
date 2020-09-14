/*
   drv_hmc5883l.c :  Support for HMC5883L Magnetometer

   Adapted from https://github.com/rosflight/firmware/blob/master/src/hmc5883l.cpp

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

#include <math.h>


#define HMC58X3_ADDR 			0x1E
#define HMC58X3_CRA 			0x00
#define HMC58X3_CRB 			0x01
#define HMC58X3_MODE 			0x02
#define HMC58X3_DATA 			0x03
#define HMC58X3_STATUS 	  0x09
#define HMC58X3_ID1 			0x0A
#define HMC58X3_ID2 			0x0B
#define HMC58X3_ID3 			0x0C

#define HMC58X3_CRA_NO_AVG 		0x00
#define HMC58X3_CRA_AVG_2_MEAS 	0x20
#define HMC58X3_CRA_AVG_4_MEAS 	0x40
#define HMC58X3_CRA_AVG_8_MEAS 	0x60

#define HMC58X3_CRA_DO_0_75 	0x00
#define HMC58X3_CRA_DO_1_5 		0x04
#define HMC58X3_CRA_DO_3 		0x08
#define HMC58X3_CRA_DO_7_5 		0x0C
#define HMC58X3_CRA_DO_15 		0x10
#define HMC58X3_CRA_DO_30 		0x14
#define HMC58X3_CRA_DO_75 		0x18

#define HMC58X3_CRA_MEAS_MODE_NORMAL 	0x00
#define HMC58X3_CRA_MEAS_MODE_POS_BIAS 	0x01
#define HMC58X3_CRA_MEAS_MODE_NEG_BIAS 	0x02

#define HMC58X3_CRB_GN_1370 	0x00
#define HMC58X3_CRB_GN_1090 	0x20
#define HMC58X3_CRB_GN_820 		0x40
#define HMC58X3_CRB_GN_660 		0x60
#define HMC58X3_CRB_GN_440 		0x80
#define HMC58X3_CRB_GN_390 		0xA0
#define HMC58X3_CRB_GN_330 		0xC0
#define HMC58X3_CRB_GN_230 		0xE0

#define HMC58X3_MODE_HS 		0x80
#define HMC58X3_MODE_CONTINUOUS 0x00
#define HMC58X3_MODE_SINGLE 	0x01
#define HMC58X3_MODE_IDLE 		0x02

#define HMC58X3_SR_LOCK 		0x02
#define HMC58X3_SR_RDY 			0x01

#define HMC58X3_TIMEOUT 30000

static uint8_t i2c_buf_[6];
static volatile float data_[3];
static uint32_t last_update_ms_;
static uint32_t next_update_ms_;
static bool mag_present_;
static uint8_t cmd;

static void read_cb(uint8_t result);

bool hmc5883lInit()
{
  mag_present_ = false;
  
  // Wait for the chip to power up
  
  last_update_ms_ = millis();
  next_update_ms_ = millis();

  // Detect Magnetometer
  cmd = 0;
  if (i2cWrite(HMC58X3_ADDR, 0xFF, cmd) != true)
  {
    mag_present_ = false;
    return false;
  }
  else
  {
    bool result = true;
    // Configure HMC5833L
    result &= i2cWrite(HMC58X3_ADDR, HMC58X3_CRA, HMC58X3_CRA_DO_75 | HMC58X3_CRA_NO_AVG | HMC58X3_CRA_MEAS_MODE_NORMAL ); // 75 Hz Measurement, no bias, no averaging
    result &= i2cWrite(HMC58X3_ADDR, HMC58X3_CRB, HMC58X3_CRB_GN_390); // 390 LSB/Gauss
    result &= i2cWrite(HMC58X3_ADDR, HMC58X3_MODE, HMC58X3_MODE_CONTINUOUS); // Continuous Measurement Mode
    mag_present_ = true;
    return result;
  }
}

bool hmc5883l_present()
{
  if (mag_present_ && millis() > last_update_ms_ + 1000)
    mag_present_ = false;
  return mag_present_;
}

void hmc5883l_request_async_update()
{
  if ( millis() > next_update_ms_)
  {
    i2c_queue_job(READ, HMC58X3_ADDR, HMC58X3_DATA, i2c_buf_, 6, NULL, &read_cb);
    next_update_ms_ += 10;
  }
}

void read_cb(uint8_t result)
{
  if (result != I2C_JOB_ERROR)
    mag_present_ = true;
  last_update_ms_ = millis();
  data_[0] = (float)((int16_t)((i2c_buf_[0] << 8) | i2c_buf_[1]));
  data_[1] = (float)((int16_t)((i2c_buf_[2] << 8) | i2c_buf_[3]));
  data_[2] = (float)((int16_t)((i2c_buf_[4] << 8) | i2c_buf_[5]));
}


void hmc5883l_async_read(float *mag_data)
{
  mag_data[0] = data_[0];
  mag_data[1] = data_[1];
  mag_data[2] = data_[2];
}

