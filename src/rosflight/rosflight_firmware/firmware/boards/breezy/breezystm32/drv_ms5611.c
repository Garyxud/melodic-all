/*
   drv_ms5611.c : driver for Measurement Specialties MS5611 barometer

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_ms5611.c

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
#include <limits.h>

#include <math.h>

// MS5611, Standard address 0x77
static const uint8_t ADDR = 0x77;
static const uint8_t PROM_RD = 0xA0; // Prom read command

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8


#define REBOOT_PERIOD_MS 1000 * 60 * 30 // reboot the device every 30 minutes

static uint8_t cmd;

static void master_cb(uint8_t result);

uint8_t pres_buf_[3];
uint8_t temp_buf_[3];
int32_t pres_raw_;
int32_t temp_raw_;
float pressure_;
float temperature_;
uint16_t prom[8];
uint32_t next_update_ms_;
uint32_t next_reboot_ms_;
uint32_t last_update_ms_;
bool waiting_for_cb_;
bool new_data_;
bool baro_present_;

typedef enum
{
 START_TEMP = 0,
 READ_TEMP = 1,
 START_PRESS = 2,
 READ_PRESS = 3,
} state_t;
static state_t state_;


typedef enum
{
  CB_TEMP_READ1,
  CB_TEMP_READ2,
  CB_PRES_READ1,
  CB_PRES_READ2,
  CB_TEMP_START,
  CB_PRES_START,
  CB_RESET,
  CB_WRITE_ZERO,
} callback_type_t;
static callback_type_t callback_type_;

static void reset(void)
{
  i2cWrite(ADDR, CMD_RESET, 1);
  delayMicroseconds(2800);
}

static bool read_prom()
{
  uint8_t buf[2] = {0, 0};
  
  // try a few times
  for (int i = 0; i < 8; i++)
  {
    i2cWrite(ADDR, 0xFF, PROM_RD + 2* i);
    if (i2cRead(ADDR, 0xFF, 2, buf) == true)
      prom[i] = (uint16_t)(buf[0] << 8 | buf[1]);
    else
    {
      reset();
      delay(3);
      i2cWrite(0, 0, 0);
      delay(3);
      // didn't work, try again
      return false;
    }
  }
  return true;
}

int8_t calc_crc()
{
  uint32_t res = 0;
  uint8_t crc = prom[7] & 0xF;
  prom[7] &= 0xFF00;

  bool blank = true;

  for (int i = 0; i < 16; i++)
  {
    if (prom[i >> 1])
    {
      blank = false;
    }
    if (i & 1)
      res ^= ((prom[i >> 1]) & 0x00FF);
    else
      res ^= (prom[i >> 1] >> 8);
    for (int j = 8; j > 0; j--)
    {
      if (res & 0x8000)
        res ^= 0x1800;
      res <<= 1;
    }
  }
  prom[7] |= crc;
  if (!blank && crc == ((res >> 12) & 0xF))
    return 0;

  return -1;
}


bool ms5611_init()
{
  baro_present_ = false;
  while (millis() < 10); // wait for chip to power on
  
  next_update_ms_ = millis();
  last_update_ms_ = millis();
  
  i2cWrite(0, 0, 0); // cycle the I2C clk signal (from the datasheet)
  delay(1);
  if (i2cWrite(ADDR, RESET, 1) != true)
  {
    baro_present_ = false;
    return false;
  }
  else
  {
    baro_present_ = true;
  }
  delay(3);
  
  // Read the PROM (try a couple of times if it fails)
  bool got_valid_prom = false;
  for (int i = 0; i < 5; i++)
  {
    if (read_prom() == true)
    {
      if (calc_crc() != 0)
        continue;
      else
      {
        got_valid_prom = true;
        break;
      }
    }
  }
  
  if (got_valid_prom)
  {
    state_ = START_TEMP;
    new_data_ = false;  
    baro_present_ = true;    
    next_reboot_ms_ = REBOOT_PERIOD_MS;
    waiting_for_cb_ = false;
    return true;
  }
  else
  {
    return false;
  }
}

bool ms5611_present()
{
  if (baro_present_ && waiting_for_cb_ && (millis() > last_update_ms_ + 200))
      baro_present_ = false;
  return baro_present_;
}

static void convert()
{
  int32_t press = 0;
  int32_t temp = 0;
  int64_t delta = 0;
  temp_raw_ = (temp_buf_[0] << 16) | (temp_buf_[1] << 8) | temp_buf_[2];
  pres_raw_ = (pres_buf_[0] << 16) | (pres_buf_[1] << 8) | pres_buf_[2];
  if(pres_raw_ > 9085466 * 2 / 3 && temp_raw_ > 0)
  {  
    int32_t dT = temp_raw_ - ((int32_t)(prom[5]) << 8);
    int64_t off = ((int64_t)(prom[2]) << 16) + (((int64_t)(prom[4]) * dT) >> 7);
    int64_t sens = ((int64_t)(prom[1]) << 15) + (((int64_t)(prom[3]) * dT) >> 8);
    temp = 2000 + ((dT * (int64_t)(prom[6])) >> 23);

    // temperature lower than 20degC
    if (temp < 2000)
    {
      delta = temp - 2000;
      delta = 5 * delta * delta;
      off -= delta >> 1;
      sens -= delta >> 2;

      // temperature lower than -15degC
      if (temp < -1500)
      {
        delta = temp + 1500;
        delta = delta * delta;
        off -= 7 * delta;
        sens -= (11 * delta) >> 1;
      }

      temp -= ((dT * dT) >> 31);
    }

    press = ((((uint64_t)(pres_raw_) * sens) >> 21) - off) >> 15;
    
    pressure_ = (float)(press); // Pa
    temperature_ = (float)(temp) / 100.0 + 273.0; // K
  }
  new_data_ = false;
}

void temp_read_cb1(uint8_t result)
{
  (void) result;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  callback_type_ = CB_TEMP_READ2;
  i2c_queue_job(READ, ADDR, 0xFF, temp_buf_, 3, NULL, &master_cb);  
}

void pres_read_cb1(uint8_t result)
{
  (void) result;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  callback_type_ = CB_PRES_READ2;
  i2c_queue_job(READ, ADDR, 0xFF, pres_buf_, 3, NULL, &master_cb);  
}


void temp_read_cb2(uint8_t result)
{
  (void) result;
  state_ = START_PRESS;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
  new_data_ = true;
}

void pres_read_cb2(uint8_t result)
{
  (void) result;
  state_ = START_TEMP;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
  new_data_ = true;
}

void temp_start_cb(uint8_t result)
{
  (void) result;
  state_ = READ_TEMP;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
}

void pres_start_cb(uint8_t result)
{
  (void) result;
  state_ = READ_PRESS;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
}

void reset_cb(uint8_t result)
{
  (void) result;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
  next_reboot_ms_ = last_update_ms_ + REBOOT_PERIOD_MS;
  waiting_for_cb_ = false;
  callback_type_ = CB_WRITE_ZERO;
  uint8_t data = 0;
  i2c_queue_job(WRITE, 0, 0xFF, &data, 1, NULL, &master_cb);
}

void write_zero_cb(uint8_t result)
{
  (void) result;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
  next_reboot_ms_ = last_update_ms_ + REBOOT_PERIOD_MS;
  waiting_for_cb_ = false;
  state_ = START_TEMP;
}

void master_cb(uint8_t result)
{
  if (result != I2C_JOB_ERROR)
    baro_present_ = true;
  switch (callback_type_)
  {
  case CB_TEMP_READ1:
    temp_read_cb1(result);
    break;
  case CB_TEMP_READ2:
    temp_read_cb2(result);
    break;
  case CB_PRES_READ1:
    pres_read_cb1(result);
    break;
  case CB_PRES_READ2:
    pres_read_cb2(result);
    break;
  case CB_TEMP_START:
    temp_start_cb(result);
    break;
  case CB_PRES_START:
    pres_start_cb(result);
    break;
  case CB_RESET:
    reset_cb(result);
    break;
  case CB_WRITE_ZERO:
    write_zero_cb(result);
    break;
  }
}

bool start_temp_meas()
{
  waiting_for_cb_ = true;
  last_update_ms_ = millis();
  callback_type_ = CB_TEMP_START;
  cmd = CMD_ADC_CONV + CMD_ADC_D2 + CMD_ADC_4096;
  i2c_queue_job(WRITE, ADDR, 0xFF, &cmd, 1, NULL, &master_cb);
  return true;
}

bool start_pres_meas()
{
  waiting_for_cb_ = true;
  last_update_ms_ = millis();
  callback_type_ = CB_PRES_START;
  cmd = CMD_ADC_CONV + CMD_ADC_D1 + CMD_ADC_4096;
  i2c_queue_job(WRITE, ADDR, 0xFF, &cmd, 1, NULL, &master_cb);
  return true;
}

bool read_pres_mess()
{
  waiting_for_cb_ = true;
  last_update_ms_ = millis();
  callback_type_ = CB_PRES_READ1;
  cmd = CMD_ADC_READ;
  i2c_queue_job(WRITE, ADDR, 0xFF, &cmd, 1, NULL, &master_cb);
  return true;
}

bool read_temp_mess()
{
  waiting_for_cb_ = true;
  last_update_ms_ = millis();
  callback_type_ = CB_TEMP_READ1;
  cmd = CMD_ADC_READ;
  i2c_queue_job(WRITE, ADDR, 0xFF, &cmd, 1, NULL, &master_cb);
  return true;
}


void ms5611_async_update()
{
  uint32_t now_ms = millis();
  
  // Sometimes the barometer fails to respond.  If this happens, then reset it
  // the barometer also seems to stop responding after 72 minutes (suspiciously close to a overflow of uint32_t with a microsecond timer)
  // to avoid that, just reboot periodically
  if ((waiting_for_cb_ && now_ms) > last_update_ms_ + 20 || (now_ms > next_reboot_ms_))
  {
    last_update_ms_ = now_ms;
    callback_type_ = CB_RESET;
    uint8_t command = RESET;
    i2c_queue_job(READ, ADDR, 0xFF, &command, 1, NULL, &master_cb);
  }

  else if (now_ms > next_update_ms_)
  {
    switch (state_)
    {
    case START_TEMP:
      if (start_temp_meas())
        next_update_ms_ += 100;
      break;
    case READ_TEMP:
      if (read_temp_mess())
        next_update_ms_ += 100;
      break;
    case START_PRESS:
      if (start_pres_meas())
        next_update_ms_ += 100;
      break;
    case READ_PRESS:
      if (read_pres_mess())
        next_update_ms_ += 100;
      break;
    default:
      state_ = START_TEMP;
      break;
    }
  }
  
  if (new_data_)
  {
    convert();
  }
}

void ms5611_async_read(float *pressure, float *temperature)
{
  (*pressure) = pressure_;
  (*temperature) = temperature_;
}
