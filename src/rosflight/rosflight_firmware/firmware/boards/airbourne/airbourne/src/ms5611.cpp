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

#include "ms5611.h"

MS5611* baro_ptr;
static void cb(uint8_t result);

#define REBOOT_PERIOD_MS 1000 * 60 * 30 // reboot the device every 30 minutes

bool MS5611::init(I2C* _i2c)
{
  baro_ptr = this;
  i2c_ = _i2c;
  baro_present_ = false;
  while (millis() < 10);  // wait for chip to power on
  
  next_update_ms_ = millis();
  last_update_ms_ = millis();
  
  i2c_->write(0, 0, 0);
  delay(1);
  if (i2c_->write(ADDR, RESET, 1) != I2C::RESULT_SUCCESS)
  {
    baro_present_ = false;
    return false;
  }
  else
  {
    baro_present_ = true;
  }

  delay(3);

  // Read the PROM (try a couple times if it fails)
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
    return true;
  }
  else
  {
    return false;
  }
}

bool MS5611::present()
{
  if (baro_present_ && waiting_for_cb_ && (millis() > last_update_ms_ + 200))
      baro_present_ = false;
  return baro_present_;
}

void MS5611::update()
{
  uint32_t now_ms = millis();
  
  // Sometimes the barometer fails to respond.  If this happens, then reset it
  // the barometer also seems to stop responding after 72 minutes (suspiciously close to a overflow of uint32_t with a microsecond timer)
  // to avoid that, just reboot periodically
  if ((waiting_for_cb_ && now_ms) > last_update_ms_ + 20 || (now_ms > next_reboot_ms_))
  {
    last_update_ms_ = now_ms;
    callback_type_ = CB_RESET;
    i2c_->write(ADDR, RESET, 1, &cb, false);
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


void MS5611::reset()
{

}

bool MS5611::read_prom()
{
  uint8_t buf[2] = {0, 0};
  
  // try a few times
  for (int i = 0; i < 8; i++)
  {
    i2c_->write(ADDR, 0xFF, PROM_RD + 2* i);
    if (i2c_->read(ADDR, 0xFF, 2, buf, nullptr, true) == I2C::RESULT_SUCCESS)
      prom[i] = static_cast<uint16_t>(buf[0] << 8 | buf[1]);
    else
    {
      reset();
      delay(3);
      i2c_->write(0, 0, 0);
      delay(3);
      // didn't work, try again
      return false;
    }
  }
  return true;
}

int8_t MS5611::calc_crc()
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

//int8_t MS5611::calc_crc()
//{    
//  uint16_t n_rem;                      // crc reminder 
//  uint16_t crc_read;                   // original value of the crc 
//  uint8_t  n_bit; 
//  n_rem = 0x00; 
//  crc_read=prom[7];                      //save read CRC 
//  prom[7]=(0xFF00 & (prom[7]));        //CRC byte is replaced by 0 
//  for (uint8_t cnt = 0; cnt < 16; cnt++)                 // operation is performed on bytes 
//  {// choose LSB or MSB 
//    if (cnt%2==1) n_rem ^= ((prom[cnt>>1]) & 0x00FF); 
//    else n_rem ^=  (prom[cnt>>1]>>8); 
//    for (n_bit = 8; n_bit > 0; n_bit--) 
//    { 
//      if (n_rem & (0x8000)) 
//      { 
//        n_rem = (n_rem << 1) ^ 0x3000; 
//      } 
//      else 
//      { 
//        n_rem = (n_rem << 1); 
//      } 
//    } 
//  } 
//  n_rem=  (0x000F & (n_rem >> 12));       // final 4-bit reminder is CRC code 
//  prom[7]=crc_read;               // restore the crc_read to its original place 
//  return (n_rem ^ 0x0); 
//}   

void MS5611::convert()
{
  int32_t press = 0;
  int32_t temp = 0;
  int64_t delta = 0;
  temp_raw_ = (temp_buf_[0] << 16) | (temp_buf_[1] << 8) | temp_buf_[2];
  pres_raw_ = (pres_buf_[0] << 16) | (pres_buf_[1] << 8) | pres_buf_[2];
  if(pres_raw_ > 9085466 * 2 / 3 && temp_raw_ > 0)
  {  
    int32_t dT = temp_raw_ - (static_cast<int32_t>(prom[5]) << 8);
    int64_t off = (static_cast<int64_t>(prom[2]) << 16) + ((static_cast<int64_t>(prom[4]) * dT) >> 7);
    int64_t sens = (static_cast<int64_t>(prom[1]) << 15) + ((static_cast<int64_t>(prom[3]) * dT) >> 8);
    temp = 2000 + ((dT * static_cast<int64_t>(prom[6])) >> 23);

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

    press = (((static_cast<uint64_t>(pres_raw_) * sens) >> 21) - off) >> 15;
    
    pressure_ = static_cast<float>(press); // Pa
    temperature_ = static_cast<float>(temp) / 100.0 + 273.0; // K
  }
  new_data_ = false;
}

bool MS5611::start_temp_meas()
{
  waiting_for_cb_ = true;
  last_update_ms_ = millis();
  callback_type_ = CB_TEMP_START;
  return i2c_->write(ADDR, 0xFF, ADC_CONV + ADC_D2 + ADC_4096, &cb) > 0;
}

bool MS5611::start_pres_meas()
{
  waiting_for_cb_ = true;
  last_update_ms_ = millis();
  callback_type_ = CB_PRES_START;
  return i2c_->write(ADDR, 0XFF, ADC_CONV + ADC_D1 + ADC_4096, &cb) > 0;
}

bool MS5611::read_pres_mess()
{
  waiting_for_cb_ = true;
  last_update_ms_ = millis();
  callback_type_ = CB_PRES_READ1;
  return i2c_->write(ADDR, 0xFF, ADC_READ, &cb) > 0;
}

bool MS5611::read_temp_mess()
{
  waiting_for_cb_ = true;
  last_update_ms_ = millis();
  callback_type_ = CB_TEMP_READ1;
  return (i2c_->write(ADDR, 0xFF, ADC_READ, &cb) > 0);
}

void MS5611::temp_read_cb1(uint8_t result)
{
  (void) result;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  callback_type_ = CB_TEMP_READ2;
  i2c_->read(ADDR, 0xFF, 3, temp_buf_, &cb);  
}

void MS5611::pres_read_cb1(uint8_t result)
{
  (void) result;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  callback_type_ = CB_PRES_READ2;
  i2c_->read(ADDR, 0xFF, 3, pres_buf_, &cb);  
}


void MS5611::temp_read_cb2(uint8_t result)
{
  (void) result;
  state_ = START_PRESS;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
  new_data_ = true;
}

void MS5611::pres_read_cb2(uint8_t result)
{
  (void) result;
  state_ = START_TEMP;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
  new_data_ = true;
}

void MS5611::temp_start_cb(uint8_t result)
{
  (void) result;
  state_ = READ_TEMP;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
}

void MS5611::pres_start_cb(uint8_t result)
{
  (void) result;
  state_ = READ_PRESS;
  waiting_for_cb_ = false;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
}

void MS5611::reset_cb(uint8_t result)
{
  (void) result;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
  next_reboot_ms_ = last_update_ms_ + REBOOT_PERIOD_MS;
  waiting_for_cb_ = false;
  callback_type_ = CB_WRITE_ZERO;
  i2c_->write(0, 0, 0, &cb, false);
}

void MS5611::write_zero_cb(uint8_t result)
{
  (void) result;
  last_update_ms_ = millis();
  next_update_ms_ = last_update_ms_ + 10;
  next_reboot_ms_ = last_update_ms_ + REBOOT_PERIOD_MS;
  waiting_for_cb_ = false;
  state_ = START_TEMP;
}

void MS5611::read(float * press, float* temp)
{
  (*press) = pressure_;
  (*temp) = temperature_;
}

void MS5611::master_cb(uint8_t result)
{
  if (result == I2C::RESULT_SUCCESS)
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

void cb(uint8_t result)
{
  baro_ptr->master_cb(result);
}
