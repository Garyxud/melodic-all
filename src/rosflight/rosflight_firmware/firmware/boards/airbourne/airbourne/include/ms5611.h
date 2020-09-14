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

#ifndef MS5611_H
#define MS5611_H

#include "system.h"
#include "i2c.h"


class MS5611
{
private:

  enum : uint8_t
  {
    RESET    = 0x1E, // ADC reset command
    ADC_READ = 0x00, // ADC read command
    ADC_CONV = 0x40, // ADC conversion command
    ADC_D1   = 0x00, // ADC D1 conversion
    ADC_D2   = 0x10, // ADC D2 conversion
    ADC_256  = 0x00, // ADC OSR=256
    ADC_512  = 0x02, // ADC OSR=512
    ADC_1024 = 0x04, // ADC OSR=1024
    ADC_2048 = 0x06, // ADC OSR=2048
    ADC_4096 = 0x08, // ADC OSR=4096
    PROM_RD  = 0xA0 // Prom read command
  };

  typedef enum
  {
    START_TEMP = 0,
    READ_TEMP = 1,
    START_PRESS = 2,
    READ_PRESS = 3,
  } state_t;
  state_t state_;
  
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

  static const uint8_t ADDR = 0x77;

  void reset();
  bool read_prom();
  int8_t calc_crc();
  bool read_pres_mess();
  bool read_temp_mess();
  bool start_temp_meas();
  bool start_pres_meas();
  void convert();

  I2C* i2c_;
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
  
  callback_type_t callback_type_;

public:
  bool init(I2C* _i2c);
  void update();
  void read(float *press, float *temp);
  bool present();

  void master_cb(uint8_t result);
  void temp_read_cb1(uint8_t result);
  void pres_read_cb1(uint8_t result);
  void temp_read_cb2(uint8_t result);
  void pres_read_cb2(uint8_t result);
  void temp_start_cb(uint8_t result);
  void pres_start_cb(uint8_t result);
  void write_zero_cb(uint8_t result);
  void reset_cb(uint8_t result);
};


#endif // MS5611_H
