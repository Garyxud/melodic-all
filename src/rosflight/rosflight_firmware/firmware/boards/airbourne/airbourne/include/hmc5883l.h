/*
 * Copyright (c) 2017, James Jackson and Tyler Miller
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

#pragma once

#include "system.h"
#include "i2c.h"

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


class HMC5883L
{
public:
  bool init(I2C* i2c_drv);
  void update();
  bool read(float mag_data[]);
  bool present();
  void cb(uint8_t result);

private:
  I2C* i2c_;
  uint8_t i2c_buf_[6];
  volatile float data_[3];
  uint32_t last_update_ms_;
  uint32_t next_update_ms_;
  bool mag_present_;
};
