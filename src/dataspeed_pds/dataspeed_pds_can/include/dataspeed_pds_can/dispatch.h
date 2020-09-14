/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef _DATASPEED_PDS_CAN_DISPATCH_H
#define _DATASPEED_PDS_CAN_DISPATCH_H
#include <stdint.h>

namespace dataspeed_pds_can
{

typedef struct {
  // Master and slave status messages have the same structure.
  // 4 Reserved/Unused bits.
  uint8_t :4;
  // Inverter Last Request (bool)
  uint8_t inverter_request :1;
  // Inverter Status (bool)
  uint8_t inverter_status :1;
  // Inverter Overloading (bool)
  uint8_t inverter_overload :1;
  // Inverter Over Temperature (bool)
  uint8_t inverter_overtemp :1;
  // Mode (enum)
  //   The current mode of the PDS.
  uint8_t mode :4;
  // Script (enum)
  //   The currently running script on the PDS, if any.
  uint8_t script :4;
  // Channel Status X (enum)
  //   The current status of channel X on the PDS.
  // Union for master channels 1-12 and slave channels 13-24
  union { struct {uint8_t status_01 :4; uint8_t status_02 :4;}; struct {uint8_t status_13 :4; uint8_t status_14 :4;}; };
  union { struct {uint8_t status_03 :4; uint8_t status_04 :4;}; struct {uint8_t status_15 :4; uint8_t status_16 :4;}; };
  union { struct {uint8_t status_05 :4; uint8_t status_06 :4;}; struct {uint8_t status_17 :4; uint8_t status_18 :4;}; };
  union { struct {uint8_t status_07 :4; uint8_t status_08 :4;}; struct {uint8_t status_19 :4; uint8_t status_20 :4;}; };
  union { struct {uint8_t status_09 :4; uint8_t status_10 :4;}; struct {uint8_t status_21 :4; uint8_t status_22 :4;}; };
  union { struct {uint8_t status_11 :4; uint8_t status_12 :4;}; struct {uint8_t status_23 :4; uint8_t status_24 :4;}; };
} MsgStatus1;

typedef struct {
  // Master and slave status messages have the same structure.
  // Board Temperature (float32)
  //   Internal board temperature, in Celsius.
  int8_t board_temp;
  // Thermocouple Temperature (float32)
  //   Thermocouple temperature, in Celsius.
  int8_t thermocouple_temp;
  // Voltage (float32)
  //   Voltage, in V.
  uint16_t voltage :12;
  // 4 Reserved/Unused bits.
  uint8_t :4;
} MsgStatus2;

typedef struct {
  // Union for multiple messages with the same structure: Current1Master, Current2Master, Current3Master, Current1Slave, Current2Slave, Current3Slave
  union { int16_t current_01; int16_t current_05; int16_t current_09; int16_t current_13; int16_t current_17; int16_t current_21; };
  union { int16_t current_02; int16_t current_06; int16_t current_10; int16_t current_14; int16_t current_18; int16_t current_22; };
  union { int16_t current_03; int16_t current_07; int16_t current_11; int16_t current_15; int16_t current_19; int16_t current_23; };
  union { int16_t current_04; int16_t current_08; int16_t current_12; int16_t current_16; int16_t current_20; int16_t current_24; };
} MsgCurrent;

typedef struct {
  uint8_t channel;
  uint8_t request;
} MsgRelay;

typedef struct {
  uint8_t mode;
} MsgMode;

typedef struct {
  uint8_t script;
} MsgScript;

#define BUILD_ASSERT(cond) do { (void) sizeof(char [1 - 2*!(cond)]); } while(0)
static void dispatchAssertSizes() {
  BUILD_ASSERT(8 == sizeof(MsgCurrent));
  BUILD_ASSERT(8 == sizeof(MsgStatus1));
  BUILD_ASSERT(4 == sizeof(MsgStatus2));
  BUILD_ASSERT(2 == sizeof(MsgRelay));
  BUILD_ASSERT(1 == sizeof(MsgMode));
  BUILD_ASSERT(1 == sizeof(MsgScript));
}
#undef BUILD_ASSERT

enum {
  ID_REQUEST   = 0x410,
  ID_MODE      = 0x411,
  ID_SCRIPT    = 0x412,
  ID_RESERVED1 = 0x413,
  ID_RESERVED2 = 0x430,
  ID_RESERVED3 = 0x431,
  ID_RESERVED4 = 0x432,
  ID_STATUS1_MASTER  = 0x420,
  ID_STATUS1_SLAVE1  = 0x421,
  ID_STATUS1_SLAVE2  = 0x422,
  ID_STATUS1_SLAVE3  = 0x423,
  ID_CURRENT1_MASTER = 0x424,
  ID_CURRENT1_SLAVE1 = 0x425,
  ID_CURRENT1_SLAVE2 = 0x426,
  ID_CURRENT1_SLAVE3 = 0x427,
  ID_CURRENT2_MASTER = 0x428,
  ID_CURRENT2_SLAVE1 = 0x429,
  ID_CURRENT2_SLAVE2 = 0x42A,
  ID_CURRENT2_SLAVE3 = 0x42B,
  ID_CURRENT3_MASTER = 0x42C,
  ID_CURRENT3_SLAVE1 = 0x42D,
  ID_CURRENT3_SLAVE2 = 0x42E,
  ID_CURRENT3_SLAVE3 = 0x42F,
  ID_STATUS2_MASTER  = 0x43C,
  ID_STATUS2_SLAVE1  = 0x43D,
  ID_STATUS2_SLAVE2  = 0x43E,
  ID_STATUS2_SLAVE3  = 0x43F,
};

} //namespace dataspeed_pds_can

#endif // _DATASPEED_PDS_CAN_DISPATCH_H
