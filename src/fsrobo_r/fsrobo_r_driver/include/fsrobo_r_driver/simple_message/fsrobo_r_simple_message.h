/*********************************************************************
* FSRobo-R Package BSDL
* ---------
* Copyright (C) 2019 FUJISOFT. All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef FSROBO_R_DRIVER_SIMPLE_MESSAGE_FSROBO_R_SIMPLE_MESSAGE_H
#define FSROBO_R_DRIVER_SIMPLE_MESSAGE_FSROBO_R_SIMPLE_MESSAGE_H

#include "simple_message/simple_message.h"

namespace fsrobo_r_driver
{
namespace simple_message
{
/**
 * \brief Enumeration of FSRobo-R-specific message types.
 *        See simple_message.h for a listing of "standard" message types
 */
namespace FSRoboRMsgTypes
{
enum FSRoboRMsgType
{
  SET_IO = 9001,
  SET_IO_REPLY = 9002,
  IO_STATE = 9003,
  EXECUTE_PROGRAM = 9004,
  SET_POSTURE = 9005,
  GET_POSTURE = 9006,
  WRENCH = 9007,
  SYS_STAT = 9008,
  SET_TOOL_OFFSET = 9009
};
}
typedef FSRoboRMsgTypes::FSRoboRMsgType FSRoboRMsgType;
}
}

#endif // FSROBO_R_DRIVER_SIMPLE_MESSAGE_FSROBO_R_SIMPLE_MESSAGE_H