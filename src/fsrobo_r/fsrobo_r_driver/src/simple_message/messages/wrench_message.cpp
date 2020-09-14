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

#include "fsrobo_r_driver/simple_message/messages/wrench_message.h"
#include "fsrobo_r_driver/simple_message/wrench.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace wrench_message
{

WrenchMessage::WrenchMessage(void)
{
  this->init();
}

WrenchMessage::~WrenchMessage(void)
{

}

bool WrenchMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->wrench_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload wrench data");
  }
  return rtn;
}

void WrenchMessage::init(fsrobo_r_driver::simple_message::wrench::Wrench &wrench)
{
  this->init();
  this->wrench_.copyFrom(wrench);
}

void WrenchMessage::init()
{
  this->setMessageType(FSRoboRMsgTypes::WRENCH);
  this->wrench_.init();
}

bool WrenchMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing wrench message load");
  if (buffer->load(this->wrench_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load wrench data");
  }
  return rtn;
}

bool WrenchMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing wrench message unload");

  if (buffer->unload(this->wrench_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload wrench data");
  }
  return rtn;
}

}
}
}

