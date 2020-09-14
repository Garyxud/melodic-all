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

#include "fsrobo_r_driver/simple_message/messages/set_tool_offset_reply_message.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"

using industrial::byte_array::ByteArray;
using industrial::simple_message::SimpleMessage;
using fsrobo_r_driver::simple_message::set_tool_offset_reply::SetToolOffsetReply;

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace set_tool_offset_reply_message
{

SetToolOffsetReplyMessage::SetToolOffsetReplyMessage(void)
{
  this->init();
}

SetToolOffsetReplyMessage::~SetToolOffsetReplyMessage(void)
{
}

bool SetToolOffsetReplyMessage::init(SimpleMessage & msg)
{
  ByteArray data = msg.getData();
  this->init();

  if (!data.unload(this->reply_))
  {
    LOG_ERROR("Failed to unload SetToolOffsetReplyMessage data");
    return false;
  }
  return true;
}

void SetToolOffsetReplyMessage::init(SetToolOffsetReply & reply)
{
  this->init();
  this->reply_.copyFrom(reply);
}

void SetToolOffsetReplyMessage::init()
{
  this->setMessageType(FSRoboRMsgTypes::SET_IO_REPLY);
  this->reply_.init();
}

bool SetToolOffsetReplyMessage::load(ByteArray *buffer)
{
  LOG_COMM("Executing SetToolOffsetReply message load");
  if (!buffer->load(this->reply_))
  {
    LOG_ERROR("Failed to load SetToolOffsetReply message");
    return false;
  }

  return true;
}

bool SetToolOffsetReplyMessage::unload(ByteArray *buffer)
{
  LOG_COMM("Executing SetToolOffsetReply message unload");

  if (!buffer->unload(this->reply_))
  {
    LOG_ERROR("Failed to unload SetToolOffsetReply message");
    return false;
  }

  return true;
}

}
}
}
