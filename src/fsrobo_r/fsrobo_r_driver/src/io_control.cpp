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

#include "fsrobo_r_driver/io_control.h"
#include "fsrobo_r_driver/simple_message/messages/set_io_message.h"
#include "fsrobo_r_driver/simple_message/messages/set_io_reply_message.h"
#include "ros/ros.h"
#include "simple_message/simple_message.h"
#include <string>


namespace SetIOReplyResults = fsrobo_r_driver::simple_message::io_control_reply::SetIOReplyResults;

using fsrobo_r_driver::simple_message::io_control::SetIO;
using fsrobo_r_driver::simple_message::io_control_message::SetIOMessage;
using fsrobo_r_driver::simple_message::io_control_reply_message::SetIOReplyMessage;
using industrial::simple_message::SimpleMessage;
using industrial::shared_types::shared_int;
using std::vector;


namespace fsrobo_r_driver
{
namespace io_control
{

bool IOControl::init(SmplMsgConnection* connection)
{
  connection_ = connection;
  return true;
}

bool IOControl::setIO(shared_int fun, shared_int address, std::vector<shared_int> &data)
{
  SetIOReply reply;

  if (!sendAndReceive(fun, address, data, reply))
  {
    ROS_ERROR("Failed to send WRITE_SINGLE_IO command");
    return false;
  }

  return (reply.getResultCode() == SetIOReplyResults::SUCCESS);
}

bool IOControl::sendAndReceive(shared_int fun, shared_int address, vector<shared_int> &data, SetIOReply &reply)
{
  SimpleMessage req, res;
  SetIO set_io;
  SetIOMessage set_io_msg;
  SetIOReplyMessage set_io_reply;

  set_io.init(fun, address, data);
  set_io_msg.init(set_io);
  set_io_msg.toRequest(req);

  if (!this->connection_->sendAndReceiveMsg(req, res))
  {
    ROS_ERROR("Failed to send SetIO message");
    return false;
  }

  set_io_reply.init(res);
  reply.copyFrom(set_io_reply.reply_);

  return true;
}

}

}
