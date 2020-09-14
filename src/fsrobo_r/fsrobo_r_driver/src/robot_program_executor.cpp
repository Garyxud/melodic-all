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

#include "fsrobo_r_driver/robot_program_executor.h"
#include "fsrobo_r_driver/simple_message/simple_string.h"
#include "fsrobo_r_driver/simple_message/messages/execute_program_message.h"
#include "fsrobo_r_driver/simple_message/messages/execute_program_reply_message.h"
#include "ros/ros.h"
#include "simple_message/simple_message.h"
#include <string>


namespace ExecuteProgramReplyResults = fsrobo_r_driver::simple_message::execute_program_reply::ExecuteProgramReplyResults;

using fsrobo_r_driver::simple_message::simple_string::SimpleString;
using fsrobo_r_driver::simple_message::execute_program::ExecuteProgram;
using fsrobo_r_driver::simple_message::execute_program_message::ExecuteProgramMessage;
using fsrobo_r_driver::simple_message::execute_program_reply_message::ExecuteProgramReplyMessage;
using industrial::simple_message::SimpleMessage;
using industrial::shared_types::shared_int;
using std::string;
using std::vector;

namespace fsrobo_r_driver
{
namespace robot_program_executor
{

bool RobotProgramExecutor::init(SmplMsgConnection* connection)
{
  connection_ = connection;
  return true;
}

bool RobotProgramExecutor::execute(string name, string param, bool &result)
{
  ExecuteProgramReply reply;

  if (!sendAndReceive(name, param, reply))
  {
    ROS_ERROR("Failed to send EXECUTE_PROGRAM command");
    return false;
  }

  //return (reply.getResultCode() == ExecuteProgramReplyResults::SUCCESS);
  result = (reply.getResultCode() == ExecuteProgramReplyResults::SUCCESS);
  return true;
}

bool RobotProgramExecutor::sendAndReceive(string name, string param,ExecuteProgramReply &reply)
{
  SimpleMessage req, res;
  SimpleString ss_name;
  SimpleString ss_param;
  ExecuteProgram execute_program;
  ExecuteProgramMessage execute_program_msg;
  ExecuteProgramReplyMessage execute_program_reply_msg;

  ss_name.init(name);
  ss_param.init(param);
  ROS_ERROR("%s", name.c_str());
  ROS_ERROR("%s", param.c_str());
  execute_program.init(ss_name, ss_param);
  execute_program_msg.init(execute_program);
  execute_program_msg.toRequest(req);

  if (!this->connection_->sendAndReceiveMsg(req, res))
  {
    ROS_ERROR("Failed to send RobotProgram message");
    return false;
  }

  execute_program_reply_msg.init(res);
  reply.copyFrom(execute_program_reply_msg.reply_);

  return true;
}

}

}
