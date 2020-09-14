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

#include "fsrobo_r_driver/robot_configurator.h"
#include "fsrobo_r_driver/simple_message/messages/get_posture_message.h"
#include "fsrobo_r_driver/simple_message/messages/get_posture_reply_message.h"
#include "fsrobo_r_driver/simple_message/messages/set_posture_message.h"
#include "fsrobo_r_driver/simple_message/messages/set_posture_reply_message.h"
#include "fsrobo_r_driver/simple_message/set_tool_offset.h"
#include "fsrobo_r_driver/simple_message/messages/set_tool_offset_message.h"
#include "fsrobo_r_driver/simple_message/messages/set_tool_offset_reply_message.h"
#include "ros/ros.h"
#include "simple_message/simple_message.h"



using fsrobo_r_driver::simple_message::posture::Posture;
using fsrobo_r_driver::simple_message::set_posture_message::SetPostureMessage;
using fsrobo_r_driver::simple_message::set_posture_reply_message::SetPostureReplyMessage;
using fsrobo_r_driver::simple_message::get_posture_message::GetPostureMessage;
using fsrobo_r_driver::simple_message::get_posture_reply_message::GetPostureReplyMessage;
using fsrobo_r_driver::simple_message::set_tool_offset::SetToolOffset;
using fsrobo_r_driver::simple_message::set_tool_offset_message::SetToolOffsetMessage;
using fsrobo_r_driver::simple_message::set_tool_offset_reply_message::SetToolOffsetReplyMessage;
using industrial::simple_message::SimpleMessage;
using industrial::simple_message::ReplyType;
using industrial::shared_types::shared_int;
using industrial::shared_types::shared_real;

namespace fsrobo_r_driver
{
namespace robot_configurator
{

bool RobotConfigurator::init(SmplMsgConnection* connection)
{
  connection_ = connection;
  return true;
}

bool RobotConfigurator::setPosture(shared_int posture, bool &result)
{
  if (!sendAndReceiveSetPostureMsg(posture, result))
  {
    ROS_ERROR("Failed to send SET_POSTURE command");
    return false;
  }

  return true;
}

bool RobotConfigurator::sendAndReceiveSetPostureMsg(shared_int posture, bool &result)
{
  SimpleMessage req, rep;
  Posture req_body;
  SetPostureMessage req_msg;
  SetPostureReplyMessage rep_msg;

  req_body.init(posture);
  req_msg.init(req_body);
  req_msg.toRequest(req);

  if (!this->connection_->sendAndReceiveMsg(req, rep))
  {
    ROS_ERROR("Failed to send SET_POSTURE message");
    return false;
  }

  rep_msg.init(rep);
  result = (rep.getReplyCode() == ReplyType::SUCCESS);

  return true;
}

bool RobotConfigurator::getPosture(shared_int &posture, bool &result)
{
  if (!sendAndReceiveGetPostureMsg(posture, result))
  {
    ROS_ERROR("Failed to send GET_POSTURE command");
    return false;
  }

  return true;
}

bool RobotConfigurator::sendAndReceiveGetPostureMsg(shared_int &posture, bool &result)
{
  SimpleMessage req, rep;
  Posture rep_body;
  GetPostureMessage req_msg;
  GetPostureReplyMessage rep_msg;

  rep_body.init();
  req_msg.init(rep_body);
  req_msg.toRequest(req);

  if (!this->connection_->sendAndReceiveMsg(req, rep))
  {
    ROS_ERROR("Failed to send GET_POSTURE message");
    return false;
  }

  rep_msg.init(rep);
  posture = rep_msg.reply_.getPosture();
  result = (rep.getReplyCode() == ReplyType::SUCCESS);

  return true;
}

bool RobotConfigurator::setToolOffset(shared_real x, shared_real y, shared_real z, shared_real rz, shared_real ry, shared_real rx, bool &result)
{
  if (!sendAndReceiveSetToolOffsetMsg(x, y, z, rz, ry, rx, result))
  {
    ROS_ERROR("Failed to send SET_TOOL_OFFSET command");
    return false;
  }

  return true;
}

bool RobotConfigurator::sendAndReceiveSetToolOffsetMsg(shared_real x, shared_real y, shared_real z, shared_real rz, shared_real ry, shared_real rx, bool &result)
{
  SimpleMessage req, rep;
  SetToolOffset req_body;
  SetToolOffsetMessage req_msg;
  SetToolOffsetReplyMessage rep_msg;

  req_body.init(x, y, z, rz, ry, rx);
  req_msg.init(req_body);
  req_msg.toRequest(req);

  if (!this->connection_->sendAndReceiveMsg(req, rep))
  {
    ROS_ERROR("Failed to send SET_TOOL_OFFSET message");
    return false;
  }

  rep_msg.init(rep);
  result = (rep.getReplyCode() == ReplyType::SUCCESS);

  return true;
}

}
}
