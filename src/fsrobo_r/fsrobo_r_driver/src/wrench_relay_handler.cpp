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

#include "fsrobo_r_driver/wrench_relay_handler.h"
#include "geometry_msgs/WrenchStamped.h"
#include "simple_message/log_wrapper.h"
#include <sstream>

using namespace industrial::shared_types;
using namespace industrial::smpl_msg_connection;
using namespace industrial::simple_message;
using namespace fsrobo_r_driver::simple_message::wrench;
using namespace fsrobo_r_driver::simple_message::wrench_message;

namespace fsrobo_r_driver
{
namespace wrench_relay_handler
{

bool WrenchRelayHandler::init(SmplMsgConnection *connection)
{
  ROS_WARN("WrenchRelayHandler::init!");
  this->pub_wrench_stamped_ = this->node_.advertise<geometry_msgs::WrenchStamped>("raw_force", 1);
  ros::param::param<std::string>("frame_id", force_frame_id_, "/sensor");
  return init((int)fsrobo_r_driver::simple_message::FSRoboRMsgType::WRENCH, connection);
}

bool WrenchRelayHandler::internalCB(SimpleMessage &in)
{
  WrenchMessage wrench_msg;

  if (!wrench_msg.init(in))
  {
    LOG_ERROR("Failed to initialize wrench message");
    return false;
  }

  return internalCB(wrench_msg);
}

bool WrenchRelayHandler::internalCB(WrenchMessage &in)
{
  geometry_msgs::WrenchStamped wrench_stamped;
  bool rtn = true;

  // ROS_WARN("%f %f %f %f %f %f", in.wrench_.getForce(0), in.wrench_.getForce(1), in.wrench_.getForce(2),
  //         in.wrench_.getTorque(0), in.wrench_.getTorque(1), in.wrench_.getTorque(2));

  wrench_stamped.header.stamp = ros::Time::now();
  wrench_stamped.header.frame_id = force_frame_id_;
  wrench_stamped.wrench.force.x = in.wrench_.getForce(0);
  wrench_stamped.wrench.force.y = in.wrench_.getForce(1);
  wrench_stamped.wrench.force.z = in.wrench_.getForce(2);
  wrench_stamped.wrench.torque.x = in.wrench_.getTorque(0);
  wrench_stamped.wrench.torque.y = in.wrench_.getTorque(1);
  wrench_stamped.wrench.torque.z = in.wrench_.getTorque(2);
  this->pub_wrench_stamped_.publish(wrench_stamped);

  // Reply back to the controller if the sender requested it.
  if (CommTypes::SERVICE_REQUEST == in.getCommType())
  {
    SimpleMessage reply;
    in.toReply(reply, rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }

  return rtn;
}

}
}

