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

#include "fsrobo_r_driver/io_state_relay_handler.h"
#include "fsrobo_r_msgs/IOStates.h"
#include "simple_message/log_wrapper.h"
#include <sstream>

using namespace industrial::shared_types;
using namespace industrial::smpl_msg_connection;
using namespace industrial::simple_message;
using namespace fsrobo_r_driver::simple_message::io_state;
using namespace fsrobo_r_driver::simple_message::io_state_message;

namespace fsrobo_r_driver
{
namespace io_state_relay_handler
{

bool IOStateRelayHandler::init(SmplMsgConnection *connection)
{
  ROS_WARN("IOStateRelayHandler::init!");
  this->pub_io_states_ = this->node_.advertise<fsrobo_r_msgs::IOStates>("io_states", 1);
  return init((int)fsrobo_r_driver::simple_message::FSRoboRMsgType::IO_STATE, connection);
}

bool IOStateRelayHandler::internalCB(SimpleMessage &in)
{
  IOStateMessage state_msg;

  if (!state_msg.init(in))
  {
    LOG_ERROR("Failed to initialize state message");
    return false;
  }

  return internalCB(state_msg);
}

bool IOStateRelayHandler::internalCB(IOStateMessage &in)
{
  fsrobo_r_msgs::IOStates io_states;
  bool rtn = true;

  //ROS_WARN("%08x %08x %08x", in.state_.getDigital(0), in.state_.getDigital(1), in.state_.getAnalog(0));

  std::vector<fsrobo_r_msgs::Digital> d_in_list;
  for (int i = 0; i < 16; i++) {
    fsrobo_r_msgs::Digital d;
    createDigitalMessage(in.state_.getDigital(0), i, 0, d);
    d_in_list.push_back(d);
  }
  for (int i = 0; i < 16; i++) {
    fsrobo_r_msgs::Digital d;
    createDigitalMessage(in.state_.getDigital(1), i, 32, d);
    d_in_list.push_back(d);
  }

  std::vector<fsrobo_r_msgs::Digital> d_out_list;
  for (int i = 0; i < 16; i++) {
    fsrobo_r_msgs::Digital d;
    createDigitalMessage(in.state_.getDigital(0), i + 16, 0, d);
    d_out_list.push_back(d);
  }
  for (int i = 0; i < 16; i++) {
    fsrobo_r_msgs::Digital d;
    createDigitalMessage(in.state_.getDigital(1), i + 16, 32, d);
    d_out_list.push_back(d);
  }

  std::vector<fsrobo_r_msgs::Analog> a_in_list;
  for (int i = 0; i < 2; i++) {
    fsrobo_r_msgs::Analog a;
    createAnalogMessage(in.state_.getAnalog(0), i, 0, a);
    a_in_list.push_back(a);
  }
 
  io_states.digital_in_states = d_in_list;
  io_states.digital_out_states = d_out_list;
  io_states.analog_in_states = a_in_list;
  
  this->pub_io_states_.publish(io_states);

  // Reply back to the controller if the sender requested it.
  if (CommTypes::SERVICE_REQUEST == in.getCommType())
  {
    SimpleMessage reply;
    in.toReply(reply, rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }

  return rtn;
}

void IOStateRelayHandler::createDigitalMessage(int data, int n, int base_addr, fsrobo_r_msgs::Digital &d)
{
  int mask = 1 << n;

  d.addr = base_addr + n;
  d.state = (bool)(data & mask);
  // ROS_WARN("%d %d %08x %08x %08x %d %d %d", base_addr, n, mask, data, data & mask, !!(data & mask), d.state, (bool)(data & mask));
}

void IOStateRelayHandler::createAnalogMessage(int data, int n, int base_ch, fsrobo_r_msgs::Analog &a)
{
  const int data_mask = 0x0fff;
  const int mode_mask = 0x3000;

  int d = data >> (16 * n);

  a.ch = base_ch + n;
  a.state = d & data_mask;
  a.mode = (d & mode_mask) >> 12;
}

}
}

