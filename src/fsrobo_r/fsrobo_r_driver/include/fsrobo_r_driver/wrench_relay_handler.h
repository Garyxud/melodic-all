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

#ifndef FSROBO_R_DRIVER_WRENCH_RELAY_HANDLER_H
#define FSROBO_R_DRIVER_WRENCH_RELAY_HANDLER_H

#include "ros/ros.h"
#include "simple_message/message_handler.h"
#include "fsrobo_r_driver/simple_message/messages/wrench_message.h"


namespace fsrobo_r_driver
{
namespace wrench_relay_handler
{

/**
 * \brief Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class WrenchRelayHandler : public industrial::message_handler::MessageHandler
{
  // since this class defines a different init(), this helps find the base-class init()
  using industrial::message_handler::MessageHandler::init;

public:

 /**
  * \brief Constructor
  */
  WrenchRelayHandler() {};


 /**
  * \brief Class initializer
  *
  * \param connection simple message connection that will be used to send replies.
  *
  * \return true on success, false otherwise (an invalid message type)
  */
  bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection);

protected:

  ros::Publisher pub_wrench_stamped_;
  ros::NodeHandle node_;

  /**
   * \brief Callback executed upon receiving a robot status message
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  bool internalCB(fsrobo_r_driver::simple_message::wrench_message::WrenchMessage &in);

private:
  /**
  * \brief Callback executed upon receiving a message
  *
  * \param in incoming message
  *
  * \return true on success, false otherwise
  */
  bool internalCB(industrial::simple_message::SimpleMessage &in);
  std::string force_frame_id_;
};

} // namespace wrench_relay_handler
} // namespace fsrobo_r_driver

#endif // FSROBO_R_DRIVER_WRENCH_RELAY_HANDLER_H