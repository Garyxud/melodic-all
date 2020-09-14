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

#ifndef FSROBO_R_DRIVER_ROBOT_PROGRAM_EXECUTOR_H
#define FSROBO_R_DRIVER_ROBOT_PROGRAM_EXECUTOR_H

#include "simple_message/smpl_msg_connection.h"
#include "fsrobo_r_driver/simple_message/execute_program.h"
#include "fsrobo_r_driver/simple_message/execute_program_reply.h"
#include <string>
#include <vector>

namespace fsrobo_r_driver
{
namespace robot_program_executor
{
using industrial::smpl_msg_connection::SmplMsgConnection;
using fsrobo_r_driver::simple_message::execute_program_reply::ExecuteProgramReply;

/**
 * \brief Wrapper class to exeute program on robot controller
 */

class RobotProgramExecutor
{
public:
  /**
   * \brief Default constructor
   */
  RobotProgramExecutor() {}

  bool init(SmplMsgConnection* connection);

public:
  /**
   * \brief Execute robot program on the controller.
   * 
   * \param name The name of robot program
   * \param result Program execution result
   * \return True IFF service call was successful
   */
  bool execute(std::string name, std::string param, bool &result);

protected:
  SmplMsgConnection* connection_;

  bool sendAndReceive(std::string name, std::string param, ExecuteProgramReply &reply);
};

} 
} 

#endif // FSROBO_R_DRIVER_ROBOT_PROGRAM_EXECUTOR_H
