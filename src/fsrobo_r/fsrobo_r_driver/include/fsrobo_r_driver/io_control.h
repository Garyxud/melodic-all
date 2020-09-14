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

#ifndef FSROBO_R_DRIVER_IO_CONTROL_H
#define FSROBO_R_DRIVER_IO_CONTROL_H

#include "simple_message/smpl_msg_connection.h"
#include "fsrobo_r_driver/simple_message/set_io.h"
#include "fsrobo_r_driver/simple_message/set_io_reply.h"
#include <vector>

namespace fsrobo_r_driver
{
namespace io_control
{
using industrial::smpl_msg_connection::SmplMsgConnection;
using fsrobo_r_driver::simple_message::io_control_reply::SetIOReply;

/**
 * \brief Wrapper class around FSRobo-R-specific io control commands
 */

class IOControl
{
public:
  /**
   * \brief Default constructor
   */
  IOControl() {}

  bool init(SmplMsgConnection* connection);

public:
  /**
   * \brief Writes to a single IO point on the controller.
   *
   * \param address The address (index) of the IO point
   * \param value The value to set the IO element to
   * \return True IFF writing was successful
   */
  bool setIO(industrial::shared_types::shared_int fun,
    industrial::shared_types::shared_int,
    std::vector<industrial::shared_types::shared_int> &data);

protected:
  SmplMsgConnection* connection_;

  bool sendAndReceive(industrial::shared_types::shared_int address,
    industrial::shared_types::shared_int value,
    std::vector<industrial::shared_types::shared_int> &data,
    SetIOReply &reply);
};

} 
} 

#endif // FSROBO_R_DRIVER_IO_CONTROL_H
