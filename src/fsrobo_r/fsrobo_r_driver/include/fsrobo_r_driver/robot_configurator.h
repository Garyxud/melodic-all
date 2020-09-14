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

#ifndef FSROBO_R_DRIVER_ROBOT_CONFIGURATOR_H
#define FSROBO_R_DRIVER_ROBOT_CONFIGURATOR_H

#include "simple_message/smpl_msg_connection.h"
#include <vector>

namespace fsrobo_r_driver
{
namespace robot_configurator
{
using industrial::smpl_msg_connection::SmplMsgConnection;

/**
 * \brief Wrapper class around FSRobo-R-specific configuration commands
 */

class RobotConfigurator
{
public:
  /**
   * \brief Default constructor
   */
  RobotConfigurator() {}

  bool init(SmplMsgConnection* connection);

public:
  /**
   * \brief Set posture on the controller.
   *
   * \param posture
   * \param result True IFF setting was successful
   * \return True IFF message sending was successful
   */
  bool setPosture(industrial::shared_types::shared_int posture, bool &result);

  /**
   * \brief Get posture on the controller.
   *
   * \param posture
   * \param result True IFF getting was successful
   * \return True IFF message sending was successful
   */
  bool getPosture(industrial::shared_types::shared_int &posture, bool &result);

  /**
   * \brief Set tool offset on the controller.
   *
   * \param x
   * \param y
   * \param z
   * \param rz
   * \param ry
   * \param rx
   * \param result True IFF setting was successful
   * \return True IFF message sending was successful
   */
  bool setToolOffset(industrial::shared_types::shared_real x,
    industrial::shared_types::shared_real y,
    industrial::shared_types::shared_real z,
    industrial::shared_types::shared_real rz,
    industrial::shared_types::shared_real ry,
    industrial::shared_types::shared_real rx,
    bool &result);

protected:
  SmplMsgConnection* connection_;

  bool sendAndReceiveSetPostureMsg(industrial::shared_types::shared_int posture, bool &result);
  bool sendAndReceiveGetPostureMsg(industrial::shared_types::shared_int &posture, bool &result);
  bool sendAndReceiveSetToolOffsetMsg(industrial::shared_types::shared_real x,
    industrial::shared_types::shared_real y,
    industrial::shared_types::shared_real z,
    industrial::shared_types::shared_real rz,
    industrial::shared_types::shared_real ry,
    industrial::shared_types::shared_real rx,
    bool &result);
};

} 
} 

#endif // FSROBO_R_DRIVER_ROBOT_CONFIGURATOR_H