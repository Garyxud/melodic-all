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
#ifndef FSROBO_R_DRIVER_ROBOT_SERVICE_INTERFACE_H
#define FSROBO_R_DRIVER_ROBOT_SERVICE_INTERFACE_H

#include <map>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/tcp_client.h"
#include "fsrobo_r_msgs/SetIO.h"
#include "fsrobo_r_driver/io_control.h"
#include "fsrobo_r_msgs/SetPosture.h"
#include "fsrobo_r_msgs/GetPosture.h"
#include "fsrobo_r_driver/robot_configurator.h"
#include "fsrobo_r_msgs/SetToolOffset.h"

namespace fsrobo_r_driver
{
namespace robot_service_interface
{
using fsrobo_r_driver::io_control::IOControl;
using fsrobo_r_driver::robot_configurator::RobotConfigurator;
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::tcp_client::TcpClient;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

/**
 * \brief Message handler that relays robot service to the robot controller
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class FSRoboRRobotServiceInterface
{

public:
  /**
   * \brief Default constructor.
   */
  FSRoboRRobotServiceInterface();
  ~FSRoboRRobotServiceInterface();

  /**
   * \brief Initialize robot connection using default method.
   *
   * \param default_ip default IP address to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "robot_ip_address" cannot be read
   * \param default_port default port to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "~port" cannot be read
   *
   * \return true on success, false otherwise
   */
  virtual bool init(std::string default_ip = "", int default_port = StandardSocketPorts::IO);

  /**
   * \brief Initialize robot connection using specified method.
   *
   * \param connection new robot-connection instance (ALREADY INITIALIZED).
   *
   * \return true on success, false otherwise
   */
  virtual bool init(SmplMsgConnection *connection);


  /**
   * \brief Begin processing messages and publishing topics.
   */
  virtual void run() { ros::spin(); }

protected:
  ros::ServiceServer srv_set_io_; // handle for set_io service
  ros::ServiceServer srv_set_posture;
  ros::ServiceServer srv_get_posture;
  ros::ServiceServer srv_set_tool_offset;

  IOControl io_ctrl_;
  RobotConfigurator robot_configurator_;

  /**
   * \brief Callback function registered to ROS set_io service.
   *  Transform message into SimpleMessage objects and send commands to robot.
   * \param req SetIO request from service call
   * \param res SetIO response to service call
   * \return true if success, false otherwise
   */
  bool setIOCB(fsrobo_r_msgs::SetIO::Request &req, fsrobo_r_msgs::SetIO::Response &res);

  /**
   * \brief Callback function registered to ROS set_posture service.
   *  Transform message into SimpleMessage objects and send commands to robot.
   * \param req SetPosture request from service call
   * \param res SetPosture response to service call
   * \return true if success, false otherwise
   */
  bool setPostureCB(fsrobo_r_msgs::SetPosture::Request &req, fsrobo_r_msgs::SetPosture::Response &res);

  /**
   * \brief Callback function registered to ROS get_posture service.
   *  Transform message into SimpleMessage objects and send commands to robot.
   * \param req GetPosture request from service call
   * \param res GetPosture response to service call
   * \return true if success, false otherwise
   */
  bool getPostureCB(fsrobo_r_msgs::GetPosture::Request &req, fsrobo_r_msgs::GetPosture::Response &res);

  /**
   * \brief Callback function registered to ROS set_tool_offset service.
   *  Transform message into SimpleMessage objects and send commands to robot.
   * \param req SetToolOffset request from service call
   * \param res SetToolOffset response to service call
   * \return true if success, false otherwise
   */
  bool setToolOffsetCB(fsrobo_r_msgs::SetToolOffset::Request &req, fsrobo_r_msgs::SetToolOffset::Response &res);

  TcpClient default_tcp_connection_;

  ros::NodeHandle node_;
  SmplMsgConnection *connection_;
};

} // namespace robot_service_interface
} // namespace fsrobo_r_driver

#endif /* FSROBO_R_DRIVER_ROBOT_SERVICE_INTERFACE_H */
