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
#include "fsrobo_r_driver/fsrobo_r_robot_service_interface.h"
#include <sstream>

namespace fsrobo_r_driver
{
namespace robot_service_interface
{

FSRoboRRobotServiceInterface::FSRoboRRobotServiceInterface()
{  
}

FSRoboRRobotServiceInterface::~FSRoboRRobotServiceInterface()
{  
}

bool FSRoboRRobotServiceInterface::init(std::string default_ip, int default_port)
{
  std::string ip;
  int port;

  // override IP/port with ROS params, if available
  ros::param::param<std::string>("robot_ip_address", ip, default_ip);
  ros::param::param<int>("~port", port, default_port);

  // check for valid parameter values
  if (ip.empty())
  {
    ROS_ERROR("No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
    return false;
  }
  if (port <= 0)
  {
    ROS_ERROR("No valid robot IP port found.  Please set ROS '~port' param");
    return false;
  }

  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  ROS_INFO("IO Interface connecting to IP address: '%s:%d'", ip_addr, port);
  default_tcp_connection_.init(ip_addr, port);
  free(ip_addr);


  return init(&default_tcp_connection_);
}

bool FSRoboRRobotServiceInterface::init(SmplMsgConnection* connection)
{
  this->connection_ = connection;
  connection_->makeConnect();

  io_ctrl_.init(connection);
  this->srv_set_io_ = this->node_.advertiseService("set_io", &FSRoboRRobotServiceInterface::setIOCB, this);

  robot_configurator_.init(connection);
  this->srv_set_posture = this->node_.advertiseService("set_posture", &FSRoboRRobotServiceInterface::setPostureCB, this);
  this->srv_get_posture = this->node_.advertiseService("get_posture", &FSRoboRRobotServiceInterface::getPostureCB, this);
  this->srv_set_tool_offset = this->node_.advertiseService("set_tool_offset", &FSRoboRRobotServiceInterface::setToolOffsetCB, this);

  return true;
}

bool FSRoboRRobotServiceInterface::setIOCB(fsrobo_r_msgs::SetIO::Request &req, fsrobo_r_msgs::SetIO::Response &res)
{
  ROS_WARN("SetIO!");

  industrial::shared_types::shared_int io_val = -1;

  std::stringstream ss;
  std::vector<industrial::shared_types::shared_int> data;
  for (int x : req.data)
  {
    data.push_back(x);
    ss << x;
    ss << " ";
  }
  ROS_WARN("%s", ss.str().c_str());

  bool result = io_ctrl_.setIO(req.fun, req.address, data);

  res.success = result;

  if (!result)
  {
    ROS_ERROR("Writing IO element %d failed", req.address);
    return false;
  }

  return true;
}

bool FSRoboRRobotServiceInterface::setPostureCB(fsrobo_r_msgs::SetPosture::Request &req, fsrobo_r_msgs::SetPosture::Response &res)
{
  ROS_WARN("SetPosture!");

  bool exec_result;

  bool send_result = robot_configurator_.setPosture(req.posture, exec_result);

  bool result = send_result && exec_result;

  if (!result)
  {
    ROS_ERROR("Setting Posture failed");
    return false;
  }

  return true;
}

bool FSRoboRRobotServiceInterface::getPostureCB(fsrobo_r_msgs::GetPosture::Request &req, fsrobo_r_msgs::GetPosture::Response &res)
{
  ROS_WARN("GetPosture!");

  bool exec_result;
  industrial::shared_types::shared_int posture;

  bool send_result = robot_configurator_.getPosture(posture, exec_result);

  res.posture = posture;
  bool result = send_result && exec_result;

  if (!result)
  {
    ROS_ERROR("Getting posture failed");
    return false;
  }

  return true;
}

bool FSRoboRRobotServiceInterface::setToolOffsetCB(fsrobo_r_msgs::SetToolOffset::Request &req, fsrobo_r_msgs::SetToolOffset::Response &res)
{
  ROS_WARN("SetToolOffset!");

  bool exec_result;

  bool send_result = robot_configurator_.setToolOffset(req.origin.x, req.origin.y, req.origin.z, req.rotation.z, req.rotation.y, req.rotation.x, exec_result);

  bool result = send_result && exec_result;

  if (!result)
  {
    ROS_ERROR("Setting tool offset failed");
    return false;
  }

  return true;
}

} // fsrobo_r_driver
} // robot_service_interface

