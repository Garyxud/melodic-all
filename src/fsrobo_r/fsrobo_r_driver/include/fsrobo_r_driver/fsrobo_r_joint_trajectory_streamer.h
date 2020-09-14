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

#ifndef FSROBO_R_DRIVER_JOINT_TRAJECTORY_STREAMER_H
#define FSROBO_R_DRIVER_JOINT_TRAJECTORY_STREAMER_H

#include <map>
#include <string>
#include <vector>
#include <boost/thread/thread.hpp>
#include "industrial_robot_client/joint_trajectory_interface.h"
#include "simple_message/joint_data.h"
#include "simple_message/simple_message.h"
#include "fsrobo_r_driver/simple_message/sys_stat.h"
#include "fsrobo_r_driver/simple_message/messages/sys_stat_message.h"
#include "fsrobo_r_driver/simple_message/sys_stat_reply.h"
#include "fsrobo_r_driver/simple_message/messages/sys_stat_reply_message.h"
#include "fsrobo_r_msgs/ExecuteRobotProgram.h"
#include "fsrobo_r_driver/robot_program_executor.h"

using industrial::simple_message::SimpleMessage;

namespace fsrobo_r_driver
{
namespace joint_trajectory_streamer
{
using industrial_robot_client::joint_trajectory_interface::JointTrajectoryInterface;
using industrial::joint_traj_pt_message::JointTrajPtMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;
using fsrobo_r_driver::robot_program_executor::RobotProgramExecutor;

namespace TransferStates
{
enum TransferState
{
  IDLE = 0, STREAMING =1 //,STARTING, //, STOPPING
};
}

typedef TransferStates::TransferState TransferState;

class FSRoboRJointTrajectoryStreamer : public JointTrajectoryInterface
{
public:
  // since this class defines a different init(), this helps find the base-class init()
  using JointTrajectoryInterface::init;

  /**
   * \brief Default constructor
   *
   * \param min_buffer_size minimum number of points as required by robot implementation
   */
  FSRoboRJointTrajectoryStreamer(int min_buffer_size = 1) : min_buffer_size_(min_buffer_size) { default_vel_ratio_ = 0.01; };

  ~FSRoboRJointTrajectoryStreamer();

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send commands to robot (ALREADY INITIALIZED)
   * \param joint_names list of expected joint-names.
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to insert a placeholder joint position (typ. 0.0).
   *   - Joints in the incoming JointTrajectory stream that are NOT listed here will be ignored.
   * \param velocity_limits map of maximum velocities for each joint
   *   - leave empty to lookup from URDF
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(SmplMsgConnection *connection, const std::vector<std::string> &joint_names,
                    const std::map<std::string, double> &velocity_limits = std::map<std::string, double>());

  virtual void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);

  virtual bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr &traj, std::vector<JointTrajPtMessage> *msgs);

  void streamingThread();

  bool send_to_robot(const std::vector<JointTrajPtMessage> &messages);

protected:
  void trajectoryStop();

  boost::thread *streaming_thread_;
  boost::mutex mutex_;
  int current_point_;
  std::vector<JointTrajPtMessage> current_traj_;
  TransferState state_;
  ros::Time streaming_start_;
  int min_buffer_size_;

  ros::ServiceServer srv_execute_robot_program;
  RobotProgramExecutor robot_program_executor_;

  bool executeRobotProgramCB(fsrobo_r_msgs::ExecuteRobotProgram::Request &req, fsrobo_r_msgs::ExecuteRobotProgram::Response &res);
};

} // namespace joint_trajectory_streamer
} // namespace fsrobo_r_driver

#endif