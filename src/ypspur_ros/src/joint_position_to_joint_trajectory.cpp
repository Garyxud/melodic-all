/*
 * Copyright (c) 2015-2017, the ypspur_ros authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ypspur_ros/JointPositionControl.h>

#include <map>
#include <string>

#include <compatibility.h>

class ConvertNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_joint_;
  ros::Subscriber sub_joint_state_;
  ros::Publisher pub_joint_;

  std::map<std::string, double> state_;

  double accel_;
  bool skip_same_;

  void cbJointState(const sensor_msgs::JointState::ConstPtr& msg)
  {
    for (size_t i = 0; i < msg->name.size(); i++)
    {
      state_[msg->name[i]] = msg->position[i];
    }
  }
  ypspur_ros::JointPositionControl cmd_prev;
  void cbJointPosition(const ypspur_ros::JointPositionControl::ConstPtr& msg)
  {
    while (true)
    {
      if (msg->joint_names.size() != cmd_prev.joint_names.size())
        break;
      {
        bool eq = true;
        for (unsigned int i = 0; i < msg->joint_names.size(); i++)
        {
          if (msg->joint_names[i].compare(cmd_prev.joint_names[i]) != 0)
            eq = false;
        }
        if (!eq)
          break;
      }
      if (msg->positions.size() != cmd_prev.positions.size())
        break;
      {
        bool eq = true;
        for (unsigned int i = 0; i < msg->positions.size(); i++)
        {
          if (msg->positions[i] != cmd_prev.positions[i])
            eq = false;
        }
        if (!eq)
          break;
      }
      if (msg->velocities.size() != cmd_prev.velocities.size())
        break;
      {
        bool eq = true;
        for (unsigned int i = 0; i < msg->velocities.size(); i++)
        {
          if (msg->velocities[i] != cmd_prev.velocities[i])
            eq = false;
        }
        if (!eq)
          break;
      }
      if (msg->accelerations.size() != cmd_prev.accelerations.size())
        break;
      {
        bool eq = true;
        for (unsigned int i = 0; i < msg->accelerations.size(); i++)
        {
          if (msg->accelerations[i] != cmd_prev.accelerations[i])
            eq = false;
        }
        if (!eq)
          break;
      }
      return;
    }
    cmd_prev = *msg;

    trajectory_msgs::JointTrajectory cmd;
    cmd.header = msg->header;
    cmd.joint_names = msg->joint_names;
    cmd.points.resize(1);
    cmd.points[0].velocities.resize(msg->positions.size());
    cmd.points[0].positions = msg->positions;
    cmd.points[0].accelerations.resize(msg->positions.size());

    float t_max = 0;
    int i = 0;
    for (auto& p : msg->positions)
    {
      float t = fabs(p - state_[msg->joint_names[i]]) / msg->velocities[i];
      if (t_max < t)
        t_max = t;

      i++;
    }
    cmd.points[0].time_from_start = ros::Duration(t_max);

    pub_joint_.publish(cmd);
  }

public:
  ConvertNode()
    : nh_()
    , pnh_("~")
  {
    sub_joint_ = compat::subscribe(
        nh_, "joint_position",
        pnh_, "joint_position", 5, &ConvertNode::cbJointPosition, this);
    sub_joint_state_ = compat::subscribe(
        nh_, "joint_states",
        pnh_, "joint", 5, &ConvertNode::cbJointState, this);
    pub_joint_ = compat::advertise<trajectory_msgs::JointTrajectory>(
        nh_, "joint_trajectory",
        pnh_, "joint_trajectory", 5, false);

    pnh_.param("accel", accel_, 0.3);
    pnh_.param("skip_same", skip_same_, true);
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "joint_position_to_joint_trajectory");

  ConvertNode conv;
  ros::spin();

  return 0;
}
