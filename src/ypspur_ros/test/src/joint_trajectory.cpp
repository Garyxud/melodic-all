/*
 * Copyright (c) 2019, the ypspur_ros authors
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

#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <gtest/gtest.h>

TEST(JointTrajectory, CommandValidation)
{
  ros::WallDuration wait(0.005);
  ros::Duration clock_step(0.05);

  ros::NodeHandle nh;
  ros::Publisher pub_cmd =
      nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1, true);
  ros::Publisher pub_clock =
      nh.advertise<rosgraph_msgs::Clock>("clock", 100);

  sensor_msgs::JointState::ConstPtr joint_states;
  const boost::function<void(const sensor_msgs::JointState::ConstPtr &)> cb_joint =
      [&joint_states](const sensor_msgs::JointState::ConstPtr &msg) -> void
  {
    joint_states = msg;
  };
  ros::Subscriber sub_joint_states =
      nh.subscribe("joint_states", 100, cb_joint);

  rosgraph_msgs::Clock clock;
  clock.clock.fromNSec(ros::WallTime::now().toNSec());
  pub_clock.publish(clock);

  // Wait until ypspur_ros
  for (int i = 0; i < 1000; ++i)
  {
    wait.sleep();
    ros::spinOnce();
    if (joint_states)
      break;
  }

  // Publish valid command
  trajectory_msgs::JointTrajectory cmd;
  cmd.header.stamp = clock.clock;
  cmd.joint_names.resize(1);
  cmd.joint_names[0] = "joint0";
  cmd.points.resize(1);
  cmd.points[0].time_from_start = ros::Duration(1);
  cmd.points[0].positions.resize(1);
  cmd.points[0].positions[0] = 1.0;
  cmd.points[0].velocities.resize(1);
  cmd.points[0].velocities[0] = 1.0;
  pub_cmd.publish(cmd);
  wait.sleep();

  for (int i = 0; i < 50; ++i)
  {
    clock.clock += clock_step;
    pub_clock.publish(clock);
    wait.sleep();
    ros::spinOnce();
  }

  // Valid command must not be ignored
  ASSERT_TRUE(static_cast<bool>(joint_states));
  ASSERT_EQ(joint_states->name.size(), 1);
  ASSERT_EQ(joint_states->name[0], "joint0");
  ASSERT_EQ(joint_states->velocity.size(), 1);
  ASSERT_NEAR(joint_states->velocity[0], 1.0, 0.1)
      << "Valid joint_trajectory must not be ignored";

  // Stop
  cmd.header.stamp = clock.clock;
  cmd.points[0].positions[0] = 0.0;
  cmd.points[0].velocities[0] = 0.0;
  pub_cmd.publish(cmd);
  wait.sleep();
  for (int i = 0; i < 50; ++i)
  {
    clock.clock += clock_step;
    pub_clock.publish(clock);
    wait.sleep();
    ros::spinOnce();
  }
  ASSERT_NEAR(joint_states->velocity[0], 0.0, 0.1)
      << "Valid joint_trajectory must not be ignored";

  // Publish outdated command
  cmd.header.stamp = clock.clock - ros::Duration(2.0);
  cmd.points[0].positions[0] = 1.0;
  cmd.points[0].velocities[0] = 1.0;
  pub_cmd.publish(cmd);
  wait.sleep();
  for (int i = 0; i < 50; ++i)
  {
    clock.clock += clock_step;
    pub_clock.publish(clock);
    wait.sleep();
    ros::spinOnce();
  }
  ASSERT_NEAR(joint_states->velocity[0], 0.0, 0.1)
      << "Outdated joint_trajectory must be ignored";
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_joint_trajectory");

  return RUN_ALL_TESTS();
}
