/*
 * Copyright (c) 2018, the neonavigation authors
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

#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <trajectory_tracker_msgs/PathWithVelocity.h>

#include <trajectory_tracker_msgs/converter.h>

namespace
{
nav_msgs::Path generatePath()
{
  nav_msgs::Path path;
  path.header.frame_id = "frame-name";
  path.header.stamp = ros::Time(1234.5);
  path.poses.resize(2);
  path.poses[0].pose.orientation.w = 1.0;
  path.poses[0].pose.position.x = 2.0;
  path.poses[0].pose.position.y = 3.0;
  path.poses[1].pose.orientation.z = 4.0;
  path.poses[1].pose.position.x = 5.0;
  path.poses[1].pose.position.y = 6.0;
  return path;
}

geometry_msgs::Vector3 generateVector3(
    const double x,
    const double y,
    const double z)
{
  geometry_msgs::Vector3 vec;
  vec.x = x;
  vec.y = y;
  vec.z = z;
  return vec;
}
}  // namespace

TEST(Converter, ToPathWithVelocityScalar)
{
  const nav_msgs::Path in = generatePath();
  const trajectory_tracker_msgs::PathWithVelocity out =
      trajectory_tracker_msgs::toPathWithVelocity(in, 1.23);

  ASSERT_EQ(in.header.frame_id, out.header.frame_id);
  ASSERT_EQ(in.header.stamp, out.header.stamp);
  for (size_t i = 0; i < in.poses.size(); ++i)
  {
    ASSERT_EQ(out.poses[i].header.frame_id, in.poses[i].header.frame_id);
    ASSERT_EQ(out.poses[i].header.stamp, in.poses[i].header.stamp);
    ASSERT_EQ(out.poses[i].pose.position.x, in.poses[i].pose.position.x);
    ASSERT_EQ(out.poses[i].pose.position.y, in.poses[i].pose.position.y);
    ASSERT_EQ(out.poses[i].pose.position.z, in.poses[i].pose.position.z);
    ASSERT_EQ(out.poses[i].pose.orientation.x, in.poses[i].pose.orientation.x);
    ASSERT_EQ(out.poses[i].pose.orientation.y, in.poses[i].pose.orientation.y);
    ASSERT_EQ(out.poses[i].pose.orientation.z, in.poses[i].pose.orientation.z);
    ASSERT_EQ(out.poses[i].pose.orientation.w, in.poses[i].pose.orientation.w);
    ASSERT_EQ(out.poses[i].linear_velocity.x, 1.23);
    ASSERT_EQ(out.poses[i].linear_velocity.y, 0.0);
    ASSERT_EQ(out.poses[i].linear_velocity.z, 0.0);
  }
}

TEST(Converter, ToPathWithVelocityVector)
{
  const nav_msgs::Path in = generatePath();
  const geometry_msgs::Vector3 vel = generateVector3(1.23, 0.12, 0.23);
  const trajectory_tracker_msgs::PathWithVelocity out =
      trajectory_tracker_msgs::toPathWithVelocity(in, vel);

  ASSERT_EQ(in.header.frame_id, out.header.frame_id);
  ASSERT_EQ(in.header.stamp, out.header.stamp);
  for (size_t i = 0; i < in.poses.size(); ++i)
  {
    ASSERT_EQ(out.poses[i].header.frame_id, in.poses[i].header.frame_id);
    ASSERT_EQ(out.poses[i].header.stamp, in.poses[i].header.stamp);
    ASSERT_EQ(out.poses[i].pose.position.x, in.poses[i].pose.position.x);
    ASSERT_EQ(out.poses[i].pose.position.y, in.poses[i].pose.position.y);
    ASSERT_EQ(out.poses[i].pose.position.z, in.poses[i].pose.position.z);
    ASSERT_EQ(out.poses[i].pose.orientation.x, in.poses[i].pose.orientation.x);
    ASSERT_EQ(out.poses[i].pose.orientation.y, in.poses[i].pose.orientation.y);
    ASSERT_EQ(out.poses[i].pose.orientation.z, in.poses[i].pose.orientation.z);
    ASSERT_EQ(out.poses[i].pose.orientation.w, in.poses[i].pose.orientation.w);
    ASSERT_EQ(out.poses[i].linear_velocity.x, vel.x);
    ASSERT_EQ(out.poses[i].linear_velocity.y, vel.y);
    ASSERT_EQ(out.poses[i].linear_velocity.z, vel.z);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
