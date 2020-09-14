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

#ifndef TRAJECTORY_TRACKER_MSGS_CONVERTER_H
#define TRAJECTORY_TRACKER_MSGS_CONVERTER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <trajectory_tracker_msgs/PathWithVelocity.h>

namespace trajectory_tracker_msgs
{
inline PathWithVelocity toPathWithVelocity(
    const nav_msgs::Path& src,
    const double vel)
{
  PathWithVelocity dest;
  dest.header = src.header;
  dest.poses.clear();
  dest.poses.reserve(src.poses.size());
  for (const geometry_msgs::PoseStamped& p : src.poses)
  {
    PoseStampedWithVelocity pv;
    pv.header = p.header;
    pv.pose = p.pose;
    pv.linear_velocity.x = vel;
    dest.poses.push_back(pv);
  }
  return dest;
}
inline PathWithVelocity toPathWithVelocity(
    const nav_msgs::Path& src,
    const geometry_msgs::Vector3& vel)
{
  PathWithVelocity dest;
  dest.header = src.header;
  dest.poses.clear();
  dest.poses.reserve(src.poses.size());
  for (const geometry_msgs::PoseStamped& p : src.poses)
  {
    PoseStampedWithVelocity pv;
    pv.header = p.header;
    pv.pose = p.pose;
    pv.linear_velocity = vel;
    dest.poses.push_back(pv);
  }
  return dest;
}
}  // namespace trajectory_tracker_msgs

#endif  // TRAJECTORY_TRACKER_MSGS_CONVERTER_H
