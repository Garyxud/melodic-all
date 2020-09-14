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

#ifndef TRAJECTORY_TRACKER_RVIZ_PLUGINS_VALIDATE_FLOATS_H
#define TRAJECTORY_TRACKER_RVIZ_PLUGINS_VALIDATE_FLOATS_H

#include <vector>

#include <rviz/validate_floats.h>
#ifdef HAVE_VALIDATE_QUATERNION_H
#include <rviz/validate_quaternions.h>
#endif

#include <trajectory_tracker_msgs/PathWithVelocity.h>
#include <trajectory_tracker_msgs/PoseStampedWithVelocity.h>

namespace trajectory_tracker_rviz_plugins
{
inline bool validateFloats(const trajectory_tracker_msgs::PoseStampedWithVelocity& msg)
{
  return rviz::validateFloats(msg.pose.position) &&
         rviz::validateFloats(msg.pose.orientation);
  // NaN value in linear_velocity means "Don't Care"; don't validate linear_velocity field
}

template <typename T>
inline bool validateFloats(const std::vector<T>& vec)
{
  for (const T& e : vec)
  {
    if (!validateFloats(e))
      return false;
  }
  return true;
}

inline bool validateFloats(const trajectory_tracker_msgs::PathWithVelocity& msg)
{
  return validateFloats(msg.poses);
}

#ifdef HAVE_VALIDATE_QUATERNION_H
inline bool validateQuaternions(const trajectory_tracker_msgs::PoseStampedWithVelocity& msg)
{
  return rviz::validateQuaternions(msg.pose.orientation);
}

template <typename T>
inline bool validateQuaternions(const std::vector<T>& vec)
{
  for (const T& e : vec)
  {
    if (!validateQuaternions(e))
      return false;
  }
  return true;
}
#endif

}  // namespace trajectory_tracker_rviz_plugins

#endif  // TRAJECTORY_TRACKER_RVIZ_PLUGINS_VALIDATE_FLOATS_H
