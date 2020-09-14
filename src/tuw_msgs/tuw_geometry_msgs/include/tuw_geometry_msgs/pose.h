/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Markus Bader <markus.bader@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#ifndef TUW_GEOMETRY_MSGS_POSE_H
#define TUW_GEOMETRY_MSGS_POSE_H

// ROS
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace tuw
{
namespace ros_msgs
{
class Pose;
typedef boost::shared_ptr<Pose> PosePtr;
typedef boost::shared_ptr<Pose const> PoseConstPtr;

class Pose : public geometry_msgs::Pose
{
public:
  Pose();
  Pose(double x, double y, double z, double roll, double pitch, double yaw);
  Pose &set(double x, double y, double z, double roll, double pitch, double yaw);
  Pose &setXYZ(double x, double y, double z);
  Pose &setOrientation(double x, double y, double z, double w);
  Pose &setRPY(double roll, double pitch, double yaw);
  geometry_msgs::PosePtr create();
};
};
void SetPositionXYZ(geometry_msgs::PosePtr &pose, double x, double y, double z);
void SetOrientation(geometry_msgs::PosePtr &pose, double x, double y, double z, double w);
void SetRPY(geometry_msgs::PosePtr &pose, double roll, double pitch, double yaw);
void Set(geometry_msgs::PosePtr &pose, double x, double y, double z, double roll, double pitch, double yaw);
void Set(geometry_msgs::PosePtr &des, const tuw::ros_msgs::Pose &src);
};
#endif  // TUW_GEOMETRY_MSGS_POSE_H