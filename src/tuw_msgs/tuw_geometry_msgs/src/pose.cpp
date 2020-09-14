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

#include <tuw_geometry_msgs/pose.h>
#include <memory.h>

using namespace tuw::ros_msgs;

Pose::Pose(){

};

Pose::Pose(double x, double y, double z, double roll, double pitch, double yaw)
{
  set(x, y, z, roll, pitch, yaw);
}

Pose &Pose::set(double x, double y, double z, double roll, double pitch, double yaw)
{
  setXYZ(x, y, z);
  setRPY(roll, pitch, yaw);
  return *this;
}

Pose &Pose::setXYZ(double x, double y, double z)
{
  position.x = x, position.y = y, position.z = z;
  return *this;
}
Pose &Pose::setOrientation(double x, double y, double z, double w)
{
  orientation.x = x, orientation.y = y, orientation.z = z, orientation.w = w;
}
Pose &Pose::setRPY(double roll, double pitch, double yaw)
{
  double halfYaw = double(yaw) * double(0.5), halfPitch = double(pitch) * double(0.5),
         halfRoll = double(roll) * double(0.5);
  double cosYaw = cos(halfYaw), sinYaw = sin(halfYaw), cosPitch = cos(halfPitch), sinPitch = sin(halfPitch),
         cosRoll = cos(halfRoll), sinRoll = sin(halfRoll);
  setOrientation(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,   // x
                 cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,   // y
                 cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,   // z
                 cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);  // formerly yzx
  return *this;
}
geometry_msgs::PosePtr Pose::create()
{
  geometry_msgs::PosePtr p = geometry_msgs::PosePtr(new geometry_msgs::Pose);
  Set(p, *this);
  return p;
}
void tuw::SetPositionXYZ(geometry_msgs::PosePtr &pose, double x, double y, double z)
{
  pose->position.x = x, pose->position.y = y, pose->position.z = z;
}
void tuw::SetOrientation(geometry_msgs::PosePtr &pose, double x, double y, double z, double w)
{
  pose->orientation.x = x, pose->orientation.y = y, pose->orientation.z = z, pose->orientation.w = w;
}
void tuw::SetRPY(geometry_msgs::PosePtr &pose, double roll, double pitch, double yaw)
{
  double halfYaw = double(yaw) * double(0.5), halfPitch = double(pitch) * double(0.5),
         halfRoll = double(roll) * double(0.5);
  double cosYaw = cos(halfYaw), sinYaw = sin(halfYaw), cosPitch = cos(halfPitch), sinPitch = sin(halfPitch),
         cosRoll = cos(halfRoll), sinRoll = sin(halfRoll);
  pose->orientation.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;  // x
  pose->orientation.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;  // y
  pose->orientation.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;  // z
  pose->orientation.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;  // formerly yzx
}
void tuw::Set(geometry_msgs::PosePtr &pose, double x, double y, double z, double roll, double pitch, double yaw)
{
  SetPositionXYZ(pose, x, y, z), SetRPY(pose, roll, pitch, yaw);
}
void tuw::Set(geometry_msgs::PosePtr &des, const Pose &src)
{
  SetPositionXYZ(des, src.position.x, src.position.y, src.position.z);
  SetOrientation(des, src.orientation.x, src.orientation.y, src.orientation.z, src.orientation.w);
}
