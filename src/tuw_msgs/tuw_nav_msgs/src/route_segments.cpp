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

#include <tuw_nav_msgs/route_segment.h>
#include <tuw_nav_msgs/route_segments.h>
#include <tf/tf.h>

using namespace tuw::ros_msgs;

RouteSegments::RouteSegments(){

};

RouteSegments::RouteSegments(size_t n)
{
  segments.resize(n);
}

void RouteSegments::set_ids(const std::vector<unsigned int> &id)
{
  if (segments.size() != id.size())
    segments.resize(id.size());
  for (int i = 0; i < segments.size(); i++)
    segments[i].id = id[i];
}
void RouteSegments::set_type(const std::vector<unsigned int> &type)
{
  if (segments.size() != type.size())
    segments.resize(type.size());
  for (int i = 0; i < segments.size(); i++)
    segments[i].type = type[i];
}
void RouteSegments::set_orientation(const std::vector<unsigned int> &orientation)
{
  if (segments.size() != orientation.size())
    segments.resize(orientation.size());
  for (int i = 0; i < segments.size(); i++)
    segments[i].orientation = orientation[i];
}
void RouteSegments::set_motion_type(const std::vector<unsigned int> &motion_type)
{
  if (segments.size() != motion_type.size())
    segments.resize(motion_type.size());
  for (int i = 0; i < segments.size(); i++)
    segments[i].motion_type = motion_type[i];
}
void RouteSegments::set_start(const std::vector<double> &x, const std::vector<double> &y,
                              const std::vector<double> &theta)
{
  if (segments.size() != x.size())
    segments.resize(x.size());
  for (int i = 0; i < segments.size(); i++)
  {
    segments[i].start.position.x = x[i];
    segments[i].start.position.y = y[i];
    segments[i].start.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, theta[i]);
    segments[i].start.orientation.x = q.getX();
    segments[i].start.orientation.y = q.getY();
    segments[i].start.orientation.z = q.getZ();
    segments[i].start.orientation.w = q.getW();
  }
}
void RouteSegments::set_end(const std::vector<double> &x, const std::vector<double> &y,
                            const std::vector<double> &theta)
{
  if (segments.size() != x.size())
    segments.resize(x.size());
  for (int i = 0; i < segments.size(); i++)
  {
    segments[i].end.position.x = x[i];
    segments[i].end.position.y = y[i];
    segments[i].end.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, theta[i]);
    segments[i].end.orientation.x = q.getX();
    segments[i].end.orientation.y = q.getY();
    segments[i].end.orientation.z = q.getZ();
    segments[i].end.orientation.w = q.getW();
  }
}
void RouteSegments::set_center(const std::vector<double> &x, const std::vector<double> &y,
                               const std::vector<double> &theta)
{
  if (segments.size() != x.size())
    segments.resize(x.size());
  for (int i = 0; i < segments.size(); i++)
  {
    segments[i].center.position.x = x[i];
    segments[i].center.position.y = y[i];
    segments[i].center.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, theta[i]);
    segments[i].center.orientation.x = q.getX();
    segments[i].center.orientation.y = q.getY();
    segments[i].center.orientation.z = q.getZ();
    segments[i].center.orientation.w = q.getW();
  }
}
void RouteSegments::set_level(const std::vector<int> &level)
{
  if (segments.size() != level.size())
    segments.resize(level.size());
  for (int i = 0; i < segments.size(); i++)
    segments[i].level = level[i];
}

void RouteSegments::convert(nav_msgs::Path &path, double distance) const
{
  path.header = header;
  double offset = 0;
  std::vector<geometry_msgs::PosePtr> waypoints;
  for (size_t i = 0; i < segments.size(); i++)
  {
    const tuw::ros_msgs::RouteSegment &segment = (const tuw::ros_msgs::RouteSegment &)segments[i];
    offset = segment.sample_equal_distance(waypoints, distance, offset);
  }
  /// check if an additional end point is needed
  if (offset > 0)
  {
    path.poses.resize(waypoints.size() + 1);
  }
  else
  {
    path.poses.resize(waypoints.size());
  }
  for (size_t i = 0; i < waypoints.size(); i++)
  {
    path.poses[i].header = header;
    path.poses[i].pose = *waypoints[i];
  }
  if (offset > 0)
  {
    /// add the additional end point
    path.poses[waypoints.size()].header = header;
    path.poses[waypoints.size()].pose = segments.back().end;
  }
}
