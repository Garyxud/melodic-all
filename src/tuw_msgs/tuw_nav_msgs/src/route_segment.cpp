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

#include <tuw_geometry_msgs/point.h>
#include <tuw_nav_msgs/route_segment.h>

using namespace tuw::ros_msgs;

RouteSegment::RouteSegment(){

};

double RouteSegment::sample_equal_distance(std::vector<geometry_msgs::PosePtr> &poses, double distance_resolution,
                                           double distance_offset) const
{
  if (type == LINE)
  {
    double dx = end.position.x - start.position.x, dy = end.position.y - start.position.y;
    double d = sqrt(dx * dx + dy * dy);
    double ux = dx / d, uy = dy / d;
    tuw::ros_msgs::Pose pose;
    double l = distance_offset;
    while (l < d)
    {
      pose.setXYZ(start.position.x + ux * l, start.position.y + uy * l, 0);
      pose.setRPY(0, 0, atan2(uy, ux));
      poses.push_back(pose.create());
      l += distance_resolution;
    }
    distance_offset = l - d;
  }
  else if (type == ARC)
  {
    static const double LEFT = -1.;
    static const double RIGHT = +1.;
    double direction = LEFT;
    if (orientation == tuw::ros_msgs::RouteSegment::CLOCKWISE)
      direction = LEFT;
    else if (orientation == tuw::ros_msgs::RouteSegment::COUNTER_CLOCKWISE)
      direction = RIGHT;
    else
      throw 0;

    double dx0 = start.position.x - center.position.x, dy0 = start.position.y - center.position.y;
    double a0 = atan2(dy0, dx0);
    double radius = sqrt(dx0 * dx0 + dy0 * dy0);
    double dx1 = end.position.x - center.position.x, dy1 = end.position.y - center.position.y;
    double a1 = atan2(dy1, dx1);

    double da = atan2(sin(a0 - a1), cos(a0 - a1));  /// signed minimal delta angle difference
    if (direction == RIGHT)
    {
      da = da < 0 ? da : da - 2 * M_PI;
    }
    else
    {
      da = da >= 0 ? da : da + 2 * M_PI;
    }
    double d = fabs(da) * radius;

    tuw::ros_msgs::Pose pose;
    double l = distance_offset;
    while (l < d)
    {
      double a = a0 + l / radius * direction;
      pose.setXYZ(center.position.x + cos(a) * radius, center.position.y + sin(a) * radius, 0);
      pose.setRPY(0, 0, a + (M_PI / 2. * direction));
      poses.push_back(pose.create());
      l += distance_resolution;
    }
    distance_offset = l - d;
  }
  return distance_offset;
}

double RouteSegment::sample_equal_angle(std::vector<geometry_msgs::PosePtr> &poses, double angle_resolution,
                                        double distance_offset) const
{
  if (type == ARC)
  {
    double radius = tuw::Distance(start.position, center.position);
    double distance_resolution = angle_resolution * radius;
    return sample_equal_distance(poses, distance_resolution, distance_offset);
  }
  if (type == LINE)
  {
    double distance_resolution = tuw::Distance(start.position, end.position);
    return sample_equal_distance(poses, distance_resolution, distance_offset);
  }
}
