/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// ROS and messages
#include <ros/ros.h>
#include <can_msgs/Frame.h>

// CAN message definitions
#include <dataspeed_pds_can/dispatch.h>
using namespace dataspeed_pds_can;

// Publisher
ros::Publisher g_pub;

can_msgs::Frame buildMsg(int unit_id, uint32_t can_id) {
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.dlc = 8;
  msg.id = unit_id + can_id;
  return msg;
}

void timerCallback(const ros::TimerEvent&, int id)
{
  ROS_ASSERT((0 <= id) && (id <= 3));
  g_pub.publish(buildMsg(id, ID_STATUS1_MASTER));
  g_pub.publish(buildMsg(id, ID_CURRENT1_MASTER));
  g_pub.publish(buildMsg(id, ID_CURRENT2_MASTER));
  g_pub.publish(buildMsg(id, ID_CURRENT3_MASTER));
  g_pub.publish(buildMsg(id, ID_STATUS2_MASTER));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Configurable period
  double hz = 20; // 20 Hz
  nh_priv.getParam("hz", hz);

  // Multiple units (optional)
  int unit_id = 0;
  nh_priv.getParam("unit_id", unit_id);
  if (unit_id < 0) {
    unit_id = 0;
  } else if (unit_id > 3) {
    unit_id = 3;
  }

  // Setup Publishers
  g_pub = nh.advertise<can_msgs::Frame>("can_tx", 100);

  // Setup Timers
  std::vector<ros::Timer> timers;
  for (int i = 0; i <= unit_id; i++) {
    timers.push_back(nh.createTimer(ros::Duration(1.0 / hz), boost::bind(&timerCallback, _1, i)));
  }

  // Handle callbacks until shutdown
  ros::spin();

  return 0;
}

