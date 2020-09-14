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

// LCM and messages
#include <lcm/lcm-cpp.hpp>
#include <dataspeed_pds_lcm/status_t.hpp>
using namespace dataspeed_pds_lcm;

// LCM handle
lcm::LCM * g_lcm = NULL;

void timerCallback(const ros::TimerEvent&, int id)
{
  ROS_ASSERT((0 <= id) && (id <= 3));
  status_t msg;
  memset(&msg, 0x00, sizeof(msg));
  msg.unit_id = id;
  g_lcm->publish("STATUS", &msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Configurable period
  double hz = 50; // 50 Hz
  nh_priv.getParam("hz", hz);

  // Multiple units (optional)
  int unit_id = 0;
  nh_priv.getParam("unit_id", unit_id);
  if (unit_id < 0) {
    unit_id = 0;
  } else if (unit_id > 3) {
    unit_id = 3;
  }

  // LCM URL
  std::string lcm_url;
  nh_priv.getParam("lcm_url", lcm_url);
  if (lcm_url.empty()) {
    lcm_url = "udpm://225.0.0.0:7667?ttl=0";
  }

  // Initialize LCM
  lcm::LCM * lcm;
  while(ros::ok()) {
    lcm = new lcm::LCM(lcm_url);
    if(lcm->good()) {
      break;
    } else {
      ROS_WARN_THROTTLE(10.0,"lcm is not initialized, is the network ready?");
      delete lcm;
    }
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("LCM connected to %s", lcm_url.c_str());

  // Hold onto the LCM handle
  g_lcm = lcm;

  // Setup Timers
  std::vector<ros::Timer> timers;
  for (int i = 0; i <= unit_id; i++) {
    timers.push_back(nh.createTimer(ros::Duration(1.0 / hz), boost::bind(&timerCallback, _1, i)));
  }

  // Handle callbacks until shutdown
  ros::spin();

  return 0;
}

