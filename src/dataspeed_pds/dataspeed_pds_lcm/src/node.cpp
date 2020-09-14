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

#include <ros/ros.h>
#include "PdsNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pds");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  const int ROS_RESPONSE_TIME_MS = 100;
  const float TRY_CONNECT_EVERY_S = 1.0;
  const float WARN_EVERY_S = 10.0;

  std::string lcm_url;
  priv_nh.getParam("lcm_url", lcm_url);
  if (lcm_url.empty()) {
    lcm_url = "udpm://225.0.0.0:7667?ttl=1";
  }

  // Initialize LCM
  lcm::LCM * lcm;
  while(ros::ok()) {
    lcm = new lcm::LCM(lcm_url);
    if(lcm->good()) {
      break;
    } else {
      ROS_WARN_THROTTLE(WARN_EVERY_S,"lcm is not initialized, is the network ready?");
      delete lcm;
    }
    ros::Duration(TRY_CONNECT_EVERY_S).sleep();
  }
  ROS_INFO("LCM connected to %s", lcm_url.c_str());

  // Create PdsNode class
  dataspeed_pds_lcm::PdsNode n(node, priv_nh, lcm);

  // Handle ros callbacks asynchronous to lcm
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // TODO: This is a little jenky, lcm handling could get its own thread
  while(ros::ok()) {
    lcm->handleTimeout(ROS_RESPONSE_TIME_MS);
  }

  return 0;
}

