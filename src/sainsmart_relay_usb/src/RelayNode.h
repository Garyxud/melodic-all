/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Dataspeed Inc.
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

#ifndef _RELAY_NODE_H_
#define _RELAY_NODE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/String.h>

struct ftdi_context;

namespace sainsmart_relay_usb
{

class RelayNode
{
public:
  RelayNode(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
  ~RelayNode();

private:
  void serviceDevice();
  void timerCallback(const ros::WallTimerEvent& event);
  void recv(const std_msgs::Byte::ConstPtr& msg);

  void publishReady(bool ready) {
    std_msgs::Bool msg; msg.data = ready;
    pub_ready_.publish(msg);
  }
  void publishSerial(const std::string &serial) {
    std_msgs::String msg; msg.data = serial;
    pub_serial_.publish(msg);
  }

  // Parameters
  std::string param_serial_; // Serial number
  std::string param_desc_; // Description

  // Timer
  ros::WallTimer timer_;

  // FTDI context
  ftdi_context *ctx_;

  // Subscribed topics
  ros::Subscriber sub_;

  // Published topics
  ros::Publisher pub_ready_;
  ros::Publisher pub_serial_;

  // Device serial number
  std::string serial_live_;

  // Keep track if USB device is open
  bool open_;
};

} // namespace sainsmart_relay_usb

#endif // _RELAY_NODE_H_

