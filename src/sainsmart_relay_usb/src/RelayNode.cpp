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

#include "RelayNode.h"
#include <ftdi.h>

namespace sainsmart_relay_usb
{

RelayNode::RelayNode(ros::NodeHandle &nh, ros::NodeHandle &nh_priv) : open_(false)
{
  // Initialize FTDI
  ctx_ = ftdi_new();
  ftdi_init(ctx_);

  // Get parameters
  nh_priv.getParam("serial", param_serial_);
  nh_priv.getParam("desc", param_desc_);

  // Setup publishers
  pub_ready_ = nh.advertise<std_msgs::Bool>("ready", 1, true);
  pub_serial_ = nh.advertise<std_msgs::String>("serial", 1, true);

  // Setup subscribers
  sub_ = nh.subscribe("relay_cmd", 10, &RelayNode::recv, this);

  // Connect to device
  serviceDevice();

  // Setup timer
  timer_ = nh.createWallTimer(ros::WallDuration(0.1), &RelayNode::timerCallback, this);
}

RelayNode::~RelayNode()
{
  // De-initialize FTDI
  if (ctx_) {
    ftdi_deinit(ctx_);
    ftdi_free(ctx_);
    ctx_ = NULL;
  }
}

void RelayNode::recv(const std_msgs::Byte::ConstPtr& msg)
{
  if (open_) {
    uint8_t value = msg->data;
    int ret = ftdi_write_data(ctx_, &value, sizeof(value));
    if (ret == sizeof(value)) {
      ROS_INFO("FTDI %s: Writing 0x%02X", serial_live_.c_str(), value);
    } else {
      ROS_WARN("FTDI %s: Writing 0x%02X failed, %i: %s", serial_live_.c_str(), value, ret, ftdi_get_error_string(ctx_));
      ftdi_usb_close(ctx_);
      open_ = false;
      publishReady(false);
    }
  }
}

void RelayNode::serviceDevice()
{
  if (!open_) {
    int ret = 0;
    struct ftdi_device_list *devlist;
    if (ftdi_usb_find_all(ctx_, &devlist, 0x0403, 0x6001) >= 0) {
      struct ftdi_device_list *curdev;
      char buf_manu[128], buf_desc[128], buf_serial[128];
      for (curdev = devlist; curdev != NULL; curdev = curdev->next) {
        if (ftdi_usb_get_strings(ctx_, curdev->dev, buf_manu, 128, buf_desc, 128, buf_serial, 128) >= 0) {
          if (!param_serial_.length() || (param_serial_ == std::string(buf_serial))) {
            if (!param_desc_.length() || (param_desc_ == std::string(buf_desc))) {
              if (ftdi_usb_open_dev(ctx_, curdev->dev) >= 0) {
                if (ftdi_set_bitmode(ctx_, -1, BITMODE_BITBANG) >= 0) {
                  ROS_INFO("Opened device %s %s %s", buf_manu, buf_desc, buf_serial);
                  serial_live_ = buf_serial;
                  open_ = true;
                  publishSerial(serial_live_);
                  publishReady(true);
                  break;
                } else {
                  ftdi_usb_close(ctx_);
                }
              }
            }
          }
        }
      }
      ftdi_list_free(&devlist);
    }
  } else {
    unsigned char buf[64] = {0};
    int ret = ftdi_read_data(ctx_, buf, sizeof(buf));
    if (ret != sizeof(buf)) {
      ROS_WARN("FTDI %s: Disconnected, %i: %s", serial_live_.c_str(), ret, ftdi_get_error_string(ctx_));
      ftdi_usb_close(ctx_);
      open_ = false;
      publishReady(false);
    } else {
#if 0
      ROS_INFO("FTDI %s: Ping, %i: %s", serial_live_.c_str(), ret, ftdi_get_error_string(ctx_));
#endif
    }
  }
}

void RelayNode::timerCallback(const ros::WallTimerEvent& event)
{
  serviceDevice();
}

} // namespace sainsmart_relay_usb

