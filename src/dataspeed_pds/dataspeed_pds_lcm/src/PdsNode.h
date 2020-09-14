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

#ifndef _PDS_NODE_H_
#define _PDS_NODE_H_

#include <ros/ros.h>

// ROS messages
#include <dataspeed_pds_msgs/Mode.h>
#include <dataspeed_pds_msgs/Relay.h>
#include <dataspeed_pds_msgs/Script.h>
#include <dataspeed_pds_msgs/Status.h>

// Sync messages from multiple units
#include <message_filters/pass_through.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// LCM and messages
#include <lcm/lcm-cpp.hpp>
#include <dataspeed_pds_lcm/mode_t.hpp>
#include <dataspeed_pds_lcm/relay_request_t.hpp>
#include <dataspeed_pds_lcm/script_request_t.hpp>
#include <dataspeed_pds_lcm/status_t.hpp>

namespace dataspeed_pds_lcm
{

typedef enum {
  MASTER = 0,
  SLAVE1 = 1,
  SLAVE2 = 2,
  SLAVE3 = 3,
} UnitId;

const ros::Duration TIMEOUT(0.5);

class PdsNode
{
public:
  PdsNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh, lcm::LCM *lcm);
  ~PdsNode();

private:
  void lcmRecvStatus(const lcm::ReceiveBuffer* buf, const std::string &chan, const status_t *lcm);
  void recvRelay(const dataspeed_pds_msgs::Relay::ConstPtr &msg);
  void recvMode(const dataspeed_pds_msgs::Mode::ConstPtr &msg);
  void recvScript(const dataspeed_pds_msgs::Script::ConstPtr &msg);

  // Subscribed topics
  ros::Subscriber sub_relay_;
  ros::Subscriber sub_mode_;
  ros::Subscriber sub_script_;

  // Published topics
  ros::Publisher pub_status_;

  // Detect presence of multiple units
  ros::Time stamp_slave1_;
  ros::Time stamp_slave2_;
  ros::Time stamp_slave3_;

  // Time synchronization of multiple units
  typedef dataspeed_pds_msgs::Status SyncMsg;
  typedef dataspeed_pds_msgs::Status::ConstPtr SyncPtr;
  typedef message_filters::sync_policies::ApproximateTime<SyncMsg, SyncMsg> SyncPolicy1;
  typedef message_filters::sync_policies::ApproximateTime<SyncMsg, SyncMsg, SyncMsg> SyncPolicy2;
  typedef message_filters::sync_policies::ApproximateTime<SyncMsg, SyncMsg, SyncMsg, SyncMsg> SyncPolicy3;
  message_filters::Synchronizer<SyncPolicy1> *sync_ros_slave1_;
  message_filters::Synchronizer<SyncPolicy2> *sync_ros_slave2_;
  message_filters::Synchronizer<SyncPolicy3> *sync_ros_slave3_;
  message_filters::PassThrough<SyncMsg> sync_msg_master_;
  message_filters::PassThrough<SyncMsg> sync_msg_slave1_;
  message_filters::PassThrough<SyncMsg> sync_msg_slave2_;
  message_filters::PassThrough<SyncMsg> sync_msg_slave3_;
  void recvSyncSlave1(const SyncPtr& master, const SyncPtr& slave1);
  void recvSyncSlave2(const SyncPtr& master, const SyncPtr& slave1, const SyncPtr& slave2);
  void recvSyncSlave3(const SyncPtr& master, const SyncPtr& slave1, const SyncPtr& slave2, const SyncPtr& slave3);

  // LCM object
  lcm::LCM lcm_;

}; // class PdsNode

} // namespace dataspeed_pds_lcm

#endif // _PDS_NODE_H_

