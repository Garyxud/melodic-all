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

#include "PdsNode.h"

namespace dataspeed_pds_lcm
{

PdsNode::PdsNode(ros::NodeHandle &nh, ros::NodeHandle &priv_nh, lcm::LCM *lcm)
{
  // Setup message synchronizers
  sync_ros_slave1_ = new message_filters::Synchronizer<SyncPolicy1>(SyncPolicy1(5), sync_msg_master_, sync_msg_slave1_);
  sync_ros_slave2_ = new message_filters::Synchronizer<SyncPolicy2>(SyncPolicy2(5), sync_msg_master_, sync_msg_slave1_, sync_msg_slave2_);
  sync_ros_slave3_ = new message_filters::Synchronizer<SyncPolicy3>(SyncPolicy3(5), sync_msg_master_, sync_msg_slave1_, sync_msg_slave2_, sync_msg_slave3_);
  sync_ros_slave1_->registerCallback(boost::bind(&PdsNode::recvSyncSlave1, this, _1, _2));
  sync_ros_slave2_->registerCallback(boost::bind(&PdsNode::recvSyncSlave2, this, _1, _2, _3));
  sync_ros_slave3_->registerCallback(boost::bind(&PdsNode::recvSyncSlave3, this, _1, _2, _3, _4));

  // Reduce synchronization delay
  const ros::Duration SYNC_20_MS(0.006); // 30% of 20ms period
  sync_ros_slave1_->setInterMessageLowerBound(0, SYNC_20_MS);
  sync_ros_slave1_->setInterMessageLowerBound(1, SYNC_20_MS);
  sync_ros_slave2_->setInterMessageLowerBound(0, SYNC_20_MS);
  sync_ros_slave2_->setInterMessageLowerBound(1, SYNC_20_MS);
  sync_ros_slave2_->setInterMessageLowerBound(2, SYNC_20_MS);
  sync_ros_slave3_->setInterMessageLowerBound(0, SYNC_20_MS);
  sync_ros_slave3_->setInterMessageLowerBound(1, SYNC_20_MS);
  sync_ros_slave3_->setInterMessageLowerBound(2, SYNC_20_MS);
  sync_ros_slave3_->setInterMessageLowerBound(3, SYNC_20_MS);

  // Setup Publishers
  pub_status_ = nh.advertise<dataspeed_pds_msgs::Status>("status", 10);

  // Setup Subscribers
  sub_relay_ = nh.subscribe("relay", 10, &PdsNode::recvRelay, this, ros::TransportHints().tcpNoDelay(true));
  sub_mode_ = nh.subscribe("mode", 10, &PdsNode::recvMode, this, ros::TransportHints().tcpNoDelay(true));
  sub_script_ = nh.subscribe("script", 10, &PdsNode::recvScript, this, ros::TransportHints().tcpNoDelay(true));

  // Hold onto the LCM handle
  lcm_ = *lcm;

  // LCM Subscribers
  lcm_.subscribe("STATUS", &PdsNode::lcmRecvStatus, this);
}
PdsNode::~PdsNode()
{
  if (sync_ros_slave1_) {
    delete sync_ros_slave1_;
    sync_ros_slave1_ = NULL;
  }
  if (sync_ros_slave2_) {
    delete sync_ros_slave2_;
    sync_ros_slave2_ = NULL;
  }
  if (sync_ros_slave3_) {
    delete sync_ros_slave3_;
    sync_ros_slave3_ = NULL;
  }
}

void PdsNode::lcmRecvStatus(const lcm::ReceiveBuffer* buf, const std::string &chan, const status_t *lcm) {
#if 0
  ROS_INFO("Received LCM message from channel '%s'", chan.c_str());
#endif

  // Timestamp
  const ros::Time now = ros::Time::now();

  // Convert to ROS message
  dataspeed_pds_msgs::Status msg;
  msg.header.stamp = now;
  msg.mode.mode = lcm->mode;
  msg.script.script = 0; ///@TODO: LCM message doesn't report script status
  msg.chan.resize(12);
  for (size_t i = 0; i < 12; i++) {
    msg.chan[i].current = lcm->current[i];
    msg.chan[i].status  = lcm->status[i];
  }
  msg.master.inverter.request  = (lcm->inverter_status & (1 << 0)) ? true : false;
  msg.master.inverter.status   = (lcm->inverter_status & (1 << 1)) ? true : false;
  msg.master.inverter.overload = (lcm->inverter_status & (1 << 2)) ? true : false;
  msg.master.inverter.overtemp = (lcm->inverter_status & (1 << 3)) ? true : false;
  msg.master.temp.internal = lcm->board_temp;
  msg.master.temp.external = lcm->thermocouple;
  msg.master.voltage = lcm->voltage;

  // Publish for single unit, or forward to multi-unit synchronization
  const dataspeed_pds_msgs::Status::ConstPtr ptr(new dataspeed_pds_msgs::Status(msg));
  switch (lcm->unit_id) {
    case MASTER:
      sync_msg_master_.add(ptr);
      if ((now - stamp_slave1_) > TIMEOUT) {
        pub_status_.publish(msg);
      }
      break;
    case SLAVE1:
      stamp_slave1_ = now;
      sync_msg_slave1_.add(ptr);
      break;
    case SLAVE2:
      stamp_slave2_ = now;
      sync_msg_slave2_.add(ptr);
      break;
    case SLAVE3:
      stamp_slave3_ = now;
      sync_msg_slave3_.add(ptr);
      break;
  }
}

void PdsNode::recvRelay(const dataspeed_pds_msgs::Relay::ConstPtr &msg)
{
  relay_request_t out;
  out.channel = msg->channel;
  out.request = msg->request;
  lcm_.publish("RELAY", &out);
}

void PdsNode::recvMode(const dataspeed_pds_msgs::Mode::ConstPtr &msg)
{
  mode_t out;
  out.mode = msg->mode;
  lcm_.publish("MODE", &out);
}

void PdsNode::recvScript(const dataspeed_pds_msgs::Script::ConstPtr &msg)
{
  script_request_t out;
  out.script = msg->script;
  lcm_.publish("SCRIPT", &out);
}

void PdsNode::recvSyncSlave1(const SyncPtr& master, const SyncPtr& slave1)
{
  if ((ros::Time::now() - stamp_slave2_) > TIMEOUT) {
    dataspeed_pds_msgs::Status msg = *master;
    msg.chan.insert(msg.chan.end(), slave1->chan.begin(), slave1->chan.end());
    msg.slave.push_back(slave1->master);
    pub_status_.publish(msg);
  }
}
void PdsNode::recvSyncSlave2(const SyncPtr& master, const SyncPtr& slave1, const SyncPtr& slave2)
{
  if ((ros::Time::now() - stamp_slave3_) > TIMEOUT) {
    dataspeed_pds_msgs::Status msg = *master;
    msg.chan.insert(msg.chan.end(), slave1->chan.begin(), slave1->chan.end());
    msg.chan.insert(msg.chan.end(), slave2->chan.begin(), slave2->chan.end());
    msg.slave.push_back(slave1->master);
    msg.slave.push_back(slave2->master);
    pub_status_.publish(msg);
  }
}
void PdsNode::recvSyncSlave3(const SyncPtr& master, const SyncPtr& slave1, const SyncPtr& slave2, const SyncPtr& slave3)
{
  if (1) { // There is no slave4
    dataspeed_pds_msgs::Status msg = *master;
    msg.chan.insert(msg.chan.end(), slave1->chan.begin(), slave1->chan.end());
    msg.chan.insert(msg.chan.end(), slave2->chan.begin(), slave2->chan.end());
    msg.chan.insert(msg.chan.end(), slave3->chan.begin(), slave3->chan.end());
    msg.slave.push_back(slave1->master);
    msg.slave.push_back(slave2->master);
    msg.slave.push_back(slave3->master);
    pub_status_.publish(msg);
  }
}

} // namespace dataspeed_pds_lcm

