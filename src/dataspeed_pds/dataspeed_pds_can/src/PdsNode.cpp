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
#include <dataspeed_pds_can/dispatch.h>

namespace dataspeed_pds_can
{

PdsNode::PdsNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
: sync_can_master_(8, boost::bind(&PdsNode::recvSync, this, _1, MASTER), ID_STATUS1_MASTER, ID_STATUS2_MASTER, ID_CURRENT1_MASTER, ID_CURRENT2_MASTER, ID_CURRENT3_MASTER)
, sync_can_slave1_(8, boost::bind(&PdsNode::recvSync, this, _1, SLAVE1), ID_STATUS1_SLAVE1, ID_STATUS2_SLAVE1, ID_CURRENT1_SLAVE1, ID_CURRENT2_SLAVE1, ID_CURRENT3_SLAVE1)
, sync_can_slave2_(8, boost::bind(&PdsNode::recvSync, this, _1, SLAVE2), ID_STATUS1_SLAVE2, ID_STATUS2_SLAVE2, ID_CURRENT1_SLAVE2, ID_CURRENT2_SLAVE2, ID_CURRENT3_SLAVE2)
, sync_can_slave3_(8, boost::bind(&PdsNode::recvSync, this, _1, SLAVE3), ID_STATUS1_SLAVE3, ID_STATUS2_SLAVE3, ID_CURRENT1_SLAVE3, ID_CURRENT2_SLAVE3, ID_CURRENT3_SLAVE3)
{
  // Setup message synchronizers
  sync_ros_slave1_ = new message_filters::Synchronizer<SyncPolicy1>(SyncPolicy1(4), sync_msg_master_, sync_msg_slave1_);
  sync_ros_slave2_ = new message_filters::Synchronizer<SyncPolicy2>(SyncPolicy2(4), sync_msg_master_, sync_msg_slave1_, sync_msg_slave2_);
  sync_ros_slave3_ = new message_filters::Synchronizer<SyncPolicy3>(SyncPolicy3(4), sync_msg_master_, sync_msg_slave1_, sync_msg_slave2_, sync_msg_slave3_);
  sync_ros_slave1_->registerCallback(boost::bind(&PdsNode::recvSyncSlave1, this, _1, _2));
  sync_ros_slave2_->registerCallback(boost::bind(&PdsNode::recvSyncSlave2, this, _1, _2, _3));
  sync_ros_slave3_->registerCallback(boost::bind(&PdsNode::recvSyncSlave3, this, _1, _2, _3, _4));

  // Reduce synchronization delay
  const ros::Duration SYNC_50_MS(0.016); // 30% of 50ms period
  sync_can_master_.setInterMessageLowerBound(SYNC_50_MS);
  sync_can_slave1_.setInterMessageLowerBound(SYNC_50_MS);
  sync_can_slave2_.setInterMessageLowerBound(SYNC_50_MS);
  sync_can_slave3_.setInterMessageLowerBound(SYNC_50_MS);
  sync_ros_slave1_->setInterMessageLowerBound(SYNC_50_MS);
  sync_ros_slave2_->setInterMessageLowerBound(SYNC_50_MS);
  sync_ros_slave3_->setInterMessageLowerBound(SYNC_50_MS);

  // Setup Publishers
  pub_status_ = node.advertise<dataspeed_pds_msgs::Status>("status", 10);
  pub_can_ = node.advertise<can_msgs::Frame>("can_tx", 10);

  // Setup Subscribers
  sub_relay_ = node.subscribe("relay", 10, &PdsNode::recvRelay, this, ros::TransportHints().tcpNoDelay(true));
  sub_mode_ = node.subscribe("mode", 10, &PdsNode::recvMode, this, ros::TransportHints().tcpNoDelay(true));
  sub_script_ = node.subscribe("script", 10, &PdsNode::recvScript, this, ros::TransportHints().tcpNoDelay(true));
  sub_can_ = node.subscribe("can_rx", 80, &PdsNode::recvCAN, this, ros::TransportHints().tcpNoDelay(true));
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

void PdsNode::recvCAN(const can_msgs::Frame::ConstPtr& msg)
{
  if (!msg->is_rtr && !msg->is_error && !msg->is_extended) {
    sync_can_master_.processMsg(msg);
    sync_can_slave1_.processMsg(msg);
    sync_can_slave2_.processMsg(msg);
    sync_can_slave3_.processMsg(msg);
    switch (msg->id) {
      case ID_REQUEST: // These command messages aren't handled.
      case ID_MODE:
      case ID_SCRIPT:
        break;
      case ID_RESERVED1: // Reserved for the Touchscreen Display
      case ID_RESERVED2:
      case ID_RESERVED3:
      case ID_RESERVED4:
        break;
      case ID_STATUS1_MASTER:
      case ID_STATUS2_MASTER:
      case ID_CURRENT1_MASTER:
      case ID_CURRENT2_MASTER:
      case ID_CURRENT3_MASTER:
        break;
      case ID_STATUS1_SLAVE1:
      case ID_STATUS2_SLAVE1:
      case ID_CURRENT1_SLAVE1:
      case ID_CURRENT2_SLAVE1:
      case ID_CURRENT3_SLAVE1:
        break;
      case ID_STATUS1_SLAVE2:
      case ID_STATUS2_SLAVE2:
      case ID_CURRENT1_SLAVE2:
      case ID_CURRENT2_SLAVE2:
      case ID_CURRENT3_SLAVE2:
        break;
      case ID_STATUS1_SLAVE3:
      case ID_STATUS2_SLAVE3:
      case ID_CURRENT1_SLAVE3:
      case ID_CURRENT2_SLAVE3:
      case ID_CURRENT3_SLAVE3:
        break;
      default:
#if 0
        ROS_WARN("Unknown message ID: %1$d (0x%1$01x)", msg->id);
#endif
        break;
    }
  }
}

void PdsNode::recvRelay(const dataspeed_pds_msgs::Relay::ConstPtr &msg)
{
  can_msgs::Frame out;
  ((MsgRelay*)out.data.elems)->channel = msg->channel;
  ((MsgRelay*)out.data.elems)->request = msg->request;
  out.id = ID_REQUEST;
  out.dlc = sizeof(MsgRelay);
  out.is_extended = false;
  pub_can_.publish(out);
}

void PdsNode::recvMode(const dataspeed_pds_msgs::Mode::ConstPtr &msg)
{
  can_msgs::Frame out;
  ((MsgMode*)out.data.elems)->mode = msg->mode;
  out.id = ID_MODE;
  out.dlc = sizeof(MsgMode);
  out.is_extended = false;
  pub_can_.publish(out);
}

void PdsNode::recvScript(const dataspeed_pds_msgs::Script::ConstPtr &msg)
{
  can_msgs::Frame out;
  MsgScript *ptr = (MsgScript*)out.data.elems;
  ((MsgScript*)out.data.elems)->script = msg->script;
  out.id = ID_SCRIPT;
  out.dlc = sizeof(MsgScript);
  out.is_extended = false;
  pub_can_.publish(out);
}

void PdsNode::recvSync(const std::vector<can_msgs::Frame::ConstPtr> &msgs, UnitId id)
{
  ROS_ASSERT((MASTER <= id) && (id <= SLAVE3));
  ROS_ASSERT(msgs.size() == 5);
  ROS_ASSERT(msgs[0]->id == ID_STATUS1_MASTER + id);
  ROS_ASSERT(msgs[1]->id == ID_STATUS2_MASTER + id);
  ROS_ASSERT(msgs[2]->id == ID_CURRENT1_MASTER + id);
  ROS_ASSERT(msgs[3]->id == ID_CURRENT2_MASTER + id);
  ROS_ASSERT(msgs[4]->id == ID_CURRENT3_MASTER + id);
  if ((msgs[0]->dlc >= sizeof(MsgStatus1))
   && (msgs[1]->dlc >= sizeof(MsgStatus2))
   && (msgs[2]->dlc >= sizeof(MsgCurrent))
   && (msgs[3]->dlc >= sizeof(MsgCurrent))
   && (msgs[4]->dlc >= sizeof(MsgCurrent))) {
    const MsgStatus1 *ptrS1 = (const MsgStatus1*)msgs[0]->data.elems;
    const MsgStatus2 *ptrS2 = (const MsgStatus2*)msgs[1]->data.elems;
    const MsgCurrent *ptrC1 = (const MsgCurrent*)msgs[2]->data.elems;
    const MsgCurrent *ptrC2 = (const MsgCurrent*)msgs[3]->data.elems;
    const MsgCurrent *ptrC3 = (const MsgCurrent*)msgs[4]->data.elems;

    // Merge CAN messages into a single ROS message
    dataspeed_pds_msgs::Status msg;
    msg.header.stamp = msgs[0]->header.stamp;
    msg.mode.mode     = ptrS1->mode;
    msg.script.script = ptrS1->script;
    msg.chan.resize(12);
    msg.chan[ 0].status = ptrS1->status_01;
    msg.chan[ 1].status = ptrS1->status_02;
    msg.chan[ 2].status = ptrS1->status_03;
    msg.chan[ 3].status = ptrS1->status_04;
    msg.chan[ 4].status = ptrS1->status_05;
    msg.chan[ 5].status = ptrS1->status_06;
    msg.chan[ 6].status = ptrS1->status_07;
    msg.chan[ 7].status = ptrS1->status_08;
    msg.chan[ 8].status = ptrS1->status_09;
    msg.chan[ 9].status = ptrS1->status_10;
    msg.chan[10].status = ptrS1->status_11;
    msg.chan[11].status = ptrS1->status_12;
    msg.chan[ 0].current = bytesToAmperes(ptrC1->current_01);
    msg.chan[ 1].current = bytesToAmperes(ptrC1->current_02);
    msg.chan[ 2].current = bytesToAmperes(ptrC1->current_03);
    msg.chan[ 3].current = bytesToAmperes(ptrC1->current_04);
    msg.chan[ 4].current = bytesToAmperes(ptrC2->current_05);
    msg.chan[ 5].current = bytesToAmperes(ptrC2->current_06);
    msg.chan[ 6].current = bytesToAmperes(ptrC2->current_07);
    msg.chan[ 7].current = bytesToAmperes(ptrC2->current_08);
    msg.chan[ 8].current = bytesToAmperes(ptrC3->current_09);
    msg.chan[ 9].current = bytesToAmperes(ptrC3->current_10);
    msg.chan[10].current = bytesToAmperes(ptrC3->current_11);
    msg.chan[11].current = bytesToAmperes(ptrC3->current_12);
    msg.master.inverter.request  = ptrS1->inverter_request;
    msg.master.inverter.status   = ptrS1->inverter_status;
    msg.master.inverter.overload = ptrS1->inverter_overload;
    msg.master.inverter.overtemp = ptrS1->inverter_overtemp;
    msg.master.temp.internal = bytesToCelsius(ptrS2->board_temp);
    msg.master.temp.external = bytesToCelsius(ptrS2->thermocouple_temp);
    msg.master.voltage = bytesToVoltage(ptrS2->voltage);

    // Publish for single unit, or forward to multi-unit synchronization
    const dataspeed_pds_msgs::Status::ConstPtr ptr(new dataspeed_pds_msgs::Status(msg));
    const ros::Time now = ros::Time::now();
    switch (id) {
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

} // namespace dataspeed_pds_can

