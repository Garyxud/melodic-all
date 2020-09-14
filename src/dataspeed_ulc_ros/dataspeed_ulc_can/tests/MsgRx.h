/*********************************************************************
 * Software License Agreement (Proprietary and Confidential)
 *
 *  Copyright (c) 2017-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  NOTICE:  All information contained herein is, and remains the
 *  property of Dataspeed Inc. The intellectual and technical concepts
 *  contained herein are proprietary to Dataspeed Inc. and may be
 *  covered by U.S. and Foreign Patents, patents in process, and are
 *  protected by trade secret or copyright law. Dissemination of this
 *  information or reproduction of this material is strictly forbidden
 *  unless prior written permission is obtained from Dataspeed Inc.
 *********************************************************************/

#ifndef _MSG_RX_H_
#define _MSG_RX_H_

#include <ros/ros.h>

template <typename MsgT>
class MsgRx {
public:
  MsgRx(const ros::WallDuration& thresh) : dur_(thresh) {};
  MsgRx(const ros::WallDuration& thresh, const MsgT& msg) : dur_(thresh) { set(msg); };
  void set(const MsgT& msg) { msg_ = msg; stamp_ = ros::WallTime::now(); }
  void clear() { stamp_ = ros::WallTime(0); }
  bool fresh(ros::WallDuration delta) const { return age() < delta; }
  bool fresh() const { return fresh(dur_); }
  ros::WallDuration age() const { return !stamp_.isZero() ? ros::WallTime::now() - stamp_ : ros::WallDuration(9999.0); }
  const MsgT& get() const { return msg_; }
  const ros::WallTime& stamp() const { return stamp_; }
private:
  MsgT msg_;
  ros::WallTime stamp_;
  ros::WallDuration dur_;
};

#endif // _MSG_RX_H_
