/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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
 *   * Neither the name of the PAL Robotics nor the names of its
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

/*
 * Author: Bence Magyar
 */

#include <tf/transform_datatypes.h>

#include <urdf_parser/urdf_parser.h>

#include <boost/assign.hpp>

#include <double_diff_drive_controller/double_diff_drive_controller.h>


namespace double_diff_drive_controller
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DoubleDiffDriveController::DoubleDiffDriveController()
  : open_loop_(false)
  , command_struct_()
  , wheel_radius_(0)
  , wheel_separation_(0)
  , drive_motor_gear_ratio_(0)
  , steer_motor_gear_ratio_(0)
  , cmd_vel_timeout_(0.5)
  , base_frame_id_("base_link")
  , enable_odom_tf_(true)
  , wheel_joints_size_(0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DoubleDiffDriveController::init(hardware_interface::VelocityJointInterface* hw,
                                     ros::NodeHandle& root_nh,
                                     ros::NodeHandle& controller_nh)
{
  const std::string complete_ns = controller_nh.getNamespace();
  std::size_t id = complete_ns.find_last_of("/");
  name_ = complete_ns.substr(id + 1);

  // Get joint names from the parameter server
  std::string drive_motor_name;
  controller_nh.param("drive_motor_joint", drive_motor_name, drive_motor_name);
  ROS_INFO_STREAM_NAMED(name_, "Drive motor joint (drive_motor) is : " << drive_motor_name);

  std::string steer_motor_name;
  controller_nh.param("steer_motor_joint", steer_motor_name, steer_motor_name);
  ROS_INFO_STREAM_NAMED(name_, "Steer motor joint (steer_motor) is : " << steer_motor_name);

  // Odometry related:
  double publish_rate;
  controller_nh.param("publish_rate", publish_rate, 20.0);
  ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                        << publish_rate << "Hz.");
  publish_period_ = ros::Duration(1.0 / publish_rate);

  controller_nh.param("open_loop", open_loop_, open_loop_);

  // Twist command related:
  controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
  ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                        << cmd_vel_timeout_ << "s.");

  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

  controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
  ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

  // Velocity and acceleration limits:
  controller_nh.param("linear/x/has_velocity_limits"    , limiter_lin_.has_velocity_limits    , limiter_lin_.has_velocity_limits    );
  controller_nh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
  controller_nh.param("linear/x/max_velocity"           , limiter_lin_.max_velocity           ,  limiter_lin_.max_velocity          );
  controller_nh.param("linear/x/min_velocity"           , limiter_lin_.min_velocity           , -limiter_lin_.max_velocity          );
  controller_nh.param("linear/x/max_acceleration"       , limiter_lin_.max_acceleration       ,  limiter_lin_.max_acceleration      );
  controller_nh.param("linear/x/min_acceleration"       , limiter_lin_.min_acceleration       , -limiter_lin_.max_acceleration      );

  controller_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
  controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
  controller_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
  controller_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
  controller_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
  controller_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );

  // Get the joint objects to use in the realtime loop
  drive_motor_input_ = hw->getHandle(drive_motor_name);  // throws on failure
  steer_motor_input_ = hw->getHandle(steer_motor_name);  // throws on failure

  // Pass params through and setup publishers and subscribers
  if (!setWheelParamsFromUrdf(root_nh, controller_nh, drive_motor_name, steer_motor_name))
  {
    return false;
  }

  setupRtPublishersMsg(root_nh, controller_nh);

  sub_command_ = controller_nh.subscribe("cmd_vel", 1, &DoubleDiffDriveController::cmdVelCallback, this);

  return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DoubleDiffDriveController::update(const ros::Time& time, const ros::Duration& period)
{
  // COMPUTE AND PUBLISH ODOMETRY
  if (open_loop_)
  {
    odometry_.updateOpenLoop(last_cmd_.lin, last_cmd_.ang, time);
  }
  else
  {
    double drive_vel = drive_motor_input_.getVelocity();
    double steer_vel = steer_motor_input_.getVelocity();
    if (std::isnan(drive_vel) || std::isnan(steer_vel))
      return;

    // Estimate twist (using joint information) and integrate
    odometry_.update(drive_vel, steer_vel, time);
  }

  // Publish odometry message
  if(last_state_publish_time_ + publish_period_ < time)
  {
    last_state_publish_time_ += publish_period_;
    // Compute and store orientation info
    const geometry_msgs::Quaternion orientation(
          tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

    // Populate odom message and publish
    if(odom_pub_->trylock())
    {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
      odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
      odom_pub_->msg_.pose.pose.orientation = orientation;
      odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinear();
      odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
      odom_pub_->unlockAndPublish();
    }

    // Publish tf /odom frame
    if (enable_odom_tf_ && tf_odom_pub_->trylock())
    {
      geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
      odom_frame.header.stamp = time;
      odom_frame.transform.translation.x = odometry_.getX();
      odom_frame.transform.translation.y = odometry_.getY();
      odom_frame.transform.rotation = orientation;
      tf_odom_pub_->unlockAndPublish();
    }
  }

  // MOVE ROBOT
  // Retrieve current velocity command and time step:
  Commands curr_cmd = *(command_.readFromRT());
  const double dt = (time - curr_cmd.stamp).toSec();

  // Brake if cmd_vel has timeout:
  if (dt > cmd_vel_timeout_)
  {
    curr_cmd.lin = 0.0;
    curr_cmd.ang = 0.0;
  }

  // Limit velocities and accelerations:
  const double cmd_dt(period.toSec());
  limiter_lin_.limit(curr_cmd.lin, last_cmd_.lin, cmd_dt);
  limiter_ang_.limit(curr_cmd.ang, last_cmd_.ang, cmd_dt);
  last_cmd_ = curr_cmd;

  // Compute wheels velocities (this is the actual ik):
  // NOTE: the input desired twist (from topic /cmd_vel) is a body twist.
  const double drive_vel = (1.0 / wheel_radius_) * (curr_cmd.lin * drive_motor_gear_ratio_);
  const double steer_vel = (1.0 / wheel_radius_) * (2.0 / wheel_separation_)
                         * (curr_cmd.ang * steer_motor_gear_ratio_);

  // Set wheels velocities:
  drive_motor_input_.setCommand(drive_vel);
  steer_motor_input_.setCommand(steer_vel);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DoubleDiffDriveController::starting(const ros::Time& time)
{
  brake();

  // Register starting time used to keep fixed rate
  last_state_publish_time_ = time;

  odometry_.init(time);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DoubleDiffDriveController::stopping(const ros::Time& time)
{
  brake();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DoubleDiffDriveController::brake()
{
  drive_motor_input_.setCommand(0.0);
  steer_motor_input_.setCommand(0.0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DoubleDiffDriveController::cmdVelCallback(const geometry_msgs::Twist& command)
{
  if(isRunning())
  {
    command_struct_.ang  = command.angular.z;
    command_struct_.lin  = command.linear.x;
    command_struct_.stamp = ros::Time::now();
    command_.writeFromNonRT (command_struct_);
    ROS_DEBUG_STREAM_NAMED(name_,
                           "Added values to command. "
                           << "Ang: "   << command_struct_.ang << ", "
                           << "Lin: "   << command_struct_.lin << ", "
                           << "Stamp: " << command_struct_.stamp);
  }
  else
  {
    ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DoubleDiffDriveController::setWheelParamsFromUrdf(ros::NodeHandle& root_nh,
                                                       ros::NodeHandle &controller_nh,
                                                       const std::string& drive_motor_name,
                                                       const std::string& steer_motor_name)
{
  bool has_wheel_separation = controller_nh.getParam("wheel_separation", wheel_separation_);
  if (!has_wheel_separation)
  {
    ROS_ERROR_STREAM_NAMED(name_, " wheel_separation couldn't be retrieved");
    return false;
  }

  bool has_wheel_radius = controller_nh.getParam("wheel_radius", wheel_radius_);
  if (!has_wheel_radius)
  {
    ROS_ERROR_STREAM_NAMED(name_, " wheel_radius couldn't be retrieved");
    return false;
  }

  bool has_drive_motor_gear_ratio = controller_nh.getParam("drive_motor_gear_ratio", drive_motor_gear_ratio_);
  if (!has_drive_motor_gear_ratio)
  {
    ROS_ERROR_STREAM_NAMED(name_, " drive_motor_gear_ratio couldn't be retrieved");
    return false;
  }

  bool has_steer_motor_gear_ratio = controller_nh.getParam("steer_motor_gear_ratio", steer_motor_gear_ratio_);
  if (!has_steer_motor_gear_ratio)
  {
    ROS_ERROR_STREAM_NAMED(name_, " steer_motor_gear_ratio couldn't be retrieved");
    return false;
  }

  ROS_INFO_STREAM("Wheel radius: " << wheel_radius_);
  ROS_INFO_STREAM("Wheel seperation: "  << wheel_separation_);
  ROS_INFO_STREAM("Drive motor gear ratio: " << drive_motor_gear_ratio_);
  ROS_INFO_STREAM("Steer motor gear ratio: " << steer_motor_gear_ratio_);

  // Set wheel params for the odometry computation
  odometry_.setWheelsParams(wheel_radius_, wheel_separation_);
  odometry_.setGearRatios(drive_motor_gear_ratio_, steer_motor_gear_ratio_);

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DoubleDiffDriveController::setupRtPublishersMsg(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Get covariance parameters for odometry.
  XmlRpc::XmlRpcValue pose_cov_list;
  controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
  ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pose_cov_list.size() == 6);
  for (int i = 0; i < pose_cov_list.size(); ++i)
    ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  XmlRpc::XmlRpcValue twist_cov_list;
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
  ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(twist_cov_list.size() == 6);
  for (int i = 0; i < twist_cov_list.size(); ++i)
    ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  // Setup odometry msg.
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = base_frame_id_;
  odom_pub_->msg_.pose.pose.position.z = 0;
  odom_pub_->msg_.pose.covariance = boost::assign::list_of
      (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
      (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
      (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
      (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
      (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
      (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
  odom_pub_->msg_.twist.twist.linear.y  = 0;
  odom_pub_->msg_.twist.twist.linear.z  = 0;
  odom_pub_->msg_.twist.twist.angular.x = 0;
  odom_pub_->msg_.twist.twist.angular.y = 0;
  odom_pub_->msg_.twist.covariance = boost::assign::list_of
      (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
      (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
      (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
      (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
      (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
      (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));

  // Setup tf msg.
  tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
  tf_odom_pub_->msg_.transforms.resize(1);
  tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
  tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
  tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
}

} // namespace double_diff_drive_controller
