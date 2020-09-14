/*
 * Copyright (c) 2013, Osnabrueck University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the Osnabrueck University nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <string>

#include "volksbot.h"

class ROSComm : public Comm
{
  public:
    ROSComm(
        const ros::NodeHandle &n,
        double sigma_x,
        double sigma_theta,
        double cov_x_y,
        double cov_x_theta,
        double cov_y_theta,
	size_t num_wheels,
	std::vector<std::string> joint_names) :
      n_(n),
      sigma_x_(sigma_x),
      sigma_theta_(sigma_theta),
      cov_x_y_(cov_x_y),
      cov_x_theta_(cov_x_theta),
      cov_y_theta_(cov_y_theta),
      publish_tf_(false),
      odom_pub_(n_.advertise<nav_msgs::Odometry> ("odom", 10)),
      joint_pub_(n_.advertise<sensor_msgs::JointState> ("joint_states", 1)),
      num_wheels_(num_wheels),
      joint_names_(joint_names){ }

    virtual void send_odometry(double x, double y, double theta, double v_x,
        double v_theta, double wheelpos_l, double wheelpos_r);

    void setTFPrefix(const std::string &tf_prefix);

  private:
    void populateCovariance(nav_msgs::Odometry &msg, double v_x, double
        v_theta);

    ros::NodeHandle n_;
    double sigma_x_, sigma_theta_, cov_x_y_, cov_x_theta_, cov_y_theta_;
    bool publish_tf_;
    std::string tf_prefix_;

    tf::TransformBroadcaster odom_broadcaster_;
    ros::Publisher odom_pub_;
    ros::Publisher joint_pub_;
    size_t num_wheels_;
    std::vector<std::string> joint_names_;
};

void ROSComm::setTFPrefix(const std::string &tf_prefix)
{
  tf_prefix_ = tf_prefix;
}

void ROSComm::populateCovariance(nav_msgs::Odometry &msg, double v_x, double v_theta)
{
  double odom_multiplier = 1.0;

  if (fabs(v_x) <= 1e-8 && fabs(v_theta) <= 1e-8)
  {
    //nav_msgs::Odometry has a 6x6 covariance matrix
    msg.twist.covariance[0] = 1e-12;
    msg.twist.covariance[35] = 1e-12;

    msg.twist.covariance[30] = 1e-12;
    msg.twist.covariance[5] = 1e-12;
  }
  else
  {
    //nav_msgs::Odometry has a 6x6 covariance matrix
    msg.twist.covariance[0] = odom_multiplier * pow(sigma_x_, 2);
    msg.twist.covariance[35] = odom_multiplier * pow(sigma_theta_, 2);

    msg.twist.covariance[30] = odom_multiplier * cov_x_theta_;
    msg.twist.covariance[5] = odom_multiplier * cov_x_theta_;
  }

  msg.twist.covariance[7] = DBL_MAX;
  msg.twist.covariance[14] = DBL_MAX;
  msg.twist.covariance[21] = DBL_MAX;
  msg.twist.covariance[28] = DBL_MAX;

  msg.pose.covariance = msg.twist.covariance;

  if (fabs(v_x) <= 1e-8 && fabs(v_theta) <= 1e-8)
  {
    msg.pose.covariance[7] = 1e-12;

    msg.pose.covariance[1] = 1e-12;
    msg.pose.covariance[6] = 1e-12;

    msg.pose.covariance[31] = 1e-12;
    msg.pose.covariance[11] = 1e-12;
  }
  else
  {
    msg.pose.covariance[7] = odom_multiplier * pow(sigma_x_, 2) * pow(sigma_theta_, 2);

    msg.pose.covariance[1] = odom_multiplier * cov_x_y_;
    msg.pose.covariance[6] = odom_multiplier * cov_x_y_;

    msg.pose.covariance[31] = odom_multiplier * cov_y_theta_;
    msg.pose.covariance[11] = odom_multiplier * cov_y_theta_;
  }
}

void ROSComm::send_odometry(double x, double y, double theta, double v_x, double v_theta, double wheelpos_l, double wheelpos_r)
{
  nav_msgs::Odometry odom;
  odom.header.frame_id = tf::resolve(tf_prefix_, "odom_combined");
  odom.child_frame_id = tf::resolve(tf_prefix_, "base_footprint");

  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  odom.twist.twist.linear.x = v_x;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = v_theta;
  populateCovariance(odom, v_x, v_theta);

  odom_pub_.publish(odom);

  if (publish_tf_)
  {
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = tf::resolve(tf_prefix_, "odom_combined");
    odom_trans.child_frame_id = tf::resolve(tf_prefix_, "base_footprint");

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);

    odom_broadcaster_.sendTransform(odom_trans);
  }

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(num_wheels_);
  joint_state.position.resize(num_wheels_);
  joint_state.name = joint_names_;

  if(num_wheels_ == 6)
  {
    joint_state.position[0] = joint_state.position[1] = joint_state.position[2] = wheelpos_l;
    joint_state.position[3] = joint_state.position[4] = joint_state.position[5] = wheelpos_r;
  }
  else
  {
    joint_state.position[0] = joint_state.position[1] = wheelpos_l;
    joint_state.position[2] = joint_state.position[3] = wheelpos_r; 
  }

  joint_pub_.publish(joint_state);
}

class ROSCall
{
  public:
    ROSCall(Volksbot &volksbot, double axis_length) :
      volksbot_(volksbot),
      axis_length_(axis_length),
      last_cmd_vel_time_(0.0) { }
    void velCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void cmd_velWatchdog(const ros::TimerEvent& event);

  private:
    Volksbot &volksbot_;
    double axis_length_;
    ros::Time last_cmd_vel_time_;
};

void ROSCall::velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  last_cmd_vel_time_ = ros::Time::now();
  double max_vel = volksbot_.get_max_vel();
  double velocity = msg->linear.x;

  velocity = std::min(max_vel, velocity);
  velocity = std::max(-max_vel, velocity);
  volksbot_.set_wheel_speed(velocity - axis_length_ * msg->angular.z * 0.5, velocity + axis_length_ * msg->angular.z * 0.5);
}

void ROSCall::cmd_velWatchdog(const ros::TimerEvent& event)
{
  if (ros::Time::now() - last_cmd_vel_time_ > ros::Duration(0.6))
    volksbot_.set_wheel_speed(0.0, 0.0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "volksbot");
  ros::NodeHandle n;
  ros::NodeHandle nh_ns("~");

  /* This is the wheel Radius for the odometry, accounting for some slip in the movement.
   * therefor it's not the same as the one in the volksbot.urdf.xacro */
  double wheel_radius;
  nh_ns.param("wheel_radius", wheel_radius, 0.0985);
  double axis_length;
  nh_ns.param("axis_length", axis_length, 0.41);
  int gear_ratio;
  nh_ns.param("gear_ratio", gear_ratio, 74);
  int max_vel_l;
  nh_ns.param("max_vel_l", max_vel_l, 8250);
  int max_vel_r;
  nh_ns.param("max_vel_r", max_vel_r, 8400);
  int max_acc_l;
  nh_ns.param("max_acc_l", max_acc_l, 10000);
  int max_acc_r;
  nh_ns.param("max_acc_r", max_acc_r, 10000);
  bool drive_backwards;
  nh_ns.param("drive_backwards", drive_backwards, false);

  double turning_adaptation;
  nh_ns.param("turning_adaptation", turning_adaptation, 0.95);

  int num_wheels;
  // 4 or 6 wheels are supported
  nh_ns.param("num_wheels", num_wheels, 6);
  if(num_wheels != 4 && num_wheels != 6)
  {
    ROS_FATAL("Wrong configuration of the volksbot driver: Only four or six wheels are supported! See param \"num_wheels\".");
    exit(1);
  }

  std::vector<std::string> joint_names;
  if(nh_ns.getParam("joint_names", joint_names))
  {
    if(num_wheels != joint_names.size())
    {
      ROS_FATAL("Wrong configuration of the volksbot driver: The number of joint names must equak the number of wheels!");
      exit(1);
    }
    ROS_INFO("Using joint names from \"joint_names\" parameter list");    
  }
  else if(num_wheels == 6)
  {
    ROS_INFO("Using default joint names for six wheels.");    
    // default values;
    joint_names = {
      "left_front_wheel_joint",
      "left_middle_wheel_joint",
      "left_rear_wheel_joint",
      "right_front_wheel_joint",
      "right_middle_wheel_joint",
      "right_rear_wheel_joint"
    };
  }
  else
  {
    // default values;
    ROS_INFO("Using default joint names for four wheels.");    
    joint_names = {
      "left_front_wheel_joint",
      "left_rear_wheel_joint",
      "right_front_wheel_joint",
      "right_rear_wheel_joint"
    };
  }

  double sigma_x, sigma_theta, cov_x_y, cov_x_theta, cov_y_theta;
  nh_ns.param("x_stddev", sigma_x, 0.002);
  nh_ns.param("rotation_stddev", sigma_theta, 0.017);
  nh_ns.param("cov_xy", cov_x_y, 0.0);
  nh_ns.param("cov_xrotation", cov_x_theta, 0.0);
  nh_ns.param("cov_yrotation", cov_y_theta, 0.0);

  ROSComm roscomm(n, sigma_x, sigma_theta, cov_x_y, cov_x_theta, cov_y_theta, num_wheels, joint_names);

  Volksbot volksbot(roscomm, wheel_radius, axis_length, turning_adaptation, gear_ratio, max_vel_l, max_vel_r, max_acc_l, max_acc_r, drive_backwards);

  bool publish_tf;
  nh_ns.param("publish_tf", publish_tf, false);
  std::string tf_prefix;
  tf_prefix = tf::getPrefixParam(nh_ns);
  roscomm.setTFPrefix(tf_prefix);

  ROSCall roscall(volksbot, axis_length);

  ros::Timer timer = n.createTimer(ros::Duration(0.1), &ROSCall::cmd_velWatchdog, &roscall);
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, &ROSCall::velCallback, &roscall);

  while (ros::ok())
  {
    volksbot.odometry();
    ros::spinOnce();
  }

  return 0;
}
