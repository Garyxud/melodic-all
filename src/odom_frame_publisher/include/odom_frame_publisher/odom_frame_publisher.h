#ifndef ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_H_INCLUDED
#define ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_H_INCLUDED

/**
 * @file odom_frame_publisher.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Definition of the OdomFramePublisher Class
 * @version 0.1
 * @date 2019-09-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// Headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <quaternion_operation/quaternion_operation.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/impl/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Headers in Boost
#include <boost/circular_buffer.hpp>

class OdomFramePublisher
{
public:
    OdomFramePublisher(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~OdomFramePublisher();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void currentTwistCallback(const geometry_msgs::TwistStamped::ConstPtr msg);
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
    std::string current_pose_topic_;
    std::string current_twist_topic_;
    std::string odom_frame_id_;
    std::string map_frame_id_;
    std::string robot_frame_id_;
    ros::Subscriber current_twist_sub_;
    ros::Subscriber current_pose_sub_;
    tf2_ros::TransformBroadcaster broadcaster_;
    boost::circular_buffer<geometry_msgs::TwistStamped> data_;
    geometry_msgs::PoseStamped current_odom_pose_;
    geometry_msgs::PoseStamped current_pose_;
    ros::Publisher odom_pose_pub_;
    bool current_pose_recieved_;
    tf2_ros::TransformListener listenter_;
    tf2_ros::Buffer buffer_;
};

#endif  //ODOM_FRAME_PUBLISHER_ODOM_FRAME_PUBLISHER_H_INCLUDED