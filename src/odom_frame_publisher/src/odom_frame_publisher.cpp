/**
 * @file odom_frame_publisher.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Implimentation of the OdomFramePublisher Class
 * @version 0.1
 * @date 2019-09-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// headers in this package
#include <odom_frame_publisher/odom_frame_publisher.h>

/**
 * @brief Constructor of the OdomFramePublisher Class
 * 
 */
OdomFramePublisher::OdomFramePublisher(ros::NodeHandle nh,ros::NodeHandle pnh):listenter_(buffer_)
{
    buffer_.setUsingDedicatedThread(true);
    current_pose_recieved_ = false;
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("current_pose_topic", current_pose_topic_, "current_pose");
    pnh_.param<std::string>("current_twist_topic", current_twist_topic_, "current_twist");
    pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    pnh_.param<std::string>("map_frame_id", map_frame_id_, "map");
    pnh_.param<std::string>("robot_frame_id", robot_frame_id_, "base_link");
    data_ = boost::circular_buffer<geometry_msgs::TwistStamped>(2);
    current_odom_pose_.header.frame_id = odom_frame_id_;
    current_odom_pose_.header.stamp = ros::Time::now();
    current_odom_pose_.pose.position.x = 0;
    current_odom_pose_.pose.position.y = 0;
    current_odom_pose_.pose.position.z = 0;
    current_odom_pose_.pose.orientation.x = 0;
    current_odom_pose_.pose.orientation.y = 0;
    current_odom_pose_.pose.orientation.z = 0;
    current_odom_pose_.pose.orientation.w = 1;
    odom_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("odom_pose",1);
    current_twist_sub_ = nh_.subscribe(current_twist_topic_,1,&OdomFramePublisher::currentTwistCallback,this);
    current_pose_sub_ = nh_.subscribe(current_pose_topic_,1,&OdomFramePublisher::currentPoseCallback,this);
}

/**
 * @brief Destructor of the OdomFramePublisher class
 * 
 */
OdomFramePublisher::~OdomFramePublisher()
{

}

/**
 * @brief ROS callback function for the current pose topic
 * 
 */
void OdomFramePublisher::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    current_pose_recieved_ = true;
    current_pose_ = *msg;
    return;
}

/**
 * @brief ROS callback function for the current tiwst topic
 * 
 */
void OdomFramePublisher::currentTwistCallback(const geometry_msgs::TwistStamped::ConstPtr msg)
{
    data_.push_back(*msg);
    if(data_.size()==2 && current_pose_recieved_)
    {
        current_odom_pose_.header.stamp = current_pose_.header.stamp;
        double duration = (current_pose_.header.stamp - data_[0].header.stamp).toSec();
        geometry_msgs::Vector3 orientation;
        orientation.x = (data_[0].twist.angular.x+data_[1].twist.angular.x) * duration * 0.5;
        orientation.y = (data_[0].twist.angular.y+data_[1].twist.angular.y) * duration * 0.5;
        orientation.z = (data_[0].twist.angular.z+data_[1].twist.angular.z) * duration * 0.5;
        geometry_msgs::Quaternion twist_angular_quat = 
            quaternion_operation::convertEulerAngleToQuaternion(orientation);
        current_odom_pose_.pose.orientation = quaternion_operation::rotation(current_odom_pose_.pose.orientation,twist_angular_quat);
        Eigen::Vector3d trans_vec;
        trans_vec(0) = (data_[0].twist.linear.x+data_[1].twist.linear.x) * duration * 0.5;
        trans_vec(1) = (data_[0].twist.linear.y+data_[1].twist.linear.y) * duration * 0.5;
        trans_vec(2) = (data_[0].twist.linear.z+data_[1].twist.linear.z) * duration * 0.5;
        Eigen::Matrix3d rotation_mat = quaternion_operation::getRotationMatrix(current_odom_pose_.pose.orientation);
        trans_vec = rotation_mat*trans_vec;
        current_odom_pose_.pose.position.x = current_odom_pose_.pose.position.x + trans_vec(0);
        current_odom_pose_.pose.position.x = current_odom_pose_.pose.position.x + trans_vec(1);
        current_odom_pose_.pose.position.x = current_odom_pose_.pose.position.x + trans_vec(2);
        odom_pose_pub_.publish(current_odom_pose_);

        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.frame_id = odom_frame_id_;
        transform_stamped.header.stamp = current_pose_.header.stamp;
        transform_stamped.child_frame_id = robot_frame_id_;
        transform_stamped.transform.translation.x = current_odom_pose_.pose.position.x;
        transform_stamped.transform.translation.y = current_odom_pose_.pose.position.y;
        transform_stamped.transform.translation.z = current_odom_pose_.pose.position.z;
        transform_stamped.transform.rotation = current_odom_pose_.pose.orientation;
        broadcaster_.sendTransform(transform_stamped);

        tf2::Transform latest_tf;
        try
        {
            tf2::Transform odom_pose_tf2;
            tf2::convert(current_odom_pose_.pose, odom_pose_tf2);
            tf2::Quaternion q(current_pose_.pose.orientation.x,current_pose_.pose.orientation.y,current_pose_.pose.orientation.z,current_pose_.pose.orientation.w);
            tf2::Transform tmp_tf(q, tf2::Vector3(current_pose_.pose.position.x,current_pose_.pose.position.y,current_pose_.pose.position.z));
            geometry_msgs::PoseStamped tmp_tf_stamped;
            tmp_tf_stamped.header.frame_id = robot_frame_id_;
            tmp_tf_stamped.header.stamp = current_pose_.header.stamp;
            tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);
            geometry_msgs::PoseStamped odom_to_map;
            buffer_.transform(tmp_tf_stamped, odom_to_map, odom_frame_id_, ros::Duration(0.1));
            tf2::convert(odom_to_map.pose, latest_tf);
        }
        catch(...)
        {
            ROS_WARN("Failed to subtract base to odom transform");
            return;
        }
        geometry_msgs::TransformStamped tmp_tf_stamped;
        tmp_tf_stamped.header.frame_id = map_frame_id_;
        tmp_tf_stamped.header.stamp = current_pose_.header.stamp;
        tmp_tf_stamped.child_frame_id = odom_frame_id_;
        tf2::convert(latest_tf.inverse(), tmp_tf_stamped.transform);
        broadcaster_.sendTransform(tmp_tf_stamped);
    }
    return;
}