#include <ros/ros.h>
#include <cv.h>

#include <jderobot/types/laserData.h>
#include <sensor_msgs/LaserScan.h>
#include <jderobot/types/pose3d.h>
#include <vector>
#include <nav_msgs/Odometry.h>

#include <jderobot/types/image.h>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include <jderobot/types/rgbd.h>

#include <jderobot/types/cmdvel.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <jderobot/types/bumperData.h>


namespace Comm {

	/**
	 * @brief translate ROS LaserScan messages to JdeRobot LaserData
	 *
	 *
	 * @param ROS laser Scan Message
	 * 
	 *
	 * @return LaserData translated from ROS Message 
	 */
	JdeRobotTypes::LaserData translate_laser_messages(const sensor_msgs::LaserScanConstPtr& scan);

	/**
	 * @brief translate ROS BumperEvent messages to JdeRobot BumperData
	 *
	 *
	 * @param ROS kobuki Bumper Event Message
	 * 
	 *
	 * @return BumperData translated from ROS Message 
	 */
	JdeRobotTypes::BumperData translate_bumper_messages(const kobuki_msgs::BumperEventConstPtr& bump);

	/**
	 * @brief translate ROS Image messages to JdeRobot Image
	 *
	 *
	 * @param ROS Image Message
	 * 
	 *
	 * @return Image translated from ROS Message 
	 */
	JdeRobotTypes::Image translate_image_messages(const sensor_msgs::ImageConstPtr& image_msg);


	/**
	 * @brief translate ROS images messages to JdeRobot Rgbd
	 *
	 *
	 * @param ROS Image Message
	 * @param ROS Image Message
	 * 
	 *
	 * @return Rgbd translated from ROS Messages 
	 */
	//JdeRobotTypes::Rgbd translate_rgbd(const sensor_msgs::ImageConstPtr& rgb,const sensor_msgs::ImageConstPtr& d);

	/**
	 * @brief Translates from 32FC1 Image format to RGB. Inf values are represented by NaN, when converting to RGB, NaN passed to 0 
	 *
	 *
	 * @param ROS Image Message
	 * 
	 *
	 * @return Image translated from ROS Message 
	 */
	void depthToRGB(const cv::Mat& float_img, cv::Mat& rgb_img);
	
	/**
	 * @brief translate Jderobot CMDVel to ROS Twist messages
	 *
	 *
	 * @param float_img, image in 32FC1
	 * @param rgb_img, container for image in RGB
	 * 
	 *
	 */
	geometry_msgs::Twist translate_twist_messages(JdeRobotTypes::CMDVel cmdvel );

	/**
	 * @brief translate ROS Odometry messages to JdeRobot Pose3D
	 *
	 *
	 * @param ROS Odometry Message
	 * 
	 *
	 * @return Pose3D translated from ROS Message 
	 */
	JdeRobotTypes::Pose3d translate_odometry_messages(const nav_msgs::OdometryConstPtr& odom_msg);

	/**
	 * @brief Extracts Yaw angle from Quaternion
	 *
	 * @param Quaternion
	 * 
	 * @return Yaw Angle 
	 */
	float quat2Yaw(std::vector <float> q);

	/**
	 * @brief Extracts Pitch angle from Quaternion
	 *
	 * @param Quaternion
	 * 
	 * @return Pitch Angle 
	 */
	float quat2Pitch(std::vector <float> q);

	/**
	 * @brief Extracts Roll angle from Quaternion
	 *
	 * @param Quaternion
	 * 
	 * @return Roll Angle 
	 */
	float quat2Roll(std::vector <float> q);

} 
