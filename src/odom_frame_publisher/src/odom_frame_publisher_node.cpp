/**
 * @file odom_frame_publisher_node.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Main function of the odom_frame_publisher node
 * @version 0.1
 * @date 2019-09-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// headers in this package
#include <odom_frame_publisher/odom_frame_publisher.h>

// headers for ros
#include <ros/ros.h>

/**
 * @brief main function
 * 
 */
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_frame_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    OdomFramePublisher publisher(nh,pnh);
    ros::spin();
    return 0;
}