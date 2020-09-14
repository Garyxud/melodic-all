#ifndef HEIFU_SAFETY_HPP
#define HEIFU_SAFETY_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <ros_utils/Console.h>

#include "heifu_msgs/EnableSafety.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

namespace HS
{
    class HeifuSafety
    {
    public:

        HeifuSafety ();
        virtual ~HeifuSafety (){};

        void run();

    private:
        // Node handles
        ros::NodeHandle n;

        // Subscribers
        ros::Subscriber subOdomUAV;
        ros::Subscriber subJoystickVelocity;

        // Publishers
        ros::Publisher  pubSafeVelocity;

        // ROS Service Servers
        ros::ServiceServer srvEnableSafety;

        // Variables
        bool   enableSafetyFence;
        double safetyDistance;
        geometry_msgs::Point startPosition;
        geometry_msgs::Point maxPosition;
        geometry_msgs::Point minPosition;
        geometry_msgs::Twist safetyVelocity;

        geometry_msgs::Point currentUAVPosition;

        // Functions
        void cbOdomUAV(const geometry_msgs::PoseStampedConstPtr& msg);
        void cbJoystickVelocity(const geometry_msgs::Twist& ROSmsg);

        // ROS Service Functions
        bool enableSafety(heifu_msgs::EnableSafety::Request &req, heifu_msgs::EnableSafety::Response &res);
    };
}

#endif
