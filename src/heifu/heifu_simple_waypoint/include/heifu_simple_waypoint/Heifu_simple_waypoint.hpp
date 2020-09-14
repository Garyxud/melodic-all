#ifndef HEIFU_SIMPLE_WAYPOINT_HPP
#define HEIFU_SIMPLE_WAYPOINT_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <iterator>
#include <numeric>
#include <ctime>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <ros_utils/Console.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geographic_msgs/GeoPose.h>


namespace HSW
{
    class Heifu_simple_waypoint
    {
    public:

        Heifu_simple_waypoint ();
        virtual ~Heifu_simple_waypoint (){};

        void run();

    private:
        ros::NodeHandle n;

        // Subscribers
        ros::Subscriber subOdomUAV;
        ros::Subscriber subPoseCommander;

        // Publishers
        ros::Publisher pubMissionStart;
        ros::Publisher pubMissionStop;
        ros::Publisher pubDesiredPosition;

        // Parameters
        double paramNodeRate;

        // TF relation
        tf::TransformListener tfListener;
        tf::StampedTransform tfTransform;
        geometry_msgs::TwistStamped msgVel;

        // Parameters
        std_msgs::Empty emptyMsg;        

        // Variables
        geometry_msgs::PoseStamped currentPosition;

        // Functions
        void cbOdomUAV(const geometry_msgs::PoseStampedConstPtr& msg);
        void cbPoseCommander(const geometry_msgs::Pose& msg);
    };

}

#endif
