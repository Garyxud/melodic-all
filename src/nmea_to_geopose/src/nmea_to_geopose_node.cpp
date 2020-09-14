// Headers in this package
#include <nmea_to_geopose/nmea_to_geopose.h>

// Headers in ROS
#include <ros/ros.h>

// Headers in Glog
#include <glog/logging.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "nmea_to_geopose_node");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    NmeaToGeoPose converter(nh,pnh);
    ros::spin();
    return 0;
}