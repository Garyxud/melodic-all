#ifndef NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_H_INCLUDED
#define NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_H_INCLUDED

// Headers in Boost
#include <boost/optional.hpp>

// Headers in ROS
#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <geographic_msgs/GeoPose.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Quaternion.h>
#include <quaternion_operation/quaternion_operation.h>

class NmeaToGeoPose
{
public:
    NmeaToGeoPose(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~NmeaToGeoPose();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void nmeaSentenceCallback(const nmea_msgs::Sentence::ConstPtr msg);
    std::string calculateChecksum(std::string sentence);
    std::string getHexString(uint8_t value);
    ros::Publisher geopose_pub_;
    ros::Subscriber nmea_sub_;
    std::string input_topic_;
    boost::optional<geographic_msgs::GeoPoint> geopoint_;
    boost::optional<geometry_msgs::Quaternion> quat_;
    bool isGprmcSentence(nmea_msgs::Sentence sentence);
    bool isGphdtSentence(nmea_msgs::Sentence sentence);
    std::vector<std::string> split(const std::string &s,char delim);
    std::vector<std::string> splitChecksum(std::string str);
    boost::optional<std::vector<std::string> > splitSentence(nmea_msgs::Sentence sentence);
    boost::optional<ros::Time> last_timestamp_;
};

#endif  //NMEA_TO_GEOPOSE_NMEA_TO_GEOPOSE_H_INCLUDED