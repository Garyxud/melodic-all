#include <nmea_to_geopose/nmea_to_geopose.h>

NmeaToGeoPose::NmeaToGeoPose(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("input_topic", input_topic_, "/nmea_sentence");
    nmea_sub_ = nh_.subscribe(input_topic_,1,&NmeaToGeoPose::nmeaSentenceCallback,this);
    geopose_pub_ = pnh_.advertise<geographic_msgs::GeoPoseStamped>("geopose",1);
}

NmeaToGeoPose::~NmeaToGeoPose()
{

}

void NmeaToGeoPose::nmeaSentenceCallback(const nmea_msgs::Sentence::ConstPtr msg)
{
    nmea_msgs::Sentence sentence = *msg;
    if(isGprmcSentence(sentence))
    {
        geographic_msgs::GeoPoint geopoint;
        boost::optional<std::vector<std::string> > data = splitSentence(sentence);
        if(data)
        {
            std::string lat_str = data.get()[3];
            std::string north_or_south_str = data.get()[4];
            double latitude = std::stod(lat_str.substr(0,2)) + std::stod(lat_str.substr(2))/60.0;
            ROS_ASSERT(north_or_south_str == "N" || north_or_south_str == "S");
            if(north_or_south_str == "S")
            {
                latitude = latitude*-1;
            }
            std::string lon_str = data.get()[5];
            std::string east_or_west_str = data.get()[6];
            double longitude = std::stod(lon_str.substr(0,3)) + std::stod(lon_str.substr(3))/60.0;
            ROS_ASSERT(east_or_west_str == "E" || east_or_west_str == "W");
            if(east_or_west_str == "W")
            {
                longitude = longitude*-1;
            }
            geopoint.latitude = latitude;
            geopoint.longitude = longitude;
            geopoint.altitude = 0.0;
            geopoint_ = geopoint;
        }
    }
    if(isGphdtSentence(sentence))
    {
        boost::optional<std::vector<std::string> > data = splitSentence(sentence);
        if(data)
        {
            if(data.get()[2] == "T")
            {
                double heading = std::stod(data.get()[1]);
                geometry_msgs::Vector3 vec;
                vec.x = 0.0;
                vec.y = 0.0;
                vec.z = heading/180*M_PI;
                geometry_msgs::Quaternion quat = 
                    quaternion_operation::convertEulerAngleToQuaternion(vec);
                quat_ = quat;
            }
        }
    }
    if(geopoint_ && quat_)
    {
        if(!last_timestamp_ || last_timestamp_ != msg->header.stamp)
        {
            last_timestamp_ = msg->header.stamp;
            geographic_msgs::GeoPoseStamped geopose;
            geopose.pose.position = geopoint_.get();
            geopose.pose.orientation = quat_.get();
            geopose.header = msg->header;
            geopose_pub_.publish(geopose);
        }
    }
}

std::vector<std::string> NmeaToGeoPose::split(const std::string &s,char delim)
{
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (getline(ss, item, delim))
    {
        if (!item.empty())
        {
            elems.push_back(item);
        }
    }
    return elems;
}

std::vector<std::string> NmeaToGeoPose::splitChecksum(std::string str)
{
    return split(str,'*');
}

boost::optional<std::vector<std::string> > NmeaToGeoPose::splitSentence(nmea_msgs::Sentence sentence)
{
    std::vector<std::string> data = splitChecksum(sentence.sentence);
    if(data.size() != 2)
    {
        return boost::none;
    }
    if(calculateChecksum(data[0]) == data[1])
    {
        return split(data[0],',');
    }
    return boost::none;
}

bool NmeaToGeoPose::isGprmcSentence(nmea_msgs::Sentence sentence)
{
    std::string type = sentence.sentence.substr(0,6);
    if(type == "$GPRMC")
    {
        return true;
    }
    return false;
}

bool NmeaToGeoPose::isGphdtSentence(nmea_msgs::Sentence sentence)
{
    std::string type = sentence.sentence.substr(0,6);
    if(type == "$GPHDT")
    {
        return true;
    }
    return false;
}

std::string NmeaToGeoPose::calculateChecksum(std::string sentence)
{
    uint8_t checksum;
    for(int i=1; i<sentence.size(); i++)
    {
        int16_t c = sentence[i];
        checksum ^= c;
    }
    uint8_t rest = checksum%16;
    uint8_t quotient = (checksum-rest)/16;
    std::string ret = getHexString(quotient) + getHexString(rest);
    return ret;
}

std::string NmeaToGeoPose::getHexString(uint8_t value)
{
    ROS_ASSERT(value <= 16);
    std::string ret;
    if(value == 10)
    {
        ret = "A";
    }
    else if(value == 11)
    {
        ret = "B";
    }
    else if(value == 12)
    {
        ret = "C";
    }
    else if(value == 13)
    {
        ret = "D";
    }
    else if(value == 14)
    {
        ret = "E";
    }
    else if(value == 15)
    {
        ret = "F";
    }
    else
    {
        ret = std::to_string(value);
    }
    return ret;
}