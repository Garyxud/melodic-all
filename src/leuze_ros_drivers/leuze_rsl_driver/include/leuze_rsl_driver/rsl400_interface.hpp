#ifndef LEUZE_RSL400_INTERFACE_H
#define LEUZE_RSL400_INTERFACE_H

#include "leuze_rsl_driver/communication.hpp"
#include "leuze_rsl_driver/hardware_interface.hpp"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "leuze_msgs/ExtendedStatusProfileMsg.h"

class RSL400Interface : public HardwareInterface<UDPConnection>, DataParser
{
public:
    RSL400Interface(std::string address, std::string port, ros::NodeHandle* nh);
    ~RSL400Interface();

    void connect();
    void disconnect();
    int parseBuffer(std::basic_string<unsigned char> buffer);

protected:
    void resetDefault();
    bool checkScan();
    void publishScan();
    void verifyConfiguration(DatagramExtendedStatusProfile d_esp);
    DatagramExtendedStatusProfile parseExtendedStatusProfile(std::basic_string<unsigned char> buffer);
    DatagramMeasurementDataType parseScanData(std::basic_string<unsigned char> buffer, Frame* frame);
    uint16_t convertBytesToUint16(unsigned char low_byte, unsigned char high_byte);
    bool compareTwoFloats(float a, float b,float epsilon = 0.0001);

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_scan_;
    ros::Publisher pub_status_;
    std::string header_frame_;

    sensor_msgs::LaserScan laser_scan_;
    leuze_msgs::ExtendedStatusProfileMsg status_msg_;
    int last_scan_number_;
    int configuration_type_; //type 3 = Distance + Intensity / type6 = Distance
    int measure_counter_;
    int block_counter_;
    int scan_size_;
    std::vector<DatagramMeasurementDataType> scan_data_;
    //HTTP protocol object
};

#endif
