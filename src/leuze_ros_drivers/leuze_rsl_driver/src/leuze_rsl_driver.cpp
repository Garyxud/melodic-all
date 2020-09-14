#include <ros/ros.h>

#include "leuze_rsl_driver/rsl400_interface.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "leuze_driver");
    ros::NodeHandle n;
    if (argc < 3)
    {
        std::cerr << "Not enough arguments!" << std::endl;
    }
    std::string address = argv[1];
    std::string port = argv[2];

    RSL400Interface rsl_interface(address, port, &n);
    rsl_interface.connect();

    ros::spin();
    return 0;
}
