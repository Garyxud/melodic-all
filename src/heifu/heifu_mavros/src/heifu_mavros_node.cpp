#include "heifu_mavros/Heifu_mavros.hpp"

using namespace HM;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heifu_mavros_node");
    HM::Heifu_mavros heifuMavros;
    heifuMavros.run();
    return 0;
}
