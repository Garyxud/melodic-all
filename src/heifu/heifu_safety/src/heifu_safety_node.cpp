#include "heifu_safety/Heifu_safety.hpp"

using namespace HS;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heifu_safety_node");
    HS::HeifuSafety hs;
    hs.run();
    return 0;
}
