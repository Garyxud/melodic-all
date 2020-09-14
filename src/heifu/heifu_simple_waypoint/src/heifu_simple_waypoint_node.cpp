#include "heifu_simple_waypoint/Heifu_simple_waypoint.hpp"

using namespace HSW;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heifu_simple_waypoint_node");
    HSW::Heifu_simple_waypoint heifuSimpleWaypoint;
    heifuSimpleWaypoint.run();
    return 0;
}
