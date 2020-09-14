#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <dynamic_robot_state_publisher/joint_state_listener.h>

int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "robot_state_publisher");
  NodeHandle node;

  urdf::Model model;
  KDL::Tree tree;
  MimicMap mimic;
  robot_state_publisher::DynamicJointStateListener::loadModel(
      tree, mimic, model, "");

  robot_state_publisher::DynamicJointStateListener state_publisher(
      tree, mimic, model);

  ros::spin();

  return 0;
}
