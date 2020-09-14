/// Copyright 2019 Continental AG
///
/// @file hfl110dcu-test.cpp
///
/// @brief This file defines the HFL110DCU ROS unit tests
///

#include <gtest/gtest.h>
#include "camera_commander/camera_commander.h"
#include "ros/ros.h"
#include <string>
#include <vector>

///
/// ################################
///           Declare Tests
/// ################################
///

TEST(HFL110DCUTestSuite, testTEST)
{
  ASSERT_EQ("true", "true");
}

TEST(HFL110DCUTestSuite, testDefaultParams)
{
  // TODO(evan_flynn): this assumes the launch file defaults are set to HFL110DCU and left
  // for model and frame_id
  ros::NodeHandle nh("hfl110dcu_01");
  std::string model, version, frame_id, parent_frame, interface;
  std::string camera_ip, computer_ip;
  int frame_port;

  nh.getParam("model", model);
  nh.getParam("version", version);
  nh.getParam("frame_id", frame_id);
  nh.getParam("parent_frame", parent_frame);
  nh.getParam("ethernet_interface", interface);
  nh.getParam("camera_ip_address", camera_ip);
  nh.getParam("computer_ip_address", computer_ip);
  nh.getParam("frame_data_port", frame_port);

  ASSERT_EQ(model, "hfl110dcu");
  ASSERT_EQ(version, "v1");
  ASSERT_EQ(frame_id, "hfl110dcu_01");
  ASSERT_EQ(interface, "eth0");
  ASSERT_EQ(camera_ip, "192.168.10.21");
  ASSERT_EQ(computer_ip, "192.168.10.5");
  ASSERT_EQ(frame_port, 1900);
}

