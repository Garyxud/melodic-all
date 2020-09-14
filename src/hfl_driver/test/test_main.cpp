/// Copyright 2019 Continental AG
///
/// @file test_main.cpp
///
/// @brief This file defines the entry point for hfl_driver tests
///

#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "HflDriverTest");
  ros::AsyncSpinner spinner(1);

  spinner.start();

  int ret = RUN_ALL_TESTS();

  spinner.stop();
  ros::shutdown();
  return ret;
}
