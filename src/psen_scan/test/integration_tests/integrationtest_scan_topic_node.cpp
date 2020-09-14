// Copyright (c) 2020 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <gtest/gtest.h>
#include "psen_scan/ros_scanner_node.h"
#include "psen_scan/laserscan.h"
#include "psen_scan/mock_scanner.h"

using namespace psen_scan;
using namespace psen_scan_test;
using ::testing::Return;
using ::testing::DoAll;

ACTION(ROS_SHUTDOWN)
{
  sleep(1);
  ros::shutdown();
}

ACTION(PSEN_SCAN_PAUSE)
{
  usleep(30000);
}

/**
 * @brief This node is used in an integrationtest that checks that messages
 * are correctly published on the 'scan' topic.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "integrationtest_scan_topic_node");
  ros::NodeHandle nh("~");

  LaserScan laser_scan_fake(PSENscanInternalAngle(1), PSENscanInternalAngle(1), PSENscanInternalAngle(2));
  laser_scan_fake.measures_.push_back(1);
  LaserScan laser_scan_error(PSENscanInternalAngle(0), PSENscanInternalAngle(1), PSENscanInternalAngle(2));
  laser_scan_error.measures_.push_back(1);

  std::unique_ptr<MockScanner> mock_scanner = std::unique_ptr<MockScanner>(new MockScanner());

  EXPECT_CALL(*(mock_scanner), getCompleteScan()).Times(1).WillOnce(DoAll(ROS_SHUTDOWN(), Return(laser_scan_error)));

  EXPECT_CALL(*(mock_scanner), getCompleteScan())
      .Times(100)
      .WillRepeatedly(DoAll(PSEN_SCAN_PAUSE(), Return(laser_scan_fake)))
      .RetiresOnSaturation();

  ROSScannerNode ros_scanner_node(nh,
                                  "scan",
                                  "scanner",
                                  0,  // every frame
                                  Degree(137.5),
                                  std::move(mock_scanner));

  ros_scanner_node.processingLoop();

  return 0;
}
