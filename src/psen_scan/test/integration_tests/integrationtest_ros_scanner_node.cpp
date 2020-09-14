// Copyright (c) 2019-2020 Pilz GmbH & Co. KG
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
#include "psen_scan/coherent_monitoring_frames_exception.h"
#include "psen_scan/diagnostic_information_exception.h"
#include "psen_scan/parse_monitoring_frame_exception.h"
#include "psen_scan/build_ros_message_exception.h"
#include "psen_scan/fetch_monitoring_frame_exception.h"
#include "psen_scan/psen_scan_fatal_exception.h"
#include "psen_scan/mock_scanner.h"
#include <boost/thread.hpp>

using namespace psen_scan;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::Throw;

namespace psen_scan_test
{
class ros_scanner_node_test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node1_Scanner_test = std::unique_ptr<MockScanner>(new MockScanner());
    laser_scan_fake = new LaserScan(PSENscanInternalAngle(1), PSENscanInternalAngle(1), PSENscanInternalAngle(2));
    laser_scan_fake->measures_.push_back(1);
    laser_scan_error_1 = new LaserScan(PSENscanInternalAngle(1), PSENscanInternalAngle(1), PSENscanInternalAngle(2));
    laser_scan_error_1->measures_.push_back(1);
    laser_scan_error_1->measures_.push_back(2);
    laser_scan_error_2 = new LaserScan(PSENscanInternalAngle(0), PSENscanInternalAngle(1), PSENscanInternalAngle(2));
    laser_scan_error_2->measures_.push_back(1);
    laser_scan_error_3 = new LaserScan(PSENscanInternalAngle(10), PSENscanInternalAngle(2), PSENscanInternalAngle(1));
    laser_scan_error_3->measures_.push_back(1);
  }

  void TearDown() override
  {
    delete laser_scan_fake;
    delete laser_scan_error_1;
    delete laser_scan_error_2;
    delete laser_scan_error_3;
  }

  ros::NodeHandle node1_nh_test;
  std::unique_ptr<MockScanner> node1_Scanner_test;
  LaserScan* laser_scan_fake;
  LaserScan* laser_scan_error_1;
  LaserScan* laser_scan_error_2;
  LaserScan* laser_scan_error_3;
};

ACTION(ROS_SHUTDOWN)
{
  sleep(1);
  ros::shutdown();
}

class TestSubscriber
{
public:
  TestSubscriber(const ros::NodeHandle& nh, const std::string& topic) : receivedMessage_(0), ready_(false), nh_(nh)
  {
    sub_ = nh_.subscribe(topic,
                         1000,
                         &TestSubscriber::callback,
                         this,
                         ros::TransportHints().reliable().tcp().tcpNoDelay().udp().unreliable());
  }

  void spin()
  {
    while (sub_.getNumPublishers() < 1)
    {
      ros::spinOnce();
    }
    ready_ = true;
    ros::spin();
  }

  void callback(const sensor_msgs::LaserScan::ConstPtr& newMessage)
  {
    receivedMessage_++;
    message_ = *newMessage;
  }

  int receivedMessage_;
  bool ready_;
  sensor_msgs::LaserScan message_;
  ros::Subscriber sub_;
  ros::NodeHandle nh_;
};

TEST_F(ros_scanner_node_test, processingLoop_skip_eq_zero)
{
  EXPECT_CALL(*(node1_Scanner_test), start()).Times(1);

  EXPECT_CALL(*(node1_Scanner_test), stop()).Times(1);

  EXPECT_CALL(*(node1_Scanner_test), getCompleteScan())
      .Times(1)
      .WillOnce(DoAll(ROS_SHUTDOWN(), Return(*laser_scan_fake)));

  EXPECT_CALL(*(node1_Scanner_test), getCompleteScan())
      .Times(504)
      .WillRepeatedly(Return(*laser_scan_fake))
      .RetiresOnSaturation();

  ROSScannerNode ros_scanner_node(node1_nh_test,
                                  "node1_topic",
                                  "node1_frame",
                                  0,  // every frame
                                  Degree(137.5),
                                  std::move(node1_Scanner_test));

  TestSubscriber test_sub(node1_nh_test, "node1_topic");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros_scanner_node.processingLoop();

  spinner.stop();

  EXPECT_EQ(504, test_sub.receivedMessage_);
}

TEST_F(ros_scanner_node_test, processingLoop_skip_eq_one)
{
  ros::start();
  EXPECT_CALL(*(node1_Scanner_test), start()).Times(1);

  EXPECT_CALL(*(node1_Scanner_test), stop()).Times(1);

  EXPECT_CALL(*(node1_Scanner_test), getCompleteScan())
      .Times(1)
      .WillOnce(DoAll(ROS_SHUTDOWN(), Return(*laser_scan_fake)));

  EXPECT_CALL(*(node1_Scanner_test), getCompleteScan())
      .Times(504)
      .WillRepeatedly(Return(*laser_scan_fake))
      .RetiresOnSaturation();

  ROSScannerNode ros_scanner_node(node1_nh_test,
                                  "node1_topic",
                                  "node1_frame",
                                  1,  // every other frame
                                  Degree(137.5),
                                  std::move(node1_Scanner_test));

  TestSubscriber test_sub(node1_nh_test, "node1_topic");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros_scanner_node.processingLoop();
  spinner.stop();

  EXPECT_EQ(252, test_sub.receivedMessage_);
}

TEST_F(ros_scanner_node_test, processingLoop_skip_eq_99)
{
  ros::start();
  EXPECT_CALL(*(node1_Scanner_test), start()).Times(1);

  EXPECT_CALL(*(node1_Scanner_test), stop()).Times(1);

  EXPECT_CALL(*(node1_Scanner_test), getCompleteScan())
      .Times(1)
      .WillOnce(DoAll(ROS_SHUTDOWN(), Return(*laser_scan_fake)));

  EXPECT_CALL(*(node1_Scanner_test), getCompleteScan())
      .Times(504)
      .WillRepeatedly(Return(*laser_scan_fake))
      .RetiresOnSaturation();

  ROSScannerNode ros_scanner_node(node1_nh_test,
                                  "node1_topic",
                                  "node1_frame",
                                  99,  // every hundredth frame
                                  Degree(137.5),
                                  std::move(node1_Scanner_test));

  TestSubscriber test_sub(node1_nh_test, "node1_topic");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros_scanner_node.processingLoop();
  spinner.stop();

  EXPECT_EQ(5, test_sub.receivedMessage_);
}

TEST_F(ros_scanner_node_test, processingLoop_exception_catching)
{
  ros::start();
  EXPECT_CALL(*(node1_Scanner_test), start()).Times(1);

  EXPECT_CALL(*(node1_Scanner_test), getCompleteScan())
      .Times(5)
      .WillOnce(Throw(CoherentMonitoringFramesException("")))
      .WillOnce(Throw(ParseMonitoringFrameException("")))
      .WillOnce(Throw(DiagnosticInformationException("")))
      .WillOnce(Throw(BuildROSMessageException("")))
      .WillOnce(Throw(FetchMonitoringFrameException("")));

  ROSScannerNode ros_scanner_node(
      node1_nh_test, "node1_topic", "node1_frame", 0, Degree(137.5), std::move(node1_Scanner_test));

  TestSubscriber test_sub(node1_nh_test, "node1_topic");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ASSERT_THROW(ros_scanner_node.processingLoop(), FetchMonitoringFrameException);
  spinner.stop();

  EXPECT_EQ(0, test_sub.receivedMessage_);
}

TEST_F(ros_scanner_node_test, buildROSMessage)
{
  ROSScannerNode ros_scanner_node(
      node1_nh_test, "node1_topic", "node1_frame", 0, Degree(137.5), std::move(node1_Scanner_test));

  ros_scanner_node.buildRosMessage(*laser_scan_fake);
  EXPECT_THROW(ros_scanner_node.buildRosMessage(*laser_scan_error_1), BuildROSMessageException);
  EXPECT_THROW(ros_scanner_node.buildRosMessage(*laser_scan_error_2), BuildROSMessageException);
  EXPECT_THROW(ros_scanner_node.buildRosMessage(*laser_scan_error_3), BuildROSMessageException);
}

TEST_F(ros_scanner_node_test, constructor)
{
  EXPECT_THROW(new ROSScannerNode(node1_nh_test,
                                  "node1_topic",
                                  "node1_frame",
                                  0,
                                  Degree(137.5),
                                  nullptr  // This throws exception
                                  ),
               PSENScanFatalException);
}
}  // namespace psen_scan_test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "integrationtest_ros_scanner_node");
  ros::NodeHandle nh;  // keep one nh to avoid
  testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
