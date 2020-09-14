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

#include <psen_scan/ros_parameter_handler.h>
#include <psen_scan/default_parameters.h>
#include <psen_scan/psen_scan_fatal_exception.h>
#include <arpa/inet.h>
#include <gtest/gtest.h>

using namespace psen_scan;

#define DELETE_ROS_PARAM(param_name)                                                                                   \
  if (ros::param::has(param_name))                                                                                     \
  {                                                                                                                    \
    ros::param::del(param_name);                                                                                       \
  }

#define DELETE_ALL_ROS_PARAMS()                                                                                        \
  DELETE_ROS_PARAM("password");                                                                                        \
  DELETE_ROS_PARAM("sensor_ip");                                                                                       \
  DELETE_ROS_PARAM("host_ip");                                                                                         \
  DELETE_ROS_PARAM("host_udp_port");                                                                                   \
  DELETE_ROS_PARAM("angle_start");                                                                                     \
  DELETE_ROS_PARAM("angle_end");                                                                                       \
  DELETE_ROS_PARAM("frame_id");                                                                                        \
  DELETE_ROS_PARAM("skip");                                                                                            \
  DELETE_ROS_PARAM("x_axis_rotation");

namespace psen_scan_test
{
class ROSParameterHandlerTest : public ::testing::Test
{
protected:
  ROSParameterHandlerTest()
    : password_("ac0d68d033")
    , host_ip_("1.2.3.5")
    , host_udp_port_(12345)
    , sensor_ip_("1.2.3.4")
    , expected_password_("admin")
    , expected_host_ip_(htobe32(inet_network(host_ip_.c_str())))
    , expected_host_udp_port_(htole32(host_udp_port_))
    , expected_frame_id_(DEFAULT_FRAME_ID)
    , expected_skip_(DEFAULT_SKIP)
    , expected_angle_start_(DEFAULT_ANGLE_START)
    , expected_angle_end_(DEFAULT_ANGLE_END)
    , expected_x_axis_rotation_(DEFAULT_X_AXIS_ROTATION)
  {
    DELETE_ALL_ROS_PARAMS();
  }

  ros::NodeHandle node_handle_;

  // Default values to set
  std::string password_;
  std::string host_ip_;
  int host_udp_port_;
  std::string sensor_ip_;

  // Default expected values
  std::string expected_password_;
  uint32_t expected_host_ip_;
  uint32_t expected_host_udp_port_;
  std::string expected_frame_id_;
  uint16_t expected_skip_;
  PSENscanInternalAngle expected_angle_start_;
  PSENscanInternalAngle expected_angle_end_;
  Degree expected_x_axis_rotation_;
};

class ROSRequiredParameterTest : public ROSParameterHandlerTest
{
protected:
  ROSRequiredParameterTest()
  {
    ros::param::set("password", password_);
    ros::param::set("sensor_ip", sensor_ip_);
    ros::param::set("host_ip", host_ip_);
    ros::param::set("host_udp_port", host_udp_port_);
  }
};

class ROSInvalidParameterTest : public ROSParameterHandlerTest
{
protected:
  ROSInvalidParameterTest()
  {
    ros::param::set("password", password_);
    ros::param::set("host_ip", host_ip_);
    ros::param::set("host_udp_port", host_udp_port_);
    ros::param::set("sensor_ip", sensor_ip_);
    ros::param::set("frame_id", DEFAULT_FRAME_ID);
    ros::param::set("skip", DEFAULT_SKIP);
    ros::param::set("angle_start", static_cast<double>(Degree(DEFAULT_ANGLE_START)));
    ros::param::set("angle_end", static_cast<double>(Degree(DEFAULT_ANGLE_END)));
    ros::param::set("x_axis_rotation", static_cast<double>(DEFAULT_X_AXIS_ROTATION));
  }
};

TEST_F(ROSParameterHandlerTest, test_no_param)
{
  EXPECT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);
}

TEST_F(ROSRequiredParameterTest, test_required_params_only)
{
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_);
                  EXPECT_EQ(param_handler.getPassword(), expected_password_);
                  EXPECT_EQ(param_handler.getSensorIP(), sensor_ip_);
                  EXPECT_EQ(param_handler.getHostIP(), expected_host_ip_);
                  EXPECT_EQ(param_handler.getHostUDPPort(), expected_host_udp_port_);
                  EXPECT_EQ(param_handler.getFrameID(), expected_frame_id_);
                  EXPECT_EQ(param_handler.getSkip(), expected_skip_);
                  EXPECT_EQ(param_handler.getAngleStart(), expected_angle_start_);
                  EXPECT_EQ(param_handler.getAngleEnd(), expected_angle_end_);
                  EXPECT_EQ(param_handler.getXAxisRotation(), expected_x_axis_rotation_););
}

TEST_F(ROSRequiredParameterTest, test_single_required_params_missing_password)
{
  DELETE_ROS_PARAM("password");

  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);
}

TEST_F(ROSRequiredParameterTest, test_single_required_params_missing_sensor_ip)
{
  DELETE_ROS_PARAM("sensor_ip");

  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);
}

TEST_F(ROSRequiredParameterTest, test_single_required_params_missing_host_ip)
{
  DELETE_ROS_PARAM("host_ip");

  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);
}

TEST_F(ROSRequiredParameterTest, test_single_required_params_missing_host_udp_port)
{
  DELETE_ROS_PARAM("host_udp_port");

  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);
}

TEST_F(ROSRequiredParameterTest, test_all_params)
{
  int skip = 2;
  std::string frame_id = "abcdefg";
  float angle_start = 10.5;
  float angle_end = 204.7;
  double x_axis_rotation = 100.3;
  ros::param::set("angle_start", angle_start);
  ros::param::set("angle_end", angle_end);
  ros::param::set("x_axis_rotation", x_axis_rotation);
  ros::param::set("frame_id", frame_id);
  ros::param::set("skip", skip);

  uint16_t expected_skip = 2;
  PSENscanInternalAngle expected_angle_start(105);
  PSENscanInternalAngle expected_angle_end(2047);
  Degree expected_x_axis_rotation(x_axis_rotation);

  RosParameterHandler param_handler(node_handle_);
  EXPECT_EQ(param_handler.getPassword(), expected_password_);
  EXPECT_EQ(param_handler.getSensorIP(), sensor_ip_);
  EXPECT_EQ(param_handler.getHostIP(), expected_host_ip_);
  EXPECT_EQ(param_handler.getHostUDPPort(), expected_host_udp_port_);
  EXPECT_EQ(param_handler.getFrameID(), frame_id);
  EXPECT_EQ(param_handler.getSkip(), expected_skip);
  EXPECT_EQ(param_handler.getAngleStart(), expected_angle_start);
  EXPECT_EQ(param_handler.getAngleEnd(), expected_angle_end);
  EXPECT_EQ(param_handler.getXAxisRotation(), expected_x_axis_rotation);
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_password)
{
  // Set password with wrong datatype (expected string) as example for wrong datatypes on expected strings
  ros::param::set("password", 15);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  ros::param::set("password", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set password back to valid data type but not in accepted format
  ros::param::set("password", "AABBCCDDEEFFGG");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set password back to valid data type
  ros::param::set("password", password_);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_host_ip)
{
  // Set host_ip with wrong datatype (expected string) as example for wrong datatypes on expected strings
  ros::param::set("host_ip", 25);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  ros::param::set("host_ip", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set host_ip back to valid data type but not IP-Format
  ros::param::set("host_ip", "AABBCCDDEEFFGG");
  // ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);
  ASSERT_ANY_THROW(RosParameterHandler param_handler(node_handle_);
                   EXPECT_EQ(static_cast<uint32_t>(0), param_handler.getHostIP()));

  // test valid
  ros::param::set("host_ip", "1.2.3.4");
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_host_udp_port)
{
  // Set host_udp_port with wrong datatype (expected int) as example for wrong datatypes on expected integers
  ros::param::set("host_udp_port", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Wrong Datatype, but could be converted to int
  ros::param::set("host_udp_port", "52425");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Wrong Datatype, but could be converted to int, but it's too large
  ros::param::set("host_udp_port", "152425");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Wrong Datatype, but can't be converted to int
  ros::param::set("host_udp_port", "AABBCCDDEEFFGG");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Integer too large
  ros::param::set("host_udp_port", 150000);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set negative parameter
  ros::param::set("host_udp_port", -1);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // test valid
  ros::param::set("host_udp_port", 1000);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_sensor_ip)
{
  // Set sensor_ip with wrong datatype (expected string) as example for wrong datatypes on expected strings
  ros::param::set("sensor_ip", 35);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  ros::param::set("sensor_ip", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set sensor_ip back to valid data type, but not valid IP-Address
  ros::param::set("sensor_ip", "AABBCCDDEEFFGG");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // test valid
  ros::param::set("sensor_ip", "1.2.3.4");
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_frame_id)
{
  // Set frame_id with wrong datatype (expected string) as example for wrong datatypes on expected strings
  ros::param::set("frame_id", 12);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  ros::param::set("frame_id", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set frame_id back to valid data type, which could be interpreted as int
  // Numbers are allowed for frame id TODO: discussion
  ros::param::set("frame_id", "125");
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_skip)
{
  // Set skip with wrong datatype (expected int) as example for wrong datatypes on expected integers
  ros::param::set("skip", "abcde");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  ros::param::set("skip", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Wrong Datatype, but could be converted to int
  ros::param::set("skip", "12");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set skip back to valid data type, but negative
  ros::param::set("skip", -1);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // valid test
  ros::param::set("skip", 0);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_angle_start)
{
  // Set angle_start with wrong datatype (expected double) as example for wrong datatypes on expected double
  ros::param::set("angle_start", "string");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  ros::param::set("angle_start", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Wrong Datatype, but can be interpreted as int
  ros::param::set("angle_start", "12");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set angle_start back to valid data type, but too big
  ros::param::set("angle_start", 276);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set angle_start back to valid data type, but negative
  ros::param::set("angle_start", -1);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // valid test
  ros::param::set("angle_start", 20.);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_angle_end)
{
  // Set angle_end with wrong datatype (expected double) as example for wrong datatypes on expected double
  ros::param::set("angle_end", "string");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  ros::param::set("angle_end", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Wrong Datatype, but can be converted to int
  ros::param::set("angle_end", "250");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set angle_end back to valid data type, but too large
  ros::param::set("angle_end", 276);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set angle_end back to valid data type, but negative
  ros::param::set("angle_end", -1);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // valid test
  ros::param::set("angle_end", 90.);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_x_axis_rotation)
{
  // Set x_axis_rotation with wrong datatype (expected double) as example for wrong datatypes on expected double
  ros::param::set("x_axis_rotation", "string");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  ros::param::set("x_axis_rotation", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Wrong Datatype, but can be converted to float
  ros::param::set("x_axis_rotation", "137.5");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set x_axis_rotation back to valid data type, but too large
  ros::param::set("x_axis_rotation", 361);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // Set x_axis_rotation back to valid data type, but too small
  ros::param::set("x_axis_rotation", -361);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), PSENScanFatalException);

  // valid test
  ros::param::set("x_axis_rotation", 90.);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}
}  // namespace psen_scan_test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "integrationtest_ros_parameter_handler");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}