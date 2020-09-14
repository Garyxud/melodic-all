#include <gtest/gtest.h>
#include <ros/ros.h>

#include <parameter_assertions/assertions.h>
#include "macro_lib.h"

class TestAssertions : public testing::Test
{
public:
  TestAssertions() : handle_{} {};

protected:
  void SetUp() override
  {
    ros::start();
    handle_.deleteParam(parameter1);
    handle_.deleteParam(parameter2);
    handle_.deleteParam(parameter3);
  }

  void TearDown() override
  {
    if (ros::ok())
    {
      ros::shutdown();
    }
  }

  const std::string parameter1 = "first_param";
  const std::string parameter2 = "second_param";
  const std::string parameter3 = "third_param";
  ros::NodeHandle handle_{ "~" };
};

// ===========================
// = Test Positive assertion =
// ===========================

TEST_F(TestAssertions, getParamAssertNumberPositivePass)
{
  {
    double param;
    double set_param = 40.02;
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::POSITIVE });
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    float param;
    float set_param = 82.1f;
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param, { assertions::NumberAssertionType::POSITIVE });
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    int param;
    int set_param = 150;
    handle_.setParam(parameter3, set_param);
    assertions::getParam(handle_, parameter3, param, { assertions::NumberAssertionType::POSITIVE });
    EXPECT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, getParamAssertNumberPositiveFailDouble)
{
  double param;
  double set_param = -91.312;
  handle_.setParam(parameter1, set_param);
  assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::POSITIVE });
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamAssertNumberPositiveFailFloat)
{
  float param;
  float set_param = -91.312;
  handle_.setParam(parameter1, set_param);
  assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::POSITIVE });
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamAssertNumberPositiveFailInt)
{
  int param;
  int set_param = -32131;
  handle_.setParam(parameter1, set_param);
  assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::POSITIVE });
  EXPECT_ROS_DEAD();
}

// ===============================
// = Test Non-negative assertion =
// ===============================

TEST_F(TestAssertions, getParamAssertNumberNonNegativePass)
{
  {
    double param;
    double set_param = 0.0;
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::NON_NEGATIVE });
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    float param;
    float set_param = 0.1f;
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param, { assertions::NumberAssertionType::NON_NEGATIVE });
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    int param;
    int set_param = 0;
    handle_.setParam(parameter3, set_param);
    assertions::getParam(handle_, parameter3, param, { assertions::NumberAssertionType::NON_NEGATIVE });
    EXPECT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, getParamAssertNumberNonNegativeFailDouble)
{
  double param;
  double set_param = -91.312;
  handle_.setParam(parameter1, set_param);
  assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::NON_NEGATIVE });
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamAssertNumberNonNegativeFailFloat)
{
  float param;
  float set_param = -91.312;
  handle_.setParam(parameter1, set_param);
  assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::NON_NEGATIVE });
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamAssertNumberNonNegativeFailInt)
{
  int param;
  int set_param = -32131;
  handle_.setParam(parameter1, set_param);
  assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::NON_NEGATIVE });
  EXPECT_ROS_DEAD();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_getParam_with_assertions");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
