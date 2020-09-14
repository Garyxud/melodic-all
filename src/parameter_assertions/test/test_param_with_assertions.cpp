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

TEST_F(TestAssertions, paramAssertNumberPositivePass)
{
  {
    double param;
    double set_param = 40.02;
    double default_param = 0.0;
    handle_.setParam(parameter1, set_param);
    ASSERT_TRUE(
        assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::POSITIVE }));
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();

    double result =
        assertions::param(handle_, parameter1, default_param, { assertions::NumberAssertionType::POSITIVE });
    EXPECT_FLOAT_EQ(result, set_param);

    EXPECT_ROS_ALIVE();
  }
  {
    float param;
    float set_param = 82.1f;
    float default_param = 0.0f;
    handle_.setParam(parameter2, set_param);
    ASSERT_TRUE(
        assertions::param(handle_, parameter2, param, default_param, { assertions::NumberAssertionType::POSITIVE }));
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();

    float result = assertions::param(handle_, parameter2, default_param, { assertions::NumberAssertionType::POSITIVE });
    EXPECT_FLOAT_EQ(result, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    int param;
    int set_param = 150;
    int default_param = 0;
    handle_.setParam(parameter3, set_param);
    ASSERT_TRUE(
        assertions::param(handle_, parameter3, param, default_param, { assertions::NumberAssertionType::POSITIVE }));
    EXPECT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();

    int result = assertions::param(handle_, parameter3, default_param, { assertions::NumberAssertionType::POSITIVE });
    EXPECT_EQ(result, set_param);
    EXPECT_ROS_ALIVE();
  }
}

// =========================================================================
// = Test Positive assertion, parameter fails assertion so default is used =
// =========================================================================

TEST_F(TestAssertions, paramAssertNumberPositiveFailDoubleFirst)
{
  double param;
  double set_param = -91.312;
  double default_param = 0.1;
  handle_.setParam(parameter1, set_param);
  assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::POSITIVE });
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, paramAssertNumberPositiveFailFloatFirst)
{
  float param;
  float set_param = -91.312f;
  float default_param = 0.123f;
  handle_.setParam(parameter1, set_param);
  assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::POSITIVE });
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, paramAssertNumberPositiveFailIntFirst)
{
  int param;
  int set_param = -32131;
  int default_param = 45;
  handle_.setParam(parameter1, set_param);
  assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::POSITIVE });
  EXPECT_ROS_ALIVE();
}

// ==============================================================================
// = Test Positive assertion, both parameter and default fail assertion so exit =
// ==============================================================================

TEST_F(TestAssertions, paramAssertNumberPositiveFailDoubleBoth)
{
  double param;
  double set_param = -91.312;
  double default_param = -15243.0;
  handle_.setParam(parameter1, set_param);
  assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::POSITIVE });
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, paramAssertNumberPositiveFailFloatBoth)
{
  float param;
  float set_param = -91.312f;
  float default_param = -12.912f;
  handle_.setParam(parameter1, set_param);
  assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::POSITIVE });
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, paramAssertNumberPositiveFailIntBoth)
{
  int param;
  int set_param = -32131;
  int default_param = -1235;
  handle_.setParam(parameter1, set_param);
  assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::POSITIVE });
  EXPECT_ROS_DEAD();
}

// ===============================
// = Test Non-negative assertion =
// ===============================

TEST_F(TestAssertions, paramAssertNumberNonNegativePass)
{
  {
    double param;
    double set_param = 0.0;
    double default_param = 512.0;
    handle_.setParam(parameter1, set_param);
    assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::NON_NEGATIVE });
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    float param;
    float set_param = 0.1f;
    float default_param = -312.541f;
    handle_.setParam(parameter2, set_param);
    assertions::param(handle_, parameter2, param, default_param, { assertions::NumberAssertionType::NON_NEGATIVE });
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    int param;
    int set_param = 0;
    int default_param = 51;
    handle_.setParam(parameter3, set_param);
    assertions::param(handle_, parameter3, param, default_param, { assertions::NumberAssertionType::NON_NEGATIVE });
    EXPECT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
}

// =========================================================================
// = Test non-negative assertion, parameter fails assertion so default is used =
// =========================================================================

TEST_F(TestAssertions, paramAssertNumberNonNegativeFailDoubleFirst)
{
  double param;
  double set_param = -91.312;
  double default_param = 0.1;
  handle_.setParam(parameter1, set_param);
  assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::NON_NEGATIVE });
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, paramAssertNumberNonNegativeFailFloatFirst)
{
  float param;
  float set_param = -91.312f;
  float default_param = 0.123f;
  handle_.setParam(parameter1, set_param);
  assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::NON_NEGATIVE });
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, paramAssertNumberNonNegativeFailIntFirst)
{
  int param;
  int set_param = -32131;
  int default_param = 45;
  handle_.setParam(parameter1, set_param);
  assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::NON_NEGATIVE });
  EXPECT_ROS_ALIVE();
}

// ==============================================================================
// = Test NonNegative assertion, both parameter and default fail assertion so exit =
// ==============================================================================

TEST_F(TestAssertions, paramAssertNumberNonNegativeFailDoubleBoth)
{
  double param;
  double set_param = -91.312;
  double default_param = -15243.0;
  handle_.setParam(parameter1, set_param);
  assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::NON_NEGATIVE });
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, paramAssertNumberNonNegativeFailFloatBoth)
{
  float param;
  float set_param = -91.312f;
  float default_param = -12.912f;
  handle_.setParam(parameter1, set_param);
  assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::NON_NEGATIVE });
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, paramAssertNumberNonNegativeFailIntBoth)
{
  int param;
  int set_param = -32131;
  int default_param = -1235;
  handle_.setParam(parameter1, set_param);
  assertions::param(handle_, parameter1, param, default_param, { assertions::NumberAssertionType::NON_NEGATIVE });
  EXPECT_ROS_DEAD();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_param_with_assertions");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
