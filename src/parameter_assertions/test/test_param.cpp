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

// ============================
// = param uses parameter set =
// ============================
TEST_F(TestAssertions, paramGetsParamString)
{
  {
    std::string param;
    std::string set_param = "Vim>Emacs";
    std::string default_param{};
    handle_.setParam(parameter1, set_param);
    assertions::param(handle_, parameter1, param, default_param);
    EXPECT_STREQ(param.c_str(), set_param.c_str());
    EXPECT_ROS_ALIVE();

    std::string result = assertions::param(handle_, parameter1, default_param);
    EXPECT_STREQ(param.c_str(), result.c_str());
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<std::string> param;
    std::vector<std::string> set_param{ "rrt", "prm", "sst", "est" };
    std::vector<std::string> default_param{};
    handle_.setParam(parameter2, set_param);
    assertions::param(handle_, parameter2, param, default_param);
    EXPECT_STR_VEC_EQ(param, set_param);
    EXPECT_ROS_ALIVE();

    std::vector<std::string> result = assertions::param(handle_, parameter2, default_param);
    EXPECT_STR_VEC_EQ(result, set_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, paramGetsParamDouble)
{
  {
    double param;
    double set_param = 0.1234;
    double default_param{};
    handle_.setParam(parameter1, set_param);
    assertions::param(handle_, parameter1, param, default_param);
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();

    double result = assertions::param(handle_, parameter1, default_param);
    EXPECT_FLOAT_EQ(result, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<double> param;
    std::vector<double> set_param{ 1.213, 0.9123, 0.123, -123.32 };
    std::vector<double> default_param{};
    handle_.setParam(parameter2, set_param);
    assertions::param(handle_, parameter2, param, default_param);
    EXPECT_FLOAT_VEC_EQ(param, set_param);
    EXPECT_ROS_ALIVE();

    std::vector<double> result = assertions::param(handle_, parameter2, default_param);
    EXPECT_FLOAT_VEC_EQ(result, set_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, paramGetsParamFloat)
{
  {
    double param;
    double set_param = 0.1234;
    double default_param{};
    handle_.setParam(parameter1, set_param);
    assertions::param(handle_, parameter1, param, default_param);
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();

    double result = assertions::param(handle_, parameter1, default_param);
    EXPECT_FLOAT_EQ(result, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<double> param;
    std::vector<double> set_param{ 1.213, 0.9123, 0.123, -123.32 };
    std::vector<double> default_param{};
    handle_.setParam(parameter2, set_param);
    assertions::param(handle_, parameter2, param, default_param);
    EXPECT_FLOAT_VEC_EQ(param, set_param);
    EXPECT_ROS_ALIVE();

    std::vector<double> result = assertions::param(handle_, parameter2, default_param);
    EXPECT_FLOAT_VEC_EQ(result, set_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, paramGetsParamInt)
{
  {
    int param;
    int set_param = 32121;
    int default_param{};
    handle_.setParam(parameter1, set_param);
    assertions::param(handle_, parameter1, param, default_param);
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();

    int result = assertions::param(handle_, parameter1, default_param);
    EXPECT_FLOAT_EQ(result, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<int> param;
    std::vector<int> set_param{ 412, 581, 321321, -54213, 3231 };
    std::vector<int> default_param{};
    handle_.setParam(parameter2, set_param);
    assertions::param(handle_, parameter2, param, default_param);
    EXPECT_FLOAT_VEC_EQ(param, set_param);
    EXPECT_ROS_ALIVE();

    std::vector<int> result = assertions::param(handle_, parameter2, default_param);
    EXPECT_FLOAT_VEC_EQ(result, set_param);
    EXPECT_ROS_ALIVE();
  }
}

// =================================================
// = param uses default value and doesn't shutdown =
// =================================================
TEST_F(TestAssertions, paramUsesDefaultValueString)
{
  {
    std::string param;
    std::string default_param{ "yay testing" };
    assertions::param(handle_, parameter1, param, default_param);
    EXPECT_STREQ(param.c_str(), default_param.c_str());
    EXPECT_ROS_ALIVE();

    std::string result = assertions::param(handle_, parameter1, default_param);
    EXPECT_STREQ(param.c_str(), default_param.c_str());
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<std::string> param;
    std::vector<std::string> default_param{ "fwafwf", "F", "412431" };
    assertions::param(handle_, parameter2, param, default_param);
    EXPECT_STR_VEC_EQ(param, default_param);
    EXPECT_ROS_ALIVE();

    std::vector<std::string> result = assertions::param(handle_, parameter2, default_param);
    EXPECT_STR_VEC_EQ(result, default_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, paramUsesDefaultValueDouble)
{
  {
    double param;
    double default_param{ 35.1312 };
    assertions::param(handle_, parameter1, param, default_param);
    EXPECT_FLOAT_EQ(param, default_param);
    EXPECT_ROS_ALIVE();

    double result = assertions::param(handle_, parameter1, default_param);
    EXPECT_FLOAT_EQ(result, default_param);
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<double> param;
    std::vector<double> default_param{ 9.9, 9.98, 9.932 };
    assertions::param(handle_, parameter2, param, default_param);
    EXPECT_FLOAT_VEC_EQ(param, default_param);
    EXPECT_ROS_ALIVE();

    std::vector<double> result = assertions::param(handle_, parameter2, default_param);
    EXPECT_FLOAT_VEC_EQ(result, default_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, paramUsesDefaultValueFloat)
{
  {
    float param;
    float default_param{ 35.1312 };
    assertions::param(handle_, parameter1, param, default_param);
    EXPECT_FLOAT_EQ(param, default_param);
    EXPECT_ROS_ALIVE();

    float result = assertions::param(handle_, parameter1, default_param);
    EXPECT_FLOAT_EQ(result, default_param);
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<float> param;
    std::vector<float> default_param{ 9.9, 9.98, 9.932 };
    assertions::param(handle_, parameter2, param, default_param);
    EXPECT_FLOAT_VEC_EQ(param, default_param);
    EXPECT_ROS_ALIVE();

    std::vector<float> result = assertions::param(handle_, parameter2, default_param);
    EXPECT_FLOAT_VEC_EQ(result, default_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, paramUsesDefaultValueInt)
{
  {
    int param;
    int default_param{ 98481231 };
    assertions::param(handle_, parameter1, param, default_param);
    EXPECT_EQ(param, default_param);
    EXPECT_ROS_ALIVE();

    int result = assertions::param(handle_, parameter2, default_param);
    EXPECT_EQ(result, default_param);
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<int> param;
    std::vector<int> default_param{ 0, 231, 151, 131, 12, -341 };
    assertions::param(handle_, parameter2, param, default_param);
    EXPECT_VEC_EQ(param, default_param);
    EXPECT_ROS_ALIVE();

    std::vector<int> result = assertions::param(handle_, parameter2, default_param);
    EXPECT_VEC_EQ(result, default_param);
    EXPECT_ROS_ALIVE();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_param");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
