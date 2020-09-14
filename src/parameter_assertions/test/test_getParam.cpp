#include <gtest/gtest.h>
#include <log4cxx/logger.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

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
  ros::NodeHandle handle_{ "~" };
};

//================================
//=  getParam gets the parameter =
//================================
TEST_F(TestAssertions, getParamGetsParamString)
{
  {
    std::string param;
    std::string set_param = "Vim>Emacs";
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param);
    EXPECT_STREQ(param.c_str(), set_param.c_str());
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<std::string> param;
    std::vector<std::string> set_param{ "rrt", "prm", "sst", "est" };
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param);
    EXPECT_STR_VEC_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, getParamGetsParamDouble)
{
  {
    double param;
    double set_param = 40.02;
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param);
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<double> param;
    std::vector<double> set_param{ 1.0, 1.1, 0.9, 0.8 };
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param);
    EXPECT_FLOAT_VEC_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, getParamGetsParamFloat)
{
  {
    float param;
    float set_param = 40.02f;
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param);
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<float> param;
    std::vector<float> set_param{ 1.0f, 1.1f, 0.9f, 0.8f };
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param);
    EXPECT_FLOAT_VEC_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, getParamGetsParamInt)
{
  {
    int param;
    int set_param = 1;
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param);
    ASSERT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<int> param;
    std::vector<int> set_param{ 3, 1, 4, 1, 5 };
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param);
    EXPECT_VEC_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, getParamGetsParamBool)
{
  {
    bool param;
    bool set_param = true;
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param);
    EXPECT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    std::vector<bool> param;
    std::vector<bool> set_param{ true, false, true, true, false, true };
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param);
    EXPECT_VEC_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
}

//================================================
//=  getParam kills the node if param is not set =
//================================================
TEST_F(TestAssertions, getParamEnsuresParamIsSetString)
{
  std::string param;
  assertions::getParam(handle_, parameter1, param);
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamEnsuresParamIsSetDouble)
{
  double param;
  assertions::getParam(handle_, parameter1, param);
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamEnsuresParamIsSetFloat)
{
  float param;
  assertions::getParam(handle_, parameter1, param);
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamEnsuresParamIsSetInt)
{
  int param;
  assertions::getParam(handle_, parameter1, param);
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamEnsuresParamIsSetBool)
{
  bool param;
  assertions::getParam(handle_, parameter1, param);
  EXPECT_ROS_DEAD();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_getParam");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
