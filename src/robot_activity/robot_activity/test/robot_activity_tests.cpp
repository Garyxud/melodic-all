/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, University of Luxembourg
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Luxembourg nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Maciej Zurad
 *********************************************************************/
#include <gtest/gtest.h>

#include <ros/ros.h>
#include <robot_activity/robot_activity.h>
#include <robot_activity_msgs/State.h>

#include <std_srvs/Empty.h>

#include <vector>
#include <string>

using robot_activity::RobotActivity;
using robot_activity::State;
using robot_activity::IsolatedAsyncTimer;

class AnyRobotActivity : public RobotActivity
{
public:
  using RobotActivity::RobotActivity;
  ~AnyRobotActivity() {}
private:
  void onCreate() override {};
  void onTerminate() override {};

  void onConfigure() override {};
  void onUnconfigure() override {};

  void onStart() override {};
  void onStop() override {};

  void onResume() override {};
  void onPause() override {};
};

class AnyRobotActivityWithTimer : public AnyRobotActivity
{
public:
  using AnyRobotActivity::AnyRobotActivity;
  int context = 0;
  bool stoppable = true;
private:
  void onCreate() override
  {
    IsolatedAsyncTimer::LambdaCallback cb = [this]() { context++; };
    registerIsolatedTimer(cb, 1, stoppable);
  }
};

TEST(RobotActivityTests, RemappedNameInitializedStateAndNamespace)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  AnyRobotActivity test(argc, const_cast<char**>(argv));
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  EXPECT_EQ(test.getNamespace(), std::string(""));

  boost::function<void(const robot_activity_msgs::State::ConstPtr&)> empty_cb =
    [](const robot_activity_msgs::State::ConstPtr &s){};

  ros::NodeHandle nh;
  auto sub = nh.subscribe("/heartbeat", 1, empty_cb);

  test.init();
  EXPECT_EQ(test.getState(), State::STOPPED);
  EXPECT_EQ(test.getNamespace(), std::string("/remapped_name"));
}

TEST(RobotActivityTests, InitializedNonWaitingStateAndNamespace)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);

  AnyRobotActivity test(argc, const_cast<char**>(argv));
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  EXPECT_EQ(test.getNamespace(), std::string(""));
  test.init();
  EXPECT_EQ(test.getState(), State::STOPPED);
  EXPECT_EQ(test.getNamespace(), std::string("/remapped_name"));
}

TEST(RobotActivityTests, AutostartNonWaitingStateAndNamespace)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  AnyRobotActivity test(argc, const_cast<char**>(argv));
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  EXPECT_EQ(test.getNamespace(), std::string(""));
  test.init();
  EXPECT_EQ(test.getState(), State::RUNNING);
  EXPECT_EQ(test.getNamespace(), std::string("/remapped_name"));
}

TEST(RobotActivityTests, StartStopServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", false);

  AnyRobotActivity test(argc, const_cast<char**>(argv));
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::STOPPED);

  auto start = nh.serviceClient<std_srvs::Empty>("/remapped_name/start");
  std_srvs::Empty start_empty;
  EXPECT_EQ(start.call(start_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);

  auto stop = nh.serviceClient<std_srvs::Empty>("/remapped_name/stop");
  std_srvs::Empty stop_empty;
  EXPECT_EQ(stop.call(stop_empty), true);
  EXPECT_EQ(test.getState(), State::STOPPED);
}

TEST(RobotActivityTests, PauseResumeServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  AnyRobotActivity test(argc, const_cast<char**>(argv));
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::RUNNING);

  auto pause = nh.serviceClient<std_srvs::Empty>("/remapped_name/pause");
  std_srvs::Empty pause_empty;
  EXPECT_EQ(pause.call(pause_empty), true);
  EXPECT_EQ(test.getState(), State::PAUSED);

  auto start = nh.serviceClient<std_srvs::Empty>("/remapped_name/start");
  std_srvs::Empty start_empty;
  EXPECT_EQ(start.call(start_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);
}

TEST(RobotActivityTests, RestartServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  AnyRobotActivity test(argc, const_cast<char**>(argv));
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::RUNNING);

  int counter = 0;
  std::vector<State> expected_transitions
  {
    State::RUNNING,
    State::PAUSED,
    State::STOPPED,
    State::PAUSED,
    State::RUNNING
  };

  boost::function<void(const robot_activity_msgs::State::ConstPtr&)> check_transitions_cb =
  [&](const robot_activity_msgs::State::ConstPtr &s)
  {
    EXPECT_EQ(s->state, static_cast<uint8_t>(expected_transitions[counter++]));
  };

  auto sub = nh.subscribe("/heartbeat", 0, check_transitions_cb);
  auto restart = nh.serviceClient<std_srvs::Empty>("/remapped_name/restart");
  std_srvs::Empty restart_empty;
  EXPECT_EQ(restart.call(restart_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);
}

TEST(RobotActivityTests, ReconfigureServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/autostart_after_reconfigure", true);
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  AnyRobotActivity test(argc, const_cast<char**>(argv));
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::RUNNING);

  int counter = 0;
  std::vector<State> expected_transitions =
  {
    State::RUNNING,
    State::PAUSED,
    State::STOPPED,
    State::UNCONFIGURED,
    State::STOPPED,
    State::PAUSED,
    State::RUNNING
  };

  boost::function<void(const robot_activity_msgs::State::ConstPtr&)> check_transitions_cb =
  [&](const robot_activity_msgs::State::ConstPtr &s)
  {
    EXPECT_EQ(s->state, static_cast<uint8_t>(expected_transitions[counter++]));
  };

  auto sub = nh.subscribe("/heartbeat", 0, check_transitions_cb);
  auto reconfigure = nh.serviceClient<std_srvs::Empty>("/remapped_name/reconfigure");
  std_srvs::Empty reconfigure_empty;
  EXPECT_EQ(reconfigure.call(reconfigure_empty), true);
  EXPECT_EQ(test.getState(), State::RUNNING);
}

TEST(RobotActivityTests, NoAutostartAfterReconfigureServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/autostart_after_reconfigure", false);
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  AnyRobotActivity test(argc, const_cast<char**>(argv));
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::RUNNING);

  auto reconfigure = nh.serviceClient<std_srvs::Empty>("/remapped_name/reconfigure");
  std_srvs::Empty reconfigure_empty;
  EXPECT_EQ(reconfigure.call(reconfigure_empty), true);
  EXPECT_EQ(test.getState(), State::STOPPED);
}

TEST(RobotActivityTests, TerminateServiceState)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", false);

  AnyRobotActivity test(argc, const_cast<char**>(argv));
  EXPECT_EQ(test.getState(), State::LAUNCHING);
  test.init();
  test.runAsync();
  EXPECT_EQ(test.getState(), State::STOPPED);

  auto terminate = nh.serviceClient<std_srvs::Empty>("/remapped_name/terminate");
  std_srvs::Empty terminate_empty;
  EXPECT_EQ(terminate.call(terminate_empty), true);
  EXPECT_EQ(test.getState(), State::TERMINATED);
}

TEST(RobotActivityTests, IsolatedAsyncTimer)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  AnyRobotActivityWithTimer test(argc, const_cast<char**>(argv));
  test.init().runAsync();
  ros::Duration(2.1).sleep();
  EXPECT_EQ(test.context, 2);
}


TEST(RobotActivityTests, StoppableIsolatedAsyncTimer)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  auto start = nh.serviceClient<std_srvs::Empty>("/remapped_name/start");
  auto pause = nh.serviceClient<std_srvs::Empty>("/remapped_name/pause");
  std_srvs::Empty pause_empty, start_empty;

  AnyRobotActivityWithTimer test(argc, const_cast<char**>(argv));
  test.init().runAsync();

  ros::Duration(1.1).sleep();
  EXPECT_EQ(test.context, 1);
  EXPECT_EQ(pause.call(pause_empty), true);
  ros::Duration(1.0).sleep();
  EXPECT_EQ(test.context, 1);
  EXPECT_EQ(start.call(start_empty), true);
  ros::Duration(1.1).sleep();
  EXPECT_EQ(test.context, 2);
}

TEST(RobotActivityTests, NonStoppableIsolatedAsyncTimer)
{
  int argc = 2;
  const char* argv[2];
  argv[0] = "random_process_name";
  argv[1] = "__name:=remapped_name";

  ros::NodeHandle nh;
  nh.setParam("/remapped_name/wait_for_supervisor", false);
  nh.setParam("/remapped_name/autostart", true);

  auto start = nh.serviceClient<std_srvs::Empty>("/remapped_name/start");
  auto stop = nh.serviceClient<std_srvs::Empty>("/remapped_name/stop");
  std_srvs::Empty stop_empty, start_empty;

  AnyRobotActivityWithTimer test(argc, const_cast<char**>(argv));
  test.stoppable = false;
  test.init().runAsync();

  ros::Duration(1.1).sleep();
  EXPECT_EQ(test.context, 1);
  EXPECT_EQ(stop.call(stop_empty), true);
  ros::Duration(1.1).sleep();
  EXPECT_EQ(test.context, 2);
  EXPECT_EQ(start.call(start_empty), true);
  ros::Duration(1.1).sleep();
  EXPECT_EQ(test.context, 3);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
