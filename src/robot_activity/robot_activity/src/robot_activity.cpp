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
#include <robot_activity/robot_activity.h>

#include <string>
#include <vector>

#define PRINT_FUNC_CALL(state) \
  ROS_DEBUG_STREAM(#state << "() method called")

namespace robot_activity
{

RobotActivity::RobotActivity(int argc, char* argv[],
                           const std::string& name_space,
                           const std::string& name)
  : node_namespace_(name_space),
    node_name_(name),
    state_request_callback_queue_()
{
  if (ros::isInitialized())
  {
    node_name_ = ros::this_node::getName();
    return;
  }

  if (node_name_.empty())
  {
    ros::init(argc, argv, "robot_activity", ros::init_options::AnonymousName);
    node_name_ = ros::this_node::getName();
  }
  else
    ros::init(argc, argv, name);
}

RobotActivity::~RobotActivity()
{
  ROS_DEBUG_STREAM("RobotActivity dtor [" << getNamespace() << "]");
}

RobotActivity& RobotActivity::init(bool autostart)
{
  node_handle_ = ros::NodeHandlePtr(new ros::NodeHandle(node_namespace_));
  node_handle_private_ = ros::NodeHandlePtr(new ros::NodeHandle("~" + node_namespace_));

  ros::param::param<bool>("~wait_for_supervisor", wait_for_supervisor_, true);
  ROS_INFO_STREAM("wait_for_supervisor = "
                  << std::boolalpha << wait_for_supervisor_);

  process_state_pub_ = node_handle_private_->advertise<robot_activity_msgs::State>("/heartbeat", 0, true);
  process_error_pub_ = node_handle_private_->advertise<robot_activity_msgs::Error>("/error", 0, true);

  if (wait_for_supervisor_)
  {
    ros::Rate poll_rate(100);
    while (process_state_pub_.getNumSubscribers() == 0 && ros::ok())
      poll_rate.sleep();
  }

  notifyState();

  ros::param::param<bool>("~autostart_after_reconfigure", autostart_after_reconfigure_, false);
  ROS_INFO_STREAM("autostart_after_reconfigure = "
                  << std::boolalpha << autostart_after_reconfigure_);

  terminate_server_ = registerStateChangeRequest("terminate", {State::TERMINATED}); // NOLINT
  reconfigure_server_ = registerStateChangeRequest("reconfigure",
    {
      State::UNCONFIGURED,
      autostart_after_reconfigure_ ? State::RUNNING : State::STOPPED
    }); // NOLINT

  restart_server_ = registerStateChangeRequest("restart", {State::STOPPED, State::RUNNING}); // NOLINT
  start_server_   = registerStateChangeRequest("start", {State::RUNNING}); // NOLINT
  stop_server_    = registerStateChangeRequest("stop",  {State::STOPPED}); // NOLINT
  pause_server_   = registerStateChangeRequest("pause", {State::PAUSED}); // NOLINT

  float heartbeat_rate;
  ros::param::param<float>("~heartbeat_rate", heartbeat_rate, 1.0f);
  ROS_INFO("heartbeat_rate = %.3f [Hz]", heartbeat_rate);

  IsolatedAsyncTimer::LambdaCallback heartbeat_callback = [this]()
  {
    notifyState();
  };
  heartbeat_timer_ = std::make_shared<IsolatedAsyncTimer>(
                       *node_handle_private_,
                       heartbeat_callback,
                       heartbeat_rate,
                       false);

  ros::param::param<bool>("~autostart", autostart_, false);

  state_request_spinner_ = std::make_shared<ros::AsyncSpinner>(1, &state_request_callback_queue_);
  state_request_spinner_->start();

  autostart_ = autostart_ || autostart;
  ROS_INFO_STREAM("autostart = " << std::boolalpha << autostart_);

  if (autostart_)
    transitionToState(State::RUNNING);
  else
    transitionToState(State::STOPPED);

  return *this;
}

void RobotActivity::run(uint8_t threads)
{
  runAsync(threads);
  ros::waitForShutdown();
}

void RobotActivity::runAsync(uint8_t threads)
{
  global_callback_queue_spinner_ = std::make_shared<ros::AsyncSpinner>(threads);
  global_callback_queue_spinner_->start();
}

State RobotActivity::getState()
{
  return current_state_;
}

void RobotActivity::notifyError(uint8_t error_type,
                               const std::string& function,
                               const std::string& description)
{
  ROS_DEBUG_STREAM("Publishing error msg with code: "
                   << error_type << " function: " << function
                   << " description: " << description);
  robot_activity_msgs::Error error_msg;
  error_msg.header.stamp = ros::Time::now();
  error_msg.node_name = getNamespace();
  error_msg.error_type = error_type;
  error_msg.function = function;
  error_msg.description = description;
  process_error_pub_.publish(error_msg);
}

void RobotActivity::registerIsolatedTimer(
  const IsolatedAsyncTimer::LambdaCallback& callback,
  const float& frequency,
  bool stoppable)
{
  process_timers_.emplace_back(
    std::make_shared<IsolatedAsyncTimer>(
      *node_handle_private_,
      callback,
      frequency,
      stoppable,
      false,
      false));
}

std::string RobotActivity::getNamespace() const
{
  if (node_handle_private_)
    return node_handle_private_->getNamespace();
  else
    return std::string();
}


void RobotActivity::create()
{
  PRINT_FUNC_CALL("create");
  onCreate();
}

void RobotActivity::terminate()
{
  PRINT_FUNC_CALL("terminate");
  onTerminate();
  // ros::Rate(2).sleep();
  // ros::shutdown();
}

void RobotActivity::configure()
{
  PRINT_FUNC_CALL("configure");
  onConfigure();
}

void RobotActivity::unconfigure()
{
  PRINT_FUNC_CALL("unconfigure");
  onUnconfigure();
}

void RobotActivity::start()
{
  PRINT_FUNC_CALL("start");
  for (const auto & timer : process_timers_)
  {
    ROS_DEBUG("Starting timer");
    timer->start();
  }
  onStart();
}

void RobotActivity::stop()
{
  PRINT_FUNC_CALL("stop");
  for (const auto & timer : process_timers_)
  {
    ROS_DEBUG("Stopping timer");
    timer->stop();
  }
  onStop();
}

void RobotActivity::resume()
{
  PRINT_FUNC_CALL("resume");
  for (const auto & timer : process_timers_)
  {
    ROS_DEBUG("Resuming timer");
    timer->resume();
  }
  onResume();
}

void RobotActivity::pause()
{
  PRINT_FUNC_CALL("pause");
  for (const auto & timer : process_timers_)
  {
    ROS_DEBUG("Pausing timer");
    timer->pause();
  }
  onPause();
}

ros::ServiceServer RobotActivity::registerStateChangeRequest(
  const std::string& service_name,
  const std::vector<State>& states)
{
  ROS_DEBUG_STREAM(
    "Registering state transition request for state " << service_name);
  using std_srvs::Empty;
  EmptyServiceCallback callback = [ = ](Empty::Request & req, Empty::Response & res)
  {
    bool success = true;
    for (const auto & s : states)
    {
      success = success && transitionToState(s);
    }
    return success;
  };

  auto options = ros::AdvertiseServiceOptions::create<Empty>(
                   service_name,
                   callback,
                   ros::VoidConstPtr(),
                   &state_request_callback_queue_);

  return node_handle_private_->advertiseService(options);
}

void RobotActivity::notifyState() const
{
  ROS_DEBUG("Heartbeat sent!");
  robot_activity_msgs::State state_msg;
  state_msg.header.stamp = ros::Time::now();
  state_msg.node_name = getNamespace();
  state_msg.state = static_cast<uint8_t>(current_state_);
  process_state_pub_.publish(state_msg);
}

bool RobotActivity::transitionToState(const State& goal_state)
{
  if (current_state_ == goal_state)
  {
    ROS_WARN_STREAM("Node is already at state " << goal_state);
    return false;
  }

  while (current_state_ != goal_state)
  {
    auto from_state = static_cast<uint8_t>(current_state_);
    auto to_state = static_cast<uint8_t>(goal_state);
    State next_state = STATE_TRANSITIONS_PATHS[from_state][to_state];
    if (next_state == State::INVALID)
    {
      ROS_WARN_STREAM("There is no transition path from [" << current_state_
                      << "] to [" << goal_state << "]");
      return false;
    }
    changeState(next_state);
  }
  return true;
}

void RobotActivity::changeState(const State& new_state)
{
  uint8_t from_state = static_cast<uint8_t>(current_state_);
  uint8_t to_state = static_cast<uint8_t>(new_state);
  MemberLambdaCallback callback = STATE_TRANSITIONS[from_state][to_state];
  if (callback == nullptr)
  {
    ROS_FATAL_STREAM_ONCE(
      "Tried changing state from [" << current_state_
      << "] to [" << new_state << "]. Transition does NOT exist!");
    return;
  }
  ROS_DEBUG_STREAM(
    "Changing state from [" << current_state_ << "] to ["
    << new_state << "]");
  current_state_ = new_state;

  boost::bind(callback, this)();
  notifyState();
}

std::ostream& operator<<(std::ostream& os, State state)
{
  switch (state)
  {
  case State::INVALID      :
    os << "INVALID";
    break;
  case State::LAUNCHING    :
    os << "LAUNCHING";
    break;
  case State::UNCONFIGURED :
    os << "UNCONFIGURED";
    break;
  case State::STOPPED      :
    os << "STOPPED";
    break;
  case State::PAUSED       :
    os << "PAUSED";
    break;
  case State::RUNNING      :
    os << "RUNNING";
    break;
  case State::TERMINATED   :
    os << "TERMINATED";
    break;
  default                  :
    os.setstate(std::ios_base::failbit);
  }
  return os;
}

const RobotActivity::StateTransitions RobotActivity::STATE_TRANSITIONS =
{
  /* Valid transitions for State::INVALID */
  {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr},
  /* Valid transitions for  State::LAUNCHING */
  {nullptr, nullptr, &RobotActivity::create, nullptr, nullptr, nullptr, nullptr},
  /* Valid transitions for State::UNCONFIGURED */
  {nullptr, nullptr, nullptr, &RobotActivity::configure, nullptr, nullptr, &RobotActivity::terminate},
  /* Valid transitions for State::STOPPED */
  {nullptr, nullptr, &RobotActivity::unconfigure, nullptr, &RobotActivity::start, nullptr, nullptr},
  /* Valid transitions for State::PAUSED */
  {nullptr, nullptr, nullptr, &RobotActivity::stop, nullptr, &RobotActivity::resume, nullptr},
  /* Valid transitions for State::RUNNING */
  {nullptr, nullptr, nullptr, nullptr, &RobotActivity::pause, nullptr, nullptr},
  /* Valid transitions for State::TERMINATED */
  {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr}
};


const RobotActivity::StateTransitionPaths RobotActivity::STATE_TRANSITIONS_PATHS =
{
  {
    /* State::INVALID to other states */
    State::INVALID, State::INVALID, State::INVALID,
    State::INVALID, State::INVALID, State::INVALID,
    State::INVALID
  }, // NOLINT
  {
    /* State::LAUNCHING to other states */
    State::INVALID, State::LAUNCHING, State::UNCONFIGURED,
    State::UNCONFIGURED, State::UNCONFIGURED, State::UNCONFIGURED,
    State::UNCONFIGURED
  }, // NOLINT
  {
    /* State::UNCONFIGURED to other states */
    State::INVALID, State::INVALID, State::UNCONFIGURED,
    State::STOPPED, State::STOPPED, State::STOPPED,
    State::TERMINATED
  }, // NOLINT
  {
    /* State::STOPPED to other states */
    State::INVALID, State::INVALID, State::UNCONFIGURED,
    State::STOPPED, State::PAUSED, State::PAUSED,
    State::UNCONFIGURED
  }, // NOLINT
  {
    /* State::PAUSED to other states */
    State::INVALID, State::INVALID, State::STOPPED,
    State::STOPPED, State::PAUSED, State::RUNNING,
    State::STOPPED
  }, // NOLINT
  {
    /* State::RUNNING to other states */
    State::INVALID, State::INVALID, State::PAUSED,
    State::PAUSED, State::PAUSED, State::PAUSED,
    State::PAUSED
  }, // NOLINT
  {
    /* State::TERMINATED to other states */
    State::INVALID, State::INVALID, State::INVALID,
    State::INVALID, State::INVALID, State::INVALID,
    State::INVALID
  }
};

}  // namespace robot_activity
