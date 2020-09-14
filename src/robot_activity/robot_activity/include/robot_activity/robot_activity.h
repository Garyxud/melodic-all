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
/*!
   \file robot_activity.h
   \brief RobotActivity class implements ROS node lifecycle
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef ROBOT_ACTIVITY_ROBOT_ACTIVITY_H
#define ROBOT_ACTIVITY_ROBOT_ACTIVITY_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <std_srvs/Empty.h>
#include <robot_activity_msgs/State.h>
#include <robot_activity_msgs/Error.h>

#include <robot_activity/isolated_async_timer.h>

#include <string>
#include <vector>

namespace robot_activity
{

/**
 * @brief RobotActivity state enum
 * @details The enum corresponds to the robot_activity_msgs::State message
 *
 */
enum class State : std::uint8_t
{
  INVALID      = robot_activity_msgs::State::INVALID,
  LAUNCHING    = robot_activity_msgs::State::LAUNCHING,
  UNCONFIGURED = robot_activity_msgs::State::UNCONFIGURED,
  STOPPED      = robot_activity_msgs::State::STOPPED,
  PAUSED       = robot_activity_msgs::State::PAUSED,
  RUNNING      = robot_activity_msgs::State::RUNNING,
  TERMINATED   = robot_activity_msgs::State::TERMINATED,
  Count = 7
};

/**
 * @brief Overridden operator<< for easy State enum printing
 *
 * @param os left-hand std::ostream to be used for outputting
 * @param state State to be output to an std::ostream
 */
std::ostream& operator<<(std::ostream& os, State state);

/**
 * @brief Class for adding node lifecycle to ROS processes
 * @details
 */
class RobotActivity
{
public:
  /**
   * @brief Default constructor is deleted
   */
  RobotActivity() = delete;

  /**
   * @brief Constructor
   * @details UNIX process args are passed in due to ros::init being called
   *          inside. name_space allows for running multiple instances of the
   *          same class or multiple RobotActivity classes in the same UNIX
   *          process without name collisions. name argument is the name
   *          passed in to ros::init, which is usually remapped in roslaunch.
   *          If name is empty and name is not remapped, then anonymous name will
   *          be created.
   */
  RobotActivity(int argc, char* argv[],
               const std::string& name_space = std::string(),
               const std::string& name = std::string());

  /**
   * @brief Virtual destructor
   */
  virtual ~RobotActivity();

  /**
   * @brief Initializes the RobotActivity
   * @details Creates node handles, starts heartbeat thread, advertises
   *          state change request services,
   *          waits for the supervisor if requested
   *
   * @param autostart If true, transitions immediately to RUNNING state
   * @return Returns a reference to itself for method chaning
   */
  RobotActivity& init(bool autostart = false);

  /**
   * @brief Spins an amount of threads to serve the global callback queue.
   *        The call is blocking.
   *
   * @param threads Number of threads to use, 0 signifies the amount of
   *                CPU cores available to the OS
   */
  void run(uint8_t threads = 0);

  /**
   * @brief Spins an amount of threads to serve the global callback queue.
   *        The call is non-blocking.
   *
   * @param threads Number of threads to use, 0 signifies the amount of
   *                CPU cores available to the OS
   */
  void runAsync(uint8_t threads = 0);

  /**
   * @brief Returns the current state
   * @return State that the node is in
   */
  State getState();

  /**
   * @brief Returns the full private namespace
   * @details Returns the full private namespace,
   *          meaning name of the actual ROS node together with the name_space
   *          argument passed during construction.
   * @return Returned namespace
   */
  std::string getNamespace() const;

protected:
  ros::NodeHandlePtr node_handle_;
  ros::NodeHandlePtr node_handle_private_;

  /**
   * @brief Sends an error message to a global error topic "/error"
   * @details Ideally the
   *
   * @param error_type Type of error
   * @param function Function, where the error was caused
   * @param description Detailed description of the error
   */
  void notifyError(uint8_t error_type,
                   const std::string& function,
                   const std::string& description);

  /**
   * @brief Register an isolated async timer
   * @details Registers an isolated async timer, which runs on a separate
   *          callback queue and is served by a single separate thread. The timer
   *          is managed during the lifecycle and transitioning to PAUSED
   *          state will pause its execution, meaning that the callback
   *          still executes, but returns immediately. When RobotActivity
   *          transitions to STOPPED state, the timer is stopped completely
   *          and started again during transition from STOPPED to PAUSED
   *
   * @param callback Timer callback to be registered
   * @param frequency Frequency of the timer in Hz
   * @param stoppable If true, timer cannot be stopped when transitioning to
   *                  PAUSED or STOPPED state
   */
  void registerIsolatedTimer(const IsolatedAsyncTimer::LambdaCallback& callback,
                             const float& frequency,
                             bool stoppable = true);

private:
  /**
   * @brief Vector of shared pointers to isolated timers created by
   *        register isolated timer
   */
  std::vector<std::shared_ptr<robot_activity::IsolatedAsyncTimer>> process_timers_;

  /**
   * @brief Shared pointer to the isolated timer that sends heartbeat messages
   */
  std::shared_ptr<robot_activity::IsolatedAsyncTimer> heartbeat_timer_;

  /**
   * @brief Node's namespace, if empty then the private node handle
   *        resolves to ~ otheerwise private node handle
   *        resolves to ~node_namespace
   */
  std::string node_namespace_;

  /**
   * @brief Name of the actual ROS node
   */
  std::string node_name_;

  /**
   * @brief Whether to wait for the supervisor during the init function.
   *        Waiting means that there has to be at least one subscriber of the
   *        hearthbeat topic.
   *
   */
  bool wait_for_supervisor_ = true;

  /**
   * @brief Whether to automatically start (transition to RUNNING) at the end of
   *        init function if not the node will transition to STOPPED state
   */
  bool autostart_ = false;

  /**
   * @brief Whether to autostart after reconfigure service is called
   */
  bool autostart_after_reconfigure_ = false;

  /**
   * @brief Callback queue for state change request services
   */
  ros::CallbackQueue state_request_callback_queue_;

  /**
   * @brief Async spinner for serving state change requests
   */
  std::shared_ptr<ros::AsyncSpinner> state_request_spinner_;

  /**
   * @brief ServiceServer for serving terminate service
   */
  ros::ServiceServer terminate_server_;

  /**
   * @brief ServiceServer for serving reconfigure service
   */
  ros::ServiceServer reconfigure_server_;

  /**
   * @brief ServiceServer for serving restart service, where restart means
   *        transition to STOPPED and then RUNNING
   */
  ros::ServiceServer restart_server_;

  /**
   * @brief ServiceServer for serving start service
   */
  ros::ServiceServer start_server_;


  /**
   * @brief ServiceServer for serving stop service
   */
  ros::ServiceServer stop_server_;

  /**
   * @brief ServiceServer for serving pause service
   */
  ros::ServiceServer pause_server_;


  /**
   * @brief ROS Publisher of heartbeat messages and state changes messages
   */
  ros::Publisher process_state_pub_;

  /**
   * @brief ROS Publisher of error messages
   */
  ros::Publisher process_error_pub_;


  /**
   * @brief Shared pointer to async spinner that serves the global callback queue
   */
  std::shared_ptr<ros::AsyncSpinner> global_callback_queue_spinner_;


  /**
   * @brief Current state of the RobotActivity. Initially, LAUNCHING state.
   */
  State current_state_ = State::LAUNCHING;

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from LAUNCHING to UNCONFIGURED.
   */
  virtual void onCreate() = 0;

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from UNCONFIGURED to TERMINATED.
   */
  virtual void onTerminate() = 0;

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from UNCONFIGURED to STOPPED.
   */
  virtual void onConfigure() = 0;

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from STOPPED to UNCONFIGURED.
   */
  virtual void onUnconfigure() = 0;

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from STOPPED to PAUSED.
   */
  virtual void onStart() = 0;

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from PAUSED to STOPPED.
   */
  virtual void onStop() = 0;

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from RUNNING to PAUSED.
   */
  virtual void onPause() = 0;

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from PAUSED to RUNNING.
   */
  virtual void onResume() = 0;

  /**
   * @brief Called automatically, when transition from LAUNCHING to UNCONFIGURED.
   */
  void create();

  /**
   * @brief Called automatically, when transition from UNCONFIGURED to TERMINATED.
   */
  void terminate();

  /**
   * @brief Called automatically, when transition from UNCONFIGURED to STOPPED.
   */
  void configure();

  /**
   * @brief Called automatically, when transition from STOPPED to UNCONFIGURED.
   */
  void unconfigure();

  /**
   * @brief Called automatically, when transition from STOPPED to PAUSED.
   */
  void start();

  /**
   * @brief Called automatically, when transition from PAUSED to STOPPED.
   */
  void stop();

  /**
   * @brief Called automatically, when transition from PAUSED to RUNNING.
   */
  void resume();

  /**
   * @brief Called automatically, when transition from RUNNING to PAUSED.
   */
  void pause();

  /**
   * @brief Sends a heartbeat message with the current state
   */
  void notifyState() const;

  /**
   * @brief Changes state from current to new. Direct transition must exist.
   *        The appropriate function will be called during transition.
   *
   * @param new_state State to transition to.
   */
  void changeState(const State& new_state);

  /**
   * @brief Transitions to a new state. Path must exists between the current
   *        and the new state. All appropriate function will be called when
   *        transition to the goal state
   *
   * @param new_state State to transition to.
   * @return Returns true if transition succeeded
   */
  bool transitionToState(const State& new_state);

  /**
   * @brief Registers a ROS service server listening for state change requests.
   *
   * @param service_name Name of the service
   * @param states States to transition to in order
   *
   * @return Returns the created ros::ServiceServer
   */
  ros::ServiceServer registerStateChangeRequest(
    const std::string& service_name,
    const std::vector<State>& states);

  typedef void (RobotActivity::*MemberLambdaCallback)();

  typedef boost::function < bool(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res) > EmptyServiceCallback;
  typedef MemberLambdaCallback StateTransitions
  [static_cast<uint8_t>(State::Count)]
  [static_cast<uint8_t>(State::Count)];

  typedef State StateTransitionPaths
  [static_cast<uint8_t>(State::Count)]
  [static_cast<uint8_t>(State::Count)];

  /**
   * @brief 2D array of direct state transitions with values being the
   *        corresponding functions to be called during that transition.
   *        First index signifies the state we are transitioning from,
   *        while the second index signifies the state we are transitioning to.
   */
  const static StateTransitions STATE_TRANSITIONS;

  /**
   * @brief 2D array of paths between states.
   */
  const static StateTransitionPaths STATE_TRANSITIONS_PATHS;
};

}  // namespace robot_activity

#endif  // ROBOT_ACTIVITY_ROBOT_ACTIVITY_H
