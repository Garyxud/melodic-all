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
   \file isolated_async_timer.h
   \brief IsolatedAsyncTimer class implements ROS Timer served by
    a single-threaded async spinner on a separate callback queue
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef ROBOT_ACTIVITY_ISOLATED_ASYNC_TIMER_H
#define ROBOT_ACTIVITY_ISOLATED_ASYNC_TIMER_H

#include <atomic>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

namespace robot_activity
{

/**
 * @brief Wrapper around ROS Timer
 * @details ROS Timer served by a single-threaded async spinner on
 *        a separate callback queue. Timer can also be paused, which will
 *        trigger the callback, but return immediately
 */
class IsolatedAsyncTimer
{
public:
  typedef std::function<void(void)> LambdaCallback;

  /**
   * @brief Default destructor is deleted
   * @details [long description]
   */
  IsolatedAsyncTimer() = delete;

  /**
   * @brief Constructor
   * @details Constructs IsolatedAsyncTimer given a callback in form of a
   *          boost::function, which takes and returns void. Similarly to
   *          ROS's createTimer, the timer can be autostarted and run only once.
   *          Additionally, the timer can be set to unstopabble, where pausing
   *          and stopping does not have any effect
   *
   * @param node_handle [description]
   * @param callback [description]
   * @param frequency [description]
   * @param stoppable [description]
   * @param autostart [description]
   * @param oneshot [description]
   */
  IsolatedAsyncTimer(const ros::NodeHandle& node_handle,
                     const IsolatedAsyncTimer::LambdaCallback& callback,
                     const float& frequency,
                     bool stoppable = true,
                     bool autostart = true,
                     bool oneshot = false)
    : IsolatedAsyncTimer(node_handle,
                         to_timer_callback(callback),
                         frequency,
                         stoppable,
                         autostart,
                         oneshot) { }

  /**
   * @brief Constructor
   * @details Constructs IsolatedAsyncTimer given a callback in form of a
   *          ros::TimerCallback, which takes a ros::TimerEvent and
   *          returns a void. Similarly to ROS's createTimer,
   *          the timer can be autostarted and run only once.
   *          Additionally, the timer can be set to unstopabble, where pausing
   *          and stopping does not have any effect
   *
   * @param node_handle [description]
   * @param callback [description]
   * @param frequency [description]
   * @param stoppable [description]
   * @param autostart [description]
   * @param oneshot [description]
   */
  IsolatedAsyncTimer(const ros::NodeHandle& node_handle,
                     const ros::TimerCallback& callback,
                     const float& frequency,
                     bool stoppable = true,
                     bool autostart = true,
                     bool oneshot = false)
    : node_handle_(node_handle),
      timer_ops_(),
      callback_queue_(),
      stoppable_(stoppable),
      paused_(true)
  {
    callback_ = wrapTimerCallback(callback);

    frequency_ = frequency;
    period_ = ros::Duration(1.0 / frequency);

    timer_ops_.period = period_;
    timer_ops_.callback = callback_;
    timer_ops_.callback_queue = &callback_queue_;
    timer_ops_.oneshot = oneshot;
    timer_ops_.autostart = autostart;

    timer_ = std::make_shared<ros::Timer>();
    *timer_ = node_handle_.createTimer(timer_ops_);

    spinner_ = std::make_shared<ros::AsyncSpinner>(1, &callback_queue_);
    spinner_->start();
  }

  /**
   * @brief Default destructor
   */
  ~IsolatedAsyncTimer()
  {
    ROS_DEBUG("IsolatedAsyncTimer destructor");
  }

  /**
   * @brief Starts the timer
   */
  void start()
  {
    timer_->start();
  }

  /**
   * @brief Stops the timer if set to stoppable
   */
  void stop()
  {
    if (stoppable_)
      timer_->stop();
  }

  /**
   * @brief Pauses the timer
   */
  void pause()
  {
    paused_ = true;
  }

  /**
   * @brief Resumes the timer
   */
  void resume()
  {
    paused_ = false;
  }

  /**
   * @brief Returns true if timer is valid
   */
  bool isValid()
  {
    return timer_->isValid();
  }

  /**
   * @brief Sets a new rate to the timer
   * @details Sets a new rate to the timer and resets if asked for.
   *          Resetting will stop and start the timer, thus copying the callback.
   *          Copying the callback causes its state to be reset if the callback
   *          was passed as a closure (a lambda)
   *
   * @param frequency New timer frequency in Hz
   * @param reset Whether to reset the timer or not
   */
  void setRate(const float& frequency, bool reset = true)
  {
    frequency_ = frequency;
    period_ = ros::Duration(1.0 / frequency);
    timer_->setPeriod(period_, reset);
  }

  /**
   * @brief Converts a Lambda callback to valid ros::TimerCallback
   *
   * @param callback Lambda callback i.e. an std::function<void(void)>
   * @return ros::TimerCallback that can be used to create a ros::Timer
   */
  static ros::TimerCallback to_timer_callback(
    const IsolatedAsyncTimer::LambdaCallback& callback)
  {
    return [callback](const ros::TimerEvent & event)
    {
      callback();
    };
  }

private:
  /**
   * @brief Wraps a ros::TimerCallback with additional functionality
   * @details The returned callback returns immediately if it's stoppable and
   *          paused. It also checks if the loop missed it's desired rate.
   *
   * @param callback ros::TimerCallback to be wrapped
   * @return Wrapped ros::TimerCallback
   */
  ros::TimerCallback wrapTimerCallback(const ros::TimerCallback& callback) const
  {
    return [ = ](const ros::TimerEvent & ev)
    {
      if (stoppable_ == false || paused_ == false)
      {
        auto last_loop_duration = ev.profile.last_duration.toSec();
        if (ev.last_real.toSec() != 0 && last_loop_duration > period_.toSec())
        {
          auto lag = last_loop_duration - period_.toSec();
          ROS_WARN_STREAM(
            "Missed it's desired rate of " << frequency_ <<
            " [Hz], the loop actually took " << last_loop_duration
            << " [s], which is " << lag << " [s] longer");
        }
        callback(ev);
      }
    };
  }

  /**
   * @brief Node handle used to create the actual ros::Timer
   */
  ros::NodeHandle node_handle_;

  /**
   * @brief Frequency in Hz of the timer
   */
  float frequency_;

  /**
   * @brief Duration between consecutive callback executions
   */
  ros::Duration period_;

  /**
   * @brief Timer options needed for
   */
  ros::TimerOptions timer_ops_;

  /**
   * @brief Actual ros::TimerCallback used in the timer
   */
  ros::TimerCallback callback_;

  /**
   * @brief Seperate callback queue for the timer
   */
  ros::CallbackQueue callback_queue_;


  /**
   * @brief Shared pointer to the actual ros::Timer object
   */
  std::shared_ptr<ros::Timer> timer_;

  /**
   * @brief Shared pointer to the actual ros::AsyncSpiner that serves the
   *        callback queue in the background
   */
  std::shared_ptr<ros::AsyncSpinner> spinner_;

  /**
   * @brief Atomic bool that determines whether timer is stoppable or not
   */
  std::atomic<bool> stoppable_;

  /**
   * @brief Atomic bool that determines whether timer is paused or not
   */
  std::atomic<bool> paused_;
};

}  // namespace robot_activity

#endif  // ROBOT_ACTIVITY_ISOLATED_ASYNC_TIMER_H
