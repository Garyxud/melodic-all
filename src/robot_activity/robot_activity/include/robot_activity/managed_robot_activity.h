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
   \file managed_robot_activity.h
   \brief ManagedRobotActivity class implements ROS node lifecycle with
    managed subscriptions and services
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef ROBOT_ACTIVITY_MANAGED_ROBOT_ACTIVITY_H
#define ROBOT_ACTIVITY_MANAGED_ROBOT_ACTIVITY_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <std_srvs/Empty.h>
#include <robot_activity_msgs/State.h>
#include <robot_activity_msgs/Error.h>

#include <robot_activity/robot_activity.h>
#include <robot_activity/resource/resource_manager.h>

namespace robot_activity
{

/**
 * @brief Managed RobotActivity class, which adds further functionality to the
 *        RobotActivity class
 * @details ManagedRobotActivity manages ROS Subscribers and ServiceServers.
 *          It automatically pauses all Subscribers and ServiceServers
 *          during PAUSED state and resumes them when transitioning to the
 *          RUNNING state. It also shutdowns them in STOPPED state and
 *          re-acquires them (by re-subscribing or re-advertising) when
 *          transitioning from STOPPED to PAUSED.
 */
class ManagedRobotActivity : public RobotActivity
{
public:
  /**
   * @brief Default constructor inherited from RobotActivity
   */
  using RobotActivity::RobotActivity;

  /**
   * @brief Virtual destructor
   */
  virtual ~ManagedRobotActivity();

protected:
  /**
   * @brief Manager for subscribing to ROS topics.
   */
  resource::SubscriberManager subscriber_manager;

  /**
   * @brief Manager for advertising ROS services
   */
  resource::ServiceServerManager service_manager;

private:
  /**
   * @brief Overriden onCreate, which calls onManagedCreate. Cannot be
   *        overriden further by the child class of ManagedRobotActivity.
   */
  void onCreate() final;

  /**
   * @brief Overriden onTerminate, which calls onManagedTerminate. Cannot be
   *        overriden further by the child class of ManagedRobotActivity.
   */
  void onTerminate() final;

  /**
   * @brief Overriden onConfigure, which calls onManagedConfigure. Cannot be
   *        overriden further by the child class of ManagedRobotActivity.
   */
  void onConfigure() final;

  /**
   * @brief Overriden onUnconfigure, which calls onManagedUnconfigure. Cannot be
   *        overriden further by the child class of ManagedRobotActivity.
   */
  void onUnconfigure() final;

  /**
   * @brief Overriden onStart, which calls onManagedStart. Cannot be
   *        overriden further by the child class of ManagedRobotActivity.
   *        It subscribes and adverties all ROS topics and services that were
   *        subscribed and advertised with subscription_manager
   *        and service_manager before calling onManagedStart.
   */
  void onStart() final;

  /**
   * @brief Overriden onStop, which calls onManagedStop. Cannot be
   *        overriden further by the child class of ManagedRobotActivity.
   *        It shutdowns all ROS topics and services that were
   *        subscribed and advertised with subscription_manager
   *        and service_manager before calling onManagedStart.
   */
  void onStop() final;

  /**
   * @brief Overriden onPause, which calls onManagedPause. Cannot be
   *        overriden further by the child class of ManagedRobotActivity.
   *        It pauses all ROS topics and services that were
   *        subscribed and advertised with subscription_manager
   *        and service_manager before calling onManagedStart.
   */
  void onPause() final;

  /**
   * @brief Overriden onResume, which calls onManagedResume. Cannot be
   *        overriden further by the child class of ManagedRobotActivity.
   *        It resumes all ROS topics and services that were
   *        subscribed and advertised with subscription_manager
   *        and service_manager before calling onManagedStart.
   */
  void onResume() final;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        LAUNCHING to UNCONFIGURED state
   */
  virtual void onManagedCreate() = 0;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        UNCONFIGURED to TERMINATED state
   */
  virtual void onManagedTerminate() = 0;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        UNCONFIGURED to STOPPED state
   */
  virtual void onManagedConfigure() = 0;

  /**
  * @brief User-defined function that's called at the end of transition from
  *        STOPPED to UNCONFIGURED state
  */
  virtual void onManagedUnconfigure() = 0;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        STOPPED to PAUSED state
   */
  virtual void onManagedStart() = 0;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        PAUSED to STOPPED state
   */
  virtual void onManagedStop() = 0;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        RUNNING to PAUSED state
   */
  virtual void onManagedPause() = 0;

  /**
   * @brief User-defined function that's called at the end of transition from
   *        PAUSED to RUNNING state
   */
  virtual void onManagedResume() = 0;
};

}  // namespace robot_activity

#endif  // ROBOT_ACTIVITY_MANAGED_ROBOT_ACTIVITY_H
