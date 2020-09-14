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
   \file resource_manager.h
   \brief ResourceManager<Resource> class implements a resource manager for
    managed resources, such as ManagedSubscriber and ManagedServiceServer
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef ROBOT_ACTIVITY_RESOURCE_RESOURCE_MANAGER_H
#define ROBOT_ACTIVITY_RESOURCE_RESOURCE_MANAGER_H

#include <ros/ros.h>
#include <ros/console.h>

#include <robot_activity/resource/managed_subscriber.h>
#include <robot_activity/resource/managed_serviceserver.h>

#include <vector>

namespace robot_activity
{
namespace resource
{

/**
 * @brief Manages resources
 * @details Managed resources such as ManagedSubscriber and ManagedServiceServer
 *          can be added to the manager and acquired or released on demand.
 *          ManagedRobotActivity uses this class in order to correctly pause/
 *          shutdown and re-acquirer subscribers and services.
 *
 * @tparam Resource type to be managed
 */
template <class Resource>
class ResourceManager
{
public:
  /**
   * @brief Default constructor
   */
  ResourceManager() : resources_() {}

  /**
   * @brief Default destructor
   */
  ~ResourceManager() {}

  /**
   * @brief Adds a managed resource
   * @details Adds a managed resource to the list of managed resources.
   *          Arguments passed in should describe the managed resource.
   *          If the manager is specialized to deal with e.g. ManagedSubscriber
   *          then the they should match its constructor, which in turn has to
   *          match one of the ros::NodeHandle::subscribe function.
   * @param args Arguments describing the resource
   *
   * @return Returns a std::shared_ptr to the managed resource
   */
  template<typename... Args>
  typename Resource::SharedPtr add(Args&& ...args)
  {
    auto resource = std::make_shared<Resource>(std::forward<Args>(args)...);
    resources_.push_back(resource);
    return resource;
  }

  /**
   * @brief Acquires all managed resources
   *
   * @param node_handle ROS Node handle is necessary for acquiring
   */
  void acquireAll(const ros::NodeHandlePtr& node_handle);

  /**
   * @brief Releases all managed resources
   */
  void releaseAll();

  /**
   * @brief Pauses all managed resources
   */
  void pauseAll();

  /**
   * @brief Resumes all managed resources
   */
  void resumeAll();

private:
  std::vector<typename Resource::SharedPtr> resources_;
};

/**
 * @brief Wrapper around the ResourceManager
 *
 * @tparam T Resource type to be managed
 */
template <class T>
class RMWrapper : public ResourceManager<T> {};

/**
 * @brief Specialization of the RMWrapper,
 *        where the managed resource is ManagedSubscriber
 */
template<>
class RMWrapper<ManagedSubscriber> : public ResourceManager<ManagedSubscriber>
{
public:
  using ResourceManager<ManagedSubscriber>::add;

  /**
   * @brief Adds a subscriber to the list of managed subscribers
   * @details However, it does not actually subscribe to the ROS topic,
   *          this is deferred until acquireAll is called, where all
   *          managed subscribers will be actually subscribed. The goal is
   *          to allow user write code as close as possible to the original
   *          roscpp API.
   *
   * @param args Arguments describing the ROS subscriber,
   *             same as arguments passed to ros::NodeHandle::subscribe
   * @return std::shared_ptr of the ManagedSubscriber
   */
  template<typename... Args>
  ManagedSubscriber::SharedPtr subscribe(Args&& ...args)
  {
    return add(std::forward<Args>(args)...);
  }
};

/**
 * @brief Specialization of the RMWrapper,
 *        where the managed resource is ManagedServiceServer
 */
template<>
class RMWrapper<ManagedServiceServer> : public ResourceManager<ManagedServiceServer>
{
public:
  using ResourceManager<ManagedServiceServer>::add;

  /**
   * @brief Adds a service to the list of managed services
   * @details However, it does not actually advertise the ROS service,
   *          this is deferred until acquireAll is called, where all
   *          managed services will be actually advertised. The goal is
   *          to allow user write code as close as possible to the original
   *          roscpp API.
   *
   * @param args Arguments describing the ROS service,
   *             same as arguments passed to ros::NodeHandle::advertiseService
   * @return std::shared_ptr of the ManagedServiceServer
   */
  template<typename... Args>
  ManagedServiceServer::SharedPtr advertiseService(Args&& ...args)
  {
    return add(std::forward<Args>(args)...);
  }
};

typedef RMWrapper<ManagedSubscriber> SubscriberManager;
typedef RMWrapper<ManagedServiceServer> ServiceServerManager;

}  // namespace resource
}  // namespace robot_activity

#endif  // ROBOT_ACTIVITY_RESOURCE_RESOURCE_MANAGER_H
