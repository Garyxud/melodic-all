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
   \file managed_resource.h
   \brief Managed<Derived,R> class implements a base class which manages
     ROS resources, such as ros::Subscriber and ros::ServiceServer.
     It uses CRTP idiom for static polymorphism and adds functionality to
     pause and resume, as well as acquire and release the resource.
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef ROBOT_ACTIVITY_RESOURCE_MANAGED_RESOURCE_H
#define ROBOT_ACTIVITY_RESOURCE_MANAGED_RESOURCE_H

#include <atomic>

#include <ros/ros.h>
#include <ros/console.h>

namespace robot_activity
{
namespace resource
{

class ManagedSubscriber;
class ManagedServiceServer;

/**
 * @brief Wrapper around ROS resources, such as ros::Subscriber and ros::ServiceServer
 * @details This class adds additional functionality to ROS resources such
 *          as pausing, resuming and re-acquiring it. It also implements
 *          lazy acquisition of a resource due to the fact that the resource is
 *          not acquired upon instantiation, but when *acquire* is called.
 *          The class is the base wrapper class and has to be sub-classed by a
 *          specific resource. We use CRTP idiom, hence the templated class,
 *          in order to achieve static-time (compile-time) polymorphism.
 *
 *
 * @tparam Derived CRTP derived class
 * @tparam Resource ROS resource, such as ros::Subscriber or ros::ServiceServer
 */
template <class Derived, class Resource>
class Managed
{
public:
  /**
   * @brief Default empty constructor is deleted
   * @details Resource has to be fully described upon instantiation
   */
  Managed() = delete;

  /**
   * @brief Destructor
   */
  ~Managed();

  /**
   * @brief Variadic constructor
   * @details Main constructor, where args should fully specify the resource
   *          and correspond to one of the original function signatures from
   *          ROS resources (eg. ros::Subscriber). Args are forwarded
   *          to the makeLazyAcquirer function, which is bound at compile-time
   *          to the deriving class, which HAS to implement ALL function signatures
   *          available in ROS for creating that particular resource via
   *          a node handle.
   *
   * @tparam Args Types have to match ROS resource creation function signature
   *         (e.g node_handle.subscribe(...))
   * @param args Specify the resource, e.g. topic, queue_size and a callback in
   *             the case of a ros::Subscriber
   */
  template<typename... Args>
  explicit Managed(Args&& ...args)
    : acquired_(false), paused_(true), resource_(), lazy_acquirer_()
  {
    ROS_DEBUG("Managed::ctor");
    lazy_acquirer_ = makeLazyAcquirer(std::forward<Args>(args)...);
  }

  /**
   * @brief Acquires the resource if it's not already acquired
   *
   * @param node_handle NodeHandle is needed for acquisition, same as in ROS
   */
  void acquire(const ros::NodeHandlePtr& node_handle);

  /**
   * @brief Releases the resource if it's already acquired. shutdown() method in
   *        case of ros::Subscriber and ros::ServiceServer
   */
  void release();

  /**
   * @brief Pauses the resource
   * @details Depending on the deriving concrete resource, it usually means that
   *          the callback is returned before actually executing
   */
  void pause();

  /**
   * @brief Resumes the resource
   * @details Undoes the pause operation
   */
  void resume();

  typedef std::shared_ptr<Managed<Derived, Resource>> SharedPtr;

protected:
  typedef std::function<Resource(const ros::NodeHandlePtr&)> LazyAcquirer;

  /**
   * @brief Lazily acquires a resource
   * @details Creates a function such that, when called with a ROS node handle
   *          will acquire the specific resource
   *
   * @param args Specifies the resource
   * @return Function that will acquire resource when called with a node handle
   */
  template<typename... Args>
  LazyAcquirer makeLazyAcquirer(Args&& ...args) const
  {
    return static_cast<const Derived*>(this)
           ->makeLazyAcquirer(std::forward<Args>(args)...);
  }

  /**
   * @brief Atomic flag specifing whether the resource is acquired or not
   */
  std::atomic<bool> acquired_;

  /**
   * @brief Atomic flag specifing whether the resource is paused or not
   */
  std::atomic<bool> paused_;


  /**
   * @brief The actual resource controlled by this class
   */
  Resource resource_;

  /**
   * @brief Function that will acquire the resource upon calling with a node handle
   */
  LazyAcquirer lazy_acquirer_;
};

}  // namespace resource
}  // namespace robot_activity

#endif  // ROBOT_ACTIVITY_RESOURCE_MANAGED_RESOURCE_H
