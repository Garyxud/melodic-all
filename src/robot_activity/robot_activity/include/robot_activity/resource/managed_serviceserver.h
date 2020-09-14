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
   \file managed_serviceserver.h
   \brief ManagedServiceServer class implements a wrapper around ros::ServiceServer
     with same functionality as ros::ServiceServer, but also includes
     possibility to pause/resume service calls as well as re-advertise
    subscriptions and services
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef ROBOT_ACTIVITY_RESOURCE_MANAGED_SERVICESERVER_H
#define ROBOT_ACTIVITY_RESOURCE_MANAGED_SERVICESERVER_H

#include <ros/ros.h>
#include <ros/console.h>

#include <robot_activity/resource/managed_resource.h>

#include <string>

namespace robot_activity
{
namespace resource
{

/**
 * @brief Implementation of Managed class for ros::ServiceServer
 * @details Adds additional functionality to ros::ServiceServer resource by
 *          wrapping it. The service server can be shutdowned, advertised and
 *          paused. Shutting down has the same effect as in ROS, while
 *          advertising does not require any arguments, since they are passed in
 *          the inherited constructor. Pausing the service, will cause
 *          the callback to return false immediately.
 *
 */
class ManagedServiceServer : public Managed<ManagedServiceServer, ros::ServiceServer>
{
public:
  /**
   * @brief Inherited variadic constructor from the Managed class
   * @details Creates the ROS service but not yet advertises it, which is
   *          left until advertiseService is called with a node handle.
   *          The constructor polimorphically calls *makeLazyAcquirer*
   *          forwarding all the arguments to it. All function signatures of
   *          makeLazyAcquirer must match the actual function signatures for
   *          ros::NodeHandle::advertiseService function. Therefore,
   *          in order to create a managed ros::ServiceServer
   *          we have to call this constructor with the same arguments as
   *          we would call ros::NodeHandle::advertiseService.
   *          Due to the CRTP idiom, the polymorphic call is resolved
   *          at compile-time without incurring additional run-time cost.
   *
   */
  using Managed<ManagedServiceServer, ros::ServiceServer>::Managed;

  /**
   * @brief Advertises the ROS service given a node handle
   * @details Since the ROS service is fully desribed upon instantiation,
   *          advertising only requires a node handle. The function is idempotent
   *          and can be called multiple times without any side-effect. The service
   *          can also be re-advertised after being shutdowned without respecifying
   *          arguments describing the service.
   *
   * @param node_handle Node handle required for the actual call to
   *                    ros::NodeHandle::advertiseService embedded inside
   */
  void advertiseService(const ros::NodeHandlePtr& node_handle)
  {
    acquire(node_handle);
  }

  /**
   * @brief Shutdowns the ROS service
   * @details Has the same effect as ros::ServiceServer::shutdown function if
   *          the service was advertised, otherwise does not have any effect
   *
   */
  void shutdown()
  {
    release();
  }

  /**
   * @brief Typedef for ROS service callbacks
   *
   * @tparam Callback arguments
   */
  template <typename ...Args>
  using ServiceCallback = boost::function<bool(Args...)>;

  /**
   * @brief Creates a function that advertises a ROS service when called
   * @details Returns a lambda, which given a node handle will advertise the
   *          ROS service. This lambda is called during advertiseService call
   *          and the result is assigned as the managed resource, which in this
   *          case is ros::ServiceServer
   *
   * @param service Name of the service to be advertised
   * @param callback Service callback specified as boost::function accepting two
   *                 arguments
   * @param tracked_object Object to be tracked, whose destruction will terminate the service
   * @tparam MReq Service callback's message request type
   * @tparam MRes Service callback's message response type
   * @return Lambda, which return a ros::ServiceServer
   */
  template<class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service,
    const ServiceCallback<MReq&, MRes&>& callback,
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr()) const
  {
    ROS_DEBUG("makeLazyAcquirer ServiceCallback<MReq&, MRes&>&");
    return [ = ](const ros::NodeHandlePtr & nh) -> ros::ServiceServer
    {
      ROS_DEBUG("Advertising...");
      return nh->advertiseService(
        service,
        static_cast<ServiceCallback<MReq&, MRes&>>(wrapServiceCallback(callback)),
        tracked_object);
    };
  }

  /**
   * @brief Creates a function that advertises a ROS service when called
   * @details Returns a lambda, which given a node handle will advertise the
   *          ROS service. This lambda is called during advertiseService call
   *          and the result is assigned as the managed resource, which in this
   *          case is ros::ServiceServer
   *
   * @param service Name of the service to be advertised
   * @param callback Service callback specified as boost::function accepting one
   *                 argument
   * @param tracked_object Object to be tracked, whose destruction will terminate the service
   * @tparam ServiceEvent Service callback's event message type
   * @return Lambda, which return a ros::ServiceServer
   */
  template<class ServiceEvent>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service,
    const ServiceCallback<ServiceEvent&>& callback,
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr()) const
  {
    ROS_DEBUG("makeLazyAcquirer ServiceEventCallback<ServiceEvent&>&");
    return [ = ](const ros::NodeHandlePtr & nh) -> ros::ServiceServer
    {
      ROS_DEBUG("Advertising...");
      return nh->advertiseService(
        service,
        static_cast<ServiceCallback<ServiceEvent&>>(wrapServiceCallback(callback)),
        tracked_object);
    };
  }

  /**
   * @brief Creates a function that advertises a ROS service when called
   * @details Returns a lambda, which given a node handle will advertise the
   *          ROS service. This lambda is called during advertiseService call
   *          and the result is assigned as the managed resource, which in this
   *          case is ros::ServiceServer
   *
   * @param service Name of the service to be advertised
   * @param srv_func Pointer to a member function of class T, which accepts
   *                 two arguments: message request and response and returns a bool
   * @param obj Object to be used when calling the pointed function
   * @tparam T Class, where the member function is defined
   * @tparam MReq Service callback's message request type
   * @tparam MRes Service callback's message response type
   * @return Lambda, which return a ros::ServiceServer
   */
  template<class T, class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service,
    bool(T::*srv_func)(MReq&, MRes&),
    T *obj) const
  {
    ServiceCallback<MReq&, MRes&> callback = boost::bind(srv_func, obj, _1, _2);
    return makeLazyAcquirer(service, callback);
  }

  /**
   * @brief Creates a function that advertises a ROS service when called
   * @details Returns a lambda, which given a node handle will advertise the
   *          ROS service. This lambda is called during advertiseService call
   *          and the result is assigned as the managed resource, which in this
   *          case is ros::ServiceServer
   *
   * @param service Name of the service to be advertised
   * @param srv_func Pointer to a member function of class T, which accepts
   *                 one argument: service event and returns a bool
   * @param obj Raw pointer to the object to be used when
   *            calling the pointed function
   * @tparam T Class, where the member function is defined
   * @tparam MReq Service callback's message request type
   * @tparam MRes Service callback's message response type
   * @return Lambda, which return a ros::ServiceServer
   */
  template<class T, class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service,
    bool(T::*srv_func)(ros::ServiceEvent<MReq, MRes>&),
    T *obj) const
  {
    ServiceCallback<ros::ServiceEvent<MReq, MRes>&> callback
      = boost::bind(srv_func, obj, _1);
    return makeLazyAcquirer(service, callback);
  }

  /**
   * @brief Creates a function that advertises a ROS service when called
   * @details Returns a lambda, which given a node handle will advertise the
   *          ROS service. This lambda is called during advertiseService call
   *          and the result is assigned as the managed resource, which in this
   *          case is ros::ServiceServer
   *
   * @param service Name of the service to be advertised
   * @param srv_func Pointer to a member function of class T, which accepts
   *                 two arguments: message request and response and returns a bool
   * @param obj boost::shared_ptr to the object to be used when
   *            calling the pointed function, it will also be treated
   *            as the tracked object, whose destruction will trigger a
   *            service shutdown
   * @tparam T Class, where the the member function is defined
   * @tparam MReq Service callback's message request type
   * @tparam MRes Service callback's message response type
   * @return Lambda, which return a ros::ServiceServer
   */
  template<class T, class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service,
    bool(T::*srv_func)(MReq &, MRes &),
    const boost::shared_ptr<T>& obj) const
  {
    ServiceCallback<MReq&, MRes&> callback = boost::bind(srv_func, obj.get(), _1, _2);
    return makeLazyAcquirer(service, callback, obj);
  }

  /**
   * @brief Creates a function that advertises a ROS service when called
   * @details Returns a lambda, which given a node handle will advertise the
   *          ROS service. This lambda is called during advertiseService call
   *          and the result is assigned as the managed resource, which in this
   *          case is ros::ServiceServer
   *
   * @param service Name of the service to be advertised
   * @param srv_func Pointer to a member function of class T, which accepts
   *                 one argument: a service event and returns a bool
   * @param obj boost::shared_ptr to the object to be used when
   *            calling the pointed function, it will also be treated
   *            as the tracked object, whose destruction will trigger a
   *            service shutdown
   * @tparam T Class, where the the member function is defined
   * @tparam MReq Service callback's message request type
   * @tparam MRes Service callback's message response type
   * @return Lambda, which return a ros::ServiceServer
   */
  template<class T, class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service,
    bool(T::*srv_func)(ros::ServiceEvent<MReq, MRes>&),
    const boost::shared_ptr<T>& obj) const
  {
    ServiceCallback<ros::ServiceEvent<MReq, MRes>&> callback
      = boost::bind(srv_func, obj.get(), _1);
    return makeLazyAcquirer(service, callback, obj);
  }

  /**
   * @brief Creates a function that advertises a ROS service when called
   * @details Returns a lambda, which given a node handle will advertise the
   *          ROS service. This lambda is called during advertiseService call
   *          and the result is assigned as the managed resource, which in this
   *          case is ros::ServiceServer
   *
   * @param service Name of the service to be advertised
   * @param srv_func Pointer to a function, which accepts
   *                 two arguments: message request and response
   *                 and returns a bool
   * @tparam MReq Service callback's message request type
   * @tparam MRes Service callback's message response type
   * @return Lambda, which return a ros::ServiceServer
   */
  template<class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service,
    bool(*srv_func)(MReq&, MRes&)) const
  {
    ServiceCallback<MReq&, MRes&> callback = boost::bind(srv_func);
    return makeLazyAcquirer(service, callback);
  }

  /**
   * @brief Creates a function that advertises a ROS service when called
   * @details Returns a lambda, which given a node handle will advertise the
   *          ROS service. This lambda is called during advertiseService call
   *          and the result is assigned as the managed resource, which in this
   *          case is ros::ServiceServer
   *
   * @param service Name of the service to be advertised
   * @param srv_func Pointer to a function, which accepts
   *                 one argument: service event and returns a bool
   * @tparam MReq Service callback's message request type
   * @tparam MRes Service callback's message response type
   * @return Lambda, which return a ros::ServiceServer
   */
  template<class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service,
    bool(*srv_func)(ros::ServiceEvent<MReq, MRes>&)) const
  {
    ServiceCallback<ros::ServiceEvent<MReq, MRes>&> callback
      = boost::bind(srv_func);
    return makeLazyAcquirer(service, callback);
  }


  /* TODO - remove if unneccessary
  template<class MReq, class MRes>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service,
    const boost::function<bool(MReq&, MRes&)>& callback,
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr())
  {
    AdvertiseServiceOptions ops;
    ops.template init<MReq, MRes>(service, callback);
    ops.tracked_object = tracked_object;
    return makeLazyAcquirer(ops);
  }

  template<class S>
  LazyAcquirer makeLazyAcquirer(
    const std::string& service,
    const boost::function<bool(S&)>& callback,
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr())
  {
    AdvertiseServiceOptions ops;
    ops.template initBySpecType<S>(service, callback);
    ops.tracked_object = tracked_object;
    return makeLazyAcquirer(ops);
  }
  */
private:
  /**
   * @brief Wraps the service callback
   * @details Allows for pausing/resuming the servce by adding a check at
   *          run-time. If paused, the service will return false immediately.
   *
   * @param callback [description]
   * @return [description]
   */
  template <typename ...Args>
  ServiceCallback<Args...> wrapServiceCallback(
    const ServiceCallback<Args...>& callback) const
  {
    return [this, &callback](Args ... args) -> bool
    {
      if (paused_)
      {
        ROS_DEBUG("service is paused!");
        return false;
      }
      return callback(std::forward<Args>(args)...);
    };
  }
};

}  // namespace resource
}  // namespace robot_activity

#endif  // ROBOT_ACTIVITY_RESOURCE_MANAGED_SERVICESERVER_H
