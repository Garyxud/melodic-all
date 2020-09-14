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
   \file managed_subscriber.h
   \brief ManagedSubscriber class implements a wrapper around ros::Subscriber
     with same functionality as ros::Subscriber, but also includes
     possibility to pause/resume callbacks as well as re-subscribe
    subscriptions and services
   \author Maciej Marcin ZURAD
   \date 01/03/2018
*/
#ifndef ROBOT_ACTIVITY_RESOURCE_MANAGED_SUBSCRIBER_H
#define ROBOT_ACTIVITY_RESOURCE_MANAGED_SUBSCRIBER_H

#include <ros/ros.h>
#include <ros/console.h>

#include <robot_activity/resource/managed_resource.h>

#include <string>

namespace robot_activity
{
namespace resource
{

/**
 * @brief Implementation of Managed class for ros::Subscriber
 * @details Adds additional functionality to ros::Subscriber resource by
 *          wrapping it. The subsciber can be shutdowned, subscribed and
 *          paused. Shutting down has the same effect as in ROS, while
 *          subscribing does not require any arguments, since they are passed in
 *          the inherited constructor. Pausing the service, will cause
 *          the callback to return immediately without any execution.
 *
 */
class ManagedSubscriber : public Managed<ManagedSubscriber, ros::Subscriber>
{
public:
  /**
   * @brief Inherited variadic constructor from the Managed class
   * @details Creates the ROS subscriber but not yet subscribes to the topic,
   *          which is left until subscribe is called with a node handle.
   *          The constructor polimorphically calls *makeLazyAcquirer*
   *          forwarding all the arguments to it. All function signatures of
   *          makeLazyAcquirer must match the actual function signatures for
   *          ros::NodeHandle::subscribe function. Therefore,
   *          in order to create a managed ros::Subscriber
   *          we have to call this constructor with the same arguments as
   *          we would call ros::NodeHandle::subscribe.
   *          Due to the CRTP idiom, the polymorphic call is resolved
   *          at compile-time without incurring additional run-time cost.
   *
   */
  using Managed<ManagedSubscriber, ros::Subscriber>::Managed;

  /**
   * @brief Subscribes to a ROS topic given a node handle
   * @details Since the ROS subsciber is fully desribed upon instantiation,
   *          the actual subscribe call only requires a node handle.
   *          The function is idempotent and can be called multiple times
   *          without any side-effect. The topic can be also re-subscribed
   *          after being shutdowned without respecifying
   *          arguments describing the subscription.
   *
   * @param node_handle Node handle required for the actual call to
   *                    ros::NodeHandle::subscribe embedded inside
   */
  void subscribe(const ros::NodeHandlePtr& node_handle)
  {
    return acquire(node_handle);
  }

  /**
   * @brief Shutdowns the ROS subscriber
   * @details Has the same effect as ros::Subscriber::shutdown function if
   *          the topic was subscribed, otherwise does not have any effect
   *
   */
  void shutdown()
  {
    return release();
  }

  /**
   * @brief Typedef for ROS subscriber callbacks
   *
   * @tparam Callback argument - Message type
   */
  template <class Message>
  using MessageCallback = boost::function<void(Message)>;

  /**
   * @brief Creates a function that when called subscribes to a ROS topic
   * @details Returns a lambda, which given a node handle will subscribe to a
   *          ROS topic. This lambda is called during subscribe call
   *          and the result is assigned as the managed resource, which in this
   *          case is the ros::Subscriber
   *
   * @param topic Name of the ROS topic to subscribe to
   * @param queue_size Subscriber's queue size, 0 signifies an infinite queue
   * @param callback Subscriber callback specified as boost::function accepting one
   *                 argument - message type
   * @param tracked_object Object to be tracked, whose destruction will
   *                       terminate the subscriber
   * @param transport_hints Options describing the ROS subscriber
   * @tparam Message Message type
   * @return Lambda, which return a ros::Subscriber
   */
  template<class Message>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    const MessageCallback<Message>& callback,
    const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
    const ros::TransportHints& transport_hints = ros::TransportHints()) const
  {
    ROS_DEBUG("makeLazyAcquirer MessageCallback<Message>& callback form exec");
    return [ = ](const ros::NodeHandlePtr & nh) -> ros::Subscriber
    {
      ROS_DEBUG("Subscribing...");
      return nh->subscribe<Message>(
        topic,
        queue_size,
        static_cast<MessageCallback<Message>>(wrapMessageCallback(callback)),
        tracked_object,
        transport_hints);
    };
  }

  /**
   * @brief Creates a function that when called subscribes to a ROS topic
   * @details Returns a lambda, which given a node handle will subscribe to a
   *          ROS topic. This lambda is called during subscribe call
   *          and the result is assigned as the managed resource, which in this
   *          case is the ros::Subscriber
   *
   * @param topic Name of the ROS topic to subscribe to
   * @param queue_size Subscriber's queue size, 0 signifies an infinite queue
   * @param fp Subscriber callback specified as a pointer to a member function
   * @param obj Raw pointer to an object used when calling the member function.
   * @param transport_hints Options describing the ROS subscriber
   * @tparam M Message type
   * @tparam T Object type on which the member function is called
   * @return Lambda, which return a ros::Subscriber
   */
  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj,
    const ros::TransportHints& transport_hints = ros::TransportHints()) const
  {
    ROS_DEBUG("makeLazyAcquirer void(T::*fp)(M), T* obj, form exec");
    MessageCallback<M> callback = boost::bind(fp, obj, _1);
    return makeLazyAcquirer(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
  }

  /**
   * @brief Creates a function that when called subscribes to a ROS topic
   * @details Returns a lambda, which given a node handle will subscribe to a
   *          ROS topic. This lambda is called during subscribe call
   *          and the result is assigned as the managed resource, which in this
   *          case is the ros::Subscriber
   *
   * @param topic Name of the ROS topic to subscribe to
   * @param queue_size Subscriber's queue size, 0 signifies an infinite queue
   * @param fp Subscriber callback specified as a pointer to a member const function
   * @param obj Raw pointer to an object used when calling the member function
   * @param transport_hints Options describing the ROS subscriber
   * @tparam M Message type
   * @tparam T Object type on which the member function is called
   * @return Lambda, which return a ros::Subscriber
   */
  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size, void(T::*fp)(M) const, T* obj,
    const ros::TransportHints& transport_hints = ros::TransportHints()) const
  {
    ROS_DEBUG("makeLazyAcquirer void(T::*fp)(M) const, T* obj, form exec");
    MessageCallback<M> callback = boost::bind(fp, obj, _1);
    return makeLazyAcquirer(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
  }

  /**
   * @brief Creates a function that when called subscribes to a ROS topic
   * @details Returns a lambda, which given a node handle will subscribe to a
   *          ROS topic. This lambda is called during subscribe call
   *          and the result is assigned as the managed resource, which in this
   *          case is the ros::Subscriber
   *
   * @param topic Name of the ROS topic to subscribe to
   * @param queue_size Subscriber's queue size, 0 signifies an infinite queue
   * @param fp Subscriber callback specified as a pointer to a member function,
   *           which accepts a boost::shared_ptr to a const message
   * @param obj Raw pointer to an object used when calling the member function.
   * @param transport_hints Options describing the ROS subscriber
   * @tparam M Message type
   * @tparam T Object type on which the member function is called
   * @return Lambda, which return a ros::Subscriber
   */
  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr<M const>&), T* obj,
    const ros::TransportHints& transport_hints = ros::TransportHints()) const
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
                     << "void(T::*fp)(const boost::shared_ptr<M const>&), "
                     << "T* obj, form exec");
    MessageCallback<M> callback = boost::bind(fp, obj, _1);
    return makeLazyAcquirer(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
  }

  /**
   * @brief Creates a function that when called subscribes to a ROS topic
   * @details Returns a lambda, which given a node handle will subscribe to a
   *          ROS topic. This lambda is called during subscribe call
   *          and the result is assigned as the managed resource, which in this
   *          case is the ros::Subscriber
   *
   * @param topic Name of the ROS topic to subscribe to
   * @param queue_size Subscriber's queue size, 0 signifies an infinite queue
   * @param fp Subscriber callback specified as a pointer to a const member
   *           function, which accepts a boost::shared_ptr to a const message
   * @param obj Raw pointer to an object used when calling the member function.
   * @param transport_hints Options describing the ROS subscriber
   * @tparam M Message type
   * @tparam T Object type on which the member function is called
   * @return Lambda, which return a ros::Subscriber
   */
  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj,
    const ros::TransportHints& transport_hints = ros::TransportHints()) const
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
                     << "void(T::*fp)(const boost::shared_ptr<M const>&) const, "
                     << "T* obj, form exec");
    MessageCallback<M> callback = boost::bind(fp, obj, _1);
    return makeLazyAcquirer(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
  }

  /**
   * @brief Creates a function that when called subscribes to a ROS topic
   * @details Returns a lambda, which given a node handle will subscribe to a
   *          ROS topic. This lambda is called during subscribe call
   *          and the result is assigned as the managed resource, which in this
   *          case is the ros::Subscriber
   *
   * @param topic Name of the ROS topic to subscribe to
   * @param queue_size Subscriber's queue size, 0 signifies an infinite queue
   * @param fp Subscriber callback specified as a pointer to a member
   *           function, which accepts a message
   * @param obj Boost shared pointer to an object used when calling
   *            the member function. Destruction of this object causes subscriber
   *            to shutdown.
   * @param transport_hints Options describing the ROS subscriber
   * @tparam M Message type
   * @tparam T Object type on which the member function is called
   * @return Lambda, which return a ros::Subscriber
   */
  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(M), const boost::shared_ptr<T>& obj,
    const ros::TransportHints& transport_hints = ros::TransportHints()) const
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
                     << "void(T::*fp)(M), "
                     << "const boost::shared_ptr<T>& obj, form exec");
    MessageCallback<M> callback = boost::bind(fp, obj.get(), _1);
    return makeLazyAcquirer(topic, queue_size, callback, obj, transport_hints);
  }

  /**
   * @brief Creates a function that when called subscribes to a ROS topic
   * @details Returns a lambda, which given a node handle will subscribe to a
   *          ROS topic. This lambda is called during subscribe call
   *          and the result is assigned as the managed resource, which in this
   *          case is the ros::Subscriber
   *
   * @param topic Name of the ROS topic to subscribe to
   * @param queue_size Subscriber's queue size, 0 signifies an infinite queue
   * @param fp Subscriber callback specified as a pointer to a const member
   *           function, which accepts a message
   * @param obj Boost shared pointer to an object used when calling
   *            the member function. Destruction of this object causes subscriber
   *            to shutdown.
   * @param transport_hints Options describing the ROS subscriber
   * @tparam M Message type
   * @tparam T Object type on which the member function is called
   * @return Lambda, which return a ros::Subscriber
   */
  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(M) const, const boost::shared_ptr<T>& obj,
    const ros::TransportHints& transport_hints = ros::TransportHints()) const
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
                     << "void(T::*fp)(M) const, "
                     << "const boost::shared_ptr<T>& obj, form exec");
    MessageCallback<M> callback = boost::bind(fp, obj.get(), _1);
    return makeLazyAcquirer(topic, queue_size, callback, obj, transport_hints);
  }

  /**
   * @brief Creates a function that when called subscribes to a ROS topic
   * @details Returns a lambda, which given a node handle will subscribe to a
   *          ROS topic. This lambda is called during subscribe call
   *          and the result is assigned as the managed resource, which in this
   *          case is the ros::Subscriber
   *
   * @param topic Name of the ROS topic to subscribe to
   * @param queue_size Subscriber's queue size, 0 signifies an infinite queue
   * @param fp Subscriber callback specified as a pointer to a member
   *           function, which accepts a boost::shared_ptr to a const message
   * @param obj Boost shared pointer to an object used when calling
   *            the member function. Destruction of this object causes subscriber
   *            to shutdown.
   * @param transport_hints Options describing the ROS subscriber
   * @tparam M Message type
   * @tparam T Object type on which the member function is called
   * @return Lambda, which return a ros::Subscriber
   */
  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr<M const>&),
    const boost::shared_ptr<T>& obj,
    const ros::TransportHints& transport_hints = ros::TransportHints()) const
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
                     << "void(T::*fp)(const boost::shared_ptr<M const>&), "
                     << "const boost::shared_ptr<T>& obj, form exec");
    MessageCallback<M> callback = boost::bind(fp, obj.get(), _1);
    return makeLazyAcquirer(topic, queue_size, callback, obj, transport_hints);
  }

  /**
   * @brief Creates a function that when called subscribes to a ROS topic
   * @details Returns a lambda, which given a node handle will subscribe to a
   *          ROS topic. This lambda is called during subscribe call
   *          and the result is assigned as the managed resource, which in this
   *          case is the ros::Subscriber
   *
   * @param topic Name of the ROS topic to subscribe to
   * @param queue_size Subscriber's queue size, 0 signifies an infinite queue
   * @param fp Subscriber callback specified as a pointer to a const member
   *           function, which accepts a boost::shared_ptr to a const message
   * @param obj Boost shared pointer to an object used when calling
   *            the member function. Destruction of this object causes subscriber
   *            to shutdown.
   * @param transport_hints Options describing the ROS subscriber
   * @tparam M Message type
   * @tparam T Object type on which the member function is called
   * @return Lambda, which return a ros::Subscriber
   */
  template<class M, class T>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    void(T::*fp)(const boost::shared_ptr<M const>&) const,
    const boost::shared_ptr<T>& obj,
    const ros::TransportHints& transport_hints = ros::TransportHints()) const
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
                     << "void(T::*fp)(const boost::shared_ptr<M const>&) const, "
                     << "const boost::shared_ptr<T>& obj, form exec");
    MessageCallback<M> callback = boost::bind(fp, obj.get(), _1);
    return makeLazyAcquirer(topic, queue_size, callback, obj, transport_hints);
  }

  /**
    * @brief Creates a function that when called subscribes to a ROS topic
    * @details Returns a lambda, which given a node handle will subscribe to a
    *          ROS topic. This lambda is called during subscribe call
    *          and the result is assigned as the managed resource, which in this
    *          case is the ros::Subscriber
    *
    * @param topic Name of the ROS topic to subscribe to
    * @param queue_size Subscriber's queue size, 0 signifies an infinite queue
    * @param fp Subscriber callback specified as a pointer to a function
    * @param transport_hints Options describing the ROS subscriber
    * @tparam M Message type
    * @tparam T Object type on which the member function is called
    * @return Lambda, which return a ros::Subscriber
    */
  template<class M>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size, void(*fp)(M),
    const ros::TransportHints& transport_hints = ros::TransportHints()) const
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
                     << "void(*fp)(M) form exec");
    MessageCallback<M> callback = boost::bind(fp);
    return makeLazyAcquirer(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
  }

  /**
    * @brief Creates a function that when called subscribes to a ROS topic
    * @details Returns a lambda, which given a node handle will subscribe to a
    *          ROS topic. This lambda is called during subscribe call
    *          and the result is assigned as the managed resource, which in this
    *          case is the ros::Subscriber
    *
    * @param topic Name of the ROS topic to subscribe to
    * @param queue_size Subscriber's queue size, 0 signifies an infinite queue
    * @param fp Subscriber callback specified as a pointer to a function, which
    *           accepts a boost shared pointer to a const message
    * @param transport_hints Options describing the ROS subscriber
    * @tparam M Message type
    * @tparam T Object type on which the member function is called
    * @return Lambda, which return a ros::Subscriber
    */
  template<class M>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr<M const>&),
    const ros::TransportHints& transport_hints = ros::TransportHints()) const
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
                     << "void(*fp)(const boost::shared_ptr<M const>&) form exec");
    MessageCallback<M> callback = boost::bind(fp);
    return makeLazyAcquirer(topic, queue_size, callback, ros::VoidConstPtr(), transport_hints);
  }

  /* TODO - remove if unneccessary
  template<class M>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    const boost::function<void (const boost::shared_ptr<M const>&)>& callback,
    const ros::VoidConstPtr& tracked_object,
    const ros::TransportHints& transport_hints)
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
      << "const boost::function<void (const boost::shared_ptr<M const>&)>& callback, "
      << "form exec");
    //MessageCallback<const boost::shared_ptr<M const>&> callback = callback;
    return makeLazyAcquirer(topic, queue_size, callback, tracked_object, transport_hints);
  }

  template<class M, class C>
  LazyAcquirer makeLazyAcquirer(
    const std::string& topic, uint32_t queue_size,
    const boost::function<void (C)>& callback,
    const ros::VoidConstPtr& tracked_object,
    const ros::TransportHints& transport_hints)
  {
    ROS_DEBUG_STREAM("makeLazyAcquirer "
      << "const boost::function<void (C)>& callback form exec");
    return makeLazyAcquirer(topic, queue_size, callback, tracked_object, transport_hints);
  }
  */
private:
  /**
   * @brief Wraps the message callback
   * @details Allows for pausing/resuming the servce by adding a check at
   *          run-time. If paused, the service will return immediately.
   *
   * @param callback [description]
   * @return [description]
   */
  template<class Message>
  MessageCallback<Message> wrapMessageCallback(const MessageCallback<Message>& callback) const
  {
    return [this, &callback](Message message)
    {
      if (!paused_)
        callback(message);
      else
        ROS_DEBUG("callback is paused!");
    };
  }
};

}  // namespace resource
}  // namespace robot_activity

#endif  // ROBOT_ACTIVITY_RESOURCE_MANAGED_SUBSCRIBER_H
