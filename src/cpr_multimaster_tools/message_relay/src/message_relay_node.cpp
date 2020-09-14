/**
Software License Agreement (BSD)

\file      message_relay_node.cpp
\authors   Paul Bovbel <pbovbel@clearpath.ai>
\copyright Copyright (c) 2016, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "message_relay/relay_factory/topic_relay_factory.h"
#include "message_relay/relay_factory/service_relay_factory.h"
#include "message_relay/relay_factory/action_relay_factory.h"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/transport_hints.h"

#include "multimaster_msgs/GetClockOffset.h"

#include <boost/algorithm/string.hpp>

#include <vector>
#include <string>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "message_relay_node");

  ros::NodeHandle private_nh("~");

  // Get private parameters from a different namespace: allows for dynamic node names, but with a stable parameter
  // namespace (/fancy_name_relay reading parameters from /robot_relay)
  std::string parameter_namespace;
  if (private_nh.getParam("parameter_namespace", parameter_namespace))
  {
    private_nh = ros::NodeHandle(parameter_namespace);
  }

  // Get base origin and source nodehandles for this relay.
  // Topic, service, and action relays will relay from origin to source.
  std::string from = "", to = "";
  if (!private_nh.getParam("from", from) && !private_nh.getParam("to", to))
  {
    ROS_FATAL("Must provide either 'from' and 'to' namespace parameters");
    return 1;
  }

  if (from == to)
  {
    ROS_FATAL("'from' and 'to' namespaces must not be equal");
    return 1;
  }

  boost::shared_ptr<ros::NodeHandle> origin = boost::make_shared<ros::NodeHandle>(from);
  boost::shared_ptr<ros::NodeHandle> target = boost::make_shared<ros::NodeHandle>(to);

  // Determine if a frame_id transform should occur at this relay. The relay can either add or remove a tf_prefix.
  std::string tf_prefix, prefix_operation;
  if (private_nh.getParam("tf_prefix", tf_prefix) && !private_nh.getParam("prefix_operation", prefix_operation))
  {
    ROS_FATAL_STREAM("If tf_prefix is provided, a prefix_operation must be specified");
    return 1;
  }

  std::vector<std::string> global_frames;
  private_nh.getParam("global_frames", global_frames);

  message_relay::FrameIdProcessor::ConstPtr frame_id_processor = message_relay::FrameIdProcessor::create(
      tf_prefix, prefix_operation, boost::unordered_set<std::string>(global_frames.begin(), global_frames.end()));

  // Determine if a time offset should be applied to all outgoing messages/services.
  // The offset is read dynamically from a central GetClockOffset service, typically managed by a
  // specialized clock_relay_node
  std::string time_offset_operation;
  private_nh.param<std::string>("time_offset_operation", time_offset_operation, "");

  message_relay::TimeProcessor::ConstPtr time_processor;
  if (time_offset_operation != "")
  {
    // get time offset from clock_relay
    ros::ServiceClient clock_offset_client = origin->serviceClient<multimaster_msgs::GetClockOffset>
        ("/get_clock_offset");
    while (!clock_offset_client.waitForExistence(ros::Duration(5.0)))
    {
      ROS_WARN_STREAM("Waiting for /get_clock_offset service. Is clock_relay_node running?");
    }
    multimaster_msgs::GetClockOffset srv;
    while (ros::ok() && !clock_offset_client.call(srv))
    {
      ROS_ERROR_STREAM("Failed to get clock offset from /get_clock_offset");
    }
    time_processor = message_relay::TimeProcessor::create(time_offset_operation, srv.response.clock_offset.offset);
  }

  // Build topic relays from parameters
  std::vector<message_relay::TopicRelay::Ptr> topic_relays;
  XmlRpc::XmlRpcValue topics;
  if (private_nh.getParam("topics", topics))
  {
    for (int i = 0; i < topics.size(); i++)
    {
      message_relay::TopicRelayParams params;
      params.topic = static_cast<std::string>(topics[i]["topic"]);
      params.type = static_cast<std::string>(topics[i]["type"]);
      params.origin = origin;
      params.target = target;
      params.frame_id_processor = frame_id_processor;
      params.time_processor = time_processor;
      params.queue_size = static_cast<int>(topics[i]["queue_size"]);
      if (params.queue_size == 0)
      {
        std::string key;
        if (private_nh.searchParam("queue_size", key))
        {
          private_nh.getParam(key, params.queue_size);
        }
        else
        {
          // set default
          params.queue_size = 100;
        }
      }
      params.throttle_frequency = static_cast<double>(topics[i]["throttle_frequency"]);
      if (params.throttle_frequency == 0.0)
      {
        std::string key;
        if (private_nh.searchParam("throttle_frequency", key))
        {
          private_nh.getParam(key, params.throttle_frequency);
        }
      }
      params.latch = static_cast<bool>(topics[i]["latch"]);
      params.unreliable = static_cast<bool>(topics[i]["unreliable"]);
      ROS_DEBUG_STREAM("Topic " << params.topic << " type " << params.type << " latch " << params.latch <<
          " queue_size " << params.queue_size << " throttle " << params.throttle_frequency <<
          " unreliable " << params.unreliable);
      topic_relays.push_back(message_relay::createTopicRelay(params));
    }
  }

  // Determine if services should be processed on a separate queue, to prevent them from
  // accidentally blocking topic relays
  bool separate_service_queue;
  private_nh.param("separate_service_queue", separate_service_queue, true);

  // Build service relays from parameters
  boost::shared_ptr<ros::CallbackQueue> service_queue;
  if (separate_service_queue)
  {
    service_queue = boost::make_shared<ros::CallbackQueue>();
  }
  std::vector<message_relay::ServiceRelay::Ptr> service_relays;
  XmlRpc::XmlRpcValue services;
  if (private_nh.getParam("services", services))
  {
    for (int i = 0; i < services.size(); i++)
    {
      message_relay::ServiceRelayParams params;
      params.service = static_cast<std::string>(services[i]["service"]);
      params.type = static_cast<std::string>(services[i]["type"]);
      params.origin = origin;
      params.target = target;
      params.frame_id_processor = frame_id_processor;
      params.time_processor = time_processor;
      params.callback_queue = service_queue;
      ROS_DEBUG_STREAM("Service  " << params.service << " type " << params.type << " check_for_server_frequency " <<
          params.check_for_server_frequency);
      service_relays.push_back(message_relay::createServiceRelay(params));
    }
  }

  // Build action relays from parameters
  std::vector<message_relay::ActionRelay::Ptr> action_relays;
  XmlRpc::XmlRpcValue actions;
  if (private_nh.getParam("actions", actions))
  {
    for (int i = 0; i < actions.size(); i++)
    {
      message_relay::ActionRelayParams params;
      params.action = static_cast<std::string>(actions[i]["action"]);
      params.type = static_cast<std::string>(actions[i]["type"]);
      params.origin = origin;
      params.target = target;
      params.frame_id_processor = frame_id_processor;
      params.time_processor = time_processor;
      params.queue_size = static_cast<int>(actions[i]["queue_size"]);
      params.feedback_throttle_frequency = static_cast<double>(actions[i]["feedback_throttle_frequency"]);
      if (params.queue_size == 0)
      {
        std::string key;
        if (private_nh.searchParam("queue_size", key))
        {
          private_nh.getParam(key, params.queue_size);
        }
        else
        {
          // set default
          params.queue_size = 100;
        }
      }
      ROS_DEBUG_STREAM("Action " << params.action << " type " << params.type << " queue_size " << params.queue_size);
      action_relays.push_back(message_relay::createActionRelay(params));
    }
  }

  // Process services on an async queue if necessary
  boost::shared_ptr<ros::AsyncSpinner> service_spinner;
  if (separate_service_queue)
  {
    service_spinner = boost::make_shared<ros::AsyncSpinner>(1, service_queue.get());
    service_spinner->start();
  }

  ros::spin();
  return 0;
}
