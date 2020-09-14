/**
Software License Agreement (BSD)

\file      topic_relay.h
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
#ifndef MESSAGE_RELAY_RELAY_TOPIC_RELAY_H
#define MESSAGE_RELAY_RELAY_TOPIC_RELAY_H

#include "message_relay/processor/message_processor.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/transport_hints.h"

#include <string>

namespace message_relay
{

struct TopicRelayParams
{
  std::string type;
  std::string topic;
  boost::shared_ptr<ros::NodeHandle> origin;
  boost::shared_ptr<ros::NodeHandle> target;
  FrameIdProcessor::ConstPtr frame_id_processor;
  TimeProcessor::ConstPtr time_processor;
  bool latch;
  double throttle_frequency;
  int queue_size;
  boost::shared_ptr<ros::CallbackQueue> callback_queue;
  bool unreliable;

  TopicRelayParams()
    : type(), topic(), origin(), target(), frame_id_processor(), time_processor(), latch(false),
      throttle_frequency(0.0), queue_size(100), callback_queue(), unreliable(false)
  { }
};

class TopicRelay
{
public:
  typedef boost::shared_ptr<TopicRelay> Ptr;

  virtual ~TopicRelay()
  { }

protected:
  TopicRelay()
  { }
};

template<typename MessageType>
class TopicRelayImpl : public TopicRelay
{
  friend TopicRelay::Ptr createTopicRelay(const TopicRelayParams &params);

private:
  explicit TopicRelayImpl(const TopicRelayParams &params)
    : origin_(params.origin), target_(params.target), frame_id_processor_(params.frame_id_processor),
      time_processor_(params.time_processor), throttle_duration_(0.0)
  {
    if (params.throttle_frequency > 0.0)
    {
      throttle_duration_ = ros::Duration(1.0 / params.throttle_frequency);
    }

    pub_options_ = ros::AdvertiseOptions::create<MessageType>(
      params.topic,
      params.queue_size,
      ros::SubscriberStatusCallback(),
      ros::SubscriberStatusCallback(),
      ros::VoidConstPtr(),
      params.callback_queue.get());
    pub_options_.latch = params.latch;

    sub_options_ = ros::SubscribeOptions::create<MessageType>(
                pub_options_.topic,
                pub_options_.queue_size,
                boost::bind(&TopicRelayImpl<MessageType>::topicCb, this, _1),
                ros::VoidConstPtr(),
                NULL);
    sub_options_.transport_hints = params.unreliable? ros::TransportHints().unreliable() : ros::TransportHints();

    sub_ = boost::make_shared<ros::Subscriber>(origin_->subscribe(sub_options_));
    pub_ = boost::make_shared<ros::Publisher>(target_->advertise(pub_options_));
  }

  void topicCb(const typename MessageType::ConstPtr &input)
  {
    if (!throttle_duration_.isZero())
    {
      // Check if throttle duration has been met since the last relay callback
      if (ros::Time::now() > (last_relay_ + throttle_duration_))
      {
        last_relay_ = ros::Time::now();
      }
      else
      {
        // Otherwise skip this callback
        return;
      }
    }

    typename MessageType::ConstPtr output;

    if (frame_id_processor_ || time_processor_)
    {
      typename MessageType::Ptr msg = boost::make_shared<MessageType>(*input);
      if (frame_id_processor_)
      {
        MessageProcessor<MessageType, FrameIdProcessor>::processMessage(msg, frame_id_processor_);
      }
      if (time_processor_)
      {
        MessageProcessor<MessageType, TimeProcessor>::processMessage(msg, time_processor_);
      }
      output = boost::static_pointer_cast<MessageType const>(msg);
    }
    else
    {
      output = input;
    }
    pub_->publish(output);
  };

  boost::shared_ptr<ros::NodeHandle> origin_, target_;
  FrameIdProcessor::ConstPtr frame_id_processor_;
  TimeProcessor::ConstPtr time_processor_;

  ros::Duration throttle_duration_;
  ros::Time last_relay_;

  boost::shared_ptr<ros::Subscriber> sub_;
  boost::shared_ptr<ros::Publisher> pub_;
  ros::AdvertiseOptions pub_options_;
  ros::SubscribeOptions sub_options_;
};

}  // namespace message_relay

#endif  // MESSAGE_RELAY_RELAY_TOPIC_RELAY_H
