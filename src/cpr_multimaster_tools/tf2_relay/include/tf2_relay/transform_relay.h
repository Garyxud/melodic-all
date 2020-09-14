/**
Software License Agreement (BSD)

\file      transform_relay.h
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
#ifndef TF2_RELAY_TRANSFORM_RELAY_H
#define TF2_RELAY_TRANSFORM_RELAY_H

#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"

#include "message_relay/processor/frame_id_processor.h"

#include <boost/unordered_map.hpp>

#include <string>
#include <utility>

namespace tf2_relay
{

class TransformRelay
{
public:
  TransformRelay(ros::NodeHandle origin, ros::NodeHandle target, double frequency, bool is_static,
                 message_relay::FrameIdProcessor::ConstPtr frame_id_processor);

private:
  void transformCb(const tf2_msgs::TFMessageConstPtr &transforms);

  void relayCb();

  typedef std::pair<std::string, std::string> FrameIdPair;

  void processTransform(const geometry_msgs::TransformStamped &new_tf);

  ros::NodeHandle origin_, target_;
  ros::Timer relay_timer_;
  message_relay::FrameIdProcessor::ConstPtr frame_id_processor_;

  ros::Publisher tf_publisher_;
  ros::Subscriber tf_subscriber_;

  tf2_msgs::TFMessage transform_cache_;
  boost::unordered_map<FrameIdPair, std::size_t> transform_cache_index_map_;
};

}  // namespace tf2_relay

#endif  // TF2_RELAY_TRANSFORM_RELAY_H
