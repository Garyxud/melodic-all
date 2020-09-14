/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

#ifndef ROS_TRANSFORM_BROADCASTER_H_
#define ROS_TRANSFORM_BROADCASTER_H_

#include "tfMessage.h"

namespace tf
{

  class TransformBroadcaster
  {
    public:
      TransformBroadcaster() : publisher_("/tf", &internal_msg) {}

      void init(ros::NodeHandle &nh)
      {
        nh.advertise(publisher_);
      }

      void sendTransform(geometry_msgs::TransformStamped &transform)
      {
        internal_msg.transforms_length = 1;
        internal_msg.transforms = &transform;
        publisher_.publish(&internal_msg);
      }

    private:
      tf::tfMessage internal_msg;
      ros::Publisher publisher_;
  };

}

#endif

