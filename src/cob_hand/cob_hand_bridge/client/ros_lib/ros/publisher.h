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
 

#ifndef _ROS_PUBLISHER_H_
#define _ROS_PUBLISHER_H_

#include "rosserial_msgs/TopicInfo.h"
#include "node_handle.h"

namespace ros {

  /* Generic Publisher */
  class Publisher
  {
    public:
      Publisher( const char * topic_name, Msg * msg, int endpoint=rosserial_msgs::TopicInfo::ID_PUBLISHER) :
        topic_(topic_name), 
        msg_(msg),
        endpoint_(endpoint) {};

      int publish( const Msg * msg ) { return nh_->publish(id_, msg); };
      int getEndpointType(){ return endpoint_; }

      const char * topic_;
      Msg *msg_;
      // id_ and no_ are set by NodeHandle when we advertise 
      int id_;
      NodeHandleBase_* nh_;

    private:
      int endpoint_;
  };

}

#endif
