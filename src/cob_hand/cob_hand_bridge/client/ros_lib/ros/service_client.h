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
 

#ifndef _ROS_SERVICE_CLIENT_H_
#define _ROS_SERVICE_CLIENT_H_

#include "rosserial_msgs/TopicInfo.h"

#include "publisher.h"
#include "subscriber.h"

namespace ros {

  template<typename MReq , typename MRes>
  class ServiceClient : public Subscriber_  {
    public:
      ServiceClient(const char* topic_name) : 
        pub(topic_name, &req, rosserial_msgs::TopicInfo::ID_SERVICE_CLIENT + rosserial_msgs::TopicInfo::ID_PUBLISHER)
      {
        this->topic_ = topic_name;
        this->waiting = true;
      }

      virtual void call(const MReq & request, MRes & response)
      {
        if(!pub.nh_->connected()) return;
        ret = &response;
        waiting = true;
        pub.publish(&request);
        while(waiting && pub.nh_->connected())
          if(pub.nh_->spinOnce() < 0) break;
      }

      // these refer to the subscriber
      virtual void callback(unsigned char *data){
        ret->deserialize(data);
        waiting = false;
      }
      virtual const char * getMsgType(){ return this->resp.getType(); }
      virtual const char * getMsgMD5(){ return this->resp.getMD5(); }
      virtual int getEndpointType(){ return rosserial_msgs::TopicInfo::ID_SERVICE_CLIENT + rosserial_msgs::TopicInfo::ID_SUBSCRIBER; }

      MReq req;
      MRes resp;
      MRes * ret;
      bool waiting;
      Publisher pub;
  };

}

#endif
