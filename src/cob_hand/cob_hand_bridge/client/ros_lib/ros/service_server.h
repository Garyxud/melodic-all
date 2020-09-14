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
 

#ifndef _ROS_SERVICE_SERVER_H_
#define _ROS_SERVICE_SERVER_H_

#include "rosserial_msgs/TopicInfo.h"

#include "publisher.h"
#include "subscriber.h"

namespace ros {

  template<typename MReq , typename MRes>
  class ServiceServer : public Subscriber_ {
    public:
      typedef void(*CallbackT)(const MReq&,  MRes&);

      ServiceServer(const char* topic_name, CallbackT cb) :
        pub(topic_name, &resp, rosserial_msgs::TopicInfo::ID_SERVICE_SERVER + rosserial_msgs::TopicInfo::ID_PUBLISHER)
      {
        this->topic_ = topic_name;
        this->cb_ = cb;
      }

      // these refer to the subscriber
      virtual void callback(unsigned char *data){
        req.deserialize(data);
        cb_(req,resp);
        pub.publish(&resp);
      }
      virtual const char * getMsgType(){ return this->req.getType(); }
      virtual const char * getMsgMD5(){ return this->req.getMD5(); }
      virtual int getEndpointType(){ return rosserial_msgs::TopicInfo::ID_SERVICE_SERVER + rosserial_msgs::TopicInfo::ID_SUBSCRIBER; }

      MReq req;
      MRes resp;
      Publisher pub;
    private:
      CallbackT cb_;
  };

}

#endif
