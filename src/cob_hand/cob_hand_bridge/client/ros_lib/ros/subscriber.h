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
 

#ifndef ROS_SUBSCRIBER_H_
#define ROS_SUBSCRIBER_H_

#include "rosserial_msgs/TopicInfo.h"

namespace ros {

  /* Base class for objects subscribers. */
  class Subscriber_
  {
    public:
      virtual void callback(unsigned char *data)=0;
      virtual int getEndpointType()=0;

      // id_ is set by NodeHandle when we advertise 
      int id_;

      virtual const char * getMsgType()=0;
      virtual const char * getMsgMD5()=0;
      const char * topic_;
  };


  /* Actual subscriber, templated on message type. */
  template<typename MsgT>
  class Subscriber: public Subscriber_{
    public:
      typedef void(*CallbackT)(const MsgT&);
      MsgT msg;

      Subscriber(const char * topic_name, CallbackT cb, int endpoint=rosserial_msgs::TopicInfo::ID_SUBSCRIBER) :
        cb_(cb),
        endpoint_(endpoint)
      {
        topic_ = topic_name;
      };

      virtual void callback(unsigned char* data){
        msg.deserialize(data);
        this->cb_(msg);
      }

      virtual const char * getMsgType(){ return this->msg.getType(); }
      virtual const char * getMsgMD5(){ return this->msg.getMD5(); }
      virtual int getEndpointType(){ return endpoint_; }

    private:
      CallbackT cb_;
      int endpoint_;
  };

}

#endif
