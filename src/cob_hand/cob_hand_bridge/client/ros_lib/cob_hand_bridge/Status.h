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
 

#ifndef _ROS_cob_hand_bridge_Status_h
#define _ROS_cob_hand_bridge_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "cob_hand_bridge/JointValues.h"

namespace cob_hand_bridge
{

  class Status : public ros::Msg
  {
    public:
      uint32_t seq;
      ros::Time stamp;
      uint8_t status;
      uint8_t rc;
      uint32_t pins;
      cob_hand_bridge::JointValues joints;
      enum { NOT_INITIALIZED = 0 };
      enum { MASK_FINGER_READY = 1 };
      enum { MASK_GPIO_READY = 2 };
      enum { MASK_ERROR = 4 };
      enum { RC_OK = 0 };

    Status():
      seq(0),
      stamp(),
      status(0),
      rc(0),
      pins(0),
      joints()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->seq >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->seq >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->seq >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->seq >> (8 * 3)) & 0xFF;
      offset += sizeof(this->seq);
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      *(outbuffer + offset + 0) = (this->rc >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rc);
      *(outbuffer + offset + 0) = (this->pins >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pins >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pins >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pins >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pins);
      offset += this->joints.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->seq =  ((uint32_t) (*(inbuffer + offset)));
      this->seq |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->seq |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->seq |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->seq);
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      this->rc =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->rc);
      this->pins =  ((uint32_t) (*(inbuffer + offset)));
      this->pins |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pins |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pins |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pins);
      offset += this->joints.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "cob_hand_bridge/Status"; };
    const char * getMD5(){ return "d1e9c755cdcb7c29d72650239e9eb91a"; };

  };

}
#endif
