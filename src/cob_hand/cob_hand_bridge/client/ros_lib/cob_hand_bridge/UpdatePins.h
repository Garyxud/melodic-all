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
 

#ifndef _ROS_SERVICE_UpdatePins_h
#define _ROS_SERVICE_UpdatePins_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_hand_bridge
{

static const char UPDATEPINS[] = "cob_hand_bridge/UpdatePins";

  class UpdatePinsRequest : public ros::Msg
  {
    public:
      uint32_t set_pins;
      uint32_t clear_pins;

    UpdatePinsRequest():
      set_pins(0),
      clear_pins(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->set_pins >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->set_pins >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->set_pins >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->set_pins >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_pins);
      *(outbuffer + offset + 0) = (this->clear_pins >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->clear_pins >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->clear_pins >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->clear_pins >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clear_pins);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->set_pins =  ((uint32_t) (*(inbuffer + offset)));
      this->set_pins |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->set_pins |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->set_pins |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->set_pins);
      this->clear_pins =  ((uint32_t) (*(inbuffer + offset)));
      this->clear_pins |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->clear_pins |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->clear_pins |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->clear_pins);
     return offset;
    }

    const char * getType(){ return UPDATEPINS; };
    const char * getMD5(){ return "54c611b52e3951b09beb1eddc8d57861"; };

  };

  class UpdatePinsResponse : public ros::Msg
  {
    public:
      bool success;

    UpdatePinsResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return UPDATEPINS; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class UpdatePins {
    public:
    typedef UpdatePinsRequest Request;
    typedef UpdatePinsResponse Response;
  };

}
#endif
