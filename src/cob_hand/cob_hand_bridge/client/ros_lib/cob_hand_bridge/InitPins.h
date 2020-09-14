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
 

#ifndef _ROS_SERVICE_InitPins_h
#define _ROS_SERVICE_InitPins_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_hand_bridge
{

static const char INITPINS[] = "cob_hand_bridge/InitPins";

  class InitPinsRequest : public ros::Msg
  {
    public:
      uint8_t input_pins_length;
      uint8_t st_input_pins;
      uint8_t * input_pins;
      uint8_t output_pins_length;
      uint8_t st_output_pins;
      uint8_t * output_pins;

    InitPinsRequest():
      input_pins_length(0), input_pins(NULL),
      output_pins_length(0), output_pins(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = input_pins_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < input_pins_length; i++){
      *(outbuffer + offset + 0) = (this->input_pins[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->input_pins[i]);
      }
      *(outbuffer + offset++) = output_pins_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < output_pins_length; i++){
      *(outbuffer + offset + 0) = (this->output_pins[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->output_pins[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t input_pins_lengthT = *(inbuffer + offset++);
      if(input_pins_lengthT > input_pins_length)
        this->input_pins = (uint8_t*)realloc(this->input_pins, input_pins_lengthT * sizeof(uint8_t));
      offset += 3;
      input_pins_length = input_pins_lengthT;
      for( uint8_t i = 0; i < input_pins_length; i++){
      this->st_input_pins =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_input_pins);
        memcpy( &(this->input_pins[i]), &(this->st_input_pins), sizeof(uint8_t));
      }
      uint8_t output_pins_lengthT = *(inbuffer + offset++);
      if(output_pins_lengthT > output_pins_length)
        this->output_pins = (uint8_t*)realloc(this->output_pins, output_pins_lengthT * sizeof(uint8_t));
      offset += 3;
      output_pins_length = output_pins_lengthT;
      for( uint8_t i = 0; i < output_pins_length; i++){
      this->st_output_pins =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_output_pins);
        memcpy( &(this->output_pins[i]), &(this->st_output_pins), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return INITPINS; };
    const char * getMD5(){ return "842fe571cf71231e47b423e75069e2bd"; };

  };

  class InitPinsResponse : public ros::Msg
  {
    public:
      bool success;

    InitPinsResponse():
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

    const char * getType(){ return INITPINS; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class InitPins {
    public:
    typedef InitPinsRequest Request;
    typedef InitPinsResponse Response;
  };

}
#endif
