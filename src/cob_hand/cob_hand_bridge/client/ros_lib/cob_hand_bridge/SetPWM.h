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
 

#ifndef _ROS_SERVICE_SetPWM_h
#define _ROS_SERVICE_SetPWM_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_hand_bridge
{

static const char SETPWM[] = "cob_hand_bridge/SetPWM";

  class SetPWMRequest : public ros::Msg
  {
    public:
      uint8_t pins_length;
      uint8_t st_pins;
      uint8_t * pins;
      uint8_t levels_length;
      float st_levels;
      float * levels;

    SetPWMRequest():
      pins_length(0), pins(NULL),
      levels_length(0), levels(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = pins_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < pins_length; i++){
      *(outbuffer + offset + 0) = (this->pins[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pins[i]);
      }
      *(outbuffer + offset++) = levels_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < levels_length; i++){
      union {
        float real;
        uint32_t base;
      } u_levelsi;
      u_levelsi.real = this->levels[i];
      *(outbuffer + offset + 0) = (u_levelsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_levelsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_levelsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_levelsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->levels[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t pins_lengthT = *(inbuffer + offset++);
      if(pins_lengthT > pins_length)
        this->pins = (uint8_t*)realloc(this->pins, pins_lengthT * sizeof(uint8_t));
      offset += 3;
      pins_length = pins_lengthT;
      for( uint8_t i = 0; i < pins_length; i++){
      this->st_pins =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_pins);
        memcpy( &(this->pins[i]), &(this->st_pins), sizeof(uint8_t));
      }
      uint8_t levels_lengthT = *(inbuffer + offset++);
      if(levels_lengthT > levels_length)
        this->levels = (float*)realloc(this->levels, levels_lengthT * sizeof(float));
      offset += 3;
      levels_length = levels_lengthT;
      for( uint8_t i = 0; i < levels_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_levels;
      u_st_levels.base = 0;
      u_st_levels.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_levels.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_levels.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_levels.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_levels = u_st_levels.real;
      offset += sizeof(this->st_levels);
        memcpy( &(this->levels[i]), &(this->st_levels), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return SETPWM; };
    const char * getMD5(){ return "d3b671b5e2eb290c5f274725029ed0c2"; };

  };

  class SetPWMResponse : public ros::Msg
  {
    public:
      bool success;

    SetPWMResponse():
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

    const char * getType(){ return SETPWM; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetPWM {
    public:
    typedef SetPWMRequest Request;
    typedef SetPWMResponse Response;
  };

}
#endif
