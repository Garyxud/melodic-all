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
 

#ifndef _ROS_cob_hand_bridge_JointValues_h
#define _ROS_cob_hand_bridge_JointValues_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cob_hand_bridge
{

  class JointValues : public ros::Msg
  {
    public:
      int16_t position_cdeg[2];
      int16_t velocity_cdeg_s[2];
      int16_t current_100uA[2];

    JointValues():
      position_cdeg(),
      velocity_cdeg_s(),
      current_100uA()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 2; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_position_cdegi;
      u_position_cdegi.real = this->position_cdeg[i];
      *(outbuffer + offset + 0) = (u_position_cdegi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_cdegi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->position_cdeg[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_velocity_cdeg_si;
      u_velocity_cdeg_si.real = this->velocity_cdeg_s[i];
      *(outbuffer + offset + 0) = (u_velocity_cdeg_si.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_cdeg_si.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->velocity_cdeg_s[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_current_100uAi;
      u_current_100uAi.real = this->current_100uA[i];
      *(outbuffer + offset + 0) = (u_current_100uAi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_100uAi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current_100uA[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 2; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_position_cdegi;
      u_position_cdegi.base = 0;
      u_position_cdegi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_cdegi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->position_cdeg[i] = u_position_cdegi.real;
      offset += sizeof(this->position_cdeg[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_velocity_cdeg_si;
      u_velocity_cdeg_si.base = 0;
      u_velocity_cdeg_si.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_cdeg_si.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->velocity_cdeg_s[i] = u_velocity_cdeg_si.real;
      offset += sizeof(this->velocity_cdeg_s[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_current_100uAi;
      u_current_100uAi.base = 0;
      u_current_100uAi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_100uAi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current_100uA[i] = u_current_100uAi.real;
      offset += sizeof(this->current_100uA[i]);
      }
     return offset;
    }

    const char * getType(){ return "cob_hand_bridge/JointValues"; };
    const char * getMD5(){ return "a8168eaf63b3492bbb7dcd4942c4f1c0"; };

  };

}
#endif
