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
 

#ifndef _ROS_MSG_H_
#define _ROS_MSG_H_

#include <stdint.h>

namespace ros {

/* Base Message Type */
class Msg
{
public:
  virtual int serialize(unsigned char *outbuffer) const = 0;
  virtual int deserialize(unsigned char *data) = 0;
  virtual const char * getType() = 0;
  virtual const char * getMD5() = 0;

  /**
   * @brief This tricky function handles promoting a 32bit float to a 64bit
   *        double, so that AVR can publish messages containing float64
   *        fields, despite AVV having no native support for double.
   *
   * @param[out] outbuffer pointer for buffer to serialize to.
   * @param[in] f value to serialize.
   *
   * @return number of bytes to advance the buffer pointer.
   *
   */
  static int serializeAvrFloat64(unsigned char* outbuffer, const float f)
  {
    const int32_t* val = (int32_t*) &f;
    int32_t exp = ((*val >> 23) & 255);
    if (exp != 0)
    {
      exp += 1023 - 127;
    }

    int32_t sig = *val;
    *(outbuffer++) = 0;
    *(outbuffer++) = 0;
    *(outbuffer++) = 0;
    *(outbuffer++) = (sig << 5) & 0xff;
    *(outbuffer++) = (sig >> 3) & 0xff;
    *(outbuffer++) = (sig >> 11) & 0xff;
    *(outbuffer++) = ((exp << 4) & 0xF0) | ((sig >> 19) & 0x0F);
    *(outbuffer++) = (exp >> 4) & 0x7F;

    // Mark negative bit as necessary.
    if (f < 0)
    {
      *(outbuffer - 1) |= 0x80;
    }

    return 8;
  }

  /**
   * @brief This tricky function handles demoting a 64bit double to a
   *        32bit float, so that AVR can understand messages containing
   *        float64 fields, despite AVR having no native support for double.
   *
   * @param[in] inbuffer pointer for buffer to deserialize from.
   * @param[out] f pointer to place the deserialized value in.
   *
   * @return number of bytes to advance the buffer pointer.
   */
  static int deserializeAvrFloat64(const unsigned char* inbuffer, float* f)
  {
    uint32_t* val = (uint32_t*)f;
    inbuffer += 3;

    // Copy truncated mantissa.
    *val = ((uint32_t)(*(inbuffer++)) >> 5 & 0x07);
    *val |= ((uint32_t)(*(inbuffer++)) & 0xff) << 3;
    *val |= ((uint32_t)(*(inbuffer++)) & 0xff) << 11;
    *val |= ((uint32_t)(*inbuffer) & 0x0f) << 19;

    // Copy truncated exponent.
    uint32_t exp = ((uint32_t)(*(inbuffer++)) & 0xf0)>>4;
    exp |= ((uint32_t)(*inbuffer) & 0x7f) << 4;
    if (exp != 0)
    {
      *val |= ((exp) - 1023 + 127) << 23;
    }  

    // Copy negative sign.
    *val |= ((uint32_t)(*(inbuffer++)) & 0x80) << 24;

    return 8;
  }

};

}  // namespace ros

#endif
