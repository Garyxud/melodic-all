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
 

#ifndef ROS_TIME_H_
#define ROS_TIME_H_

#include <math.h>
#include <stdint.h>

#include "ros/duration.h"

namespace ros
{
  void normalizeSecNSec(uint32_t &sec, uint32_t &nsec);

  class Time
  {
    public:
      uint32_t sec, nsec;

      Time() : sec(0), nsec(0) {}
      Time(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
      {
        normalizeSecNSec(sec, nsec);
      }

      double toSec() const { return (double)sec + 1e-9*(double)nsec; };
      void fromSec(double t) { sec = (uint32_t) floor(t); nsec = (uint32_t) round((t-sec) * 1e9); };

      uint32_t toNsec() { return (uint32_t)sec*1000000000ull + (uint32_t)nsec; };
      Time& fromNSec(int32_t t);

      Time& operator +=(const Duration &rhs);
      Time& operator -=(const Duration &rhs);

      static Time now();
      static void setNow( Time & new_now);
  };

}

#endif
