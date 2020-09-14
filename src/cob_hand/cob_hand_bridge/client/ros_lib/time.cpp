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
 

#include "ros/time.h"

namespace ros
{
  void normalizeSecNSec(uint32_t& sec, uint32_t& nsec){
    uint32_t nsec_part= nsec % 1000000000UL;
    uint32_t sec_part = nsec / 1000000000UL;
    sec += sec_part;
    nsec = nsec_part;
  }

  Time& Time::fromNSec(int32_t t)
  {
    sec = t / 1000000000;
    nsec = t % 1000000000;
    normalizeSecNSec(sec, nsec);
    return *this;
  }

  Time& Time::operator +=(const Duration &rhs)
  {
    sec += rhs.sec;
    nsec += rhs.nsec;
    normalizeSecNSec(sec, nsec);
    return *this;
  }

  Time& Time::operator -=(const Duration &rhs){
    sec += -rhs.sec;
    nsec += -rhs.nsec;
    normalizeSecNSec(sec, nsec);
    return *this;
  }
}
