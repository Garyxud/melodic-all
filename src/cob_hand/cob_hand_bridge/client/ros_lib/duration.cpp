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
 

#include <math.h>
#include "ros/duration.h"

namespace ros
{
  void normalizeSecNSecSigned(int32_t &sec, int32_t &nsec)
  {
    int32_t nsec_part = nsec;
    int32_t sec_part = sec;

    while (nsec_part > 1000000000L)
    {
      nsec_part -= 1000000000L;
      ++sec_part;
    }
    while (nsec_part < 0)
    {
      nsec_part += 1000000000L;
      --sec_part;
    }
    sec = sec_part;
    nsec = nsec_part;
  }

  Duration& Duration::operator+=(const Duration &rhs)
  {
    sec += rhs.sec;
    nsec += rhs.nsec;
    normalizeSecNSecSigned(sec, nsec);
    return *this;
  }

  Duration& Duration::operator-=(const Duration &rhs){
    sec += -rhs.sec;
    nsec += -rhs.nsec;
    normalizeSecNSecSigned(sec, nsec);
    return *this;
  }

  Duration& Duration::operator*=(double scale){
    sec *= scale;
    nsec *= scale;
    normalizeSecNSecSigned(sec, nsec);
    return *this;
  }

}
