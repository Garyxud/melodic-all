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
 
 
#ifndef _ROS_DURATION_H_
#define _ROS_DURATION_H_

#include <math.h>
#include <stdint.h>

namespace ros {

  void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec);

  class Duration
  {
    public:
      int32_t sec, nsec;

      Duration() : sec(0), nsec(0) {}
      Duration(int32_t _sec, int32_t _nsec) : sec(_sec), nsec(_nsec)
      {
        normalizeSecNSecSigned(sec, nsec);
      }

      double toSec() const { return (double)sec + 1e-9*(double)nsec; };
      void fromSec(double t) { sec = (uint32_t) floor(t); nsec = (uint32_t) round((t-sec) * 1e9); };

      Duration& operator+=(const Duration &rhs);
      Duration& operator-=(const Duration &rhs);
      Duration& operator*=(double scale);
  };

}

#endif

