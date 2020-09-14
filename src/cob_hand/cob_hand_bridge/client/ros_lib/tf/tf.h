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
 


#ifndef ROS_TF_H_
#define ROS_TF_H_

#include "geometry_msgs/TransformStamped.h"

namespace tf
{
  
  static inline geometry_msgs::Quaternion createQuaternionFromYaw(double yaw)
  {
    geometry_msgs::Quaternion q;
    q.x = 0;
    q.y = 0;
    q.z = sin(yaw * 0.5);
    q.w = cos(yaw * 0.5);
    return q;
  }

}

#endif

