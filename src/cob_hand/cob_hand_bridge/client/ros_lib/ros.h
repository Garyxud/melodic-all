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
 

#ifndef _ROS_H_
#define _ROS_H_

#ifndef BUILD_LIBROSSERIALEMBEDDEDLINUX
#include "embedded_linux_comms.c"
#include "duration.cpp"
#include "time.cpp"
#endif

#include "ros/node_handle.h"
#include "embedded_linux_hardware.h"

namespace ros
{
typedef NodeHandle_<EmbeddedLinuxHardware> NodeHandle;
}

#endif
