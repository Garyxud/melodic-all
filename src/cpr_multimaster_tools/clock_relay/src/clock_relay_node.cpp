/**
Software License Agreement (BSD)

\file      clock_relay_node.cpp
\authors   Paul Bovbel <pbovbel@clearpath.ai>
\copyright Copyright (c) 2016, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "clock_relay/clock_relay.h"
#include "ros/ros.h"

#include <vector>
#include <string>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "clock_relay_node");

  ros::NodeHandle private_nh("~");

  std::string parameter_namespace;
  if (private_nh.getParam("parameter_namespace", parameter_namespace))
  {
    private_nh = ros::NodeHandle(parameter_namespace);
  }

  std::string from, to;
  if (!private_nh.getParam("from", from) && !private_nh.getParam("to", to))
  {
    ROS_FATAL("Must provide either 'from' and 'to' namespace parameters");
    return 1;
  }

  if (from == to)
  {
    ROS_FATAL("'from' and 'to' namespaces must not be equal");
    return 1;
  }

  ros::NodeHandle origin(from), target(to);

  std::string clock_relay_type;
  if (!private_nh.getParam("clock_relay_type", clock_relay_type))
  {
    ROS_FATAL_STREAM("Must specify if clock relay type is source or sink");
    return 1;
  }

  double throttle_frequency = 0.0;
  std::string key;
  if (private_nh.searchParam("throttle_frequency", key))
  {
    std::string val;
    private_nh.getParam(key, throttle_frequency);
  }

  clock_relay::ClockRelay clock_relay(from, to, clock_relay_type, throttle_frequency);

  ros::spin();

  return 0;
}
