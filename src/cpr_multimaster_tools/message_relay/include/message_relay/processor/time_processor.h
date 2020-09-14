/**
Software License Agreement (BSD)

\file      time_processor.h
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
#ifndef MESSAGE_RELAY_PROCESSOR_TIME_PROCESSOR_H
#define MESSAGE_RELAY_PROCESSOR_TIME_PROCESSOR_H

#include "ros/ros.h"

#include <boost/unordered_map.hpp>
#include <string>
#include <vector>

namespace message_relay
{

class TimeProcessor
{
public:
  typedef boost::shared_ptr<const TimeProcessor> ConstPtr;

  enum Operation
  {
    NONE, ADD_OFFSET, REMOVE_OFFSET
  };

  static ConstPtr create(std::string offset_operation_string, ros::Duration offset = ros::Duration(0.0));

  static ConstPtr create(Operation offset_operation, ros::Duration offset = ros::Duration(0.0));

  void process(ros::Time &time) const;

  static ConstPtr inverse(const ConstPtr &processor);

private:
  TimeProcessor(Operation offset_operation, ros::Duration offset);

  static const boost::unordered_map<std::string, Operation> operation_name_map_;
  static const boost::unordered_map<Operation, Operation> operation_inverse_map_;

  Operation offset_operation_;
  ros::Duration offset_;
};

}  // namespace message_relay

#endif  // MESSAGE_RELAY_PROCESSOR_TIME_PROCESSOR_H
