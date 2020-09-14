/**
Software License Agreement (BSD)

\file      time_processor.cpp
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
#include "message_relay/processor/time_processor.h"

#include <boost/assign/list_of.hpp>
#include <string>

namespace message_relay
{

const boost::unordered_map<std::string, TimeProcessor::Operation> TimeProcessor::operation_name_map_ =
    boost::assign::map_list_of("", TimeProcessor::NONE)("add_offset", TimeProcessor::ADD_OFFSET)
        ("remove_offset", TimeProcessor::REMOVE_OFFSET);

const boost::unordered_map<TimeProcessor::Operation, TimeProcessor::Operation> TimeProcessor::operation_inverse_map_ =
    boost::assign::map_list_of(TimeProcessor::NONE, TimeProcessor::NONE)
        (TimeProcessor::ADD_OFFSET, TimeProcessor::REMOVE_OFFSET)
        (TimeProcessor::REMOVE_OFFSET, TimeProcessor::ADD_OFFSET);

TimeProcessor::ConstPtr TimeProcessor::create(std::string offset_operation_string, ros::Duration offset)
{
  try
  {
    TimeProcessor::Operation offset_operation = operation_name_map_.at(offset_operation_string);
    return TimeProcessor::create(offset_operation, offset);
  }
  catch (const std::out_of_range &ex)
  {
    ROS_FATAL_STREAM("Invalid time offset operation " << offset_operation_string << " specified");
    throw ex;
  }
}

TimeProcessor::ConstPtr TimeProcessor::create(TimeProcessor::Operation offset_operation, ros::Duration offset)
{
  return TimeProcessor::ConstPtr(new const TimeProcessor(offset_operation, offset));
}

TimeProcessor::ConstPtr TimeProcessor::inverse(const TimeProcessor::ConstPtr &processor)
{
  if (processor)
  {
    return TimeProcessor::ConstPtr(new const TimeProcessor(operation_inverse_map_.at(
        processor->offset_operation_), processor->offset_));
  }
  else
  {
    return TimeProcessor::ConstPtr();
  }
}

TimeProcessor::TimeProcessor(Operation offset_operation, ros::Duration offset)
    : offset_operation_(offset_operation), offset_(offset)
{ }

void TimeProcessor::process(ros::Time &time) const
{
  switch (offset_operation_)
  {
    case TimeProcessor::NONE:
      break;

    case TimeProcessor::ADD_OFFSET:
      time += offset_;
      break;

    case TimeProcessor::REMOVE_OFFSET:
      time -= offset_;
      break;

    default:
      ROS_ASSERT_MSG(false, "Invalid time offset operation");
  }
}

}  // namespace message_relay
