/**
Software License Agreement (BSD)

\file      frame_id_processor.h
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
#ifndef MESSAGE_RELAY_PROCESSOR_FRAME_ID_PROCESSOR_H
#define MESSAGE_RELAY_PROCESSOR_FRAME_ID_PROCESSOR_H

#include <string>
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include <boost/assign/list_of.hpp>

namespace message_relay
{

class FrameIdProcessor
{
public:
  typedef boost::shared_ptr<const FrameIdProcessor> ConstPtr;

  enum Operation
  {
    NONE, ADD_PREFIX, SELECTIVE_ADD_PREFIX, REMOVE_PREFIX, SELECTIVE_REMOVE_PREFIX
  };

  static ConstPtr create(std::string tf_prefix, std::string prefix_operation_string,
      boost::unordered_set<std::string> global_frame_names = boost::unordered_set<std::string>());

  static ConstPtr create(std::string tf_prefix, FrameIdProcessor::Operation prefix_operation,
      boost::unordered_set<std::string> global_frame_names = boost::unordered_set<std::string>());

  void process(std::string &frame_id) const;

  static ConstPtr inverse(const ConstPtr &processor);

private:
  FrameIdProcessor(std::string tf_prefix, Operation prefix_operation,
        boost::unordered_set<std::string> global_frame_names);

  static const boost::unordered_map<std::string, Operation> operation_name_map_;
  static const boost::unordered_map<Operation, Operation> operation_inverse_map_;

  std::string tf_prefix_;
  Operation prefix_operation_;
  boost::unordered_set<std::string> global_frame_names_;
};

}  // namespace message_relay

#endif  // MESSAGE_RELAY_PROCESSOR_FRAME_ID_PROCESSOR_H
