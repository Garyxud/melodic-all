/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Square Robot, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Square Robot, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * ROS Node for programmatic control of multiple rosbag record processes.  This node reads in "record profiles" from
 * a configuration profile and sets up a service to allow rosbag record processes for each "record profile" to be
 * started / stopped.  Each "record profile" is comprised of a name (which is used by the service to ID which process
 * to start/stop) and options to pass rosbag record.  A latched publication of the recording status for each of the
 * "record profiles" is published every time a call to the state change service is made.
 *
 * The Bagger node was created to help compartmentalize the bagging of different publications at different / the same
 * times, and to help keep bag sizes in check, which speeds up post-mission analysis.
 */

#ifndef SRC_BAGGER_INCLUDE_BAGGER_BAGGER_H_
#define SRC_BAGGER_INCLUDE_BAGGER_BAGGER_H_

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <map>
#include <string>
#include <vector>

#include <bagger/BaggingState.h>
#include <bagger/SetBagState.h>

/// Utility class for managing each spawned rosbag record process
class RecordProcess {
public:
  RecordProcess();
  RecordProcess(std::string name, std::string record_options);
  ~RecordProcess();

  /// Getters
  std::string getName();
  pid_t getRecordPID();
  bool getRecording();
  std::vector<std::string> getRecordOptionsVector();

  /// Setters
  void setRecordPID(pid_t pid);
  void setName(std::string name);
  void setRecordOptionString(std::string record_options);
  void setRecording(bool recording);

private:
  /// name associated with the process - comes from the set record_profiles
  std::string name_;
  /// record options associated with the process - comes from the set record_profiles
  std::string record_options_;
  /// whether or not the rosbag record process is currently recording
  bool recording_;
  /// pid of the spawned process - used to eventually terminate it
  pid_t record_pid_;
};

class Bagger {
protected:
  /// For publishing the names and states of each record profile rosbag record process
  ros::Publisher bagging_state_publisher_;
  /// Service for starting / stopping rosbag record processes
  ros::ServiceServer bag_state_service_;

  // Normal node handle
  ros::NodeHandlePtr node_handle_;
  // Private node handle, used for node-specific things like parameters
  ros::NodeHandlePtr private_node_handle_;

private:
  /// Callback for receipt of set bag state service calls
  bool onBagStateSrv(bagger::SetBagState::Request &request, bagger::SetBagState::Response &response);

  /// Convenience function for publishing the recording states for each record profile
  void publishBaggingStates();

  /// Utility function which returns the current working directory of the executing program
  std::string getCurrentWorkingDirectory();
  /// Searches the passed directory for any file paths that contain the passed match string.  Returns a vector
  /// of paths containing all such matches
  std::vector<boost::filesystem::path> getMatchingFilePathsInDirectory(const boost::filesystem::path &dir_path,
                                                                       std::string match_string);
  /// Utility function which infers the most likely rosbag name for the passed record options
  std::string getBagNameFromRecordOptions(std::string record_opts);
  /// Returns a version of the passed string with the passed suffix removed
  std::string removeSuffix(std::string s, std::string suffix);

  std::map<std::string, std::string> profile_name_to_record_options_map_;
  std::map<std::string, RecordProcess> profile_name_to_record_process_map_;

public:
  Bagger();
};

#endif /* SRC_BAGGER_INCLUDE_BAGGER_BAGGER_H_ */
