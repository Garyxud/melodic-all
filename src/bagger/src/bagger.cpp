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

#include <bagger/bagger.h>

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

RecordProcess::RecordProcess() {
  name_ = "";
  record_options_ = "";
  record_pid_ = 0;
  recording_ = false;
}

RecordProcess::RecordProcess(std::string name, std::string record_options) {
  name_ = name;
  record_options_ = record_options;
  record_pid_ = 0;
  recording_ = false;
}

RecordProcess::~RecordProcess() {
}

std::string RecordProcess::getName() {
  return name_;
}

pid_t RecordProcess::getRecordPID() {
  return record_pid_;
}

std::vector<std::string> RecordProcess::getRecordOptionsVector() {
  // Split the options string at the spaces
  std::vector<std::string> options_strings;
  boost::split(options_strings, record_options_, boost::is_any_of(" "));

  std::vector<std::string> rov;

  rov.push_back("rosrun");
  rov.push_back("rosbag");
  rov.push_back("record");

  for (auto it = options_strings.begin(); it != options_strings.end(); ++it) {
    rov.push_back(*it);
  }

  return rov;
}

bool RecordProcess::getRecording() {
  return recording_;
}

void RecordProcess::setName(std::string name) {
  name_ = name;
}

void RecordProcess::setRecording(bool recording) {
  recording_ = recording;
}

void RecordProcess::setRecordPID(pid_t pid) {
  record_pid_ = pid;
}

void RecordProcess::setRecordOptionString(std::string record_options) {
  record_options_ = record_options;
}

bool Bagger::onBagStateSrv(bagger::SetBagState::Request &request, bagger::SetBagState::Response &response) {
  bool success = false;

  // double check that there is an entry for the specified profile in the profile name to options string map
  if (profile_name_to_record_options_map_.find(request.bag_profile_name) != profile_name_to_record_options_map_.end()) {
    // if trying to start a record, make sure that a bag isn't currently being recorded for the specified profile
    if (request.set) {  // request to start the bag
      if (!profile_name_to_record_process_map_[request.bag_profile_name].getRecording()) {
        // Start recording
        pid_t pid_rr = fork();
        if (pid_rr == 0) {
          // Child process - successful fork
          // pass the vector's internal array to execvp
          std::vector<std::string> command_strings =
              profile_name_to_record_process_map_[request.bag_profile_name].getRecordOptionsVector();
          std::vector<char *> commands;

          for (auto it = command_strings.begin(); it != command_strings.end(); ++it) {
            commands.push_back(const_cast<char *>(it->c_str()));
          }

          // add null to end (execvp expects null as last element)
          commands.push_back(NULL);

          // redirect console output to /dev/null
          int dev_null_fd = open("/dev/null", O_WRONLY);
          // suppress console output from the rosbag record subprocess
          dup2(dev_null_fd, 1);
          dup2(dev_null_fd, 2);
          close(dev_null_fd);

          if (execvp(commands[0], &commands[0]) != 0) {
            // just ros error about it, its the child process, so we shouldn't interact with any service / message
            ROS_ERROR("execvp failed! %s", strerror(errno));
          }
        } else if (pid_rr > 0) {
          // parent process - successful fork
          profile_name_to_record_process_map_[request.bag_profile_name].setRecordPID(pid_rr);
          profile_name_to_record_process_map_[request.bag_profile_name].setRecording(true);
          ROS_INFO("successfully started the record.  profile: %s", request.bag_profile_name.c_str());
          success = true;

          // updateBagNames();

        } else {
          // fork failed!
          ROS_ERROR("fork() failed\n");
        }

      } else {
        // already recording, - false
        ROS_WARN("record profile %s already recording", request.bag_profile_name.c_str());
      }
    } else {
      // request to stop the record
      // make sure the requested record is recording
      if (profile_name_to_record_process_map_[request.bag_profile_name].getRecording()) {
        if (kill(profile_name_to_record_process_map_[request.bag_profile_name].getRecordPID(), SIGINT) == 0) {
          ROS_INFO("successfully stopped the record.  profile: %s", request.bag_profile_name.c_str());
          profile_name_to_record_process_map_[request.bag_profile_name].setRecording(false);
          success = true;
        } else {
          ROS_ERROR("kill() when stopping record profile %s failed", request.bag_profile_name.c_str());
        }
      } else {
        // not recording, don't need to stop
        ROS_WARN("record profile %s already not recording", request.bag_profile_name.c_str());
      }
    }
  } else {
    // The requested profile does not exist
    ROS_ERROR("Requested record profile: %s does not exist. Check the configured profile names",
              request.bag_profile_name.c_str());
  }

  response.success = success;

  // Publish the current states
  publishBaggingStates();

  return true;
}

void Bagger::publishBaggingStates() {
  bagger::BaggingState bs_msg;
  bs_msg.header.stamp = ros::Time::now();

  // Sleep for a quarter second to make sure the new bag exists
  ros::Duration(0.25).sleep();

  // First, fill out the profile names and whether or not each one has an active rosbag recording
  // Second, for active profiles fill out the currently recording bag path and name
  for (auto it = profile_name_to_record_process_map_.begin(); it != profile_name_to_record_process_map_.end(); ++it) {
    bs_msg.bag_profile_names.push_back(it->first);
    bs_msg.bagging.push_back(it->second.getRecording());

    if (it->second.getRecording()) {
      // Get the record options for the profile name
      bs_msg.bag_file_names.push_back(getBagNameFromRecordOptions(profile_name_to_record_options_map_[it->first]));
    } else {
      bs_msg.bag_file_names.push_back("");
    }
  }
  bagging_state_publisher_.publish(bs_msg);
}

std::string Bagger::getBagNameFromRecordOptions(std::string record_opts) {
  std::string bag_name = "";
  std::string cwd_string = getCurrentWorkingDirectory();

  if (record_opts.find("-o") != std::string::npos) {
    // -o is the prefix argument.  It means that rosbag record will record bags beginning with the passed
    // prefix and ending with the date time, followed by .bag.  If no directory exists in the passed prefix,
    // The bag will be recorded in the current working directory.

    // Get the passed -o argument.
    size_t start_index = record_opts.find("-o ") + 3;
    size_t next_space_index = record_opts.find(" ", start_index);
    std::string file_path = record_opts.substr(start_index, next_space_index - start_index);

    // If no directory is present in the argument, look up cwd and prepend it, resulting in file path without ".bag"
    if (file_path.find("/") == std::string::npos) {
      file_path = cwd_string + file_path;
    }

    boost::filesystem::path full_file_name(file_path);

    // look in the file's directory for all files that contain the file's prefix
    std::vector<boost::filesystem::path> matches =
        getMatchingFilePathsInDirectory(full_file_name.parent_path(), full_file_name.stem().string());

    if (matches.size() > 0) {
      boost::filesystem::path latest = matches[0];
      for (auto i : matches) {
        std::string s = i.stem().string();
        s.erase(std::remove_if(s.begin(), s.end(), std::not1(std::ptr_fun((int (*)(int)) std::isdigit))), s.end());

        std::string latest_digits(latest.string());
        latest_digits.erase(std::remove_if(latest_digits.begin(), latest_digits.end(),
                                           std::not1(std::ptr_fun((int (*)(int)) std::isdigit))),
                            latest_digits.end());
        if (s > latest_digits) {
          latest = i;
        }
      }
      bag_name = latest.string();
    }
  } else if (record_opts.find("-O") != std::string::npos) {
    // The -O argument means that the bag's name is specified.  If the passed name does not
    // end in .bag, .bag is appended.  If multiple bags are recorded with the same name,
    // when rosbag record stops, it will append the .active bag to the existing .bag file.  If no
    // directory is passed as part of the name, the bag will be recorded in the current working dir

    size_t start_index = record_opts.find("-O ") + 3;
    size_t next_space_index = record_opts.find(" ", start_index);
    std::string file_path = record_opts.substr(start_index, next_space_index - start_index);

    // If the passed arg doesn't end in .bag, append it
    if (file_path.find(".bag") == std::string::npos) {
      file_path += ".bag";
    }

    // If no directory is present in the argument, look up cwd and prepend it
    if (file_path.find("/") == std::string::npos) {
      file_path = cwd_string + file_path;
    }
    bag_name = file_path;

  } else {
    // if nothing is passed, a dated rosbag will be created in the cwd.  Find the most recent rosbag in the cwd
    // without a prefix
    const boost::filesystem::path cwd_path(cwd_string);
    std::vector<boost::filesystem::path> bag_matches = getMatchingFilePathsInDirectory(cwd_path, ".bag.active");

    if (bag_matches.size() > 0) {
      boost::filesystem::path latest("");

      for (auto i : bag_matches) {
        std::string s = i.stem().string();
        if (std::isdigit(s[0])) {
          if (latest.string() == "") {
            latest = i;
          }
          std::string latest_digits(latest.stem().string());
          latest_digits.erase(std::remove_if(latest_digits.begin(), latest_digits.end(),
                                             std::not1(std::ptr_fun((int (*)(int)) std::isdigit))),
                              latest_digits.end());

          s.erase(std::remove_if(s.begin(), s.end(), std::not1(std::ptr_fun((int (*)(int)) std::isdigit))), s.end());

          if (s > latest_digits) {
            latest = i;
          }
        }
      }

      bag_name = latest.string();
    }
  }

  bag_name = removeSuffix(bag_name, ".active");
  return removeSuffix(bag_name, ".bag");
}

std::vector<boost::filesystem::path> Bagger::getMatchingFilePathsInDirectory(const boost::filesystem::path &dir_path,
                                                                             std::string match_string) {
  std::vector<boost::filesystem::path> paths;
  if (exists(dir_path)) {
    for (auto &entry : boost::make_iterator_range(boost::filesystem::directory_iterator(dir_path), {})) {
      if (entry.path().string().find(match_string) != std::string::npos) {
        paths.push_back(entry.path());
      }
    }
  }
  return paths;
}

std::string Bagger::removeSuffix(std::string s, std::string suffix) {
  if (s != suffix && s.size() > suffix.size() && s.substr(s.size() - suffix.size()) == suffix) {
    s = s.substr(0, s.size() - suffix.size());
  }
  return s;
}

std::string Bagger::getCurrentWorkingDirectory() {
  char path[FILENAME_MAX];
  if (!getcwd(path, sizeof(path))) {
    return "";
  }
  return std::string(path) + "/";
}

Bagger::Bagger() {
  node_handle_ = ros::NodeHandlePtr(new ros::NodeHandle("bagger"));

  private_node_handle_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));

  // Publish bag states - latching = true;
  bagging_state_publisher_ = node_handle_->advertise<bagger::BaggingState>("bag_states", 1, true);

  bag_state_service_ = node_handle_->advertiseService("set_bag_state", &Bagger::onBagStateSrv, this);

  // Make sure that there is at least one pair of name and options, then read it in
  if (private_node_handle_->hasParam("profile_name_to_record_options_map")) {
    private_node_handle_->getParam("profile_name_to_record_options_map", profile_name_to_record_options_map_);
  } else {
    ROS_ERROR("Failed to find profile_name_to_bag_options_map in param server - using default");
    profile_name_to_record_options_map_["everything"] = "-a -O /tmp/everything.bag";
  }

  // Go through the profile_name_to_record_options_map and set relevant info in the profile_name_to_record_process_map_
  for (auto it = profile_name_to_record_options_map_.begin(); it != profile_name_to_record_options_map_.end(); ++it) {
    RecordProcess rp(it->first, it->second);
    profile_name_to_record_process_map_[it->first] = rp;
  }

  publishBaggingStates();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bagger");
  Bagger bg;
  ros::spin();
}
