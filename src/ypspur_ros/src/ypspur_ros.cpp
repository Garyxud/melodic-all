/*
 * Copyright (c) 2015-2017, the ypspur_ros authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ypspur_ros/ControlMode.h>
#include <ypspur_ros/DigitalInput.h>
#include <ypspur_ros/DigitalOutput.h>
#include <ypspur_ros/JointPositionControl.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <signal.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <boost/atomic.hpp>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/thread/future.hpp>

#include <exception>
#include <map>
#include <string>
#include <vector>

#include <compatibility.h>

namespace YP
{
#include <ypspur.h>
}  // namespace YP

bool g_shutdown = false;
void sigintHandler(int sig)
{
  g_shutdown = true;
}

class YpspurRosNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::map<std::string, ros::Publisher> pubs_;
  std::map<std::string, ros::Subscriber> subs_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  std::string port_;
  std::string param_file_;
  std::string ypspur_bin_;
  std::map<std::string, std::string> frames_;
  std::map<std::string, double> params_;
  int key_;
  bool simulate_;
  bool simulate_control_;

  double tf_time_offset_;

  pid_t pid_;

  enum OdometryMode
  {
    DIFF,
    NONE
  };
  OdometryMode mode_;
  class JointParams
  {
  public:
    int id_;
    std::string name_;
    double accel_;
    double vel_;
    double angle_ref_;
    double vel_ref_;
    double vel_end_;
    enum control_mode_
    {
      STOP,
      VELOCITY,
      POSITION,
      TRAJECTORY
    };
    control_mode_ control_;
    trajectory_msgs::JointTrajectory cmd_joint_;
  };
  std::vector<JointParams> joints_;
  std::map<std::string, int> joint_name_to_num_;

  class AdParams
  {
  public:
    bool enable_;
    std::string name_;
    double gain_;
    double offset_;
  };
  class DioParams
  {
  public:
    bool enable_;
    std::string name_;
    bool input_;
    bool output_;
  };
  bool digital_input_enable_;
  std::vector<AdParams> ads_;
  std::vector<DioParams> dios_;
  const int ad_num_ = 8;
  unsigned int dio_output_;
  unsigned int dio_dir_;
  unsigned int dio_output_default_;
  unsigned int dio_dir_default_;
  const int dio_num_ = 8;
  std::map<int, ros::Time> dio_revert_;

  int device_error_state_;
  int device_error_state_prev_;
  ros::Time device_error_state_time_;

  geometry_msgs::Twist::ConstPtr cmd_vel_;
  ros::Time cmd_vel_time_;
  ros::Duration cmd_vel_expire_;

  int control_mode_;

  void cbControlMode(const ypspur_ros::ControlMode::ConstPtr& msg)
  {
    control_mode_ = msg->vehicle_control_mode;
    switch (control_mode_)
    {
      case ypspur_ros::ControlMode::OPEN:
        YP::YP_openfree();
        break;
      case ypspur_ros::ControlMode::TORQUE:
        YP::YPSpur_free();
        break;
      case ypspur_ros::ControlMode::VELOCITY:
        break;
    }
  }
  void cbCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
  {
    cmd_vel_ = msg;
    cmd_vel_time_ = ros::Time::now();
    if (control_mode_ == ypspur_ros::ControlMode::VELOCITY)
    {
      YP::YPSpur_vel(msg->linear.x, msg->angular.z);
    }
  }
  void cbJoint(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
  {
    const ros::Time now = ros::Time::now();

    std_msgs::Header header = msg->header;
    if (header.stamp == ros::Time(0))
      header.stamp = now;

    std::map<std::string, trajectory_msgs::JointTrajectory> new_cmd_joints;
    size_t i = 0;
    for (const std::string& name : msg->joint_names)
    {
      trajectory_msgs::JointTrajectory cmd_joint;
      cmd_joint.header = header;
      cmd_joint.joint_names.resize(1);
      cmd_joint.joint_names[0] = name;
      cmd_joint.points.clear();
      std::string err_msg;
      for (auto& cmd : msg->points)
      {
        if (header.stamp + cmd.time_from_start < now)
        {
          ROS_ERROR(
              "Ignored outdated JointTrajectory command "
              "(joint: %s, now: %0.6lf, stamp: %0.6lf, time_from_start: %0.6lf)",
              name.c_str(), now.toSec(), header.stamp.toSec(), cmd.time_from_start.toSec());
          break;
        }

        trajectory_msgs::JointTrajectoryPoint p;
        p.time_from_start = cmd.time_from_start;
        p.positions.resize(1);
        p.velocities.resize(1);
        if (cmd.velocities.size() <= i)
          p.velocities[0] = 0.0;
        else
          p.velocities[0] = cmd.velocities[i];
        p.positions[0] = cmd.positions[i];

        cmd_joint.points.push_back(p);
      }
      i++;

      if (cmd_joint.points.size() != msg->points.size())
        return;

      new_cmd_joints[name] = cmd_joint;
    }
    // Apply if all JointTrajectoryPoints are valid
    for (auto& new_cmd_joint : new_cmd_joints)
    {
      const int joint_num = joint_name_to_num_[new_cmd_joint.first];
      joints_[joint_num].control_ = JointParams::TRAJECTORY;
      joints_[joint_num].cmd_joint_ = new_cmd_joint.second;
    }
  }
  void cbSetVel(const std_msgs::Float32::ConstPtr& msg, int num)
  {
    // printf("set_vel %d %d %f\n", num, joints_[num].id_, msg->data);
    joints_[num].vel_ = msg->data;
    YP::YP_set_joint_vel(joints_[num].id_, joints_[num].vel_);
  }
  void cbSetAccel(const std_msgs::Float32::ConstPtr& msg, int num)
  {
    // printf("set_accel %d %d %f\n", num, joints_[num].id_, msg->data);
    joints_[num].accel_ = msg->data;
    YP::YP_set_joint_accel(joints_[num].id_, joints_[num].accel_);
  }
  void cbVel(const std_msgs::Float32::ConstPtr& msg, int num)
  {
    // printf("vel_ %d %d %f\n", num, joints_[num].id_, msg->data);
    joints_[num].vel_ref_ = msg->data;
    joints_[num].control_ = JointParams::VELOCITY;
    YP::YP_joint_vel(joints_[num].id_, joints_[num].vel_ref_);
  }
  void cbAngle(const std_msgs::Float32::ConstPtr& msg, int num)
  {
    joints_[num].angle_ref_ = msg->data;
    joints_[num].control_ = JointParams::POSITION;
    YP::YP_joint_ang(joints_[num].id_, joints_[num].angle_ref_);
  }
  void cbJointPosition(const ypspur_ros::JointPositionControl::ConstPtr& msg)
  {
    int i = 0;
    for (auto& name : msg->joint_names)
    {
      if (joint_name_to_num_.find(name) == joint_name_to_num_.end())
      {
        ROS_ERROR("Unknown joint name '%s'", name.c_str());
        continue;
      }
      int num = joint_name_to_num_[name];
      // printf("%s %d %d  %f", name.c_str(), num, joints_[num].id_, msg->positions[i]);
      joints_[num].vel_ = msg->velocities[i];
      joints_[num].accel_ = msg->accelerations[i];
      joints_[num].angle_ref_ = msg->positions[i];
      joints_[num].control_ = JointParams::POSITION;

      YP::YP_set_joint_vel(joints_[num].id_, joints_[num].vel_);
      YP::YP_set_joint_accel(joints_[num].id_, joints_[num].accel_);
      YP::YP_joint_ang(joints_[num].id_, joints_[num].angle_ref_);
      i++;
    }
  }

  void cbDigitalOutput(const ypspur_ros::DigitalOutput::ConstPtr& msg, int id_)
  {
    const auto dio_output_prev = dio_output_;
    const auto dio_dir_prev = dio_dir_;
    const unsigned int mask = 1 << id_;

    switch (msg->output)
    {
      case ypspur_ros::DigitalOutput::HIGH_IMPEDANCE:
        dio_output_ &= ~mask;
        dio_dir_ &= ~mask;
        break;
      case ypspur_ros::DigitalOutput::LOW:
        dio_output_ &= ~mask;
        dio_dir_ |= mask;
        break;
      case ypspur_ros::DigitalOutput::HIGH:
        dio_output_ |= mask;
        dio_dir_ |= mask;
        break;
      case ypspur_ros::DigitalOutput::PULL_UP:
        dio_output_ |= mask;
        dio_dir_ &= ~mask;
        break;
      case ypspur_ros::DigitalOutput::PULL_DOWN:
        ROS_ERROR("Digital IO pull down is not supported on this system");
        break;
    }
    if (dio_output_ != dio_output_prev)
      YP::YP_set_io_data(dio_output_);
    if (dio_dir_ != dio_dir_prev)
      YP::YP_set_io_dir(dio_dir_);

    if (msg->toggle_time > ros::Duration(0))
    {
      dio_revert_[id_] = ros::Time::now() + msg->toggle_time;
    }
  }
  void revertDigitalOutput(int id_)
  {
    const auto dio_output_prev = dio_output_;
    const auto dio_dir_prev = dio_dir_;
    const unsigned int mask = 1 << id_;

    dio_output_ &= ~mask;
    dio_output_ |= dio_output_default_ & mask;
    dio_dir_ &= ~mask;
    dio_dir_ |= dio_output_default_ & mask;

    if (dio_output_ != dio_output_prev)
      YP::YP_set_io_data(dio_output_);
    if (dio_dir_ != dio_dir_prev)
      YP::YP_set_io_dir(dio_dir_);

    dio_revert_[id_] = ros::Time(0);
  }
  void updateDiagnostics(const ros::Time& now, const bool connection_down = false)
  {
    const int connection_error = connection_down ? 1 : YP::YP_get_error_state();
    double t = 0;

    int err = 0;
    if (!connection_error)
      t = YP::YP_get_device_error_state(0, &err);
    device_error_state_ |= err;

    if (device_error_state_time_ + ros::Duration(1.0) < now || connection_down ||
        device_error_state_ != device_error_state_prev_)
    {
      device_error_state_time_ = now;
      device_error_state_prev_ = device_error_state_;

      diagnostic_msgs::DiagnosticArray msg;
      msg.header.stamp = now;
      msg.status.resize(1);
      msg.status[0].name = "YP-Spur Motor Controller";
      msg.status[0].hardware_id = "ipc-key" + std::to_string(key_);
      if (device_error_state_ == 0 && connection_error == 0)
      {
        if (t == 0)
        {
          msg.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
          msg.status[0].message = "Motor controller doesn't "
                                  "provide device error state.";
        }
        else
        {
          if (ros::Time(t) < now - ros::Duration(1.0))
          {
            msg.status[0].level = diagnostic_msgs::DiagnosticStatus::ERROR;
            msg.status[0].message = "Motor controller doesn't "
                                    "update latest device error state.";
          }
          else
          {
            msg.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
            msg.status[0].message = "Motor controller is running without error.";
          }
        }
      }
      else
      {
        msg.status[0].level = diagnostic_msgs::DiagnosticStatus::ERROR;
        if (connection_error)
          msg.status[0].message +=
              "Connection to ypspur-coordinator is down.";
        if (device_error_state_)
          msg.status[0].message +=
              std::string((msg.status[0].message.size() > 0 ? " " : "")) +
              "Motor controller reported error id " +
              std::to_string(device_error_state_) + ".";
      }
      msg.status[0].values.resize(2);
      msg.status[0].values[0].key = "connection_error";
      msg.status[0].values[0].value = std::to_string(connection_error);
      msg.status[0].values[1].key = "device_error";
      msg.status[0].values[1].value = std::to_string(device_error_state_);

      pubs_["diag"].publish(msg);
      device_error_state_ = 0;
    }
  }

public:
  YpspurRosNode()
    : nh_()
    , pnh_("~")
    , device_error_state_(0)
    , device_error_state_prev_(0)
    , device_error_state_time_(0)
  {
    compat::checkCompatMode();

    pnh_.param("port", port_, std::string("/dev/ttyACM0"));
    pnh_.param("ipc_key", key_, 28741);
    pnh_.param("simulate", simulate_, false);
    pnh_.param("simulate_control", simulate_control_, false);
    if (simulate_control_)
      simulate_ = true;
    pnh_.param("ypspur_bin", ypspur_bin_, std::string("ypspur-coordinator"));
    pnh_.param("param_file", param_file_, std::string(""));
    pnh_.param("tf_time_offset", tf_time_offset_, 0.0);

    double cmd_vel_expire_s;
    pnh_.param("cmd_vel_expire", cmd_vel_expire_s, -1.0);
    cmd_vel_expire_ = ros::Duration(cmd_vel_expire_s);

    std::string ad_mask("");
    ads_.resize(ad_num_);
    for (int i = 0; i < ad_num_; i++)
    {
      pnh_.param(std::string("ad") + std::to_string(i) + std::string("_enable"),
                 ads_[i].enable_, false);
      pnh_.param(std::string("ad") + std::to_string(i) + std::string("_name"),
                 ads_[i].name_, std::string("ad") + std::to_string(i));
      pnh_.param(std::string("ad") + std::to_string(i) + std::string("_gain"),
                 ads_[i].gain_, 1.0);
      pnh_.param(std::string("ad") + std::to_string(i) + std::string("_offset"),
                 ads_[i].offset_, 0.0);
      ad_mask = (ads_[i].enable_ ? std::string("1") : std::string("0")) + ad_mask;
      pubs_["ad/" + ads_[i].name_] = compat::advertise<std_msgs::Float32>(
          nh_, "ad/" + ads_[i].name_,
          pnh_, "ad/" + ads_[i].name_, 1);
    }
    digital_input_enable_ = false;
    dio_output_default_ = 0;
    dio_dir_default_ = 0;
    dios_.resize(dio_num_);
    for (int i = 0; i < dio_num_; i++)
    {
      DioParams param;
      pnh_.param(std::string("dio") + std::to_string(i) + std::string("_enable"),
                 param.enable_, false);
      if (param.enable_)
      {
        pnh_.param(std::string("dio") + std::to_string(i) + std::string("_name"),
                   param.name_, std::string(std::string("dio") + std::to_string(i)));

        pnh_.param(std::string("dio") + std::to_string(i) + std::string("_output"),
                   param.output_, true);
        pnh_.param(std::string("dio") + std::to_string(i) + std::string("_input"),
                   param.input_, false);

        if (param.output_)
        {
          subs_[param.name_] = compat::subscribe<ypspur_ros::DigitalOutput>(
              nh_, param.name_,
              pnh_, param.name_, 1,
              boost::bind(&YpspurRosNode::cbDigitalOutput, this, _1, i));
        }

        std::string output_default;
        pnh_.param(std::string("dio") + std::to_string(i) + std::string("_default"),
                   output_default, std::string("HIGH_IMPEDANCE"));
        if (output_default.compare("HIGH_IMPEDANCE") == 0)
        {
        }
        else if (output_default.compare("LOW") == 0)
        {
          dio_dir_default_ |= 1 << i;
        }
        else if (output_default.compare("HIGH") == 0)
        {
          dio_dir_default_ |= 1 << i;
          dio_output_default_ |= 1 << i;
        }
        else if (output_default.compare("PULL_UP") == 0)
        {
          dio_output_default_ |= 1 << i;
        }
        else if (output_default.compare("PULL_DOWN") == 0)
        {
          ROS_ERROR("Digital IO pull down is not supported on this system");
        }
        if (param.input_)
          digital_input_enable_ = true;
      }
      dios_[i] = param;
    }
    dio_output_ = dio_output_default_;
    dio_dir_ = dio_dir_default_;
    if (digital_input_enable_)
    {
      pubs_["din"] = compat::advertise<ypspur_ros::DigitalInput>(
          nh_, "digital_input",
          pnh_, "digital_input", 2);
    }

    pnh_.param("odom_id", frames_["odom"], std::string("odom"));
    pnh_.param("base_link_id", frames_["base_link"], std::string("base_link"));
    pnh_.param("origin_id", frames_["origin"], std::string(""));
    pnh_.param("hz", params_["hz"], 200.0);

    std::string mode_name;
    pnh_.param("OdometryMode", mode_name, std::string("diff"));
    if (mode_name.compare("diff") == 0)
    {
      mode_ = DIFF;
      pubs_["wrench"] = compat::advertise<geometry_msgs::WrenchStamped>(
          nh_, "wrench",
          pnh_, "wrench", 1);
      pubs_["odom"] = compat::advertise<nav_msgs::Odometry>(
          nh_, "odom",
          pnh_, "odom", 1);
      subs_["cmd_vel"] = compat::subscribe(
          nh_, "cmd_vel",
          pnh_, "cmd_vel", 1, &YpspurRosNode::cbCmdVel, this);
    }
    else if (mode_name.compare("none") == 0)
    {
    }
    else
    {
      throw(std::runtime_error("unknown mode: " + mode_name));
    }

    int max_joint_id;
    bool separated_joint;
    pnh_.param("max_joint_id", max_joint_id, 32);
    pnh_.param("separated_joint_control", separated_joint, false);
    int num = 0;
    for (int i = 0; i < max_joint_id; i++)
    {
      std::string name;
      name = std::string("joint") + std::to_string(i);
      if (pnh_.hasParam(name + std::string("_enable")))
      {
        bool en;
        pnh_.param(name + std::string("_enable"), en, false);
        if (en)
        {
          JointParams jp;
          jp.id_ = i;
          pnh_.param(name + std::string("_name"), jp.name_, name);
          pnh_.param(name + std::string("_accel"), jp.accel_, 3.14);
          joint_name_to_num_[jp.name_] = num;
          joints_.push_back(jp);
          // printf("%s %d %d", jp.name_.c_str(), jp.id_, joint_name_to_num_[jp.name_]);
          if (separated_joint)
          {
            subs_[jp.name_ + std::string("_setVel")] = compat::subscribe<std_msgs::Float32>(
                nh_, jp.name_ + std::string("/set_vel"),
                pnh_, jp.name_ + std::string("_setVel"), 1,
                boost::bind(&YpspurRosNode::cbSetVel, this, _1, num));
            subs_[jp.name_ + std::string("_setAccel")] = compat::subscribe<std_msgs::Float32>(
                nh_, jp.name_ + std::string("/set_accel"),
                pnh_, jp.name_ + std::string("_setAccel"), 1,
                boost::bind(&YpspurRosNode::cbSetAccel, this, _1, num));
            subs_[jp.name_ + std::string("_vel")] = compat::subscribe<std_msgs::Float32>(
                nh_, jp.name_ + std::string("/vel"),
                pnh_, jp.name_ + std::string("_vel"), 1,
                boost::bind(&YpspurRosNode::cbVel, this, _1, num));
            subs_[jp.name_ + std::string("_pos")] = compat::subscribe<std_msgs::Float32>(
                nh_, jp.name_ + std::string("/pos"),
                pnh_, jp.name_ + std::string("_pos"), 1,
                boost::bind(&YpspurRosNode::cbAngle, this, _1, num));
          }
          subs_[std::string("joint_position")] = compat::subscribe(
              nh_, std::string("joint_position"),
              pnh_, std::string("joint_position"), 1, &YpspurRosNode::cbJointPosition, this);
          num++;
        }
      }
    }
    if (joints_.size() > 0)
    {
      pubs_["joint"] = compat::advertise<sensor_msgs::JointState>(
          nh_, "joint_states",
          pnh_, "joint", 2);
      subs_["joint"] = compat::subscribe(
          nh_, "joint_trajectory",
          pnh_, "cmd_joint", joints_.size() * 2, &YpspurRosNode::cbJoint, this);
    }
    subs_["control_mode"] = compat::subscribe(
        nh_, "control_mode",
        pnh_, "control_mode", 1, &YpspurRosNode::cbControlMode, this);
    control_mode_ = ypspur_ros::ControlMode::VELOCITY;

    pubs_["diag"] = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        "/diagnostics", 1);

    pid_ = 0;
    for (int i = 0; i < 2; i++)
    {
      if (i > 0 || YP::YPSpur_initex(key_) < 0)
      {
        std::vector<std::string> args =
            {
              ypspur_bin_,
              "-d", port_,
              "--admask", ad_mask,
              "--msq-key", std::to_string(key_)
            };
        if (digital_input_enable_)
          args.push_back(std::string("--enable-get-digital-io"));
        if (simulate_)
          args.push_back(std::string("--without-device"));
        if (param_file_.size() > 0)
        {
          args.push_back(std::string("-p"));
          args.push_back(param_file_);
        }

        char** argv = new char*[args.size() + 1];
        for (unsigned int i = 0; i < args.size(); i++)
        {
          argv[i] = new char[args[i].size() + 1];
          memcpy(argv[i], args[i].c_str(), args[i].size());
          argv[i][args[i].size()] = 0;
        }
        argv[args.size()] = nullptr;

        int msq = msgget(key_, 0666 | IPC_CREAT);
        msgctl(msq, IPC_RMID, nullptr);

        ROS_WARN("launching ypspur-coordinator");
        pid_ = fork();
        if (pid_ == -1)
        {
          const int err = errno;
          throw(std::runtime_error(std::string("failed to fork process: ") + strerror(err)));
        }
        else if (pid_ == 0)
        {
          execvp(ypspur_bin_.c_str(), argv);
          throw(std::runtime_error("failed to start ypspur-coordinator"));
        }

        for (unsigned int i = 0; i < args.size(); i++)
          delete argv[i];
        delete argv;

        for (int i = 4; i >= 0; i--)
        {
          int status;
          if (waitpid(pid_, &status, WNOHANG) == pid_)
          {
            throw(std::runtime_error("ypspur-coordinator dead immediately"));
          }
          else if (i == 0)
          {
            throw(std::runtime_error("failed to init libypspur"));
          }
          ros::WallDuration(1).sleep();
          if (YP::YPSpur_initex(key_) >= 0)
            break;
        }
      }
      double ret;
      boost::atomic<bool> done(false);
      auto get_vel_thread = [&ret, &done]
      {
        double test_v, test_w;
        ret = YP::YPSpur_get_vel(&test_v, &test_w);
        done = true;
      };
      boost::thread spur_test = boost::thread(get_vel_thread);
      ros::WallDuration(0.1).sleep();
      if (!done)
      {
        // There is no way to kill thread safely in C++11
        // So, just leave it detached.
        spur_test.detach();
        ROS_WARN("ypspur-coordinator seems to be down - launching");
        continue;
      }
      spur_test.join();
      if (ret < 0)
      {
        ROS_WARN("ypspur-coordinator returns error - launching");
        continue;
      }
      ROS_WARN("ypspur-coordinator launched");
      break;
    }

    ROS_INFO("ypspur-coordinator conneceted");
    signal(SIGINT, sigintHandler);

    YP::YP_get_parameter(YP::YP_PARAM_MAX_VEL, &params_["vel"]);
    YP::YP_get_parameter(YP::YP_PARAM_MAX_ACC_V, &params_["acc"]);
    YP::YP_get_parameter(YP::YP_PARAM_MAX_W, &params_["angvel"]);
    YP::YP_get_parameter(YP::YP_PARAM_MAX_ACC_W, &params_["angacc"]);

    if (!pnh_.hasParam("vel"))
      ROS_WARN("default \"vel\" %0.3f used", (float)params_["vel"]);
    if (!pnh_.hasParam("acc"))
      ROS_WARN("default \"acc\" %0.3f used", (float)params_["acc"]);
    if (!pnh_.hasParam("angvel"))
      ROS_WARN("default \"angvel\" %0.3f used", (float)params_["angvel"]);
    if (!pnh_.hasParam("angacc"))
      ROS_WARN("default \"angacc\" %0.3f used", (float)params_["angacc"]);

    pnh_.param("vel", params_["vel"], params_["vel"]);
    pnh_.param("acc", params_["acc"], params_["acc"]);
    pnh_.param("angvel", params_["angvel"], params_["angvel"]);
    pnh_.param("angacc", params_["angacc"], params_["angacc"]);

    YP::YPSpur_set_vel(params_["vel"]);
    YP::YPSpur_set_accel(params_["acc"]);
    YP::YPSpur_set_angvel(params_["angvel"]);
    YP::YPSpur_set_angaccel(params_["angacc"]);

    YP::YP_set_io_data(dio_output_);
    YP::YP_set_io_dir(dio_dir_);
  }
  ~YpspurRosNode()
  {
    // Kill ypspur-coordinator if the communication is still active.
    if (pid_ > 0 && YP::YP_get_error_state() == 0)
    {
      ROS_INFO("killing ypspur-coordinator (%d)", (int)pid_);
      kill(pid_, SIGINT);
      int status;
      waitpid(pid_, &status, 0);
      ROS_INFO("ypspur-coordinator is killed (status: %d)", status);
    }
  }
  bool spin()
  {
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = frames_["odom"];
    odom_trans.child_frame_id = frames_["base_link"];

    nav_msgs::Odometry odom;
    geometry_msgs::WrenchStamped wrench;
    odom.header.frame_id = frames_["odom"];
    odom.child_frame_id = frames_["base_link"];
    wrench.header.frame_id = frames_["base_link"];

    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;

    std::map<int, geometry_msgs::TransformStamped> joint_trans;
    sensor_msgs::JointState joint;
    if (joints_.size() > 0)
    {
      joint.header.frame_id = std::string("");
      joint.velocity.resize(joints_.size());
      joint.position.resize(joints_.size());
      joint.effort.resize(joints_.size());
      for (auto& j : joints_)
        joint.name.push_back(j.name_);

      for (unsigned int i = 0; i < joints_.size(); i++)
      {
        joint_trans[i].header.frame_id = joints_[i].name_ + std::string("_in");
        joint_trans[i].child_frame_id = joints_[i].name_ + std::string("_out");
        joint.velocity[i] = 0;
        joint.position[i] = 0;
        joint.effort[i] = 0;
      }
    }

    ROS_INFO("ypspur_ros main loop started");
    ros::Rate loop(params_["hz"]);
    while (!g_shutdown)
    {
      const auto now = ros::Time::now();
      const float dt = 1.0 / params_["hz"];

      if (cmd_vel_ && cmd_vel_expire_ > ros::Duration(0))
      {
        if (cmd_vel_time_ + cmd_vel_expire_ < now)
        {
          // cmd_vel is too old and expired
          cmd_vel_ = nullptr;
          if (control_mode_ == ypspur_ros::ControlMode::VELOCITY)
            YP::YPSpur_vel(0.0, 0.0);
        }
      }

      if (mode_ == DIFF)
      {
        double x, y, yaw, v(0), w(0);
        double t;

        if (!simulate_control_)
        {
          t = YP::YPSpur_get_pos(YP::CS_BS, &x, &y, &yaw);
          if (t <= 0.0)
            break;
          YP::YPSpur_get_vel(&v, &w);
        }
        else
        {
          t = ros::Time::now().toSec();
          if (cmd_vel_)
          {
            v = cmd_vel_->linear.x;
            w = cmd_vel_->angular.z;
          }
          yaw = tf::getYaw(odom.pose.pose.orientation) + dt * w;
          x = odom.pose.pose.position.x + dt * v * cosf(yaw);
          y = odom.pose.pose.position.y + dt * v * sinf(yaw);
        }

        odom.header.stamp = ros::Time(t);
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        odom.twist.twist.linear.x = v;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = w;
        pubs_["odom"].publish(odom);

        odom_trans.header.stamp = ros::Time(t) + ros::Duration(tf_time_offset_);
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
        tf_broadcaster_.sendTransform(odom_trans);

        if (!simulate_control_)
        {
          t = YP::YPSpur_get_force(&wrench.wrench.force.x, &wrench.wrench.torque.z);
          if (t <= 0.0)
            break;
        }
        wrench.header.stamp = ros::Time(t);
        wrench.wrench.force.y = 0;
        wrench.wrench.force.z = 0;
        wrench.wrench.torque.x = 0;
        wrench.wrench.torque.y = 0;
        pubs_["wrench"].publish(wrench);

        if (frames_["origin"].length() > 0)
        {
          try
          {
            tf::StampedTransform transform;
            tf_listener_.lookupTransform(
                frames_["origin"], frames_["base_link"],
                ros::Time(0), transform);

            tfScalar yaw, pitch, roll;
            transform.getBasis().getEulerYPR(yaw, pitch, roll);
            YP::YPSpur_adjust_pos(YP::CS_GL, transform.getOrigin().x(),
                                  transform.getOrigin().y(),
                                  yaw);
          }
          catch (tf::TransformException ex)
          {
            ROS_ERROR("Failed to feedback localization result to YP-Spur (%s)", ex.what());
          }
        }
      }
      if (joints_.size() > 0)
      {
        double t;
        if (!simulate_control_)
        {
          t = -1.0;
          while (t < 0.0)
          {
            int i = 0;
            for (auto& j : joints_)
            {
              const double t0 = YP::YP_get_joint_ang(j.id_, &joint.position[i]);
              const double t1 = YP::YP_get_joint_vel(j.id_, &joint.velocity[i]);
              const double t2 = YP::YP_get_joint_torque(j.id_, &joint.effort[i]);

              if (t0 != t1 || t1 != t2)
              {
                // Retry if updated during this joint
                t = -1.0;
                break;
              }
              if (t < 0.0)
              {
                t = t0;
              }
              else if (t != t0)
              {
                // Retry if updated during loops
                t = -1.0;
                break;
              }
              i++;
            }
          }
          if (t <= 0.0)
            break;
          joint.header.stamp = ros::Time(t);
        }
        else
        {
          t = ros::Time::now().toSec();
          for (unsigned int i = 0; i < joints_.size(); i++)
          {
            auto vel_prev = joint.velocity[i];
            switch (joints_[i].control_)
            {
              case JointParams::STOP:
                break;
              case JointParams::TRAJECTORY:
              case JointParams::POSITION:
              case JointParams::VELOCITY:
                switch (joints_[i].control_)
                {
                  case JointParams::POSITION:
                  {
                    float position_err = joints_[i].angle_ref_ - joint.position[i];
                    joints_[i].vel_ref_ = sqrtf(2.0 * joints_[i].accel_ * fabs(position_err));
                    if (joints_[i].vel_ref_ > joints_[i].vel_)
                      joints_[i].vel_ref_ = joints_[i].vel_;
                    if (position_err < 0)
                      joints_[i].vel_ref_ = -joints_[i].vel_ref_;
                  }
                  break;
                  case JointParams::TRAJECTORY:
                  {
                    float position_err = joints_[i].angle_ref_ - joint.position[i];
                    float v_sq = joints_[i].vel_end_ * joints_[i].vel_end_ + 2.0 * joints_[i].accel_ * position_err;
                    joints_[i].vel_ref_ = sqrtf(fabs(v_sq));

                    float vel_max;
                    if (fabs(joints_[i].vel_) < fabs(joints_[i].vel_end_))
                    {
                      if (fabs(position_err) <
                          (joints_[i].vel_end_ * joints_[i].vel_end_ - joints_[i].vel_ * joints_[i].vel_) /
                              (2.0 * joints_[i].accel_))
                        vel_max = fabs(joints_[i].vel_end_);
                      else
                        vel_max = joints_[i].vel_;
                    }
                    else
                      vel_max = joints_[i].vel_;

                    if (joints_[i].vel_ref_ > vel_max)
                      joints_[i].vel_ref_ = vel_max;
                    if (position_err < 0)
                      joints_[i].vel_ref_ = -joints_[i].vel_ref_;
                  }
                  break;
                  default:
                    break;
                }
                joint.velocity[i] = joints_[i].vel_ref_;
                if (joint.velocity[i] < vel_prev - dt * joints_[i].accel_)
                {
                  joint.velocity[i] = vel_prev - dt * joints_[i].accel_;
                }
                else if (joint.velocity[i] > vel_prev + dt * joints_[i].accel_)
                {
                  joint.velocity[i] = vel_prev + dt * joints_[i].accel_;
                }
                joint.position[i] += joint.velocity[i] * dt;
                break;
            }
          }
          joint.header.stamp = ros::Time(t);
        }
        pubs_["joint"].publish(joint);

        for (unsigned int i = 0; i < joints_.size(); i++)
        {
          joint_trans[i].transform.rotation =
              tf::createQuaternionMsgFromYaw(joint.position[i]);
          joint_trans[i].header.stamp = ros::Time(t) + ros::Duration(tf_time_offset_);
          tf_broadcaster_.sendTransform(joint_trans[i]);
        }

        for (unsigned int jid = 0; jid < joints_.size(); jid++)
        {
          if (joints_[jid].control_ != JointParams::TRAJECTORY)
            continue;

          auto& cmd_joint_ = joints_[jid].cmd_joint_;
          auto t = now - cmd_joint_.header.stamp;
          if (t < ros::Duration(0))
            continue;

          bool done = true;
          for (auto& cmd : cmd_joint_.points)
          {
            if (cmd.time_from_start < ros::Duration(0))
              continue;
            if (now > cmd_joint_.header.stamp + cmd.time_from_start)
              continue;
            done = false;

            double ang_err = cmd.positions[0] - joint.position[jid];
            double& vel_end_ = cmd.velocities[0];
            double& vel_start = joint.velocity[jid];
            auto t_left = cmd.time_from_start - t;

            double v;
            double v_min;
            bool v_found = true;
            while (true)
            {
              // ROS_INFO("st: %0.3f, en: %0.3f, err: %0.3f, t: %0.3f",
              //          vel_start, vel_end_, ang_err, t_left.toSec());
              int s;
              if (vel_end_ > vel_start)
                s = 1;
              else
                s = -1;
              v = (s * (vel_start + vel_end_) * (vel_start - vel_end_) +
                   ang_err * joints_[jid].accel_ * 2.0) /
                  (2.0 * s * (vel_start - vel_end_) + joints_[jid].accel_ * 2.0 * (t_left.toSec()));

              double err_deacc;
              err_deacc = fabs(vel_end_ * vel_end_ - v * v) / (joints_[jid].accel_ * 2.0);
              // ROS_INFO("v+-: %0.3f", v);
              v_min = fabs(v);
              if ((vel_start * s <= v * s || err_deacc >= fabs(ang_err)) &&
                  v * s <= vel_end_ * s)
                break;

              v_min = DBL_MAX;

              auto vf = [](const double& st, const double& en,
                           const double& acc, const double& err, const double& t,
                           const int& sig, const int& sol, double& ret)
              {
                double sq;
                sq = -4.0 * st * st +
                     8.0 * st * en -
                     4.0 * en * en +
                     4.0 * sig * acc * 2 * (t * (st + en) - 2.0 * err) +
                     4.0 * acc * acc * t * t;
                if (sq < 0)
                  return false;

                ret = (2.0 * sig * (st + en) + 2.0 * acc * t + sol * sqrt(sq)) / (4.0 * sig);

                return true;
              };

              v_found = false;

              if (vf(vel_start, vel_end_, joints_[jid].accel_, ang_err, t_left.toSec(),
                     1, 1, v))
              {
                // ROS_INFO("v++: sol+ %0.3f", v);
                if (v >= vel_start && v >= vel_end_)
                {
                  if (v_min > fabs(v))
                    v_min = fabs(v);
                  v_found = true;
                }
              }
              if (vf(vel_start, vel_end_, joints_[jid].accel_, ang_err, t_left.toSec(),
                     -1, 1, v))
              {
                // ROS_INFO("v--: sol+ %0.3f", v);
                if (v <= vel_start && v <= vel_end_)
                {
                  if (v_min > fabs(v))
                    v_min = fabs(v);
                  v_found = true;
                }
              }
              if (vf(vel_start, vel_end_, joints_[jid].accel_, ang_err, t_left.toSec(),
                     1, -1, v))
              {
                // ROS_INFO("v++: sol- %0.3f", v);
                if (v >= vel_start && v >= vel_end_)
                {
                  if (v_min > fabs(v))
                    v_min = fabs(v);
                  v_found = true;
                }
              }
              if (vf(vel_start, vel_end_, joints_[jid].accel_, ang_err, t_left.toSec(),
                     -1, -1, v))
              {
                // ROS_INFO("v--: sol- %0.3f", v);
                if (v <= vel_start && v <= vel_end_)
                {
                  if (v_min > fabs(v))
                    v_min = fabs(v);
                  v_found = true;
                }
              }
              break;
            }
            if (v_found)
            {
              // ROS_INFO("v: %0.3f", v_min);
              joints_[jid].angle_ref_ = cmd.positions[0];
              joints_[jid].vel_end_ = vel_end_;
              joints_[jid].vel_ = v_min;

              YP::YP_set_joint_vel(joints_[jid].id_, v_min);
              YP::YP_set_joint_accel(joints_[jid].id_, joints_[jid].accel_);
              YP::YP_joint_ang_vel(joints_[jid].id_, cmd.positions[0], vel_end_);
            }
            else
            {
              ROS_ERROR("Impossible trajectory given");
            }
            break;
          }

          if (done)
          {
            if ((joints_[jid].vel_end_ > 0.0 &&
                 joints_[jid].angle_ref_ > joint.position[jid] &&
                 joints_[jid].angle_ref_ < joint.position[jid] + joints_[jid].vel_ref_ * dt) ||
                (joints_[jid].vel_end_ < 0.0 &&
                 joints_[jid].angle_ref_ < joint.position[jid] &&
                 joints_[jid].angle_ref_ > joint.position[jid] + joints_[jid].vel_ref_ * dt))
            {
              joints_[jid].control_ = JointParams::VELOCITY;
              joints_[jid].vel_ref_ = joints_[jid].vel_end_;
            }
          }
        }
      }

      for (int i = 0; i < ad_num_; i++)
      {
        if (ads_[i].enable_)
        {
          std_msgs::Float32 ad;
          ad.data = YP::YP_get_ad_value(i) * ads_[i].gain_ + ads_[i].offset_;
          pubs_["ad/" + ads_[i].name_].publish(ad);
        }
      }

      if (digital_input_enable_)
      {
        ypspur_ros::DigitalInput din;

        din.header.stamp = ros::Time::now();
        int in = YP::YP_get_ad_value(15);
        for (int i = 0; i < dio_num_; i++)
        {
          if (!dios_[i].enable_)
            continue;
          din.name.push_back(dios_[i].name_);
          if (in & (1 << i))
            din.state.push_back(true);
          else
            din.state.push_back(false);
        }
        pubs_["din"].publish(din);
      }

      for (int i = 0; i < dio_num_; i++)
      {
        if (dio_revert_[i] != ros::Time(0))
        {
          if (dio_revert_[i] < now)
          {
            revertDigitalOutput(i);
          }
        }
      }
      updateDiagnostics(now);

      if (YP::YP_get_error_state())
        break;

      ros::spinOnce();
      loop.sleep();

      int status;
      if (waitpid(pid_, &status, WNOHANG) == pid_)
      {
        if (WIFEXITED(status))
        {
          ROS_ERROR("ypspur-coordinator exited");
        }
        else
        {
          if (WIFSTOPPED(status))
          {
            ROS_ERROR("ypspur-coordinator dead with signal %d",
                      WSTOPSIG(status));
          }
          else
          {
            ROS_ERROR("ypspur-coordinator dead");
          }
          updateDiagnostics(now, true);
        }
        break;
      }
    }
    ROS_INFO("ypspur_ros main loop terminated");

    if (YP::YP_get_error_state())
    {
      ROS_ERROR("ypspur-coordinator is not active");
      return false;
    }
    return true;
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ypspur_ros");
  ros::NodeHandle nh;

  int ret = 0;

  try
  {
    YpspurRosNode yr;
    if (!yr.spin())
      ret = 1;
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR("%s", e.what());
    ret = 1;
  }

  ros::WallDuration(0.1).sleep();
  return ret;
}
