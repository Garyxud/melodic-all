/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018-2019, Dataspeed Inc.
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
 *   * Neither the name of Dataspeed Inc. nor the names of its
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
 *********************************************************************/

#include "JoystickDemo.h"

JoystickDemo::JoystickDemo(ros::NodeHandle &node, ros::NodeHandle &priv_nh) : counter_(0)
{
  joy_.axes.resize(AXIS_COUNT_X, 0);
  joy_.buttons.resize(BTN_COUNT_X, 0);

  brake_ = true;
  throttle_ = true;
  steer_ = true;
  shift_ = true;
  signal_ = true;
  priv_nh.getParam("brake", brake_);
  priv_nh.getParam("throttle", throttle_);
  priv_nh.getParam("steer", steer_);
  priv_nh.getParam("shift", shift_);
  priv_nh.getParam("signal", signal_);

  brake_gain_ = 1.0;
  throttle_gain_ = 1.0;
  priv_nh.getParam("brake_gain", brake_gain_);
  priv_nh.getParam("throttle_gain", throttle_gain_);
  brake_gain_    = std::min(std::max(brake_gain_,    (float)0), (float)1);
  throttle_gain_ = std::min(std::max(throttle_gain_, (float)0), (float)1);

  ignore_ = false;
  enable_ = true;
  count_ = false;
  strq_ = false;
  svel_ = 0.0;
  last_steering_filt_output_ = 0.0;
  priv_nh.getParam("ignore", ignore_);
  priv_nh.getParam("enable", enable_);
  priv_nh.getParam("count", count_);
  priv_nh.getParam("strq", strq_);
  priv_nh.getParam("svel", svel_);

  sub_joy_ = node.subscribe("/joy", 1, &JoystickDemo::recvJoy, this);

  data_.brake_joy = 0.0;
  data_.gear_cmd = dbw_fca_msgs::Gear::NONE;
  data_.steering_joy = 0.0;
  data_.steering_mult = false;
  data_.throttle_joy = 0.0;
  data_.turn_signal_cmd = dbw_fca_msgs::TurnSignal::NONE;
  data_.door_select = dbw_fca_msgs::DoorCmd::NONE;
  data_.door_action = dbw_fca_msgs::DoorCmd::NONE;
  data_.joy_throttle_valid = false;
  data_.joy_brake_valid = false;

  if (brake_) {
    pub_brake_ = node.advertise<dbw_fca_msgs::BrakeCmd>("brake_cmd", 1);
  }
  if (throttle_) {
    pub_throttle_ = node.advertise<dbw_fca_msgs::ThrottleCmd>("throttle_cmd", 1);
  }
  if (steer_) {
    pub_steering_ = node.advertise<dbw_fca_msgs::SteeringCmd>("steering_cmd", 1);
  }
  if (shift_) {
    pub_gear_ = node.advertise<dbw_fca_msgs::GearCmd>("gear_cmd", 1);
  }
  if (signal_) {
    pub_turn_signal_ = node.advertise<dbw_fca_msgs::TurnSignalCmd>("turn_signal_cmd", 1);
  }
  if (enable_) {
    pub_enable_ = node.advertise<std_msgs::Empty>("enable", 1);
    pub_disable_ = node.advertise<std_msgs::Empty>("disable", 1);
  }

  timer_ = node.createTimer(ros::Duration(0.02), &JoystickDemo::cmdCallback, this);
}

void JoystickDemo::cmdCallback(const ros::TimerEvent& event)
{
  // Detect joy timeouts and reset
  if (event.current_real - data_.stamp > ros::Duration(0.1)) {
    data_.joy_throttle_valid = false;
    data_.joy_brake_valid = false;
    last_steering_filt_output_ = 0.0;
    return;
  }

  // Optional watchdog counter
  if (count_) {
    counter_++;
  }

  // Brake
  if (brake_) {
    dbw_fca_msgs::BrakeCmd msg;
    msg.enable = true;
    msg.ignore = ignore_;
    msg.count = counter_;
    msg.pedal_cmd_type = dbw_fca_msgs::BrakeCmd::CMD_PERCENT;
    msg.pedal_cmd = data_.brake_joy * brake_gain_;
    pub_brake_.publish(msg);
  }

  // Throttle
  if (throttle_) {
    dbw_fca_msgs::ThrottleCmd msg;
    msg.enable = true;
    msg.ignore = ignore_;
    msg.count = counter_;
    msg.pedal_cmd_type = dbw_fca_msgs::ThrottleCmd::CMD_PERCENT;
    msg.pedal_cmd = data_.throttle_joy * throttle_gain_;
    pub_throttle_.publish(msg);
  }

  // Steering
  if (steer_) {
    dbw_fca_msgs::SteeringCmd msg;
    msg.enable = true;
    msg.ignore = ignore_;
    msg.count = counter_;
    if (!strq_) {
      msg.cmd_type = dbw_fca_msgs::SteeringCmd::CMD_ANGLE;

      float raw_steering_cmd;
      if (data_.steering_mult) {
        raw_steering_cmd = dbw_fca_msgs::SteeringCmd::ANGLE_MAX * data_.steering_joy;
      } else {
        raw_steering_cmd = 0.5 * dbw_fca_msgs::SteeringCmd::ANGLE_MAX * data_.steering_joy;
      }

      float tau = 0.1;
      float filtered_steering_cmd = 0.02 / tau * raw_steering_cmd + (1 - 0.02 / tau) * last_steering_filt_output_;
      last_steering_filt_output_ = filtered_steering_cmd;

      msg.steering_wheel_angle_velocity = svel_;
      msg.steering_wheel_angle_cmd = filtered_steering_cmd;
    } else {
      msg.cmd_type = dbw_fca_msgs::SteeringCmd::CMD_TORQUE;
      msg.steering_wheel_torque_cmd = dbw_fca_msgs::SteeringCmd::TORQUE_MAX * data_.steering_joy;
    }
    pub_steering_.publish(msg);
  }

  // Gear
  if (shift_) {
    if (data_.gear_cmd != dbw_fca_msgs::Gear::NONE) {
      dbw_fca_msgs::GearCmd msg;
      msg.cmd.gear = data_.gear_cmd;
      pub_gear_.publish(msg);
    }
  }

  // Turn signal
  if (signal_) {
    dbw_fca_msgs::TurnSignalCmd msg;
    msg.cmd.value = data_.turn_signal_cmd;
    msg.door.select = data_.door_select;
    msg.door.action = data_.door_action;
    pub_turn_signal_.publish(msg);
  }
}

void JoystickDemo::recvJoy(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Check for expected sizes
  if (msg->axes.size() != (size_t)AXIS_COUNT_X && msg->buttons.size() != (size_t)BTN_COUNT_X) {
    if (msg->axes.size() == (size_t)AXIS_COUNT_D && msg->buttons.size() == (size_t)BTN_COUNT_D) {
      ROS_ERROR_THROTTLE(2.0, "Detected Logitech Gamepad F310 in DirectInput (D) mode. Please select (X) with the switch on the back to select XInput mode.");
    }
    if (msg->axes.size() != (size_t)AXIS_COUNT_X) {
      ROS_ERROR_THROTTLE(2.0, "Expected %zu joy axis count, received %zu", (size_t)AXIS_COUNT_X, msg->axes.size());
    }
    if (msg->buttons.size() != (size_t)BTN_COUNT_X) {
      ROS_ERROR_THROTTLE(2.0, "Expected %zu joy button count, received %zu", (size_t)BTN_COUNT_X, msg->buttons.size());
    }
    return;
  }

  // Handle joystick startup
  if (msg->axes[AXIS_THROTTLE] != 0.0) {
    data_.joy_throttle_valid = true;
  }
  if (msg->axes[AXIS_BRAKE] != 0.0) {
    data_.joy_brake_valid = true;
  }

  // Throttle
  if (data_.joy_throttle_valid) {
    data_.throttle_joy = 0.5 - 0.5 * msg->axes[AXIS_THROTTLE];
  }

  // Brake
  if (data_.joy_brake_valid) {
    data_.brake_joy = 0.5 - 0.5 * msg->axes[AXIS_BRAKE];
  }

  // Gear
  if (msg->buttons[BTN_PARK]) {
    data_.gear_cmd = dbw_fca_msgs::Gear::PARK;
  } else if (msg->buttons[BTN_REVERSE]) {
    data_.gear_cmd = dbw_fca_msgs::Gear::REVERSE;
  } else if (msg->buttons[BTN_DRIVE]) {
    data_.gear_cmd = dbw_fca_msgs::Gear::DRIVE;
  } else if (msg->buttons[BTN_NEUTRAL]) {
    data_.gear_cmd = dbw_fca_msgs::Gear::NEUTRAL;
  } else {
    data_.gear_cmd = dbw_fca_msgs::Gear::NONE;
  }

  // Steering
  data_.steering_joy = (fabs(msg->axes[AXIS_STEER_1]) > fabs(msg->axes[AXIS_STEER_2])) ? msg->axes[AXIS_STEER_1] : msg->axes[AXIS_STEER_2];
  data_.steering_mult = msg->buttons[BTN_STEER_MULT_1] || msg->buttons[BTN_STEER_MULT_2];

  // Turn signal
  if (msg->axes[AXIS_TURN_SIG] != joy_.axes[AXIS_TURN_SIG]) {
    if (fabs(msg->axes[AXIS_DOOR_ACTION]) < 0.5) {
      switch (data_.turn_signal_cmd) {
        case dbw_fca_msgs::TurnSignal::NONE:
          if (msg->axes[AXIS_TURN_SIG] < -0.5) {
            data_.turn_signal_cmd = dbw_fca_msgs::TurnSignal::RIGHT;
          } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
            data_.turn_signal_cmd = dbw_fca_msgs::TurnSignal::LEFT;
          }
          break;
        case dbw_fca_msgs::TurnSignal::LEFT:
          if (msg->axes[AXIS_TURN_SIG] < -0.5) {
            data_.turn_signal_cmd = dbw_fca_msgs::TurnSignal::RIGHT;
          } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
            data_.turn_signal_cmd = dbw_fca_msgs::TurnSignal::NONE;
          }
          break;
        case dbw_fca_msgs::TurnSignal::RIGHT:
          if (msg->axes[AXIS_TURN_SIG] < -0.5) {
            data_.turn_signal_cmd = dbw_fca_msgs::TurnSignal::NONE;
          } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
            data_.turn_signal_cmd = dbw_fca_msgs::TurnSignal::LEFT;
          }
          break;
      }
    }
  }

  // Doors and trunk
  data_.door_select = dbw_fca_msgs::DoorCmd::NONE;
  data_.door_action = dbw_fca_msgs::DoorCmd::NONE;
  if (msg->buttons[BTN_TRUNK_OPEN]) {
    data_.door_select = dbw_fca_msgs::DoorCmd::TRUNK;
    data_.door_action = dbw_fca_msgs::DoorCmd::OPEN;
  } else if (msg->buttons[BTN_TRUNK_CLOSE]) {
    data_.door_select = dbw_fca_msgs::DoorCmd::TRUNK;
    data_.door_action = dbw_fca_msgs::DoorCmd::CLOSE;
  }
  if (msg->axes[AXIS_DOOR_ACTION] > 0.5) {
    if (msg->axes[AXIS_DOOR_SELECT] < -0.5) {
      data_.door_select = dbw_fca_msgs::DoorCmd::RIGHT;
      data_.door_action = dbw_fca_msgs::DoorCmd::OPEN;
    } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
      data_.door_select = dbw_fca_msgs::DoorCmd::LEFT;
      data_.door_action = dbw_fca_msgs::DoorCmd::OPEN;
    }
  }
  if (msg->axes[AXIS_DOOR_ACTION] < -0.5) {
    if (msg->axes[AXIS_DOOR_SELECT] < -0.5) {
      data_.door_select = dbw_fca_msgs::DoorCmd::RIGHT;
      data_.door_action = dbw_fca_msgs::DoorCmd::CLOSE;
    } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
      data_.door_select = dbw_fca_msgs::DoorCmd::LEFT;
      data_.door_action = dbw_fca_msgs::DoorCmd::CLOSE;
    }
  }

  // Optional enable and disable buttons
  if (enable_) {
    const std_msgs::Empty empty;
    if (msg->buttons[BTN_ENABLE]) {
      pub_enable_.publish(empty);
    }
    if (msg->buttons[BTN_DISABLE]) {
      pub_disable_.publish(empty);
    }
  }

  data_.stamp = ros::Time::now();
  joy_ = *msg;
}

