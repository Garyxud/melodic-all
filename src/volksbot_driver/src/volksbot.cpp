/*
 * Copyright (c) 2013, Osnabrueck University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the Osnabrueck University nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/console.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdint.h>

#include "comm.h"
#include "volksbot.h"

Volksbot::Volksbot(
        Comm &comm,
        double wheel_radius,
        double axis_length,
        double turning_adaptation,
        int gear_ratio,
        int max_vel_l,
        int max_vel_r,
        int max_acc_l,
        int max_acc_r,
        bool drive_backwards) :
      epos2_left_(drive_backwards ? 0x01 : 0x02),
      epos2_right_(drive_backwards ? 0x02 : 0x01),
      comm_(comm),
      wheel_radius_(wheel_radius),
      axis_length_(axis_length),
      turning_adaptation_(turning_adaptation),
      gear_ratio_(gear_ratio),
      max_vel_l_(max_vel_l),
      max_vel_r_(max_vel_r),
      max_acc_l_(max_acc_l),
      max_acc_r_(max_acc_r)
{
  epos2_left_.init();
  epos2_right_.init();

  epos2_left_.enableController();
  epos2_right_.enableController();

  epos2_left_.enableMotor(epos2_left_.VELOCITY);
  epos2_right_.enableMotor(epos2_right_.VELOCITY);

  epos2_left_.setProfileData(
      0, // Velocity
      max_vel_l_, // Max Velocity
      0, // Acceleration
      0, // Deceleration
      0, // QS Decel
      max_acc_l_, // Max acc
      0 // Type: Trapecoidal
      );

  epos2_right_.setProfileData(
      0, // Velocity
      max_vel_r_, // Max Velocity
      0, // Acceleration
      0, // Deceleration
      0, // QS Decel
      max_acc_r_, // Max acc
      0 // Type: Trapecoidal
      );

  epos2_left_.setOperationMode(epos2_left_.VELOCITY);
  epos2_right_.setOperationMode(epos2_right_.VELOCITY);
}

Volksbot::~Volksbot()
{
  epos2_left_.setTargetVelocity(0.0);
  epos2_right_.setTargetVelocity(0.0);
  epos2_left_.close();
  epos2_right_.close();
}

double Volksbot::get_max_vel(){
  return (max_vel_r_ + max_vel_l_) * M_PI * wheel_radius_ / (60.0 * gear_ratio_);
}

void Volksbot::set_wheel_speed(double _v_l_soll, double _v_r_soll)
{
  epos2_left_.setTargetVelocity(_v_l_soll / ( 2.0 * M_PI * wheel_radius_) * 60.0 * gear_ratio_);
  epos2_right_.setTargetVelocity(-1.0 * _v_r_soll / ( 2.0 * M_PI * wheel_radius_) * 60.0 * gear_ratio_);
}

void Volksbot::odometry()
{
  static double x = 0.0;
  static double y = 0.0;
  static double theta = 0.0;
  static long enc_left_last = epos2_left_.readEncoderCounter();
  static long enc_right_last = epos2_right_.readEncoderCounter();
  static long enc_per_turn_left = 4 * epos2_left_.getEncoderPulseNumber() * gear_ratio_;
  static long enc_per_turn_right = 4 * epos2_right_.getEncoderPulseNumber() * gear_ratio_;

  long enc_left = epos2_left_.readEncoderCounter();
  long enc_right = epos2_right_.readEncoderCounter();
  long wheel_l = enc_left - enc_left_last;
  long wheel_r = enc_right - enc_right_last;

  // handle overflow (> 10000 required to ensure we don't do this on zero crossings)
  if((abs(enc_left) > 10000) && (std::signbit(enc_left) != std::signbit(enc_left_last)))
  {
    if(std::signbit(enc_left))
      wheel_l = std::numeric_limits<int32_t>::max() - enc_left_last - std::numeric_limits<int32_t>::min() + enc_left;
    else
      wheel_l = std::numeric_limits<int32_t>::max() - enc_left - std::numeric_limits<int32_t>::min() + enc_left_last;
  }

  if((abs(enc_right) > 10000) && (std::signbit(enc_right) != std::signbit(enc_right_last)))
  {
    if(std::signbit(enc_right))
      wheel_r = std::numeric_limits<int32_t>::max() - enc_right_last - std::numeric_limits<int32_t>::min() + enc_right;
    else
      wheel_r = std::numeric_limits<int32_t>::max() - enc_right - std::numeric_limits<int32_t>::min() + enc_right_last;
  }

  enc_left_last = enc_left;
  enc_right_last = enc_right;

  double wheel_L = 2.0 * M_PI * wheel_radius_ * wheel_l / enc_per_turn_left;
  double wheel_R = -2.0 * M_PI * wheel_radius_ * wheel_r / enc_per_turn_right;

  double dtheta = (wheel_R - wheel_L) / axis_length_ * turning_adaptation_;
  double hypothenuse = 0.5 * (wheel_L + wheel_R);

  x += hypothenuse * cos(theta + dtheta * 0.5);
  y += hypothenuse * sin(theta + dtheta * 0.5);
  theta += dtheta;

  if (theta > M_PI)
    theta -= 2.0 * M_PI;
  if (theta < -M_PI)
    theta += 2.0 * M_PI;

  double v_left = epos2_left_.readVelocity() / 60.0 / gear_ratio_ * 2.0 * M_PI * wheel_radius_;
  double v_right = -epos2_right_.readVelocity() / 60.0 / gear_ratio_ * 2.0 * M_PI * wheel_radius_;
  double v_x = (v_left + v_right) * 0.5;
  double v_theta = (v_right - v_left) / axis_length_ * turning_adaptation_;

  double wheelpos_l = 2.0 * M_PI * (enc_left % enc_per_turn_left) / enc_per_turn_left;
  if (wheelpos_l > M_PI)
    wheelpos_l -= 2.0 * M_PI;
  if (wheelpos_l < -M_PI)
    wheelpos_l += 2.0 * M_PI;

  double wheelpos_r = 2.0 * M_PI * (enc_right % enc_per_turn_right) / enc_per_turn_right;
  if (wheelpos_r > M_PI)
    wheelpos_r -= 2.0 * M_PI;
  if (wheelpos_r < -M_PI)
    wheelpos_r += 2.0 * M_PI;

  comm_.send_odometry(x, y, theta, v_x, v_theta, wheelpos_l, wheelpos_r);
}
