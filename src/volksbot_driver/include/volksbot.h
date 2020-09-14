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

#ifndef _VOLKSBOT_H_
#define _VOLKSBOT_H_

#include <Epos2.h>

#include "comm.h"

class Volksbot
{
  public:
    Volksbot(
        Comm &comm,
        double wheel_radius,
        double axis_length,
        double turning_adaptation,
        int gear_ratio,
        int max_vel_l,
        int max_vel_r,
        int max_acc_l,
        int max_acc_r,
        bool drive_backwards);
    ~Volksbot();
    void set_wheel_speed(double _v_l_soll, double _v_r_soll);
    double get_max_vel();
    void odometry();

  private:
    CEpos2 epos2_left_;
    CEpos2 epos2_right_;
    Comm &comm_;
    double wheel_radius_;
    double axis_length_;
    double turning_adaptation_;
    int gear_ratio_;
    int max_vel_l_;
    int max_vel_r_;
    int max_acc_l_;
    int max_acc_r_;
};

#endif
