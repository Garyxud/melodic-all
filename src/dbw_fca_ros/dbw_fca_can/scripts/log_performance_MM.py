#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Dataspeed Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright notice,
#       this list of conditions and the following disclaimer in the documentation
#       and/or other materials provided with the distribution.
#     * Neither the name of Dataspeed Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from this
#       software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import csv
import math
import sys
import argparse

from dbw_fca_msgs.msg import BrakeCmd, BrakeReport, BrakeInfoReport
from dbw_fca_msgs.msg import ThrottleCmd, ThrottleReport, ThrottleInfoReport
from dbw_fca_msgs.msg import Gear, GearCmd, GearReport
from dbw_fca_msgs.msg import SteeringCmd, SteeringReport

parser = argparse.ArgumentParser(description='Log Pacfica performance data')
parser.add_argument('--steer', action='store_true', default=False, help='Enable steering tests')
parser.add_argument('--brake', action='store_true', default=False, help='Enable brake tests')
parser.add_argument('--lever', action='store_true', default=False, help='Enable lever tests')
parser.add_argument('--gas',   action='store_true', default=False, help='Enable gas tests')
parser.add_argument('--quick', action='store_true', default=False, help='Do a couple of quick tests')
parser.add_argument('--all',   action='store_true', default=False, help='Enable all tests (default)')
tests, _ = parser.parse_known_args()
if (tests.steer or tests.brake or tests.lever or tests.gas):
    tests.all = False
else:
    tests.all = True

if tests.all:
    tests.steer = tests.brake = tests.lever = tests.gas = True
tests.strict_MM = False # set to true to release to MM

#
#static const struct {float pedal; float torque;} BRAKE_TABLE[] = {
#// Duty,   Nm
# {0.150,    0},
# {0.166,    0},
# {0.168,    4},
# {0.200,   56},
# {0.225,  194},
# {0.250,  456},
# {0.300, 1312},
# {0.350, 2352},
# {0.400, 3716},
# {0.434, 4740},
# {0.566, 6888},
# {0.600, 6888},
#};
#static const struct {float pedal; float percent;} THROTTLE_TABLE[] = {
#// Duty,   %
# {0.080, 0.000},
# {0.114, 0.001},
# {0.497, 0.500},
# {0.890, 0.998},
# {0.892, 1.000},
#};

# The Tests below actuate the Brake, Steering, Gas (Throttle), and Shift Lever
# with the purpose of measuring the performace of these individual systems in terms
# of response time and accuracy. All data is logged to csv so that the system performance
# can be analyzed easily.

# {0.080},
# {0.114},
# {0.497},
# {0.890},
# {0.892},
brake_duty = [ 0.150, 0.166, 0.168, 0.200, 0.225, 0.250, 0.300, 0.350, 0.400, 0.434, 0.566, 0.600]
brake_Nm =   [ 0.000, 0.000, 4.000, 56.00, 194.0, 456.0, 1312,  2352,  3716,  4740,  6888,  6888]

throttle_pct = [ 0.000, 0.000, 0.001, 0.500, 0.998, 1.000, 1.000]
throttle_duty = [ 0.000, 0.080, 0.114, 0.497, 0.890, 0.892, 1.000]

dont_check = False

# timing specification
command_resolution_s = 0.04
initial_check_wait_s = 1.0
first_cmd_hold_s     = 5.0

test_counts = {}
class CarSysTest:

    def check_initial_conditions(self):
        if dont_check: return 
        # Make sure the system is in a safe configuration at the start of a test
        if not self.all_ready(['SteeringReport', 'GearReport']):
            self.stop()
            return
        steering_report = self.get_msg('SteeringReport')
        if steering_report.speed > 0.0:
            rospy.logerr('Speed check failed. Vehicle is moving.')
            self.stop()
            return
        gear_report = self.get_msg('GearReport')
        # this wont work for gear testing!
        if self.name() != "LeverTest":
            if not gear_report.state.gear == gear_report.state.PARK:
                rospy.logerr('Gear check failed. Vehicle not in park.')
                self.stop()
                return

    def tick(self, event):
        if self.done: return
        if self.repeat_ct == self.repeats:
            self.repeat_ct = 0
            self.idx += 1
        else:
            self.repeat_ct += 1

        start_ct = -int(math.ceil(first_cmd_hold_s/command_resolution_s))
        if self.repeat_ct < start_ct:
            pass # wait a bit for system to be ready
        elif self.repeat_ct == start_ct:
            self.check_initial_conditions()
        else:
            try:
                value = self.sequence[self.idx]
                self.pub(value)
            except IndexError:
                self.stop()

    def __init__(self, description, sequence, units="percent", num_reps=1, period_s=1):
        self.description = description
        self.repeats = int(math.ceil(period_s/command_resolution_s))
        self.sequence = sequence
        self.units = units
        self.done = False
        # give a counted name to each test
        global test_counts
        try:
            test_counts[self.name()] += 1
        except:
            test_counts[self.name()] = 1
        self.test_num = test_counts[self.name()]

    def name(self):
        return self.__class__.__name__

    def test_name(self):
        return "%s%d" % (self.name(), self.test_num)

    def start(self, msgs):
        self.msgs = msgs

        rospy.loginfo("Starting %s: %s" % (self.test_name(), self.description))
        self.csv_file = open(self.test_name() + '.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',')
        self.write_csv_header(self.csv_writer)
        # we want to wait a bit and check initial conditions before actually performing the tests
        self.repeat_ct = -int((initial_check_wait_s + first_cmd_hold_s)/command_resolution_s)
        self.idx = 0
        self.done = False
        self.pedal_cmd = 0.0
        self.start_time = rospy.get_rostime()

    def stop(self):
        rospy.loginfo('Saving csv file')
        self.csv_file.close()
        self.done = True

    def get_msg(self, msg_name):
        _, msg = self.msgs[msg_name]
        return msg

    def mark_not_ready(self, msg_name):
        try:
            ready, msg = self.msgs[msg_name]
            if ready:
                self.msgs[msg_name] = (False, msg)
        except:
            rospy.logerr("%s is not a valid message name" % msg_name)
    def mark_all_not_ready(self):
        for msg_name, _ in self.msgs.items():
            mark_not_ready(msg_name)

    def is_ready(self, msg_name):
        try:
            ready, _ = self.msgs[msg_name]
            if not ready:
                rospy.logwarn('no new %s messages' % msg_name)
            return ready
        except:
            return False

    def all_ready(self, msg_names):
        for name in msg_names:
            if not self.is_ready(name):
                return False
        return True

    def elapsed_time_s(self):
        return (rospy.get_rostime() - self.start_time).to_sec() - initial_check_wait_s - first_cmd_hold_s + 0.02

# Brake Tests
#    Make sure GEAR=PARK and SPEED=0
# Save the folowing data:
#    /vehicle/brake_cmd:pedal_cmd
#    /vehicle/brake_report:pedal_output
#    /vehicle/brake_info_report:brake_pc
#    /vehicle/brake_info_report:brake_pressure
class BrakeTest(CarSysTest):

    def write_csv_header(self, csv_writer):
        csv_writer.writerow(['Elapsed Time (s)', 'Brake Cmd (%)', 'Measured (%)', 'Pressure (bar)', 'User Input'])

    def pub(self, brake_val):
        if self.repeat_ct == 0:
            rospy.loginfo("Setting brake to %f" % brake_val)
        msg = BrakeCmd()
        if brake_val >= 0:
            msg.enable = True
            if self.units == "percent":
                msg.pedal_cmd_type = BrakeCmd.CMD_PERCENT
                self.pedal_cmd = brake_val/100.0
                msg.pedal_cmd = self.pedal_cmd
            elif self.units == "torque":
                msg.pedal_cmd_type = BrakeCmd.CMD_TORQUE_RQ
                self.pedal_cmd = brake_val
                msg.pedal_cmd = self.pedal_cmd
            else:
                raise Exception("units %s unsupported for BrakeTest" % self.units)
            brake_pub.publish(msg)

        # we are watching these, lets make sure we keep getting messages
        self.check_conditions()
        self.mark_not_ready('BrakeReport')
        self.mark_not_ready('BrakeInfoReport')

    def check_conditions(self):
        if dont_check: return
        # Make sure we are getting new messages
        if not self.all_ready(['BrakeReport', 'BrakeInfoReport']):
            self.stop()
            return
        # TODO: check if brake_report.enabled = True?
        # simple check doesn't work due to startup condition

    def recv_msg(self, msg):
        # don't record start sequence
        if self.repeat_ct < 0: return
        # align reporting to brake report
        if type(msg).__name__ == 'BrakeReport':
            if self.units == 'percent':
                self.csv_writer.writerow(["{:10.06f}".format(self.elapsed_time_s()),
                                          "{:7.03f}".format(self.pedal_cmd*100),
                                          "{:7.03f}".format(msg.pedal_cmd),
                                          "{:7.03f}".format(msg.pedal_input),
                                          "{:7.03f}".format(msg.pedal_output),
                                          "{:d}".format(msg.user_input)])
            elif self.units == 'torque':
                self.csv_writer.writerow(["{:10.06f}".format(self.elapsed_time_s()),
                                          "{:7.03f}".format(msg.pedal_cmd),
                                          "{:7.03f}".format(msg.pedal_input),
                                          "{:7.03f}".format(msg.pedal_output),
                                          "{:7.03f}".format(self.pedal_cmd)])

# Steering Tests
#    Make sure GEAR=PARK and SPEED=0
# Save the following data:
#    /vehicle/steering_cmd:steering_wheel_angle_cmd
#    /vehicle/steering_report:steering_wheel_cmd
#    /vehicle/steering_report:steering_wheel_angle
class SteeringTest(CarSysTest):

    def write_csv_header(self, csv_writer):
        self.steering_wheel_cmd = 0.0
        csv_writer.writerow(['Elapsed Time (s)', 'Steering Cmd Sent (Degrees)', 'Steering Cmd Reported (Degrees)', 'Measured (Degrees)'])

    def pub(self, steering_val):
        if self.repeat_ct == 0:
            rospy.loginfo("Setting steering to %f" % steering_val)
        msg = SteeringCmd()
        msg.enable = True
        if self.units == "percent": # a percent corresponds to 5 degrees
            self.steering_wheel_cmd = math.radians(steering_val*5)
        elif self.units == "degrees":
            self.steering_wheel_cmd = math.radians(steering_val)
        else:
            raise Exception("units %s unsupported for SteeringTest" % self.units)
        msg.steering_wheel_angle_cmd = self.steering_wheel_cmd
        steering_pub.publish(msg)

        # we are watching these, lets make sure we keep getting messages
        self.check_conditions()
        self.mark_not_ready('SteeringReport')

    def check_conditions(self):
        if dont_check: return
        # Make sure we are getting new messages
        if not self.all_ready(['SteeringReport']):
            self.stop()
            return
        # TODO: check if brake_report.enabled = True?
        # simple check doesn't work due to startup condition

    def recv_msg(self, msg):
        # don't record start sequence
        if self.repeat_ct < 0: return
        # align reporting to steering report
        if type(msg).__name__ == 'SteeringReport':
            self.csv_writer.writerow(["{:10.06f}".format(self.elapsed_time_s()),
                                      "{: 6.1f}".format(math.degrees(self.steering_wheel_cmd)),
                                      "{: 6.1f}".format(math.degrees(msg.steering_wheel_cmd)),
                                      "{: 6.1f}".format(math.degrees(msg.steering_wheel_angle))])

    lock2lock_degrees = math.degrees(SteeringCmd().ANGLE_MAX*2.0)

# Gas Tests
#    Make sure GEAR=PARK and SPEED=0
# Save the following data:
#    /vehicle/throttle_cmd:pedal_cmd
#    /vehicle/throttle_report:pedal_output
#    /vehicle/throttle_info_report:throttle_pc
class ThrottleTest(CarSysTest):

    def write_csv_header(self, csv_writer):
        csv_writer.writerow(['Elapsed Time (s)', 'Throttle Cmd (%)', 'Measured (%)'])

    def pub(self, throttle_val):
        if self.repeat_ct == 0:
            rospy.loginfo("Setting throttle to %f" % throttle_val)
        msg = ThrottleCmd()
        if throttle_val >= 0:
            msg.enable = True
            msg.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            if self.units == "percent":
                self.pedal_cmd = throttle_val/100.0
                msg.pedal_cmd = self.pedal_cmd
            else:
                raise Exception("units %s unsupported for ThrottleTest" % self.units)
            throttle_pub.publish(msg)

        # we are watching these, lets make sure we keep getting messages
        self.check_conditions()
        self.mark_not_ready('ThrottleReport')
        self.mark_not_ready('ThrottleInfoReport')

    def check_conditions(self):
        if dont_check: return
        # Make sure we are getting new messages
        if not self.all_ready(['ThrottleReport', 'ThrottleInfoReport']):
            self.stop()
            return
        # TODO: check if throttle_report.enabled = True?
        # simple check doesn't work due to startup condition

    def recv_msg(self, msg):
        # don't record start sequence
        if self.repeat_ct < 0: return
        # align reporting to throttle report
        if type(msg).__name__ == 'ThrottleInfoReport':
            self.csv_writer.writerow(["{:10.06f}".format(self.elapsed_time_s()),
                                      "{:5.01f}".format(self.pedal_cmd*100),
                                      "{:5.01f}".format(msg.throttle_pc)])

# Lever Tests
# "To perfomer tests in safety condition, must be activated parking brake lever.
# This test have to guarantee that there no necessity to active brake pedal by Magneti Marelli ACU.
# IF THERE IS NECESSITY TO MANAGE BRAKE PEDAL PRESSION DURING LEVER CHANGE, THIS HAS TO BE COMMUNICATED TO MAGNETI MARELLI
#    Make sure SPEED=0
# Save the folowing data:
#    /vehicle/gear_cmd:cmd
#    /vehicle/gear_report:state
# TODO: add lever tests class
class LeverTest(CarSysTest):

    def write_csv_header(self, csv_writer):
        csv_writer.writerow(['Elapsed Time (s)', 'Lever Cmd Requested', 'Lever Cmd Reported', 'Lever Position Reported'])

    def pub(self, gear):
        if self.repeat_ct == 0:
            rospy.loginfo("Setting gear to %s" % gear)
        msg = GearCmd()
        if self.units == "gear":
            if gear == 'P':
                msg.cmd.gear = Gear().PARK
            elif gear == 'R':
                msg.cmd.gear = Gear().REVERSE
            elif gear == 'D':
                msg.cmd.gear = Gear().DRIVE
            elif gear == 'N':
                msg.cmd.gear = Gear().NEUTRAL
            else:
                raise Exception("Gear %s is not supported" % gear)
            self.cmd = gear
        else:
            raise Exception("units %s unsupported for LeverTest" % self.units)
        gear_pub.publish(msg)
        # publish a brake message as well
        bmsg = BrakeCmd()
        bmsg.enable = True
        bmsg.pedal_cmd_type = BrakeCmd.CMD_PERCENT
        bmsg.pedal_cmd = 0.4
        brake_pub.publish(bmsg)

        # we are watching these, lets make sure we keep getting messages
        self.check_conditions()
        #self.mark_not_ready('GearReport') # these dont come fast enough

    def check_conditions(self):
        if dont_check: return
        # Make sure we are getting new messages
        if not self.all_ready(['GearReport']):
            self.stop()
            return

    def to_gear(self, gear_num):
        if gear_num == Gear().PARK:
            return 'P'
        elif gear_num == Gear().REVERSE:
            return 'R'
        elif gear_num == Gear().DRIVE:
            return 'D'
        elif gear_num == Gear().NEUTRAL:
            return 'N'
        elif gear_num == Gear().LOW:
            return 'L'
        else:
            return 'None'

    def recv_msg(self, msg):
        # don't record start sequence
        if self.repeat_ct < 0: return
        # align reporting to gear report
        if type(msg).__name__ == 'GearReport':
            self.csv_writer.writerow(["{:.06f}".format(self.elapsed_time_s()),
                                      self.cmd, self.to_gear(msg.cmd.gear), self.to_gear(msg.state.gear)])

def reset_gear(gear):
    rospy.loginfo("Resetting gear to %s before starting test" % gear)
    times = int(5/command_resolution_s)
    r = rospy.Rate(1/command_resolution_s)
    for i in range(0, times):
        msg = GearCmd()
        if gear == 'P':
            msg.cmd.gear = Gear().PARK
        elif gear == 'R':
            msg.cmd.gear = Gear().REVERSE
        elif gear == 'D':
            msg.cmd.gear = Gear().DRIVE
        elif gear == 'N':
            msg.cmd.gear = Gear().NEUTRAL
        else:
            raise Exception("Gear %s is not supported" % gear)
        gear_pub.publish(msg)

        bmsg = BrakeCmd()
        bmsg.enable = True
        bmsg.pedal_cmd_type = BrakeCmd.CMD_PERCENT
        bmsg.pedal_cmd = 0.4
        brake_pub.publish(bmsg)
        r.sleep()

perf_tests = []
if tests.brake:
    perf_tests.append(BrakeTest("Square wave", [ 0, 10, 0, 10, 0, 10, 0, 10, 0, 10], period_s=1, num_reps=4))
    perf_tests.append(BrakeTest("Square wave", [ 0, 50, 0, 50, 0, 50, 0, 50], period_s=1, num_reps=4))
    if not tests.strict_MM:
        perf_tests.append(BrakeTest("Square wave", [ 0, 100, 0, 100, 0, 100, 0, 100, 0, 100, 0, 100, 0, 100, 0, 100], period_s=1, num_reps=4))
        perf_tests.append(BrakeTest("Square wave", [ 0, 100, 100, 100, 100, 100, 100, 100, 100], period_s=1, num_reps=4))
    if not tests.quick:
        perf_tests.append(BrakeTest("Increasing impulse wave",
                                    [ 20, 0, 20.5, 0, 21, 0, 21.5, 0, 22, 0, 22.5, 0, 23, 0, 23.5, 0, 24, 0, 24.5, 0, 25],
                                    period_s=5, num_reps=4))
        perf_tests.append(BrakeTest("Square wave",
                                    [0, 10, 0, 15, 0, 20, 0, 25, 0, 30, 0, 35, 0, 40, 0, 45, 0, 50,
                                     0, 55, 0, 60, 0, 65, 0, 70, 0, 75, 0, 80, 0, 85, 0, 90, 0, 95, 0, 100],
                                    period_s=10, num_reps=4))
        perf_tests.append(BrakeTest("Square Wave", [ 0, 10, 15, 20, 25, 30, 35, 40, 45, 50,
                                                     55, 60, 65, 70, 75, 80, 85, 90, 95, 100],
                                    period_s=10, num_reps=4))
        perf_tests.append(BrakeTest("Increasing Wave",
                                    [ 20, 21, 20, 21.5, 20, 22, 20, 22.5, 20, 23, 20, 23.5, 20, 24, 20, 24.5, 20, 25, 20, 25.5,
                                      20, 26, 20, 26.5, 20, 27, 20, 27.5, 20, 28, 20, 28.5, 20, 29, 20, 29.5, 20, 30],
                                    period_s=10, num_reps=4))
if tests.steer:
    perf_tests.append(SteeringTest("Square wave",
                                   [0, SteeringTest.lock2lock_degrees / 4, 0, SteeringTest.lock2lock_degrees / 4, 0, SteeringTest.lock2lock_degrees / 4, 0, SteeringTest.lock2lock_degrees / 4, 0],
                                   period_s=2, units="degrees", num_reps=5))  # 0 degree - Max( from zero to lock side)
    perf_tests.append(SteeringTest("Increasing wave",
                                   [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100],
                                   period_s=2, num_reps=2))
    perf_tests.append(SteeringTest("Increasing wave",
                                   [0, 10, 0, 20, 0, 30, 0, 40, 0, 50, 0, 60, 0, 70, 0, 80, 0, 90, 0, 100, 0],
                                   period_s=2, num_reps=2))
    # perf_tests.append(SteeringTest("Increasing impulsive wave",
    #                [ 20, 0, 20.1, 0, 20.2, 0, 20.3, 0, 20.4, 0,  20.5, 0, 20.6, 0, 20.7, 0, 20.8, 0, 20.9, 0, 21],
    #                units="degrees", period_s=5, num_reps=4))
    perf_tests.append(SteeringTest("Increasing impulsive wave",
                                   [0, 0.5, 0, -0.5, 0, 1, 0, -1, 0, 1.5, 0, -1.5, 0, 2, 0, -2, 0, 2.5, 0, -2.5],
                                   units="degrees", period_s=2, num_reps=2))

if tests.gas:
    perf_tests.append(ThrottleTest("Step test", [ 0, 25 ], num_reps=4))
    perf_tests.append(ThrottleTest("Step test", [ 0, 50 ], num_reps=4))
    perf_tests.append(ThrottleTest("Step test", [ 0, 75 ], num_reps=4))
    perf_tests.append(ThrottleTest("Step test", [ 0, 100 ], num_reps=4))

if tests.lever:
    perf_tests.append(LeverTest("Transition", [ 'P', 'N'], num_reps=2, units="gear"))
    perf_tests.append(LeverTest("Transition", [ 'P', 'D'], num_reps=2, units="gear"))
    perf_tests.append(LeverTest("Transition", [ 'P', 'R'], num_reps=2, units="gear"))
    perf_tests.append(LeverTest("Transition", [ 'N', 'D'], num_reps=2, units="gear"))
    perf_tests.append(LeverTest("Transition", [ 'N', 'R'], num_reps=2, units="gear"))
    perf_tests.append(LeverTest("Transition", [ 'N', 'P'], num_reps=2, units="gear"))
    perf_tests.append(LeverTest("Transition", [ 'D', 'N'], num_reps=2, units="gear"))
    perf_tests.append(LeverTest("Transition", [ 'D', 'P'], num_reps=2, units="gear"))
    perf_tests.append(LeverTest("Transition", [ 'D', 'R'], num_reps=2, units="gear"))
    perf_tests.append(LeverTest("Transition", [ 'R', 'D'], num_reps=2, units="gear"))
    perf_tests.append(LeverTest("Transition", [ 'R', 'P'], num_reps=2, units="gear"))
    perf_tests.append(LeverTest("Transition", [ 'R', 'N'], num_reps=2, units="gear"))

    # just to put it back in park
    perf_tests.append(LeverTest("Transition", [ 'P'], num_reps=2, units="gear"))

class TestNode:
    def __init__(self):
        rospy.init_node('test_runner')
        self.current_test = None

        # Variables for logging
        self.brake_cmd = 0.0

        # Keep all of the different types of message and whether or not they are ready
        self.msgs = {} # ( Ready?, Report )
        self.msgs['BrakeReport'] = ( False, BrakeReport() )
        self.msgs['BrakeInfoReport'] = ( False, BrakeInfoReport() )
        self.msgs['GearReport'] = ( False, GearReport() )
        self.msgs['SteeringReport'] = ( False, SteeringReport() )
        self.msgs['ThrottleReport'] = ( False, ThrottleReport() )
        self.msgs['ThrottleInfoReport'] = ( False, ThrottleInfoReport() )

        # Open CSV file
        self.csv_file = open('brake_sweep_data.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',')
        self.csv_writer.writerow(['Brake (%)', 'Measured (%)', 'Pressure (bar)'])

        # Publishers and subscribers
        # Note: publishers are global so that each of the specific test classes can publish to them
        # without having to explicity allocate and manage them
        global brake_pub, gear_pub, steering_pub, throttle_pub
        brake_pub    = rospy.Publisher('/vehicle/brake_cmd',    BrakeCmd,    queue_size=1)
        gear_pub     = rospy.Publisher('/vehicle/gear_cmd',     GearCmd,     queue_size=1)
        steering_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)

        rospy.Subscriber('/vehicle/brake_report',         BrakeReport,         self.recv_msg)
        rospy.Subscriber('/vehicle/brake_info_report',    BrakeInfoReport,     self.recv_msg)
        rospy.Subscriber('/vehicle/throttle_report',      ThrottleReport,      self.recv_msg)
        rospy.Subscriber('/vehicle/throttle_info_report', ThrottleInfoReport,  self.recv_msg)
        rospy.Subscriber('/vehicle/gear_report',          GearReport,          self.recv_msg)
        rospy.Subscriber('/vehicle/steering_report',      SteeringReport,      self.recv_msg)
        
        rospy.Timer(rospy.Duration(command_resolution_s), self.timer_process)

    def timer_process(self, event):
        if not self.current_test is None and not self.current_test.done:
            self.current_test.tick(event)

    # consolidate all of these into one that uses a dictionary
    def recv_msg(self, msg):
        self.msgs[type(msg).__name__] = ( True, msg)
        if not self.current_test is None and not self.current_test.done:
            self.current_test.recv_msg(msg)

    def run_tests(self):
        rospy.sleep(rospy.Duration(5))
        if rospy.is_shutdown():
            rospy.loginfo("Stopping test")
            sys.exit(-1)
        for test in perf_tests:
            if test.__class__.__name__ == "LeverTest":
                reset_gear(test.sequence[0])
            test.start(self.msgs)
            self.current_test = test
            while not rospy.is_shutdown() and not test.done:
                rospy.sleep(rospy.Duration(0.01))
            if rospy.is_shutdown():
                break

if __name__ == '__main__':
    try:
        node = TestNode()
        node.run_tests()
    except rospy.ROSInterruptException:
        rospy.loginfo("Stopping test")
        pass
