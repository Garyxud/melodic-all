#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import nxt.locator
from nxt.sensor import *
from nxt.motor import *
import time

class NXTBrick:
  def __init__(self, bsock=None):
    """
    Ctor
    Connecting to NXT brick and creating motor object, sensor object
    and so on. Motor encoders will be reset.
    """
    if bsock:
      self.sock = bsock
    else:
      self.sock = nxt.locator.find_one_brick().connect()

    self.motors = [Motor(self.sock, PORT_A),
                   Motor(self.sock, PORT_B),
                   Motor(self.sock, PORT_C)]
            
    self.sensors = [TouchSensor(self.sock, PORT_1),
                    SoundSensor(self.sock, PORT_2),
                    LightSensor(self.sock, PORT_3),
                    UltrasonicSensor(self.sock, PORT_4)]
    self.resetPosition()

  def close(self):
    """
    Finalizing connection with NXT brick.
    """
    self.sock.close()

  def resetPosition(self, relative = 0):
    """
    Resetting encoders of NXT motors
    """
    for m in self.motors:
      m.reset_position(relative)

  def setMotors(self, vels):
    """
    This operation receives array and set them as motor power.  If the
    number of vels items does not match with the number of motors,
    smaller number of them will be taken and set respectively.
    """
    for i, v in enumerate(vels[:min(len(vels),len(self.motors))]):
      self.motors[i].power = max(min(v,127),-127)
      self.motors[i].mode = MODE_MOTOR_ON | MODE_REGULATED
      self.motors[i].regulation_mode = REGULATION_MOTOR_SYNC
      self.motors[i].run_state = RUN_STATE_RUNNING
      self.motors[i].tacho_limit = 0
      self.motors[i].set_output_state()

  def getMotors(self):
    """
    Getting motors' angle (degrees)
    """
    state = []
    for m in self.motors:
      stat = None
      for i in range(3):
        try:
          stat = m.get_output_state()
          break
        except:
          time.sleep(0.01)
          continue

      if stat == None:
        import sys
        print "Unknown motor encoder error"
        print sys.exc_info()[1]
      state.append(stat)

    return state


  def getSensors(self):
    """
    Getting sensors' values. Data will be returned as array.
    """
    state = []
    for s in self.sensors:
      stat = None
      for i in range(3):
        try:
          stat = s.get_sample()
          break
        except:
          time.sleep(0.01)
          continue
      if stat == None:
        import sys
        print "Unknown sensor error"
        print sys.exc_info()[1]
      state.append(stat)

    return state


"""
Test program
It gives appropriate values to motors, and angles of motors are
obtained and shown.  Sensor data are also obtained and shown.
"""
if __name__ == "__main__":
  import time
  nxt = NXTBrick()
  print "connected"
    
  # Testing motors
  for i in range(0):
    nxt.setMotors([80,-80,80])
    print "Motor: "
    mstat = nxt.getMotors()
    for i, m in enumerate(mstat):
      print "(" , i, "): ", m
    time.sleep(0.1)
  nxt.setMotors([0,0,0])

  # Testing sensors
  for i in range(100):
    sensors = ["Touch", "Sound", "Light", "USonic"]
    sval = nxt.getSensors()
    print sval
    time.sleep(0.1)
