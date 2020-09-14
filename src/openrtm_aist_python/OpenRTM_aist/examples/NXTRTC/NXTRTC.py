#!/usr/bin/env python
# -*- coding:utf-8 -*-
# -*- Python -*-

import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist


# import NXTBrick class
import NXTBrick


# This module's spesification
# <rtc-template block="module_spec">
nxtrtc_spec = ["implementation_id", "NXTRTC", 
               "type_name",         "NXTRTC", 
               "description",       "NXT sample component", 
               "version",           "0.1", 
               "vendor",            "AIST", 
               "category",          "example", 
               "activity_type",     "DataFlowComponent", 
               "max_instance",      "10", 
               "language",          "Python", 
               "lang_type",         "SCRIPT",
               "conf.default.map", "A,B",
               ""]

# </rtc-template>

class NXTRTC(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

    # initialize of configuration-data.
    # <rtc-template block="configurations">
    self._map = [['A', 'B']]
    self._nxtbrick = None
    self._mapping = {'A':0,'B':1,'C':2}
     
  def onInitialize(self):
    # DataPorts initialization
    # <rtc-template block="data_ports">
    self._d_vel = RTC.TimedFloatSeq(RTC.Time(0,0),[])
    self._velIn = OpenRTM_aist.InPort("vel", self._d_vel)
    self.addInPort("vel",self._velIn)
    self._d_pos = RTC.TimedFloatSeq(RTC.Time(0,0),[])
    self._posOut = OpenRTM_aist.OutPort("pos", self._d_pos)
    self.addOutPort("pos",self._posOut)
    self._d_sens = RTC.TimedFloatSeq(RTC.Time(0,0),[])
    self._sensOut = OpenRTM_aist.OutPort("sens", self._d_sens)
    self.addOutPort("sens",self._sensOut)

    # Bind variables and configuration variable
    # <rtc-template block="bind_config">
    self.bindParameter("map", self._map, "A,B")

    # create NXTBrick object
    try:
      print "Connecting to NXT brick ...."
      self._nxtbrick = NXTBrick.NXTBrick()
      print "Connection established."
    except:
      print "NXTBrick connection failed."
      return RTC.RTC_ERROR

    return RTC.RTC_OK

  def onFinalize(self):
    self._nxtbrick.close()

  def onActivated(self, ec_id):
    self._nxtbrick.setMotors([0,0,0])
    # reset NXTBrick's position.
    self._nxtbrick.resetPosition()
    return RTC.RTC_OK

  def onDeactivated(self, ec_id):
    self._nxtbrick.setMotors([0,0,0])
    # reset NXTBrick's position.
    self._nxtbrick.resetPosition()

    return RTC.RTC_OK

  def onExecute(self, ec_id):
    try:
      # check new data.
      if self._velIn.isNew():
        # read velocity data from inport.
        self._d_vel = self._velIn.read()
        
        vel_ = [0,0,0]
        vel_[self._mapping[self._map[0][0]]] = self._d_vel.data[0]
        vel_[self._mapping[self._map[0][1]]] = self._d_vel.data[1]
        # set velocity
#       print vel_
        self._nxtbrick.setMotors(vel_)
      else:
        print "buffer empty"

      # get sensor data.
      sensor_   = self._nxtbrick.getSensors()
      if sensor_:
        self._d_sens.data = sensor_
        # write sensor data to outport.
        self._sensOut.write()

      # get position data.
      position_ = self._nxtbrick.getMotors()
      if position_:
        self._d_pos.data = \
            [position_[self._mapping[self._map[0][0]]][9], \
               position_[self._mapping[self._map[0][1]]][9]]
        # write position data to outport.
        self._posOut.write()
    except:
      print sys.exc_info()[1]

    return RTC.RTC_OK



def NXTRTCInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=nxtrtc_spec)
  manager.registerFactory(profile,
                          NXTRTC,
                          OpenRTM_aist.Delete)


def MyModuleInit(manager):
  NXTRTCInit(manager)

  # Create a component
  comp = manager.createComponent("NXTRTC")



def main():
  mgr = OpenRTM_aist.Manager.init(sys.argv)
  mgr.setModuleInitProc(MyModuleInit)
  mgr.activateManager()
  mgr.runManager()

if __name__ == "__main__":
  main()


