#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys

from omniORB import CORBA

import RTC
import OpenRTM
import OpenRTM_aist


def main():

  # subscription type
  subs_type = "Flush"

  # initialization of ORB
  orb = CORBA.ORB_init(sys.argv)

  # get NamingService
  naming = OpenRTM_aist.CorbaNaming(orb, "localhost")
    
  conin = OpenRTM_aist.CorbaConsumer()
  conout = OpenRTM_aist.CorbaConsumer()

  ec0 = OpenRTM_aist.CorbaConsumer(interfaceType=OpenRTM.ExtTrigExecutionContextService)
  ec1 = OpenRTM_aist.CorbaConsumer(interfaceType=OpenRTM.ExtTrigExecutionContextService)

  # find ConsoleIn0 component
  conin.setObject(naming.resolve("ConsoleIn0.rtc"))

  # get ports
  inobj = conin.getObject()._narrow(RTC.RTObject)
  pin = inobj.get_ports()
  pin[0].disconnect_all()

  # activate ConsoleIn0
  eclisti = inobj.get_owned_contexts()
  eclisti[0].activate_component(inobj)
  ec0.setObject(eclisti[0])


  # find ConsoleOut0 component
  conout.setObject(naming.resolve("ConsoleOut0.rtc"))

  # get ports
  outobj = conout.getObject()._narrow(RTC.RTObject)
  pout = outobj.get_ports()
  pout[0].disconnect_all()
  
  # activate ConsoleOut0
  eclisto = outobj.get_owned_contexts()
  eclisto[0].activate_component(outobj)
  ec1.setObject(eclisto[0])


  # connect ports
  conprof = RTC.ConnectorProfile("connector0", "", [pin[0],pout[0]], [])
  OpenRTM_aist.CORBA_SeqUtil.push_back(conprof.properties,
                                       OpenRTM_aist.NVUtil.newNV("dataport.interface_type",
                                                                 "corba_cdr"))

  OpenRTM_aist.CORBA_SeqUtil.push_back(conprof.properties,
                                       OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type",
                                                                 "push"))

  OpenRTM_aist.CORBA_SeqUtil.push_back(conprof.properties,
                                       OpenRTM_aist.NVUtil.newNV("dataport.subscription_type",
                                                                 subs_type))

  ret = pin[0].connect(conprof)

    
  while 1:
    try:
      print "\n\n"
      print "0: tick ConsoleIn component"
      print "1: tick ConsoleOut component"
      print "2: tick both components"
      print "q: exit"
      print "cmd? >",
      cmd = str(sys.stdin.readline())
      if cmd == "0\n":
        ec0._ptr().tick()
      elif cmd == "1\n":
        ec1._ptr().tick()
      elif cmd == "2\n":
        ec0._ptr().tick()
        ec1._ptr().tick()
      elif cmd == "q\n":
        print "exit"
        break
                
    except:
      print "Exception."
      pass

if __name__ == "__main__":
  main()
