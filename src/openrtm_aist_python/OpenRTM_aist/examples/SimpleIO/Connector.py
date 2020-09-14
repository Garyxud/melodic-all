#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys

from omniORB import CORBA
from optparse import OptionParser, OptionError

import RTC
import OpenRTM_aist

def usage():
  print "usage: ConnectorComp [options]"
  print "  --flush         "
  print ": Set subscription type Flush"
  print "  --new           "
  print ": Set subscription type New"
  print "  --periodic [Hz] "
  print ": Set subscription type Periodic \n"
  print "exsample:"
  print "  ConnectorComp --flush"
  print "  ConnectorComp --new"
  print "  ConnectorComp --new --policy ALL"
  print "  ConnectorComp --new --policy SKIP --skip 100"
  print "  ConnectorComp --periodic 10"
  print "  ConnectorComp --periodic 10 --policy FIFO"
  print "  ConnectorComp --periodic 10 --policy NEW \n"

def main():

  # initialization of ORB
  orb = CORBA.ORB_init(sys.argv)

  # get NamingService
  naming = OpenRTM_aist.CorbaNaming(orb, "localhost")
    
  conin = OpenRTM_aist.CorbaConsumer()
  conout = OpenRTM_aist.CorbaConsumer()

  # find ConsoleIn0 component
  conin.setObject(naming.resolve("ConsoleIn0.rtc"))

  # get ports
  inobj = conin.getObject()._narrow(RTC.RTObject)
  pin = inobj.get_ports()
  pin[0].disconnect_all()


  # find ConsoleOut0 component
  conout.setObject(naming.resolve("ConsoleOut0.rtc"))

  # get ports
  outobj = conout.getObject()._narrow(RTC.RTObject)
  pout = outobj.get_ports()
  pout[0].disconnect_all()


  # subscription type
  subs_type = "flush"
  period = "1.0"
  push_policy = "new"
  skip_count = "0"

  for arg in sys.argv[1:]:
    if arg == "--flush":
      subs_type = "flush"
      break

    elif arg == "--new":
      subs_type = "new"
      if len(sys.argv) > 2:
        push_policy = OpenRTM_aist.normalize([sys.argv[3]])
      if push_policy == "skip":
        skip_count = sys.argv[5]
      break

    elif arg == "--periodic":
      subs_type = "periodic"
      period = sys.argv[2]
      if len(sys.argv) > 3:
        push_policy = OpenRTM_aist.normalize([sys.argv[4]])
      if push_policy == "skip":
        skip_count = sys.argv[6]
      break

    # elif sbus_type == "periodic" and type(arg) == float:
    #    period = srt(arg)
    #    break

    else:
      usage()
            
  print "Subscription Type: ", subs_type
  print "Period: ", period, " [Hz]"
  print "push policy: ", push_policy
  print "skip count: ", skip_count

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

  if subs_type == "periodic":
    OpenRTM_aist.CORBA_SeqUtil.push_back(conprof.properties,
                                         OpenRTM_aist.NVUtil.newNV("dataport.publisher.push_rate",
                                                                       period))
  OpenRTM_aist.CORBA_SeqUtil.push_back(conprof.properties,
                                       OpenRTM_aist.NVUtil.newNV("dataport.publisher.push_policy",
                                                                 push_policy))
  if push_policy == "skip":
    OpenRTM_aist.CORBA_SeqUtil.push_back(conprof.properties,
                                         OpenRTM_aist.NVUtil.newNV("dataport.publisher.skip_count",
                                                                   skip_count))


  ret,conprof = pin[0].connect(conprof)
    
  # activate ConsoleIn0
  eclistin = inobj.get_owned_contexts()
  eclistin[0].activate_component(inobj)

  # activate ConsoleOut0
  eclistout = outobj.get_owned_contexts()
  eclistout[0].activate_component(outobj)


if __name__ == "__main__":
  main()
