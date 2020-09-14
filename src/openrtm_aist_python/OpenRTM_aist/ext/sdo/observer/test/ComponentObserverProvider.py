#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file ComponentObserverProvider.py
# @brief test for ComponentObserverConsumer
# @date $Date$
# @author Shinji Kurihara
#
# Copyright (C) 2011
#     Noriaki Ando
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys

from omniORB import CORBA, PortableServer
import RTC
import OpenRTM, OpenRTM__POA
import SDOPackage
import OpenRTM_aist

class ComponentObserver_i(OpenRTM__POA.ComponentObserver):
  def __init__(self):
    pass

  def update_status(self, status_kind, hint):
    print "update_status: ", status_kind, ", ", hint
    return

def main():
  orb = CORBA.ORB_init(sys.argv)
  poa = orb.resolve_initial_references("RootPOA")
  poa._get_the_POAManager().activate()
  naming = OpenRTM_aist.CorbaNaming(orb, "localhost")
  servant = ComponentObserver_i()
  oid = poa.servant_to_id(servant)
  provider = poa.id_to_reference(oid)

  rtc = naming.resolve("ConsoleIn0.rtc")._narrow(RTC.RTObject)
  config = rtc.get_configuration()
  properties = [OpenRTM_aist.NVUtil.newNV("heartbeat.enable","YES"),
                OpenRTM_aist.NVUtil.newNV("heartbeat.interval","10"),
                OpenRTM_aist.NVUtil.newNV("observed_status","ALL")]

  id = OpenRTM_aist.toTypename(servant)
  sprof = SDOPackage.ServiceProfile("test_id", id,
                                    properties, provider)

  ret = config.add_service_profile(sprof)
  flag = True
  print "If you exit program, please input 'q'."
  sys.stdin.readline()
  ret = config.remove_service_profile("test_id")
  print "test program end. ret : ", ret
  return
    
############### test #################
if __name__ == '__main__':
  main()
