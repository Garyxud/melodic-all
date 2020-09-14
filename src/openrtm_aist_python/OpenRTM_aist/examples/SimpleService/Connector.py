#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys

from omniORB import CORBA

import RTC
import OpenRTM_aist

def usage():
    print "usage: ConnectorComp [options]"
    print " python MyServiceProvider.py  "
    print " python MyServiceConsumer.py  "
    print " python Connector.py  "


def main():

    # initialization of ORB
    orb = CORBA.ORB_init(sys.argv)

    # get NamingService
    naming = OpenRTM_aist.CorbaNaming(orb, "localhost")
    
    consumer = OpenRTM_aist.CorbaConsumer()
    provider = OpenRTM_aist.CorbaConsumer()

    # find MyServiceConsumer0 component
    consumer.setObject(naming.resolve("MyServiceConsumer0.rtc"))

    # get ports
    consobj = consumer.getObject()._narrow(RTC.RTObject)
    pcons = consobj.get_ports()
    pcons[0].disconnect_all()


    # find MyServiceProvider0 component
    provider.setObject(naming.resolve("MyServiceProvider0.rtc"))

    # get ports
    provobj = provider.getObject()._narrow(RTC.RTObject)
    prov = provobj.get_ports()
    prov[0].disconnect_all()


    # connect ports
    conprof = RTC.ConnectorProfile("connector0", "", [pcons[0],prov[0]], [])

    ret = pcons[0].connect(conprof)
    
    # activate ConsoleIn0
    eclistin = consobj.get_owned_contexts()
    eclistin[0].activate_component(consobj)

    # activate ConsoleOut0
    eclistout = provobj.get_owned_contexts()
    eclistout[0].activate_component(provobj)



if __name__ == "__main__":
	main()
