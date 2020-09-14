#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys

from omniORB import CORBA

import RTC
import OpenRTM_aist


def main():

    # subscription type
    subs_type = "Flush"

    # initialization of ORB
    orb = CORBA.ORB_init(sys.argv)

    # get NamingService
    naming = OpenRTM_aist.CorbaNaming(orb, "localhost")
    
    sl  = OpenRTM_aist.CorbaConsumer()
    tkm = OpenRTM_aist.CorbaConsumer()

    # find TkMotorComp0 component
    tkm.setObject(naming.resolve("TkMotorComp0.rtc"))

    # get ports
    inobj = tkm.getObject()._narrow(RTC.RTObject)
    pin = inobj.get_ports()
    pin[0].disconnect_all()


    # find SliderComp0 component
    sl.setObject(naming.resolve("SliderComp0.rtc"))

    # get ports
    outobj = sl.getObject()._narrow(RTC.RTObject)
    pout = outobj.get_ports()
    pout[0].disconnect_all()


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
    
    # activate TkMotorComp0
    eclistin = inobj.get_owned_contexts()
    eclistin[0].activate_component(inobj)

    # activate SliderComp0
    eclistout = outobj.get_owned_contexts()
    eclistout[0].activate_component(outobj)



if __name__ == "__main__":
	main()
