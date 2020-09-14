#!/usr/bin/env python
# -*- Python -*-


## \file test_RTObject.py
## \brief test for RT component base class
## \date $Date: $
## \author Shinji Kurihara
#
# Copyright (C) 2006
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys,time
sys.path.insert(1,"../")
sys.path.insert(1,"../RTM_IDL")

import RTC
import OpenRTM
import SDOPackage
import OpenRTM_aist
from omniORB import CORBA, PortableServer
from omniORB import any
import unittest

configsample_spec = ["implementation_id", "TestComp",
         "type_name",         "TestComp",
         "description",       "Test example component",
         "version",           "1.0",
         "vendor",            "Shinji Kurihara, AIST",
         "category",          "example",
         "activity_type",     "DataFlowComponent",
         "max_instance",      "10",
         "language",          "Python",
         "lang_type",         "script",
         # Configuration variables
         "conf.default.int_param0", "0",
         "conf.default.int_param1", "1",
         "conf.default.double_param0", "0.11",
         "conf.default.double_param1", "9.9",
         "conf.default.str_param0", "hoge",
         "conf.default.str_param1", "dara",
         "conf.default.vector_param0", "0.0,1.0,2.0,3.0,4.0",
         ""]

com = None

class TestComp(OpenRTM_aist.RTObject_impl):
  def __init__(self, orb_, poa_):
    OpenRTM_aist.RTObject_impl.__init__(self, orb=orb_, poa=poa_)

  def onInitialize(self):
    print "onInitialize"
    return RTC.RTC_OK

  def onFinalize(self):
    print "onFinalize"
    return RTC.RTC_OK
    
  def onStartup(self, ec_id):
    print "onStartup"
    return RTC.RTC_OK

  def onShutdown(self, ec_id):
    print "onSutdown"
    return RTC.RTC_OK

  def onActivated(self, ec_id):
    print "onActivated"
    return RTC.RTC_OK

  def onDeactivated(self, ec_id):
    print "onDeactivated"
    return RTC.RTC_OK

  def onExecute(self, ec_id):
    print "onExecute"
    return RTC.RTC_OK

  def onAborting(self, ec_id):
    print "onAborting"
    return RTC.RTC_OK

  def onReset(self, ec_id):
    print "onReset"
    return RTC.RTC_OK
    
  def onStateUpdate(self, ec_id):
    print "onStateUpdate"
    return RTC.RTC_OK

  def onRateChanged(self, ec_id):
    print "onRateChanged"
    return RTC.RTC_OK

    
def TestCompInit(manager):
  print "TestCompInit"
  global com
  profile = OpenRTM_aist.Properties(defaults_str=configsample_spec)
  manager.registerFactory(profile,
        TestComp,
        OpenRTM_aist.Delete)
  com = manager.createComponent("TestComp")


class MySdoServiceProviderBase(OpenRTM_aist.SdoServiceProviderBase):
  def __init__(self):
    self._profile = None
    self._rtobj = None
    return

  def __del__(self):
    return

  def init(self, rtobj, profile):
    self._rtobj = rtobj
    self._profile = profile
    return

  def reinit(self, profile):
    return

  def getProfile(self):
    return self._profile

  def finalize(self):
    return


class TestRTObject_impl(unittest.TestCase):
  def setUp(self):
    self._orb = CORBA.ORB_init(sys.argv)
    self._poa = self._orb.resolve_initial_references("RootPOA")
    self._poa._get_the_POAManager().activate()
    return

  def tearDown(self):
    #global com
    #self.rtobj.exit()
    #self.manager.terminate()
    time.sleep(0.1)
    OpenRTM_aist.Manager.instance().shutdownManager()
    #com = None
    return

  def test_is_alive(self):
    rtobj = TestComp(self._orb, self._poa)
    ec = rtobj.getObjRef().get_context(0)
    self.assertEqual(ec,None)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    self.assertNotEqual(rtobj.getObjRef().get_owned_contexts(),[])
    self.assertEqual(rtobj.is_alive(ec.getObjRef()),True)
    ec.remove_component(rtobj.getObjRef())
    ec.stop()
    del ec

    return

  def test_get_owned_contexts(self):
    rtobj = TestComp(self._orb, self._poa)
    self.assertEqual(rtobj.getObjRef().get_owned_contexts(),[])
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    self.assertNotEqual(rtobj.getObjRef().get_owned_contexts(),[])
    ec.remove_component(rtobj.getObjRef())
    ec.stop()
    del ec

    return

  def test_get_participating_contexts(self):
    rtobj = TestComp(self._orb, self._poa)
    self.assertEqual(rtobj.getObjRef().get_participating_contexts(),[])
    return

  def test_get_context(self):
    rtobj = TestComp(self._orb, self._poa)
    print rtobj.getObjRef().get_context(0)
    return

  def test_get_component_profile(self):
    rtobj = TestComp(self._orb, self._poa)
    rtobj.setInstanceName("TestComp0")
    prof = rtobj.getObjRef().get_component_profile()
    self.assertEqual(prof.instance_name, "TestComp0")
    return

  def test_get_ports(self):
    rtobj = TestComp(self._orb, self._poa)
    self.assertEqual(rtobj.getObjRef().get_ports(), [])
    return


  def test_attach_context(self):
    rtobj = TestComp(self._orb, self._poa)
    ec = OpenRTM_aist.PeriodicExecutionContext(rtobj.getObjRef(), 10)
    id = rtobj.getObjRef().attach_context(ec.getObjRef())
    print "attach_context: ", id
    print rtobj.getObjRef().detach_context(id)
    poa = OpenRTM_aist.Manager.instance().getPOA()
    poa.deactivate_object(poa.servant_to_id(ec))
    return

  def test_get_owned_organizations(self):
    rtobj = TestComp(self._orb, self._poa)
    self.assertEqual(rtobj.getObjRef().get_owned_organizations(),[])
    return
    
  def test_get_sdo_id(self):
    rtobj = TestComp(self._orb, self._poa)
    rtobj.setInstanceName("TestComp0")
    self.assertEqual(rtobj.getObjRef().get_sdo_id(), "TestComp0")
    return

  def test_get_sdo_type(self):
    rtobj = TestComp(self._orb, self._poa)
    prop = OpenRTM_aist.Properties(defaults_str=configsample_spec)
    rtobj.setProperties(prop)
    self.assertEqual(rtobj.getObjRef().get_sdo_type(), "Test example component")
    return

  def test_get_device_profile(self):
    rtobj = TestComp(self._orb, self._poa)
    prof = rtobj.getObjRef().get_device_profile()
    self.assertEqual(prof.device_type, "")
    return

  def test_get_service_profiles(self):
    rtobj = TestComp(self._orb, self._poa)
    self.assertEqual(rtobj.getObjRef().get_service_profiles(),[])
    return


  def test_get_service_profile(self):
    #rtobj.getObjRef().get_service_profile("TestComp")
    return


  def test_get_sdo_service(self):
    #rtobj.getObjRef().get_sdo_service(None)
    return

  def test_get_configuration(self):
    rtobj = TestComp(self._orb, self._poa)
    print rtobj.getObjRef().get_configuration()
    return

  def test_get_monitoring(self):
    #rtobj.getObjRef().get_monitoring()
    return

  def test_get_organizations(self):
    rtobj = TestComp(self._orb, self._poa)
    self.assertEqual(rtobj.getObjRef().get_organizations(), [])
    return

  def test_get_status_list(self):
    rtobj = TestComp(self._orb, self._poa)
    self.assertEqual(rtobj.getObjRef().get_status_list(), [])
    return

  def test_get_status(self):
    #rtobj.getObjRef().get_status("status")
    return

  def test_getPropTestCase(self):
    rtobj = TestComp(self._orb, self._poa)
    self.assertEqual(rtobj.getInstanceName(), "")
    prop = OpenRTM_aist.Properties(defaults_str=configsample_spec)
    rtobj.setInstanceName("TestComp0")
    rtobj.setProperties(prop)
    self.assertEqual(rtobj.getInstanceName(), "TestComp0")
    self.assertEqual(rtobj.getTypeName(), "TestComp")
    self.assertEqual(rtobj.getDescription(), "Test example component")
    self.assertEqual(rtobj.getVersion(), "1.0")
    self.assertEqual(rtobj.getVendor(), "Shinji Kurihara, AIST")
    self.assertEqual(rtobj.getCategory(), "example")
    self.assertNotEqual(rtobj.getNamingNames(),["TestComp0.rtc"])
    return

  def test_setObjRef(self):
    rtobj = TestComp(self._orb, self._poa)
    rtobj.setObjRef("test")
    self.assertEqual(rtobj.getObjRef(),"test")
    return

  def test_bindParameter(self):
    rtobj = TestComp(self._orb, self._poa)
    conf_ = [123]
    self.assertEqual(rtobj.bindParameter("config", conf_, 0), True)
    rtobj.updateParameters("")
    return

  def test_PortTestCase(self):
    rtobj = TestComp(self._orb, self._poa)
    ringbuf = OpenRTM_aist.RingBuffer(8)
    outp = OpenRTM_aist.OutPort("out", RTC.TimedLong(RTC.Time(0,0),0), ringbuf)
    rtobj.registerOutPort("out",outp)

    ringbuf = OpenRTM_aist.RingBuffer(8)
    inp = OpenRTM_aist.InPort("in", RTC.TimedLong(RTC.Time(0,0),0), ringbuf)
    rtobj.registerInPort("in",inp)
    
    rtobj.deletePort(outp)
    rtobj.deletePort(inp)

    rtobj.finalizePorts()
    return

  # since 1.1.0
  def test_getExecutionContext(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    self.assertNotEqual(rtobj.getExecutionContext(0),None)
    return

  def test_getExecutionRate(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    self.assertEqual(rtobj.getExecutionRate(0),1000.0)
    return

  def test_setExecutionRate(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    self.assertEqual(rtobj.setExecutionRate(0,10000),RTC.RTC_OK)
    self.assertEqual(rtobj.getExecutionRate(0),10000.0)
    return

  def test_isOwnExecutionContext(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    self.assertEqual(rtobj.isOwnExecutionContext(0),True)
    return

  def test_activate_deactivate(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.set_rate(1000.0)
    ec.bindComponent(rtobj)
    self.assertEqual(rtobj.activate(0),RTC.RTC_OK)
    ec.start()
    time.sleep(0.1)
    ret = rtobj.deactivate(0)
    time.sleep(0.1)
    self.assertEqual(ret,RTC.RTC_OK)
    ec.stop()
    return

  def test_reset(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    self.assertEqual(rtobj.activate(0),RTC.RTC_OK)
    ec.start()
    ec._comps[0]._sm._sm.goTo(RTC.ERROR_STATE)
    time.sleep(0.1)
    self.assertEqual(rtobj.reset(0),RTC.RTC_OK)
    ec.stop()
    return

  def test_addRemoveSdoServiceProvider(self):
    rtobj = TestComp(self._orb, self._poa)
    prof = SDOPackage.ServiceProfile("id","interface_type",
                                     OpenRTM_aist.NVUtil.newNV("test","any"),
                                     SDOPackage.SDOService._nil)
    prov = MySdoServiceProviderBase()
    prov.init(rtobj,prof)
    self.assertEqual(rtobj.addSdoServiceProvider(prof, prov),True)
    self.assertEqual(rtobj.removeSdoServiceProvider("id"),True)
    return

  def test_addRemoveSdoServiceConsumer(self):
    import MySdoServiceConsumer
    OpenRTM_aist.Manager.instance().load("MySdoServiceConsumer.py",
                                         "MySdoServiceConsumerInit")
    rtobj = TestComp(self._orb, self._poa)
    prof = SDOPackage.ServiceProfile(OpenRTM_aist.toTypename(OpenRTM.ComponentObserver),OpenRTM_aist.toTypename(OpenRTM.ComponentObserver),
                                     [OpenRTM_aist.NVUtil.newNV("test","any")],
                                     SDOPackage.SDOService._nil)
    self.assertEqual(rtobj.addSdoServiceConsumer(prof),True)
    self.assertEqual(rtobj.removeSdoServiceConsumer(OpenRTM_aist.toTypename(OpenRTM.ComponentObserver)),True)
    return

  def prelistenerFunc(self, id):
    print "prelistenerFunc called !!!!"
    return

  def test_addRemovePreComponentActionListener(self):
    rtobj = TestComp(self._orb, self._poa)

    rtobj.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_INITIALIZE,
                                        self.prelistenerFunc)

    rtobj.removePreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_INITIALIZE,
                                           self.prelistenerFunc)

    rtobj.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_FINALIZE,
                                        self.prelistenerFunc)

    rtobj.removePreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_FINALIZE,
                                           self.prelistenerFunc)
    return

  def postlistenerFunc(self, id, ret):
    print "postlistenerFunc called !!!!"
    return

  def test_addRemovePostComponentActionListener(self):
    rtobj = TestComp(self._orb, self._poa)

    rtobj.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_FINALIZE,
                                         self.postlistenerFunc)

    rtobj.removePostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_FINALIZE,
                                            self.postlistenerFunc)
    return

  def test_addRemovePortActionListener(self):
    rtobj = TestComp(self._orb, self._poa)

    rtobj.addPortActionListener(OpenRTM_aist.PortActionListenerType.ADD_PORT,
                                self.prelistenerFunc)

    rtobj.removePortActionListener(OpenRTM_aist.PortActionListenerType.ADD_PORT,
                                   self.prelistenerFunc)
    return

  def test_addRemoveExecutionContextActionListener(self):
    rtobj = TestComp(self._orb, self._poa)

    rtobj.addExecutionContextActionListener(OpenRTM_aist.ExecutionContextActionListenerType.EC_ATTACHED,
                                            self.prelistenerFunc)

    rtobj.removeExecutionContextActionListener(OpenRTM_aist.ExecutionContextActionListenerType.EC_ATTACHED,
                                            self.prelistenerFunc)
    return

  def test_addRemovePortConnectListener(self):
    rtobj = TestComp(self._orb, self._poa)

    rtobj.addPortConnectListener(OpenRTM_aist.PortConnectListenerType.ON_NOTIFY_CONNECT,
                                 self.postlistenerFunc)

    rtobj.removePortConnectListener(OpenRTM_aist.PortConnectListenerType.ON_NOTIFY_CONNECT,
                                    self.postlistenerFunc)
    return

  def portconretlistenerFunc(self, pname, cprof, ret):
    print "portconretlistenerFunc called !!!!"
    return

  def test_addRemovePortConnectRetListener(self):
    rtobj = TestComp(self._orb, self._poa)

    rtobj.addPortConnectRetListener(OpenRTM_aist.PortConnectRetListenerType.ON_CONNECTED,
                                    self.portconretlistenerFunc)

    rtobj.removePortConnectRetListener(OpenRTM_aist.PortConnectRetListenerType.ON_CONNECTED,
                                       self.portconretlistenerFunc)
    return

  def configparamlistenerFunc(self, pname, cprof, ret):
    print "configparamlistenerFunc called !!!!"
    return

  def test_addRemoveConfigurationParamListener(self):
    rtobj = TestComp(self._orb, self._poa)

    rtobj.addConfigurationParamListener(OpenRTM_aist.ConfigurationParamListenerType.ON_UPDATE_CONFIG_PARAM,
                                        self.configparamlistenerFunc)

    rtobj.removeConfigurationParamListener(OpenRTM_aist.ConfigurationParamListenerType.ON_UPDATE_CONFIG_PARAM,
                                           self.configparamlistenerFunc)
    return

  def test_addRemoveConfigurationSetListener(self):
    rtobj = TestComp(self._orb, self._poa)

    rtobj.addConfigurationSetListener(OpenRTM_aist.ConfigurationSetListenerType.ON_SET_CONFIG_SET,
                                      self.prelistenerFunc)

    rtobj.removeConfigurationSetListener(OpenRTM_aist.ConfigurationSetListenerType.ON_SET_CONFIG_SET,
                                         self.prelistenerFunc)
    return

  def test_addRemoveConfigurationSetNameListener(self):
    rtobj = TestComp(self._orb, self._poa)

    rtobj.addConfigurationSetNameListener(OpenRTM_aist.ConfigurationSetNameListenerType.ON_UPDATE_CONFIG_SET,
                                          self.prelistenerFunc)

    rtobj.removeConfigurationSetNameListener(OpenRTM_aist.ConfigurationSetNameListenerType.ON_UPDATE_CONFIG_SET,
                                             self.prelistenerFunc)
    return

  def test_shutdown(self):
    return

  def test_preOnInitialize(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    print "preOnInitialize()"
    rtobj.preOnInitialize(0)
    return

  def test_preOnFinalize(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.preOnFinalize(0)
    return

  def test_preOnStartup(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.preOnStartup(0)
    return

  def test_preOnShutdown(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.preOnShutdown(0)
    return

  def test_preOnActivated(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.preOnActivated(0)
    return

  def test_preOnDeactivated(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.preOnDeactivated(0)
    return

  def test_preOnAborting(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.preOnAborting(0)
    return

  def test_preOnError(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.preOnError(0)
    return

  def test_preOnReset(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.preOnReset(0)
    return

  def test_preOnExecute(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.preOnExecute(0)
    return

  def test_preOnStateUpdate(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.preOnStateUpdate(0)
    return

  def test_preOnRateChanged(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.preOnRateChanged(0)
    return

  def test_postOnInitialize(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.postOnInitialize(0,True)
    return

  def test_postOnFinalize(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.postOnFinalize(0,True)
    return

  def test_postOnStartup(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.postOnStartup(0,True)
    return

  def test_postOnShutdown(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.postOnShutdown(0,True)
    return

  def test_postOnActivated(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.postOnActivated(0,True)
    return

  def test_postOnDeactivated(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.postOnDeactivated(0,True)
    return

  def test_postOnAborting(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.postOnAborting(0,True)
    return

  def test_postOnError(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.postOnError(0,True)
    return

  def test_postOnReset(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.postOnReset(0,True)
    return

  def test_postOnExecute(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.postOnExecute(0,True)
    return

  def test_postOnStateUpdate(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.postOnStateUpdate(0,True)
    return

  def test_postOnRateChanged(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.postOnRateChanged(0,True)
    return

  def test_onAddPort(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.onAddPort(0)
    return

  def test_onRemovePort(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.onRemovePort(0)
    return

  def test_onAttachExecutionContext(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.onAttachExecutionContext(0)
    return

  def test_onDetachExecutionContext(self):
    rtobj = TestComp(self._orb, self._poa)
    ec_args = "PeriodicExecutionContext"+"?" + "rate=1000"
    ec=OpenRTM_aist.Manager.instance().createContext(ec_args)
    ec.bindComponent(rtobj)
    rtobj.onDetachExecutionContext(0)
    return


############### test #################
if __name__ == '__main__':
  unittest.main()
