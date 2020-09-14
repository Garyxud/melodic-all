#!/usr/bin/env python
# -*- Python -*-

#
# \file test_SdoConfiguration.py
# \brief test for RT component base class
# \date $Date: 2007/09/06$
# \author Shinji Kurihara
#
# Copyright (C) 2006
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys
sys.path.insert(1,"../")
sys.path.insert(1,"../RTM_IDL")

import unittest
import OpenRTM_aist
import SDOPackage, SDOPackage__POA

from SdoConfiguration import *

import CORBA
from omniORB import CORBA, PortableServer

class ServiceProf(SDOPackage__POA.SDOService):
  def __init__(self):
    pass

class OrganizationTest(SDOPackage__POA.Organization):
  def __init__(self):
    pass


configsample_spec = ["implementation_id", "ConfigSample",
           "type_name",         "ConfigSample",
           "description",       "Configuration example component",
           "version",           "1.0",
           "vendor",            "Shinji Kurihara, AIST",
           "category",          "example",
           "activity_type",     "DataFlowComponent",
           "max_instance",      "10",
           "language",          "C++",
           "lang_type",         "compile",
           # Configuration variables
           "conf.default.int_param0", "0",
           "conf.default.int_param1", "1",
           "conf.default.double_param0", "0.11",
           "conf.default.double_param1", "9.9",
           "conf.default.str_param0", "hoge",
           "conf.default.str_param1", "dara",
           "conf.default.vector_param0", "0.0,1.0,2.0,3.0,4.0",
           ""]

configsample_mode_spec = ["conf.default.int_param0", "10",
              "conf.default.int_param1", "11",
              "conf.default.double_param0", "0.22",
              "conf.default.double_param1", "9999.9",
              "conf.default.str_param0", "hogehoge",
              "conf.default.str_param1", "daradaradata",
              "conf.default.vector_param0", "0.1,1.1,2.1,3.1,4.1",
              ""]


class TestConfiguration_impl(unittest.TestCase):
  def setUp(self):
    self._orb = CORBA.ORB_init()
    self._poa = self._orb.resolve_initial_references("RootPOA")
    
    prop = OpenRTM_aist.Properties(defaults_str=configsample_spec)
    ca   = OpenRTM_aist.ConfigAdmin(prop.getNode("conf"))
    self._sdoconf = Configuration_impl(ca)

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_set_device_profile(self):
    dprof = SDOPackage.DeviceProfile("test","","","0.1.0",[])
    self._sdoconf.set_device_profile(dprof)
    self.assertEqual(self._sdoconf.getDeviceProfile().device_type, "test")
    self.assertEqual(self._sdoconf.getDeviceProfile().version, "0.1.0")
    
    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.set_device_profile, None )


  def test_add_service_profile(self):
    sprof = SDOPackage.ServiceProfile("test","MyService",[],ServiceProf())
    self._sdoconf.add_service_profile(sprof)
    profiles = self._sdoconf.getServiceProfiles()
    self.assertEqual(profiles[0].id, "test")
    self.assertEqual(self._sdoconf.getServiceProfile("test").id, "test")
    self._sdoconf.remove_service_profile("test")
    self.assertEqual(self._sdoconf.getServiceProfile("test").id, "")

    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.add_service_profile, None )
    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.remove_service_profile, None )

    # Failure pattern
    #self.assertEqual(self._sdoconf.getServiceProfile("test").id, "test")

  def test_add_organization(self):
    org = OrganizationTest()
    self._sdoconf.add_organization(org)
    self._sdoconf.add_organization(org)
    self._sdoconf.add_organization(org)
    self.assertEqual(len(self._sdoconf.getOrganizations()), 3)

    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.add_organization, None )
    self.assertRaises(SDOPackage.InternalError, self._sdoconf.remove_organization, "aaa" )
    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.remove_organization, None )


  def test_get_configuration_parameters(self):
    self._sdoconf.get_configuration_parameters()


  def test_get_configuration_parameter_values(self):
    param_val = self._sdoconf.get_configuration_parameter_values()
    self.assertEqual(param_val, [])

  def test_get_configuration_parameter_value(self):
    param_val = self._sdoconf.get_configuration_parameter_value("aaa")
    self.assertEqual(param_val, None)
    
    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.get_configuration_parameter_value, None )


  def test_set_configuration_parameter(self):
    self.assertEqual(self._sdoconf.set_configuration_parameter("name",123), True)
    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.set_configuration_parameter, None,None )
    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.set_configuration_parameter, "name",None )
    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.set_configuration_parameter, None,123 )

  def test_get_configuration_sets(self):
    conf_sets = self._sdoconf.get_configuration_sets()
    self.assertEqual(len(conf_sets), 1)


  def test_get_configuration_set(self):
    conf_set = self._sdoconf.get_configuration_set("default")
    self.assertEqual(conf_set.configuration_data[0].name, "int_param0")
    self.assertEqual(conf_set.configuration_data[0].value.value(), "0")

    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.get_configuration_set, None )

  def test_set_configuration_set_values(self):
    self._sdoconf.set_configuration_set_values(SDOPackage.ConfigurationSet("default2","",[]))

    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.set_configuration_set_values, None )

  def test_get_active_configuration_set(self):
    self.assertEqual(self._sdoconf.get_active_configuration_set().id, "default")


  def test_add_configuration_set(self):
    self._sdoconf.add_configuration_set(SDOPackage.ConfigurationSet("default2","",[]))

    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.add_configuration_set, None )

  def test_remove_configuration_set(self):
    self._sdoconf.add_configuration_set(SDOPackage.ConfigurationSet("default2","",[]))
    self.assertEqual(self._sdoconf.remove_configuration_set("default2"),True)
    
    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.remove_configuration_set, None )

  def test_activate_configuration_set(self):
    self._sdoconf.add_configuration_set(SDOPackage.ConfigurationSet("default2","",[]))
    self.assertEqual(self._sdoconf.activate_configuration_set("default2"),True)

    self.assertRaises(SDOPackage.InvalidParameter, self._sdoconf.activate_configuration_set, None )

  def test_getObjRef(self):
    self._sdoconf.getObjRef()

  def test_getOrganizations(self):
    self._sdoconf.getOrganizations()

  def test_getDeviceProfile(self):
    self._sdoconf.getDeviceProfile()

  def test_getServiceProfiles(self):
    self._sdoconf.getServiceProfiles()

  def test_getServiceProfile(self):
    self._sdoconf.getServiceProfile("default")

  def test_getOrganizations(self):
    self._sdoconf.getOrganizations()

  def test_getUUID(self):
    print self._sdoconf.getUUID()



############### test #################
if __name__ == '__main__':
        unittest.main()
