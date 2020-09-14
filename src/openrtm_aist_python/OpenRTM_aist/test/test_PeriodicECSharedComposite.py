#!/usr/bin/env python
# -*- Python -*-


## \file test_PeriodicECSharedComposite.py
## \brief test for PeriodicECSharedComposite class
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

import sys
sys.path.insert(1,"../")
sys.path.insert(1,"../RTM_IDL")

import OpenRTM_aist
import RTC

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


class TestComp(OpenRTM_aist.DataFlowComponentBase):

	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

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
	global com
	profile = OpenRTM_aist.Properties(defaults_str=configsample_spec)
	manager.registerFactory(profile,
				TestComp,
				OpenRTM_aist.Delete)
	com = manager.createComponent("TestComp")


class TestPeriodicECSharedComposite(unittest.TestCase):

	def setUp(self):
		self.manager = OpenRTM_aist.Manager.init(sys.argv)
		self.composite = OpenRTM_aist.PeriodicECSharedComposite(self.manager)
		return

	def test_organization(self):
		global com
		self.manager.setModuleInitProc(TestCompInit)
		self.manager.activateManager()
		organization = OpenRTM_aist.PeriodicECOrganization(self.composite)
		self.assertEqual(organization.set_members([com.getObjRef()]),True)
		#self.assertEqual(organization.add_members([com.getObjRef()]),True)
		self.assertEqual(organization.remove_member("TestComp0"),True)
		self.assertEqual(organization.add_members([com.getObjRef()]),True)
		organization.removeAllMembers()
		return

	
	def test_onInitialize(self):
		self.assertEqual(self.composite.onInitialize(), RTC.RTC_OK)
		return
	
	
	def test_onActivated(self):
		self.assertEqual(self.composite.onActivated(None), RTC.RTC_OK)
		return
	
	
	def test_onDeactivated(self):
		self.assertEqual(self.composite.onDeactivated(None), RTC.RTC_OK)
		return
	
	
	def test_onReset(self):
		self.assertEqual(self.composite.onReset(None), RTC.RTC_OK)
		return


	def test_delegatePort(self):
		return



############### test #################
if __name__ == '__main__':
        unittest.main()
