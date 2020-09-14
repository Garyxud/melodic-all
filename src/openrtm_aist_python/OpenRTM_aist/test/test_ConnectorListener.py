#!/usr/bin/env python
# -*- Python -*-

##
# @file test_ConnectorListener.py
# @brief test for connector listener class
# @date $Date: 2010/01/06 $
# @author Shinji Kurihara
#
# Copyright (C) 2010
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys
sys.path.insert(1,"../")

import unittest
import OpenRTM_aist
from ConnectorListener import *

import RTC, RTC__POA
import OpenRTM

from omniORB import *
from omniORB import any

class DataListener(OpenRTM_aist.ConnectorDataListenerT):
	def __init__(self, name):
		self._name = name
		self._data = None
		return

	def __del__(self):
		return

	def __call__(self, info, cdrdata):
		data = OpenRTM_aist.ConnectorDataListenerT.__call__(self, info, cdrdata, RTC.TimedLong(RTC.Time(0,0),0))
		print "------------------------------"
		print "Listener:       ", self._name
		print "------------------------------"
		self._data = data
		return

	def get_data(self):
		tmp = self._data
		self._data = None
		return tmp



class Listener(OpenRTM_aist.ConnectorListener):
	def __init__(self, name):
		self._name = name
		return

	def __del__(self):
		return

	def __call__(self, info):
		print "------------------------------"
		print "Listener:       ", self._name
		print "------------------------------"
		return


def typeToStringDataListener(type):
	if type == OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE:
		return "ON_BUFFER_WRITE"

	elif type == OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_FULL:
		return "ON_BUFFER_FULL"

	elif type == OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE_TIMEOUT:
		return "ON_BUFFER_WRITE_TIMEOUT"

	elif type == OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_OVERWRITE:
		return "ON_BUFFER_OVERWRITE"

	elif type == OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_READ:
		return "ON_BUFFER_READ"

	elif type == OpenRTM_aist.ConnectorDataListenerType.ON_SEND:
		return "ON_SEND"

	elif type == OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVED:
		return "ON_RECEIVED"

	elif type == OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_FULL:
		return "ON_RECEIVER_FULL"

	elif type == OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_TIMEOUT:
		return "ON_RECEIVER_TIMEOUT"

	elif type == OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_ERROR:
		return "ON_RECEIVER_ERROR"

	else:
		return "UNKNOWN"

		
def typeToStringListener(type):
	if type == OpenRTM_aist.ConnectorListenerType.ON_BUFFER_EMPTY:
		return "ON_BUFFER_EMPTY"

	elif type == OpenRTM_aist.ConnectorListenerType.ON_BUFFER_READ_TIMEOUT:
		return "ON_BUFFER_READ_TIMEOUT"

	elif type == OpenRTM_aist.ConnectorListenerType.ON_SENDER_EMPTY:
		return "ON_SENDER_EMPTY"

	elif type == OpenRTM_aist.ConnectorListenerType.ON_SENDER_TIMEOUT:
		return "ON_SENDER_TIMEOUT"

	elif type == OpenRTM_aist.ConnectorListenerType.ON_SENDER_ERROR:
		return "ON_SENDER_ERROR"

	else:
		return "UNKNOWN"

		
class TestConnectorListener(unittest.TestCase):
	def setUp(self):
		self._connectorListeners = ConnectorListeners()
		self._info      = OpenRTM_aist.ConnectorInfo("name",
							     "id",
							     [],
							     OpenRTM_aist.Properties())

		self._datalisteners = []
		self._listeners = []

		for i in range(OpenRTM_aist.ConnectorDataListenerType.CONNECTOR_DATA_LISTENER_NUM):
			self._datalisteners.append(DataListener(typeToStringDataListener(i)))

		for i in range(OpenRTM_aist.ConnectorListenerType.CONNECTOR_LISTENER_NUM):
			self._listeners.append(Listener(typeToStringListener(i)))

		return


	def test_ConnectorDataListenerT(self):
		# add DataListener.
		for i in range(OpenRTM_aist.ConnectorDataListenerType.CONNECTOR_DATA_LISTENER_NUM):
			self._connectorListeners.connectorData_[i].addListener(self._datalisteners[i],True)

		# test for ConnectorDataListenerT.__call__() with little endian data.
		for i in range(OpenRTM_aist.ConnectorDataListenerType.CONNECTOR_DATA_LISTENER_NUM):
			data = RTC.TimedLong(RTC.Time(0,0),i)
			cdr_data = cdrMarshal(any.to_any(data).typecode(), data, True) # little endian
			self._connectorListeners.connectorData_[i].notify(self._info, cdr_data)
			self.assertEqual(self._datalisteners[i].get_data().data, i)
			

		# test for ConnectorDataListenerT.__call__() with big endian data.
		info = OpenRTM_aist.ConnectorInfo("name",
						  "id",
						  [],
						  OpenRTM_aist.Properties())
		info.properties.setProperty("serializer.cdr.endian","big")

		for i in range(OpenRTM_aist.ConnectorDataListenerType.CONNECTOR_DATA_LISTENER_NUM):
			data = RTC.TimedLong(RTC.Time(0,0),i)
			cdr_data = cdrMarshal(any.to_any(data).typecode(), data, False) # big endian
			self._connectorListeners.connectorData_[i].notify(info, cdr_data)
			self.assertEqual(self._datalisteners[i].get_data().data, i)
			

		# remove DataListener.
		for i in range(OpenRTM_aist.ConnectorDataListenerType.CONNECTOR_DATA_LISTENER_NUM):
			self._connectorListeners.connectorData_[i].removeListener(self._datalisteners[i])

		for i in range(OpenRTM_aist.ConnectorDataListenerType.CONNECTOR_DATA_LISTENER_NUM):
			data = RTC.TimedLong(RTC.Time(0,0),i)
			cdr_data = cdrMarshal(any.to_any(data).typecode(), data, True)
			self._connectorListeners.connectorData_[i].notify(self._info, cdr_data)
			# get_data() return None, because removeListener was called.
			self.assertEqual(self._datalisteners[i].get_data(), None)
			
		return


	def test_ConnectorListener(self):
		# add Listener.
		for i in range(OpenRTM_aist.ConnectorListenerType.CONNECTOR_LISTENER_NUM):
			self._connectorListeners.connector_[i].addListener(self._listeners[i],True)

		# test for ConnectorListener.__call__()
		for i in range(OpenRTM_aist.ConnectorListenerType.CONNECTOR_LISTENER_NUM):
			self._connectorListeners.connector_[i].notify(self._info)
			
		# remove Listener.
		for i in range(OpenRTM_aist.ConnectorListenerType.CONNECTOR_LISTENER_NUM):
			self._connectorListeners.connector_[i].removeListener(self._listeners[i])

		for i in range(OpenRTM_aist.ConnectorListenerType.CONNECTOR_LISTENER_NUM):
			self._connectorListeners.connector_[i].notify(self._info)
			
		return


		


############### test #################
if __name__ == '__main__':
        unittest.main()

