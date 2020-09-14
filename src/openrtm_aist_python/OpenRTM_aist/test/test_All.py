#!/usr/bin/env python
# -*- Python -*-

import unittest
from test import test_support

from test_BufferBase import *
from test_ConfigAdmin import *
from test_CORBA_SeqUtil import *
from test_CorbaConsumer import *
from test_CorbaNaming import *
from test_CorbaPort import *
from test_ECFactory import *
from test_ExtTrigExecutionContext import *
from test_Factory import *
from test_InPort import *
from test_InPortBase import *
from test_InPortProvider import *
from test_Listener import *
from test_Manager import *
from test_ManagerConfig import *
from test_ModuleManager import *
from test_NamingManager import *
from test_NumberingPolicy import *
from test_NVUtil import *
from test_ObjectManager import *
from test_OutPort import *
from test_OutPortBase import *
from test_OutPortProvider import *
from test_PeriodicExecutionContext import *
from test_PortAdmin import *
from test_PortBase import *
from test_Properties import *
from test_PublisherNew import *
from test_PublisherPeriodic import *
from test_RingBuffer import *
from test_RTCUtil import *
from test_RTObject import *
from test_SdoConfiguration import *
from test_SdoOrganization import *
from test_SdoService import *
from test_StateMachine import *
from test_StringUtil import *
from test_SystemLogger import *
from test_Timer import *
from test_TimeValue import *


# \file test_All.py
#
# Copyright (C) 2006
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

def test_main():
  test_support.run_unittest(
    TestNullBuffer,
    TestConfigAdmin,
    TestCORBA_SeqUtil, 
    TestCorbaConsumer, 
    TestCorbaNaming, 
    TestCorbaPort, 
    TestECFactoryPython, 
    TestExtTrigExecutionContext, 
    TestFactoryPython, 
    TestInPort, 
    TestInPortBase, 
    TestInPortProvider, 
    TestListener, 
    TestManagerConfig, 
    TestModuleManager, 
    TestDefaultNumberingPolicy, 
    TestNVUtil, 
    TestObjectManager, 
    TestOutPort, 
    TestOutPortBase, 
    TestOutPortProvider, 
    TestPeriodicExecutionContext, 
    TestPortAdmin, 
    TestPortBase,
    TestProperties,
    TestPublisherNew, 
    TestPublisherPeriodic, 
    TestRingBuffer,
    TestRTCUtil, 
    TestRTObject_impl, 
    TestConfiguration_impl,
    TestOrganization_impl,
    TestSDOServiceProfile,
    TestStateMachine,
    TestStringUtil,
    TestLogger,
    TestTimer,
    TestTimeValue,
    TestManager,
    )

if __name__ == '__main__':
  test_main()
