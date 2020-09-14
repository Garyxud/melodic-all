#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file FactoryInit.py
# @brief factory initialization function
# @date $Date: 2008-03-06 06:58:40 $
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2009
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import OpenRTM_aist

def FactoryInit():
    # Buffers
    OpenRTM_aist.CdrRingBufferInit()

    # Threads
    OpenRTM_aist.DefaultPeriodicTaskInit()

    # Publishers
    OpenRTM_aist.PublisherFlushInit()
    OpenRTM_aist.PublisherNewInit()
    OpenRTM_aist.PublisherPeriodicInit()

    # Providers/Consumer
    OpenRTM_aist.InPortCorbaCdrProviderInit()
    OpenRTM_aist.InPortCorbaCdrConsumerInit()
    OpenRTM_aist.OutPortCorbaCdrConsumerInit()
    OpenRTM_aist.OutPortCorbaCdrProviderInit()
    
