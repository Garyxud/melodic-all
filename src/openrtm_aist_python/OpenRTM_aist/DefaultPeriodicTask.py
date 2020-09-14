#!/usr/bin/env python
# -*- coding: euc-jp -*-


##
# @file DefaultPeriodicTask.py
# @brief PeiodicTaskFactory class
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2009
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import OpenRTM_aist


def DefaultPeriodicTaskInit():
    OpenRTM_aist.PeriodicTaskFactory.instance().addFactory("default",
                                                           OpenRTM_aist.PeriodicTask,
                                                           OpenRTM_aist.Delete)
                                                    
