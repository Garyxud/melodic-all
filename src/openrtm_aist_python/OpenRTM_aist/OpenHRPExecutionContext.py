#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file OpenHRPExecutionContext.py
# @brief OpenHRPExecutionContext class
# @date $Date: 2007/08/29$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import OpenRTM_aist

class OpenHRPExecutionContext(OpenRTM_aist.PeriodicExecutionContext):

    def __init__(self):
        OpenRTM_aist.PeriodicExecutionContext.__init__(self)
        return


    def tick(self):
        for comp in self._comps:
            comp._sm.worker()

    def svc(self):
        return 0


def OpenHRPExecutionContextInit(manager):
  manager.registerECFactory("OpenHRPExecutionContext",
                            OpenRTM_aist.OpenHRPExecutionContext,
                            OpenRTM_aist.ECDelete)
