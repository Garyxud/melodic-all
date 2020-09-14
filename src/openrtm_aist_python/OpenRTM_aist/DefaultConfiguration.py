#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# \file DefaultConfiguration.py
# \brief RTC manager default configuration
# \date $Date: $
# \author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.



import OpenRTM_aist

##
# @if jp
# @brief Manager 用 デフォルト・コンフィギュレーション
#
# Managerクラス用デフォルトコンフィギュレーション。
#
# @since 0.4.0
#
# @else
# @endif
default_config =["config.version",                   OpenRTM_aist.openrtm_version,
                 "openrtm.name",                     OpenRTM_aist.openrtm_name,
                 "openrtm.version",                  OpenRTM_aist.openrtm_version,
                 "manager.instance_name",            "manager",
                 "manager.name",                     "manager",
                 "manager.naming_formats",           "%h.host_cxt/%n.mgr",
                 "manager.pid",                      "",
                 "manager.refstring_path",           "/var/log/rtcmanager.ref",
                 "os.name",                          "",
                 "os.release",                       "",
                 "os.version",                       "",
                 "os.arch",                          "",
                 "os.hostname",                      "",
                 "logger.enable",                    "YES",
                 "logger.file_name",                 "./rtc%p.log",
                 "logger.date_format",               "%b %d %H:%M:%S",
                 "logger.log_level",                 "INFO",
                 "logger.stream_lock",               "NO",
                 "logger.master_logger",             "",
                 "module.conf_path",                 "",
                 "module.load_path",                 "",
                 "naming.enable",                    "YES",
                 "naming.type",                      "corba",
                 "naming.formats",                   "%h.host_cxt/%n.rtc",
                 "naming.update.enable",             "YES",
                 "naming.update.interval",           "10.0",
                 "timer.enable",                     "YES",
                 "timer.tick",                       "0.1",
                 "corba.args",                       "",
                 "corba.endpoint",                   "",
                 "corba.id",                         OpenRTM_aist.corba_name,
                 "corba.nameservers",               "localhost",
                 "corba.master_manager",             "localhost:2810",
                 "corba.nameservice.replace_endpoint", "NO",
                 "exec_cxt.periodic.type",           "PeriodicExecutionContext",
                 "exec_cxt.periodic.rate",           "1000",
                 "exec_cxt.evdriven.type",           "EventDrivenExecutionContext",
                 "manager.modules.load_path",        "./",
                 "manager.modules.abs_path_allowed", "YES",
                 "manager.is_master",                "NO",
                 "manager.corba_servant",            "YES",
                 "manager.shutdown_on_nortcs",       "YES",
                 "manager.shutdown_auto",            "YES",
                 "manager.auto_shutdown_duration",   "10.0",
                 "manager.name",                     "manager",
                 "manager.command",                  "rtcd",
                 "manager.language",                 "Python",
                 "sdo.service.provider.enabled_service",  "ALL",
                 "sdo.service.consumer.enabled_service",  "ALL",
                 ""]
