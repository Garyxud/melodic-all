// -*- C++ -*-
/*!
 * @file DefaultConfiguration.h
 * @brief RTC manager default configuration
 * @date $Date: 2007-12-31 03:08:03 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006-2010
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#include "rtm/version.h"
#include "rtm/config_rtc.h"
/*!
 * @if jp
 * @brief Manager 用 デフォルト・コンフィギュレーション
 *
 * Managerクラス用デフォルトコンフィギュレーション。
 *
 * @since 0.4.0
 *
 * @else
 * @brief Default configuration for Manager
 *
 * Default configuration for Manager class
 *
 * @since 0.4.0
 *
 * @endif
 */
static const char* default_config[] =
  {
    "config.version",                        openrtm_version,
    "openrtm.name",                          openrtm_name,
    "openrtm.version",                       openrtm_version,
    "manager.instance_name",                 "manager",
    "manager.name",                          "manager",
    "manager.naming_formats",                "%h.host_cxt/%n.mgr",
    "manager.pid",                           "",
    "manager.refstring_path",                "/var/log/rtcmanager.ref",
    "os.name",                               "",
    "os.release",                            "",
    "os.version",                            "",
    "os.arch",                               "",
    "os.hostname",                           "",
    "logger.enable",                         "YES",
    "logger.file_name",                      "./rtc%p.log",
    "logger.date_format",                    "%b %d %H:%M:%S.%Q",
    "logger.log_level",                      "INFO",
    "logger.stream_lock",                    "NO",
    "logger.master_logger",                  "",
    "module.conf_path",                      "",
    "module.load_path",                      "",
    "naming.enable",                         "YES",
    "naming.type",                           "corba",
    "naming.formats",                        "%h.host_cxt/%n.rtc",
    "naming.update.enable",                  "YES",
    "naming.update.interval",                "10.0",
    "timer.enable",                          "YES",
    "timer.tick",                            "0.1",
    "corba.args",                            "",
    "corba.endpoint",                        "",  // hostname:port_number
    "corba.id",                              corba_name,
    "corba.nameservers",                     "localhost",
    "corba.master_manager",                  "localhost:2810",
    "corba.nameservice.replace_endpoint",    "NO",
    "exec_cxt.periodic.type",                "PeriodicExecutionContext",
    "exec_cxt.periodic.rate",                "1000",
    "exec_cxt.evdriven.type",                "EventDrivenExecutionContext",
    "manager.modules.load_path",             "./",
    "manager.modules.abs_path_allowed",      "YES",
    "manager.is_master",                     "NO",
    "manager.corba_servant",                 "YES",
    "manager.shutdown_on_nortcs",            "YES",
    "manager.shutdown_auto",                 "YES",
    "manager.auto_shutdown_duration",        "10.0",
    "manager.name",                          "manager",
    "manager.command",                       "rtcd",
    "manager.supported_languages",           "C++, Python, Java",
    "manager.modules.C++.manager_cmd",       "rtcd",
    "manager.modules.C++.profile_cmd",       "rtcprof",
#ifdef WIN32                                 
    "manager.modules.C++.suffixes",          "dll",
#else                                        
#ifdef RTM_OS_DARWIN                         
    "manager.modules.C++.suffixes",          "dylib",
#else                                        
    "manager.modules.C++.suffixes",          "so",
#endif                                       
#endif                                       
    "manager.modules.C++.load_paths",        "./",
    "manager.modules.Python.manager_cmd",    "rtcd_python",
    "manager.modules.Python.profile_cmd",    "rtcprof_python",
    "manager.modules.Python.suffixes",       "py",
    "manager.modules.Python.load_paths",     "./",
    "manager.modules.Java.manager_cmd",      "rtcd_java",
    "manager.modules.Java.profile_cmd",      "rtcprof_java",
    "manager.modules.Java.suffixes",         "class",
    "manager.modules.Java.load_paths",       "./",
    "sdo.service.provider.enabled_service",  "ALL",
    "sdo.service.consumer.enabled_service",  "ALL",
    ""
  };

