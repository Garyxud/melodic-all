// -*- C++ -*-
/*!
 * @file DefaultConfiguration.h
 * @brief doil default configuration
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
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

/*!
 * @if jp
 * @brief ORBManager 用 デフォルト・コンフィギュレーション
 *
 * ORBManagerクラス用デフォルトコンフィギュレーション。
 *
 * @since 1.0.0
 *
 * @else
 * @brief Default configuration for ORBManager
 *
 * Default configuration for ORBManager class
 *
 * @since 1.0.0
 *
 * @endif
 */
static const char* default_config[] =
  {
    "orb.uses",                         "corba, ice",
    "orb.module.conf_path",             "",
    //    "config.version",                   openrtm_version,
    //    "openrtm.version",                  openrtm_name,
    //    "manager.name",                     "manager",
    //    "manager.pid",                      "",
    //    "os.name",                          "",
    //    "os.release",                       "",
    //    "os.version",                       "",
    //    "os.arch",                          "",
    //    "os.hostname",                      "",
    //    "logger.enable",                    "YES",
    //    "logger.file_name",                 "./rtc%p.log",
    //    "logger.date_format",               "%b %d %H:%M:%S",
    //    "logger.log_level",                 "NORMAL",
    //    "logger.stream_lock",               "NO",
    //    "logger.master_logger",             "",
    //    "module.conf_path",                 "",
    //    "module.load_path",                 "",
    //    "naming.enable",                    "YES",
    //    "naming.type",                      "corba",
    //    "naming.formats",                   "%h.host/%n.rtc",
    //    "naming.update.enable",             "YES",
    //    "naming.update.interval",           "10.0",
    //    "timer.enable",                     "YES",
    //    "timer.tick",                       "0.1",
    //    "corba.args",                       "",
    "corba.endpoint",                   "",  // hostname:port_number
    "corba.id",                         "",
    "corba.name_servers",               "",
    //    "exec_cxt.periodic.type",           "PeriodicExecutionContext",
    //    "exec_cxt.periodic.rate",           "1000",
    //    "exec_cxt.evdriven.type",           "EventDrivenExecutionContext",
    //    "manager.modules.load_path",        "./",
    //    "manager.modules.abs_path_allowed", "YES",
    
    ""
  };

