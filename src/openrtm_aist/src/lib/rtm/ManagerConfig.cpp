// -*- C++ -*-
/*!
 * @file ManagerConfig.cpp
 * @brief RTC manager configuration
 * @date $Date: 2007-12-31 03:08:04 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2003-2008
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

#include <rtm/RTC.h>
#include <rtm/ManagerConfig.h>
#include <fstream>
#include <iostream>
#include <coil/OS.h>
#include <coil/stringutil.h>
#include <rtm/DefaultConfiguration.h>

#ifdef __QNX__
using std::sprintf;
#endif

namespace RTC
{
  
  // The list of default configuration file path.
  const char* ManagerConfig::config_file_path[] = 
    {
      "./rtc.conf",
      "/etc/rtc.conf",
      "/etc/rtc/rtc.conf",
      "/usr/local/etc/rtc.conf",
      "/usr/local/etc/rtc/rtc.conf",
      NULL
    };
  
  // Environment value to specify configuration file
  const char* ManagerConfig::config_file_env = "RTC_MANAGER_CONFIG";
  
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  ManagerConfig::ManagerConfig()
    : m_isMaster(false)
  {
  }
  
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  ManagerConfig::ManagerConfig(int argc, char** argv)
    : m_isMaster(false)
  {
    init(argc, argv);
  }
  
  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  ManagerConfig::~ManagerConfig()
  {
  }
  
  /*!
   * @if jp
   * @brief 初期化
   * @else
   * @brief Initialization
   * @endif
   */
  void ManagerConfig::init(int argc, char** argv)
  {
    parseArgs(argc, argv);
  }
  
  /*!
   * @if jp
   * @brief Configuration 情報を Property に設定する
   * @else
   * @brief Specify the configuration information to the Property
   * @endif
   */
  void ManagerConfig::configure(coil::Properties& prop)
  {
    prop.setDefaults(default_config);
    if (findConfigFile())
      {
	std::ifstream f(m_configFile.c_str());
	if (f.is_open())
	  {
	    prop.load(f);
	    f.close();
	  }
      }
    setSystemInformation(prop);
    if (m_isMaster) { prop["manager.is_master"] = "YES"; }

    // Properties from arguments are marged finally
    prop << m_argprop;
    prop["config_file"] = m_configFile;
  }
  
  /*!
   * @if jp
   * @brief コマンド引数をパースする
   * @else
   * @brief Parse the command arguments
   * @endif
   */
  void ManagerConfig::parseArgs(int argc, char** argv)
  {
    coil::GetOpt get_opts(argc, argv, "af:l:o:p:d", 0);
    int opt;
    
    //  if (argc == 0) return true;
    
    while ((opt = get_opts()) > 0)
      {
	switch (opt)
	  {
          case 'a':
            {
              m_argprop["manager.corba_servant"] = "NO";
            }
            break;
	    // Specify configuration file not default
	  case 'f':
	    m_configFile = get_opts.optarg;
	    break;
	  case 'l':
	    //	    m_configFile = get_opts.optarg;
	    break;
	  case 'o':
            {
              std::string optarg(get_opts.optarg);
              std::string::size_type pos(optarg.find(":"));
              if (pos != std::string::npos)
                {
                  m_argprop[optarg.substr(0, pos)] = optarg.substr(pos + 1);
                }
            }
	    break;
          case 'p': // ORB's port number
            {
              int portnum;
              if (coil::stringTo(portnum, get_opts.optarg))
                {
                  std::string arg(":"); arg += get_opts.optarg;
                  m_argprop["corba.endpoints"]  = arg;
                }
            }
            break;
	  case 'd':
            m_isMaster = true;
	    break;
	  default:
	    ;
	  }
      }
    return;
  }
  
  /*!
   * @if jp
   * @brief Configuration file の検索
   * @else
   * @brief Find the configuration file
   * @endif
   */
  bool ManagerConfig::findConfigFile()
  {
    // Check existance of configuration file given command arg
    if (m_configFile != "") 
      {
	if (fileExist(m_configFile))
	  {
	    return true;
	  }
      }
    
    // Search rtc configuration file from environment variable
    char* env = getenv(config_file_env);
    if (env != NULL)
      {
	if (fileExist(env))
	  {
	    m_configFile = env;
	    return true;
	  }
      }
    // Search rtc configuration file from default search path
    int i = 0;
    while (config_file_path[i] != NULL)
      {
	if (fileExist(config_file_path[i]))
	  {
	    m_configFile = config_file_path[i];
	    return true;
	  }
	++i;
      }
    return false;
  }
  
  /*!
   * @if jp
   * @brief システム情報を設定する
   * @else
   * @brief Set system information
   * @endif
   */
  void ManagerConfig::setSystemInformation(coil::Properties& prop)
  { 
    //
    // Get system information by using ACE_OS::uname (UNIX/Windows)
    // 
    coil::utsname  sysinfo;
    if (coil::uname(&sysinfo) != 0)
      {
	return;
      }
    
    //
    // Getting current proccess pid by using ACE_OS::getpid() (UNIX/Windows)
    //
    coil::pid_t pid = coil::getpid();
    char pidc[8];
    sprintf(pidc, "%d", pid);
    
    prop.setProperty("os.name",     sysinfo.sysname);
    prop.setProperty("os.release",  sysinfo.release);
    prop.setProperty("os.version",  sysinfo.version);
    prop.setProperty("os.arch",     sysinfo.machine);
    prop.setProperty("os.hostname", sysinfo.nodename);
    prop.setProperty("manager.pid",         pidc);
    
    return;
  }
  
  /*!
   * @if jp
   * @brief ファイルの存在確認
   * @else
   * @brief Check the file existence
   * @endif
   */
  bool ManagerConfig::fileExist(const std::string& filename)
  {
    std::ifstream infile;
    infile.open(filename.c_str(), std::ios::in);
    // fial() 0: ok, !0: fail
    if (infile.fail() != 0) 
      {
	infile.close();
	return false;
      }
    else
      {
	infile.close();
	return true;
      }
    return false;
  }
}
