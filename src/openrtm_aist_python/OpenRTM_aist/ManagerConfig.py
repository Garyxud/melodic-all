#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file ManagerConfig.py
# @brief RTC manager configuration
# @date $Date: $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2003-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#    All rights reserved.


import sys
import os
import re
import getopt
import platform

import OpenRTM_aist


##
# @if jp
#
# @class ManagerConfig
# @brief Manager configuration クラス
#
# Manager のコンフィギュレーションを行う、コマンドライン引数を受け取り、
# (あるいは引数なしで)インスタンス化される。
# コマンドライン引数で指定された設定ファイル、環境変数などから Manager の
# プロパティ情報を設定する。
#
# 各設定の優先度は以下のとおりである。
# <OL>
# <LI>コマンドラインオプション "-f"
# <LI>環境変数 "RTC_MANAGER_CONFIG"
# <LI>デフォルト設定ファイル "./rtc.conf"
# <LI>デフォルト設定ファイル "/etc/rtc.conf"
# <LI>デフォルト設定ファイル "/etc/rtc/rtc.conf"
# <LI>デフォルト設定ファイル "/usr/local/etc/rtc.conf"
# <LI>デフォルト設定ファイル "/usr/local/etc/rtc/rtc.conf"
# <LI>埋め込みコンフィギュレーション値
#</OL>
# ただし、コマンドラインオプション "-d" が指定された場合は、
# (たとえ -f で設定ファイルを指定しても)埋め込みコンフィギュレーション値
# が使用される。
#
# @since 0.4.0
#
# @else
#
# @class ManagerConfig
# @brief Manager configuration class
#
# Modify Manager's configuration. 
# This class receives the command line arguments and will be instantiated.
# Set property information of Manager with the configuration file specified
# by the command line argument or the environment variable etc.
#
# The priorities of each configuration are as follows:
# <OL>
# <LI>Command option "-f"
# <LI>Environment variable "RTC_MANAGER_CONFIG"
# <LI>Default configuration file "./rtc.conf"
# <LI>Default configuration file "/etc/rtc.conf"
# <LI>Default configuration file "/etc/rtc/rtc.conf"
# <LI>Default configuration file "/usr/local/etc/rtc.conf"
# <LI>Default configuration file "/usr/local/etc/rtc/rtc.conf"
# <LI>Embedded configuration value
# </OL>
# If the command option "-d" is specified (even if specify configuration file
# by "-f" option), the embedded configuration values will be used.
#
# @since 0.4.0
#
# @endif
class ManagerConfig :
  """
  """

  ##
  # @if jp
  # @brief Manager コンフィギュレーションのデフォルト・ファイル・パス
  # @else
  # @brief The default configuration file path for manager
  # @endif
  config_file_path = ["./rtc.conf",
                      "/etc/rtc.conf",
                      "/etc/rtc/rtc.conf",
                      "/usr/local/etc/rtc.conf",
                      "/usr/local/etc/rtc/rtc.conf",
                      None]


  ##
  # @if jp
  # @brief デフォルト・コンフィギュレーションのファイル・パスを識別する
  #        環境変数
  # @else
  # @brief The environment variable to distinguish the default configuration
  #        file path
  # @endif
  config_file_env = "RTC_MANAGER_CONFIG"


  ##
  # @if jp
  #
  # @brief コンストラクタ
  #
  # 与えられた引数によりコンフィギュレーション情報の初期化を行う。
  #
  # @param self
  # @param argv コマンドライン引数(デフォルト値:None)
  #
  # @else
  #
  # @brief ManagerConfig constructor
  #
  # The constructor that performs initialization at the same time with
  # given arguments.
  #
  # @param argv The command line arguments
  #
  # @endif
  def __init__(self, argv=None):

    self._configFile = ""
    self._argprop = OpenRTM_aist.Properties()
    self._isMaster   = False
    if argv:
      self.init(argv)


  ##
  # @if jp
  #
  # @brief 初期化
  #
  # コマンドライン引数に応じて初期化を実行する。コマンドラインオプションは
  # 以下のものが使用可能である。
  #
  # -f file   : コンフィギュレーションファイルを指定する。<br>
  # -l module : ロードするモジュールを指定する。(未実装)<br>
  # -o options: その他オプションを指定する。<br>
  # -d        : デフォルトのコンフィギュレーションを使う。<br>
  #
  # @param self
  # @param argv コマンドライン引数
  #
  # @else
  #
  # @brief Initialization
  #
  # Initialize with command line options. The following command options
  # are available.
  #
  # -f file   : Specify a configuration file. <br>
  # -l module : Specify modules to be loaded at the beginning. <br>
  # -o options: Other options. <br>
  # -d        : Use default static configuration. <br>
  #
  # @endif
  def init(self, argv):
    self.parseArgs(argv)
    return

  ##
  # @if jp
  # @brief Configuration 情報を Property に設定する
  # 
  # Manager のConfiguration 情報を指定された Property に設定する。
  #
  # @param self
  # @param prop Configuration 設定対象 Property
  # 
  # @else
  # @brief Specify the configuration information to the Property
  #
  # Configure to the properties specified by Manager's configuration
  #
  # @endif
  def configure(self, prop):
    prop.setDefaults(OpenRTM_aist.default_config)
    if self.findConfigFile():
      try:
        fd = file(self._configFile,"r")
        prop.load(fd)
        fd.close()
      except:
        print OpenRTM_aist.Logger.print_exception()

    self.setSystemInformation(prop)
    if self._isMaster:
      prop.setProperty("manager.is_master","YES")

    # Properties from arguments are marged finally
    prop.mergeProperties(self._argprop)
    return prop


  #######
  # @if jp
  #
  # @brief コンフィギュレーションを取得する(未実装)
  #
  # コンフィギュレーションを取得する。init()呼び出し前に呼ぶと、
  # 静的に定義されたデフォルトのコンフィギュレーションを返す。
  # init() 呼び出し後に呼ぶと、コマンドライン引数、環境変数等に
  # 基づいた初期化されたコンフィギュレーションを返す。
  #
  # @else
  #
  # @brief Get configuration value.
  #
  # This operation returns default configuration statically defined,
  # when before calling init() function. When after calling init() function,
  # this operation returns initialized configuration value according to
  # command option, environment value and so on.
  #
  # @endif
  #def getConfig(self):
  #pass


  ##
  # @if jp
  #
  # @brief コマンド引数をパースする
  #
  # -f file   : コンフィギュレーションファイルを指定する。<br>
  # -l module : ロードするモジュールを指定する。(未実装)<br>
  # -o options: その他オプションを指定する。<br>
  # -d        : デフォルトのコンフィギュレーションを使う。<br>
  #
  # @param self
  # @param argv コマンドライン引数
  #
  # @else
  #
  # @brief Parse command arguments
  #
  # -f file   : Specify a configuration file. <br>
  # -l module : Specify modules to be loaded at the beginning. <br>
  # -o options: Other options. <br>
  # -d        : Use default static configuration. <br>
  #
  # @endif
  def parseArgs(self, argv):
    try:
      opts, args = getopt.getopt(argv[1:], "adlf:o:p:")
    except getopt.GetoptError:
      print OpenRTM_aist.Logger.print_exception()
      return

    for opt, arg in opts:
      if opt == "-a":
        self._argprop.setProperty("manager.corba_servant","NO")

      if opt == "-f":
        self._configFile = arg

      if opt == "-l":
        pass

      if opt == "-o":
        pos = arg.find(":")
        if pos > 0:
          self._argprop.setProperty(arg[:pos],arg[pos+1:])

      if opt == "-p":
        num = [-1]
        ret = OpenRTM_aist.stringTo(num, arg)
        if ret:
          arg_ = ":" + arg
          self._argprop.setProperty("corba.endpoints",arg_)

      if opt == "-d":
        self._isMaster = True
        pass

    return


  ##
  # @if jp
  #
  # @brief Configuration file の検索
  #
  # Configuration file を検索し、設定する。
  # 既に Configuration file が設定済みの場合は、ファイルの存在確認を行う。
  #
  # Configuration file の優先順位<br>
  # コマンドオプション指定＞環境変数＞デフォルトファイル＞デフォルト設定
  #
  # デフォルト強制オプション(-d): デフォルトファイルがあっても無視して
  #                               デフォルト設定を使う
  #
  # @param self
  #
  # @return Configuration file 検索結果
  #
  # @else
  #
  # @brief Find the configuration file
  #
  # Find the configuration file and configure it.
  # Confirm the file existence when the configuration file has 
  # already configured.
  #
  # The priority of the configuration file<br>
  # The command option＞the environment variable＞the default file＞
  # the default configuration
  #
  # Default force option(-d): Ignore any default files and use the default 
  # configuration.
  #
  # @return Configuration file search result
  #
  # @endif
  def findConfigFile(self):
    if self._configFile != "":
      if not self.fileExist(self._configFile):
        print OpenRTM_aist.Logger.print_exception()
        return False
      return True

    env = os.getenv(self.config_file_env)
    if env:
      if self.fileExist(env):
        self._configFile = env
        return True

    i = 0
    while (self.config_file_path[i]):
      if self.fileExist(self.config_file_path[i]):
        self._configFile = self.config_file_path[i]
        return True
      i += 1

    return False


  ##
  # @if jp
  #
  # @brief システム情報を設定する
  #
  # システム情報を取得しプロパティにセットする。設定されるキーは以下の通り。
  #  - manager.os.name    : OS名
  #  - manager.os.release : OSリリース名
  #  - manager.os.version : OSバージョン名
  #  - manager.os.arch    : OSアーキテクチャ
  #  - manager.os.hostname: ホスト名
  #  - manager.pid        : プロセスID
  # 
  # @param self
  # @param prop システム情報を設定したプロパティ
  #
  # @else
  # 
  # @brief Set system information
  # 
  # Get the following system info and set them to Manager's properties.
  #  - manager.os.name    : OS name
  #  - manager.os.release : OS release name
  #  - manager.os.version : OS version
  #  - manager.os.arch    : OS architecture
  #  - manager.os.hostname: Hostname
  #  - manager.pid        : process ID
  #
  # @endif
  def setSystemInformation(self, prop):
    if sys.platform == 'win32':
      sysinfo = platform.uname()
    else:
      sysinfo = os.uname()

    prop.setProperty("manager.os.name",     sysinfo[0])
    prop.setProperty("manager.os.hostname", sysinfo[1])
    prop.setProperty("manager.os.release",  sysinfo[2])
    prop.setProperty("manager.os.version",  sysinfo[3])
    prop.setProperty("manager.os.arch",     sysinfo[4])
    prop.setProperty("manager.pid",         os.getpid())

    return prop


  ##
  # @if jp
  # @brief ファイルの存在確認
  #
  # 指定されたファイルが存在するか確認する。
  #
  # @param self
  # @param filename 確認対象ファイル名称
  #
  # @return 対象ファイル確認結果(存在する場合にtrue)
  #
  # @else
  # @brief Check the file existence
  #
  # Confirm whether the specified file exists
  #
  # @param filename The target confirmation file
  #
  # @return file existance confirmation (True if the file exists.)
  #
  # @endif
  def fileExist(self, filename):
    try:
      fp = open(filename)
    except:
      return False
    else:
      fp.close()
      return True

    return False


