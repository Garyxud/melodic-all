#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file ManagerServant.py
# @brief RTComponent manager servant implementation class
# @date $Date: 2007-12-31 03:08:04 $
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import copy
import sys
import threading
import time
from omniORB import CORBA
import OpenRTM_aist
import RTC,RTM,RTM__POA
import SDOPackage


class ManagerServant(RTM__POA.Manager):
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @else
  # @brief Constructor
  #
  # Constructor
  #
  # @endif
  #
  def __init__(self):
    self._mgr    = OpenRTM_aist.Manager.instance()
    self._owner  = None
    self._rtcout = self._mgr.getLogbuf("ManagerServant")
    self._isMaster = False
    self._masters = []
    self._slaves = []
    self._masterMutex = threading.RLock()
    self._slaveMutex = threading.RLock()
    self._objref = None

    config = copy.deepcopy(self._mgr.getConfig())

    if OpenRTM_aist.toBool(config.getProperty("manager.is_master"), "YES", "NO", True):
      # this is master manager
      self._rtcout.RTC_TRACE("This manager is master.")

      if (not self.createINSManager()):
        self._rtcout.RTC_WARN("Manager CORBA servant creation failed.")
        return
        
      self._isMaster = True
      self._rtcout.RTC_TRACE("Manager CORBA servant was successfully created.")
      return
    else:
      # this is slave manager
      self._rtcout.RTC_TRACE("This manager is slave.")
      try:
        owner = self.findManager(config.getProperty("corba.master_manager"))
        if not owner:
          self._rtcout.RTC_INFO("Master manager not found")
          return

        if not self.createINSManager():
          self._rtcout.RTC_WARN("Manager CORBA servant creation failed.")
          return

        self.add_master_manager(owner)
        owner.add_slave_manager(self._objref)
        return
      except:
        self._rtcout.RTC_ERROR("Unknown exception cought.")
        self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
        
        
    return


  ##
  # @if jp
  #
  # @brief 仮想デストラクタ
  # 
  # @else
  # 
  # @brief Virtual destructor
  # 
  # @endif
  def __del__(self):
    guard_master = OpenRTM_aist.ScopedLock(self._masterMutex)
    for i in range(len(self._masters)):
      try:
        if CORBA.is_nil(self._masters[i]):
          continue
        self._masters[i].remove_slave_manager(self._objref)
      except:
        self._masters[i] = RTM.Manager._nil
    self._masters = []

    guard_slave = OpenRTM_aist.ScopedLock(self._slaveMutex)
    for i in range(len(self._slaves)):
      try:
        if CORBA.is_nil(self._slaves[i]):
          continue
        self._slaves[i].remove_master_manager(self._objref)
      except:
        self._slaves[i] = RTM.Manager._nil
    self._slaves = []

    del guard_slave
    del guard_master
    return


  ##
  # @if jp
  # @brief モジュールをロードする
  #
  # 当該マネージャに指定されたモジュールをロードし、指定された初期化
  # 関数で初期化を行う。
  #
  # @param pathname モジュールへのパス
  # @param initfunc モジュールの初期化関数
  # @return リターンコード
  #
  # @else
  # @brief Loading a module
  #
  # This operation loads a specified loadable module、and perform
  # initialization with the specified function.
  #
  # @param pathname A path to a loading module.
  # @param initfunc Module initialization function.
  # @return The return code.
  #
  # @endif
  #
  # ReturnCode_t load_module(const char* pathname, const char* initfunc)
  def load_module(self, pathname, initfunc):
    self._rtcout.RTC_TRACE("ManagerServant::load_module(%s, %s)", (pathname, initfunc))
    self._mgr.load(pathname, initfunc)
    return RTC.RTC_OK


  ##
  # @if jp
  # @brief モジュールをアンロードする
  #
  # 当該マネージャに指定されたモジュールをアンロードする。
  #
  # @param pathname モジュールへのパス
  # @return リターンコード
  #
  # @else
  # @brief Unloading a module
  #
  # This operation unloads a specified loadable module.
  #
  # @param pathname A path to a loading module.
  # @return The return code.
  #
  # @endif
  #
  # ReturnCode_t unload_module(const char* pathname)
  def unload_module(self, pathname):
    self._rtcout.RTC_TRACE("ManagerServant::unload_module(%s)", pathname)
    self._mgr.unload(pathname)
    return RTC.RTC_OK
  

  ##
  # @if jp
  # @brief ロード可能なモジュールのプロファイルを取得する
  #
  # ロード可能なモジュールのプロファイルを取得する。
  #
  # @return モジュールプロファイル
  #
  # @else
  # @brief Getting loadable module profiles
  #
  # This operation returns loadable module profiles.
  #
  # @return A module profile list.
  #
  # @endif
  #
  # ModuleProfileList* get_loadable_modules()
  def get_loadable_modules(self):
    self._rtcout.RTC_TRACE("get_loadable_modules()")

    # copy local module profiles
    prof = self._mgr.getLoadableModules()
    cprof = [ RTM.ModuleProfile([]) for i in prof ]

    for i in range(len(prof)):
      OpenRTM_aist.NVUtil.copyFromProperties(cprof[i].properties, prof[i])

    return cprof


  ##
  # @if jp
  # @brief ロード済みのモジュールのプロファイルを取得する
  #
  # ロード済みのモジュールのプロファイルを取得する。
  #
  # @return モジュールプロファイル
  #
  # @else
  # @brief Getting loaded module profiles
  #
  # This operation returns loaded module profiles.
  #
  # @return A module profile list.
  #
  # @endif
  #
  # ModuleProfileList* get_loaded_modules()
  def get_loaded_modules(self):
    self._rtcout.RTC_TRACE("get_loaded_modules()")
    prof = self._mgr.getLoadedModules()
    cprof = [RTM.ModuleProfile([]) for i in prof]
    
    for i in range(len(prof)):
      OpenRTM_aist.NVUtil.copyFromProperties(cprof[i].properties, prof[i])

    return cprof


  ##
  # @if jp
  # @brief コンポーネントファクトリのプロファイルを取得する
  #
  # ロード済みのモジュールのうち、RTコンポーネントのモジュールが持つ
  # ファクトリのプロファイルのリストを取得する。
  #
  # @return コンポーネントファクトリのプロファイルリスト
  #
  # @else
  # @brief Getting component factory profiles
  #
  # This operation returns component factory profiles from loaded
  # RT-Component module factory profiles.
  #
  # @return An RT-Component factory profile list.
  #
  # @endif
  #
  # ModuleProfileList* get_factory_profiles()
  def get_factory_profiles(self):
    self._rtcout.RTC_TRACE("get_factory_profiles()")
    prof = self._mgr.getFactoryProfiles()
    cprof = [RTM.ModuleProfile([]) for i in prof]
    
    for i in range(len(prof)):
      OpenRTM_aist.NVUtil.copyFromProperties(cprof[i].properties, prof[i])

    return cprof


  ##
  # @if jp
  # @brief コンポーネントを生成する
  #
  # 引数に指定されたコンポーネントを生成する。
  #
  # @return 生成されたRTコンポーネント
  #
  # @else
  # @brief Creating an RT-Component
  #
  # This operation creates RT-Component according to the string
  # argument.
  #
  # @return A created RT-Component
  #
  # @endif
  #
  # RTObject_ptr create_component(const char* module_name)
  def create_component(self, module_name):
    self._rtcout.RTC_TRACE("create_component(%s)", module_name)

    arg = module_name
    pos0 = arg.find("&manager=")
    pos1 = arg.find("?manager=")

    if pos0 == -1 and pos1 == -1:
      # create on this manager
      rtc = self._mgr.createComponent(module_name)
      if not rtc:
        return RTC.RTObject._nil
      return rtc.getObjRef()

    # create other manager

    # extract manager's location
    # since Python2.5 
    # pos = (lambda x: pos0 if x == -1 else pos1)(pos0)
    if pos0 == -1:
      pos = pos1
    else:
      pos = pos0
    
    endpos = arg.find('&', pos + 1)
    if endpos == -1:
      mgrstr = arg[(pos + 1):]
    else:
      mgrstr = arg[(pos + 1): endpos]
    self._rtcout.RTC_VERBOSE("Manager arg: %s", mgrstr)
    mgrvstr = mgrstr.split(":")
    if len(mgrvstr) != 2:
      self._rtcout.RTC_WARN("Invalid manager name: %s", mgrstr)
      return RTC.RTObject._nil

    eqpos = mgrstr.find("=")
    if eqpos == -1:
      self._rtcout.RTC_WARN("Invalid argument: %s", module_name)
      return RTC.RTObject._nil

    mgrstr = mgrstr[eqpos + 1:]
    self._rtcout.RTC_DEBUG("Manager is %s", mgrstr)

    # find manager
    mgrobj = self.findManager(mgrstr)
    if CORBA.is_nil(mgrobj):
      cmd = "rtcd_python -p "
      cmd += mgrvstr[1] # port number

      self._rtcout.RTC_DEBUG("Invoking command: %s.", cmd)
      ret = OpenRTM_aist.launch_shell(cmd)
      if ret == -1:
        self._rtcout.RTC_DEBUG("%s: failed", cmd)
        return RTC.RTObject._nil

      # find manager
      time.sleep(0.01)
      count = 0
      while CORBA.is_nil(mgrobj):
        mgrobj = self.findManager(mgrstr)
        count += 1
        if count > 1000:
          break
        time.sleep(0.01)

    if CORBA.is_nil(mgrobj):
      self._rtcout.RTC_WARN("Manager cannot be found.")
      return RTC.RTObject._nil
    
    # create component on the manager
    if endpos == -1:
      arg = arg[:pos]
    else:
      arg = arg[:pos] + arg[endpos:]
    self._rtcout.RTC_DEBUG("Creating component on %s",  mgrstr)
    self._rtcout.RTC_DEBUG("arg: %s", arg)
    try:
      rtobj = mgrobj.create_component(arg)
      self._rtcout.RTC_DEBUG("Component created %s",  arg)
      return rtobj
    except CORBA.SystemException:
      self._rtcout.RTC_DEBUG("Exception was caught while creating component.")
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      return RTC.RTObject._nil
    except:
      self._rtcout.RTC_DEBUG(OpenRTM_aist.Logger.print_exception())

    return RTC.RTObject._nil

  
  ##
  # @if jp
  # @brief コンポーネントを削除する
  #
  # 引数に指定されたコンポーネントを削除する。
  #
  # @return リターンコード
  #
  # @else
  # @brief Deleting an RT-Component
  #
  # This operation delete an RT-Component according to the string
  # argument.
  #
  # @return Return code
  #
  # @endif
  #
  # ReturnCode_t delete_component(const char* instance_name)
  def delete_component(self, instance_name):
    self._rtcout.RTC_TRACE("delete_component(%s)", instance_name)
    self._mgr.deleteComponent(instance_name)
    return RTC.RTC_OK
  

  ##
  # @if jp
  # @brief 起動中のコンポーネントのリストを取得する
  #
  # 現在当該マネージャ上で起動中のコンポーネントのリストを返す。
  #
  # @return RTコンポーネントのリスト
  #
  # @else
  # @brief Getting RT-Component list running on this manager
  #
  # This operation returns RT-Component list running on this manager.
  #
  # @return A list of RT-Components
  #
  # @endif
  #
  # RTCList* get_components()
  def get_components(self):
    self._rtcout.RTC_TRACE("get_components()")

    # get local component references
    rtcs = self._mgr.getComponents()
    crtcs = []

    for rtc in rtcs:
      crtcs.append(rtc.getObjRef())

    # get slaves' component references
    self._rtcout.RTC_DEBUG("%d slave managers exists.", len(self._slaves))
    for i in range(len(self._slaves)):
      try:
        if not CORBA.is_nil(self._slaves[i]):
          srtcs = self._slaves[i].get_components()
          OpenRTM_aist.CORBA_SeqUtil.push_back_list(crtcs, srtcs)
          continue
      except:
        self._RTC_INFO("slave (%d) has disappeared.", i)
        self._slaves[i] = RTM.Manager._nil

      OpenRTM_aist.CORBA_SeqUtil.erase(self._slaves, i)
      i -= 1

    return crtcs
  

  ##
  # @if jp
  # @brief 起動中のコンポーネントプロファイルのリストを取得する
  #
  # 現在当該マネージャ上で起動中のコンポーネントのプロファイルのリス
  # トを返す。
  #
  # @return RTコンポーネントプロファイルのリスト
  #
  # @else
  # @brief Getting RT-Component's profile list running on this manager
  #
  # This operation returns RT-Component's profile list running on
  # this manager.
  #
  # @return A list of RT-Components' profiles
  #
  # @endif
  #
  # ComponentProfileList* get_component_profiles()
  def get_component_profiles(self):
    rtcs = self._mgr.getComponents()
    cprofs = [rtc.get_component_profile() for rtc in rtcs]

    # copy slaves' component profiles
    guard = OpenRTM_aist.ScopedLock(self._slaveMutex)
    self._rtcout.RTC_DEBUG("%d slave managers exists.", len(self._slaves))

    for i in range(len(self._slaves)):
      try:
        if not CORBA.is_nil(self._slaves[i]):
          sprofs = self._slaves[i].get_component_profiles()
          OpenRTM_aist.CORBA_SeqUtil.push_back_list(cprofs, sprofs)
          continue
      except:
        self._rtcout.RTC_INFO("slave (%d) has disappeared.", i)
        self._slaves[i] = RTM.Manager._nil

      OpenRTM_aist.CORBA_SeqUtil.erase(self._slaves, i)
      i -= 1

    del guard
    return cprofs


  ##
  # @if jp
  # @brief マネージャのプロファイルを取得する
  #
  # 現在当該マネージャのプロファイルを取得する。
  #
  # @return マネージャプロファイル
  #
  # @else
  # @brief Getting this manager's profile.
  #
  # This operation returns this manager's profile.
  #
  # @return Manager's profile
  #
  # @endif
  #
  # ManagerProfile* get_profile()
  def get_profile(self):
    self._rtcout.RTC_TRACE("get_profile()")
    prof = RTM.ModuleProfile([])
    OpenRTM_aist.NVUtil.copyFromProperties(prof.properties, self._mgr.getConfig().getNode("manager"))

    return prof
  

  ##
  # @if jp
  # @brief マネージャのコンフィギュレーションを取得する
  #
  # 現在当該マネージャのコンフィギュレーションを取得する。
  #
  # @return マネージャコンフィギュレーション
  #
  # @else
  # @brief Getting this manager's configuration.
  #
  # This operation returns this manager's configuration.
  #
  # @return Manager's configuration
  #
  # @endif
  #
  # NVList* get_configuration()
  def get_configuration(self):
    self._rtcout.RTC_TRACE("get_configuration()")
    nvlist = []
    OpenRTM_aist.NVUtil.copyFromProperties(nvlist, self._mgr.getConfig())
    return nvlist
  

  ##
  # @if jp
  # @brief マネージャのコンフィギュレーションを設定する
  #
  # 現在当該マネージャのコンフィギュレーションを設定する。
  #
  # @param name セットするコンフィギュレーションのキー名
  # @param value セットするコンフィギュレーションの値
  # @return リターンコード
  #
  # @else
  # @brief Setting manager's configuration
  #
  # This operation sets managers configuration.
  #  
  # @param name A configuration key name to be set
  # @param value A configuration value to be set
  # @return Return code
  #
  # @endif
  #
  # ReturnCode_t set_configuration(const char* name, const char* value)
  def set_configuration(self, name, value):
    self._rtcout.RTC_TRACE("set_configuration(name = %s, value = %s)", (name, value))
    self._mgr.getConfig().setProperty(name, value)
    return RTC.RTC_OK
  


  ##
  # @if jp
  # @brief マネージャがマスターかどうか
  #
  # この関数はマネージャがマスターかどうかを返す。Trueならば、当該マ
  # ネージャはマスターであり、それ以外は False を返す。
  #
  # @return マスターマネージャかどうかのbool値
  #
  # @else
  # @brief Whether this manager is master or not
  #
  # It returns "True" if this manager is a master, and it returns
  # "False" in other cases.
  #  
  # @return A boolean value that means it is master or not.
  #
  # @endif
  #
  # bool is_master();
  def is_master(self):
    # since Python2.5
    # self._rtcout.RTC_TRACE("is_master(): %s", (lambda x: "YES" if x else "NO")(self._isMaster))
    ret = ""
    if self._isMaster:
      ret = "YES"
    else:
      ret = "NO"
    self._rtcout.RTC_TRACE("is_master(): %s", ret)
    return self._isMaster


  ##
  # @if jp
  # @brief マスターマネージャの取得
  #
  # このマネージャがスレーブマネージャの場合、マスターとなっているマ
  # ネージャのリストを返す。このマネージャがマスターの場合、空のリス
  # トが返る。
  #
  # @return マスターマネージャのリスト
  #
  # @else
  # @brief Getting master managers
  #
  # This operation returns master manager list if this manager is
  # slave. If this manager is master, an empty sequence would be
  # returned.
  #  
  # @return Master manager list
  #
  # @endif
  #
  # RTM::ManagerList* get_master_managers();
  def get_master_managers(self):
    self._rtcout.RTC_TRACE("get_master_managers()")
    guard = OpenRTM_aist.ScopedLock(self._masterMutex)
    
    return self._masters


  ##
  # @if jp
  # @brief マスターマネージャの追加
  #
  # このマネージャのマスタとしてマネージャを一つ追加する。戻り値には、
  # 当該マネージャ上で追加されたマスターマネージャを識別するユニーク
  # なIDが返される。このマネージャがマスタの場合、当該IDで指定された
  # マスターマネージャを返す。IDで指定されたマスターマネージャがない
  # 場合、nilオブジェクトが返る。
  #
  # @return マスターマネージャ
  #
  # @else
  # @brief Getting a master manager
  #
  # This operation returns a master manager with specified id. If
  # the manager with the specified id does not exist, nil object
  # reference would be returned.
  #  
  # @return A master manager
  #
  # @endif
  #
  # RTC::ReturnCode_t add_master_manager(RTM::Manager_ptr mgr);
  def add_master_manager(self, mgr):
    guard = OpenRTM_aist.ScopedLock(self._masterMutex)
    self._rtcout.RTC_TRACE("add_master_manager(), %d masters", len(self._masters))
    index = OpenRTM_aist.CORBA_SeqUtil.find(self._masters, self.is_equiv(mgr))
    
    if not (index < 0): # found in my list
      self._rtcout.RTC_ERROR("Already exists.")
      return RTC.BAD_PARAMETER
    
    OpenRTM_aist.CORBA_SeqUtil.push_back(self._masters, mgr)
    self._rtcout.RTC_TRACE("add_master_manager() done, %d masters", len(self._masters))
    del guard
    return RTC.RTC_OK

  
  ##
  # @if jp
  # @brief マスターマネージャの削除
  #
  # このマネージャが保持するマスタのうち、指定されたものを削除する。
  #
  # @param mgr マスターマネージャ
  # @return ReturnCode_t
  #
  # @else
  # @brief Removing a master manager
  #
  # This operation removes a master manager from this manager.
  # 
  # @param mgr A master manager
  # @return ReturnCode_t 
  #
  # @endif
  #
  # RTC::ReturnCode_t remove_master_manager(RTM::Manager_ptr mgr);
  def remove_master_manager(self, mgr):
    guard = OpenRTM_aist.ScopedLock(self._masterMutex)
    self._rtcout.RTC_TRACE("remove_master_manager(), %d masters", len(self._masters))

    index = OpenRTM_aist.CORBA_SeqUtil.find(self._masters, self.is_equiv(mgr))
    
    if index < 0: # not found in my list
      self._rtcout.RTC_ERROR("Not found.")
      return RTC.BAD_PARAMETER
    
    OpenRTM_aist.CORBA_SeqUtil.erase(self._masters, index)
    self._rtcout.RTC_TRACE("remove_master_manager() done, %d masters", len(self._masters))
    del guard
    return RTC.RTC_OK


  ##
  # @if jp
  # @brief スレーブマネージャの取得
  #
  # このマネージャがスレーブマネージャの場合、スレーブとなっているマ
  # ネージャのリストを返す。このマネージャがスレーブの場合、空のリス
  # トが返る。
  #
  # @return スレーブマネージャのリスト
  #
  # @else
  # @brief Getting slave managers
  #
  # This operation returns slave manager list if this manager is
  # slave. If this manager is slave, an empty sequence would be
  # returned.
  #  
  # @return Slave manager list
  #
  # @endif
  #
  # RTM::ManagerList* get_slave_managers();
  def get_slave_managers(self):
    guard = OpenRTM_aist.ScopedLock(self._slaveMutex)
    self._rtcout.RTC_TRACE("get_slave_managers(), %d slaves", len(self._slaves))
    return self._slaves


  ##
  # @if jp
  # @brief スレーブマネージャの追加
  #
  # このマネージャのマスタとしてマネージャを一つ追加する。
  #
  # @param mgr スレーブマネージャ
  # @return ReturnCode_t
  #
  # @else
  # @brief Getting a slave manager
  #
  # This operation add a slave manager to this manager.
  #  
  # @param mgr A slave manager
  # @return ReturnCode_t
  #
  # @endif
  #
  # RTC::ReturnCode_t add_slave_manager(RTM::Manager_ptr mgr);
  def add_slave_manager(self, mgr):
    guard = OpenRTM_aist.ScopedLock(self._slaveMutex)
    self._rtcout.RTC_TRACE("add_slave_manager(), %d slaves", len(self._slaves))
    
    index = OpenRTM_aist.CORBA_SeqUtil.find(self._slaves, self.is_equiv(mgr))
    
    if not (index < 0): # found in my list
      self._rtcout.RTC_ERROR("Already exists.")
      return RTC.BAD_PARAMETER
    
    OpenRTM_aist.CORBA_SeqUtil.push_back(self._slaves, mgr)
    self._rtcout.RTC_TRACE("add_slave_manager() done, %d slaves", len(self._slaves))
    del guard
    return RTC.RTC_OK


  ##
  # @if jp
  # @brief スレーブマネージャの削除
  #
  # このマネージャが保持するマスタのうち、指定されたものを削除する。
  #
  # @param mgr スレーブマネージャ
  # @return ReturnCode_t
  #
  # @else
  # @brief Removing a slave manager
  #
  # This operation removes a slave manager from this manager.
  # 
  # @param mgr A slave manager
  # @return ReturnCode_t 
  #
  # @endif
  #
  # RTC::ReturnCode_t remove_slave_manager(RTM::Manager_ptr mgr);
  def remove_slave_manager(self, mgr):
    guard = OpenRTM_aist.ScopedLock(self._slaveMutex)
    self._rtcout.RTC_TRACE("remove_slave_manager(), %d slaves", len(self._slaves))
    index = OpenRTM_aist.CORBA_SeqUtil.find(self._slaves, self.is_equiv(mgr))
    
    if index < 0: # not found in my list
      self._rtcout.RTC_ERROR("Not found.")
      return RTC.BAD_PARAMETER
    
    OpenRTM_aist.CORBA_SeqUtil.erase(self._slaves, index)
    self._rtcout.RTC_TRACE("remove_slave_manager() done, %d slaves", len(self._slaves))
    del guard
    return RTC.RTC_OK


  ##
  # @if jp
  # @brief プロセスのコピーを生成する
  # @return ReturnCode_t
  # @else
  # @brief The copy of the process is generated. 
  # @return ReturnCode_t 
  # @endif
  #
  # ReturnCode_t fork()
  def fork(self):
    # self._mgr.fork()
    return RTC.RTC_OK

  
  ##
  # @if jp
  # @brief shutdownする
  # @return ReturnCode_t
  # @else
  # @brief This method shutdowns RTC. 
  # @return ReturnCode_t 
  # @endif
  #
  # ReturnCode_t shutdown()
  def shutdown(self):
    self._mgr.terminate()
    return RTC.RTC_OK

  
  ##
  # @if jp
  # @brief 再起動する。
  # @return ReturnCode_t
  # @else
  # @brief This method restarts RTC.  
  # @return ReturnCode_t 
  # @endif
  #
  # ReturnCode_t restart()
  def restart(self):
    # self._mgr.restart()
    return RTC.RTC_OK
  

  ##
  # @if jp
  # @brief RTCのリファレンスを取得する。
  # @return RTCのリファレンス
  # @else
  # @brief Get the reference of RTC. 
  # @return RTC reference
  # @endif
  #
  # Object_ptr get_service(const char* name)
  def get_service(self, name):
    return CORBA.Object._nil

  
  ##
  # @if jp
  # @brief Managerのリファレンスを取得する。
  # @return Managerのリファレンス
  # @else
  # @brief Get the reference of Manager. 
  # @return Manager reference
  # @endif
  #
  # Manager_ptr getObjRef() const
  def getObjRef(self):
    return self._objref


  ##
  # @if jp
  # @brief INSManagerの生成
  # @return 
  # @else 成功:true, 失敗:false
  # @brief Generate INSManager. 
  # @return Successful:true, Failed:false
  # @endif
  #
  # bool createINSManager();
  def createINSManager(self):
    try:
      poa = self._mgr.getORB().resolve_initial_references("omniINSPOA")
      poa._get_the_POAManager().activate()
      id = self._mgr.getConfig().getProperty("manager.name")
      poa.activate_object_with_id(id, self)
      mgrobj = poa.id_to_reference(id)
      self._objref = mgrobj._narrow(RTM.Manager)
    except:
      self._rtcout.RTC_DEBUG(OpenRTM_aist.Logger.print_exception())
      return False

    return True


  ##
  # @if jp
  # @brief Managerのリファレンスを検索する。
  # @return Managerのリファレンス
  # @else
  # @brief Find the reference of Manager. 
  # @return Manager reference
  # @endif
  #
  # RTM::Manager_ptr findManager(const char* host_port);
  def findManager(self, host_port):
    self._rtcout.RTC_TRACE("findManager(host_port = %s)", host_port)
    try:
      config = copy.deepcopy(self._mgr.getConfig())
      mgrloc = "corbaloc:iiop:"
      mgrloc += host_port
      mgrloc += "/" + config.getProperty("manager.name")
      self._rtcout.RTC_DEBUG("corbaloc: %s", mgrloc)

      mobj = self._mgr.getORB().string_to_object(mgrloc)
      mgr = mobj._narrow(RTM.Manager)
      return mgr

    except CORBA.SystemException:
      self._rtcout.RTC_DEBUG(OpenRTM_aist.Logger.print_exception())
      
    except:
      self._rtcout.RTC_ERROR("Unknown exception cought.")
      self._rtcout.RTC_DEBUG(OpenRTM_aist.Logger.print_exception())

    return RTM.Manager._nil


  class is_equiv:
    def __init__(self, mgr):
      self._mgr = mgr

    def __call__(self, mgr):
      if not self._mgr or not mgr:
        return self._mgr == mgr

      return self._mgr._is_equivalent(mgr)
  
