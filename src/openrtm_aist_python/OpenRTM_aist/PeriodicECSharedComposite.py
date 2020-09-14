#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file PeriodicECSharedComposite.h
# @brief Periodic Execution Context Shared Composite Component class
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#

import string
import sys
import time

from omniORB import CORBA
import OpenRTM
import RTC
import OpenRTM_aist


periodicecsharedcomposite_spec = ["implementation_id", "PeriodicECSharedComposite",
                                  "type_name",         "PeriodicECSharedComposite",
                                  "description",       "PeriodicECSharedComposite",
                                  "version",           "1.0",
                                  "vendor",            "jp.go.aist",
                                  "category",          "composite.PeriodicECShared",
                                  "activity_type",     "DataFlowComponent",
                                  "max_instance",      "0",
                                  "language",          "Python",
                                  "lang_type",         "script",
                                  "exported_ports",    "",
                                  "conf.default.members", "",
                                  "conf.default.exported_ports", "",
                                  ""]
                                  

def stringToStrVec(v, _is):
  str = [_is]
  OpenRTM_aist.eraseBlank(str)
  v[0] = str[0].split(",")
  return True


class setCallback(OpenRTM_aist.ConfigurationSetListener):
  def __init__(self, org):
    self._org = org
    pass

  def __call__(self, config_set):
    self._org.updateDelegatedPorts()



class addCallback(OpenRTM_aist.ConfigurationSetListener):
  def __init__(self, org):
    self._org = org
    pass

  def __call__(self, config_set):
    self._org.updateDelegatedPorts()
    return


##
# @if jp
# @namespace SDOPacakge
#
# @brief SDO
#
# @else
#
# @namespace SDOPackage
#
# @brief SDO
#
# @endif
#
class PeriodicECOrganization(OpenRTM_aist.Organization_impl):


  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param rtobj オブジェクト
  #
  # @else
  # @brief Constructor
  #
  # Constructor
  #
  # @param rtobj Object
  #
  # @endif
  #
  def __init__(self, rtobj):
    OpenRTM_aist.Organization_impl.__init__(self,rtobj.getObjRef())
    self._rtobj      = rtobj
    self._ec         = None
    self._rtcMembers = []
    self._rtcout = OpenRTM_aist.Manager.instance().getLogbuf("rtobject.PeriodicECOrganization")
    self._expPorts = []
    return


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Organizationメンバーを追加する
  #
  # Organization が保持するメンバーリストに与えられたSDOListを追加する。
  # 
  # @param sdo_list 追加される SDO メンバーのリスト
  # @return 追加が成功したかどうかがboolで返される
  #
  # @else
  # 
  # @brief [CORBA interface] Add Organization member
  #
  # This operation adds the given SDOList to the existing organization's 
  # member list
  # 
  # @param sdo_list SDO member list to be added
  # @return boolean will returned if the operation succeed
  #
  # @endif
  #
  # Boolean add_members(const SDOList& sdo_list)
  def add_members(self, sdo_list):
    self._rtcout.RTC_DEBUG("add_members()")
    self.updateExportedPortsList()
    for sdo in sdo_list:
      dfc = [None]
      if not self.sdoToDFC(sdo, dfc):
        continue
      member = self.Member(dfc[0])
      self.stopOwnedEC(member)
      self.addOrganizationToTarget(member)
      self.addParticipantToEC(member)
      self.addPort(member, self._expPorts)
      self._rtcMembers.append(member)

    result = OpenRTM_aist.Organization_impl.add_members(self,sdo_list)

    return result


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Organizationメンバーをセットする
  #
  # Organization が保持するメンバーリストを削除し、与えられた
  # SDOListを新規にセットする。
  # 
  # @param sdo_list 新規にセットされる SDO メンバーのリスト
  # @return 追加が成功したかどうかがboolで返される
  #
  # @else
  # 
  # @brief [CORBA interface] Set Organization member
  #
  # This operation removes existing member list and sets the given
  # SDOList to the existing organization's member list
  # 
  # @param sdo_list SDO member list to be set
  # @return boolean will returned if the operation succeed
  #
  # @endif
  #
  # Boolean set_members(const SDOList& sdo_list)
  def set_members(self, sdo_list):
    self._rtcout.RTC_DEBUG("set_members()")
    # self._rtcMembers = []
    self.removeAllMembers()
    self.updateExportedPortsList()

    for sdo in sdo_list:
      dfc = [None]
      if not self.sdoToDFC(sdo, dfc):
        continue
      
      member = self.Member(dfc[0])
      self.stopOwnedEC(member)
      self.addOrganizationToTarget(member)
      self.addParticipantToEC(member)
      self.addPort(member, self._expPorts)
      self._rtcMembers.append(member)
      
    result = OpenRTM_aist.Organization_impl.set_members(self, sdo_list)

    return result


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Organizationメンバーを削除する
  #
  # Organization が保持するメンバーリスト内の特定のSDOを削除する。
  # 
  # @param id 削除される SDO の ID
  # @return 追加が成功したかどうかがboolで返される
  #
  # @else
  # 
  # @brief [CORBA interface] Remove a member of Organization
  #
  # This operation removes a SDO from existing member list by specified ID.
  # 
  # @param id The ID of the SDO to be removed
  # @return boolean will returned if the operation succeed
  #
  # @endif
  #
  # Boolean remove_member(const char* id)
  def remove_member(self, id):
    self._rtcout.RTC_DEBUG("remove_member(id = %s)", id)
    rm_rtc = []
    for member in self._rtcMembers:
      if str(id) != str(member._profile.instance_name):
        continue
      self.removePort(member, self._expPorts)
      self._rtobj.getProperties().setProperty("conf.default.exported_ports", OpenRTM_aist.flatten(self._expPorts))
      self.removeParticipantFromEC(member)
      self.removeOrganizationFromTarget(member)
      self.startOwnedEC(member)
      rm_rtc.append(member)

    for m in rm_rtc:
      self._rtcMembers.remove(m)
            
    result = OpenRTM_aist.Organization_impl.remove_member(self, id)
    return result


  ##
  # @if jp
  # @brief Organizationメンバーを削除する
  # @else
  # @brief Remove a member of Organization
  # @endif
  #
  def removeAllMembers(self):
    self._rtcout.RTC_DEBUG("removeAllMembers()")
    self.updateExportedPortsList()
    for member in self._rtcMembers:
      self.removePort(member, self._expPorts)
      self.removeParticipantFromEC(member)
      self.removeOrganizationFromTarget(member)
      self.startOwnedEC(member)
      OpenRTM_aist.Organization_impl.remove_member(self, member._profile.instance_name)

    self._rtcMembers = []
    self._expPorts   = []
    return

        
  ##
  # @if jp
  # @brief SDOからDFCへの変換
  # @else
  # @brief Conversion from SDO to DFC
  # @endif
  #
  # bool sdoToDFC(const SDO_ptr sdo, ::OpenRTM::DataFlowComponent_ptr& dfc);
  def sdoToDFC(self, sdo, dfc):
    if CORBA.is_nil(sdo):
      return False

    dfc[0] = sdo._narrow(OpenRTM.DataFlowComponent)
    if CORBA.is_nil(dfc[0]):
      return False

    return True


  ##
  # @if jp
  # @brief Owned ExecutionContext を停止させる
  # @else
  # @brief Stop Owned ExecutionContexts
  # @endif
  #
  # void stopOwnedEC(Member& member);
  def stopOwnedEC(self, member):
    ecs = member._eclist
    for ec in ecs:
      ret = ec.stop()

    return


  ##
  # @if jp
  # @brief Owned ExecutionContext を起動する
  # @else
  # @brief Start Owned ExecutionContexts
  # @endif
  #
  def startOwnedEC(self, member):
    ecs = member._eclist
    for ec in ecs:
      ret = ec.start()

    return


  ##
  # @if jp
  # @brief DFC に Organization オブジェクトを与える
  # @else
  # @brief Set Organization object to target DFC 
  # @endif
  #
  # void addOrganizationToTarget(Member& member);
  def addOrganizationToTarget(self, member):
    conf = member._config
    if CORBA.is_nil(conf):
      return

    conf.add_organization(self._objref)
    return


  ##
  # @if jp
  # @brief Organization オブジェクトを DFCから削除する
  # @else
  # @brief Remove Organization object from a target DFC 
  # @endif
  #
  # void removeOrganizationFromTarget(Member& member)
  def removeOrganizationFromTarget(self, member):
    # get given RTC's configuration object
    if CORBA.is_nil(member._config):
      return
    
    # set organization to target RTC's conf
    ret = member._config.remove_organization(self._pId)
    return


  ##
  # @if jp
  # @brief Composite の ExecutionContext を DFC にセットする
  # @else
  # @brief Set CompositeRTC's ExecutionContext to the given DFC
  # @endif
  #
  # void addParticipantToEC(Member& member)
  def addParticipantToEC(self, member):
    if CORBA.is_nil(self._ec) or self._ec is None:
      ecs = self._rtobj.get_owned_contexts()
      if len(ecs) > 0:
        self._ec = ecs[0]
      else:
        return
    # set ec to target RTC
    ret = self._ec.add_component(member._rtobj)

    orglist = member._rtobj.get_organizations()
    for org in orglist:
      sdos = org.get_members()
      for sdo in sdos:
        dfc = [None]
        if not self.sdoToDFC(sdo, dfc):
          continue
        self._ec.add_component(dfc[0])
    return


  ##
  # @if jp
  # @brief Composite の ExecutionContext から DFC を削除する
  # @else
  # @brief Remove participant DFC from CompositeRTC's ExecutionContext
  # @endif
  #
  # void PeriodicECOrganization::removeParticipantFromEC(Member& member)
  def removeParticipantFromEC(self, member):
    if CORBA.is_nil(self._ec) or self._ec is None:
      ecs = self._rtobj.get_owned_contexts()
      if len(ecs) > 0:
        self._ec = ecs[0]
      else:
        self._rtcout.RTC_FATAL("no owned EC")
        return
    self._ec.remove_component(member._rtobj)

    orglist = member._rtobj.get_organizations()

    for org in orglist:
      sdos = org.get_members()
      for sdo in sdos:
        dfc = [None]
        if not self.sdoToDFC(sdo, dfc):
          continue
        self._ec.remove_component(dfc[0])
    return


  ##
  # @if jp
  # @brief Composite の ExecutionContext を DFC にセットする
  # @else
  # @brief Set CompositeRTC's ExecutionContext to the given DFC
  # @endif
  #
  # void setCompositeECToTarget(::OpenRTM::DataFlowComponent_ptr dfc);
  # def setCompositeECToTarget(self, dfc):
  #    if CORBA.is_nil(dfc):
  #        return
  #
  #    if CORBA.is_nil(self._ec) or self._ec is None:
  #        ecs = self._rtobj.get_owned_contexts()
  #        if len(ecs) > 0:
  #            self._ec = ecs[0]
  #        else:
  #            return
  #
  #    self._ec.add_component(dfc)

  ##
  # @if jp
  # @brief ポートを委譲する
  # @else
  # @brief Delegate given RTC's ports to the Composite
  # @endif
  #
  # void addPort(Member& member, PortList& portlist);
  def addPort(self, member, portlist):
    self._rtcout.RTC_TRACE("addPort(%s)", OpenRTM_aist.flatten(portlist))
    if len(portlist) == 0:
      return
        
    comp_name = member._profile.instance_name
    plist = member._profile.port_profiles
      
    # port delegation
    for prof in plist:
      # port name -> comp_name.port_name
      port_name = prof.name

      self._rtcout.RTC_DEBUG("port_name: %s is in %s?", (port_name,OpenRTM_aist.flatten(portlist)))
      if not port_name in portlist:
        self._rtcout.RTC_DEBUG("Not found: %s is in %s?", (port_name,OpenRTM_aist.flatten(portlist)))
        continue

      self._rtcout.RTC_DEBUG("Found: %s is in %s", (port_name,OpenRTM_aist.flatten(portlist)))
      self._rtobj.addPort(prof.port_ref)
      self._rtcout.RTC_DEBUG("Port %s was delegated.", port_name)

    return


  ##
  # @if jp
  # @brief 委譲していたポートを削除する
  # @else
  # @brief Remove delegated participatns's ports from the composite
  # @endif
  #
  # void removePort(Member& member, PortList& portlist)
  def removePort(self, member, portlist):
    self._rtcout.RTC_DEBUG("removePort()")
    if len(portlist) == 0:
      return

    comp_name = member._profile.instance_name
    plist = member._profile.port_profiles
    
    # port delegation
    for prof in plist:
      # port name -> comp_name.port_name
      port_name = prof.name
        
      self._rtcout.RTC_DEBUG("port_name: %s is in %s?", (port_name,OpenRTM_aist.flatten(portlist)))
      if not port_name in portlist:
        self._rtcout.RTC_DEBUG("Not found: %s is in %s?", (port_name,OpenRTM_aist.flatten(portlist)))
        continue

      self._rtcout.RTC_DEBUG("Found: %s is in %s", (port_name,OpenRTM_aist.flatten(portlist)))
      self._rtobj.removePort(prof.port_ref)
      portlist.remove(port_name)
      self._rtcout.RTC_DEBUG("Port %s was deleted.", port_name)

    return


  ##
  # @if jp
  # @brief PortsListを更新する
  # @else
  # @brief PortsList is updated. 
  # @endif
  #
  def updateExportedPortsList(self):
    plist = self._rtobj.getProperties().getProperty("conf.default.exported_ports")
    if plist:
      p = [plist]
      OpenRTM_aist.eraseBlank(p)
      self._expPorts = p[0].split(",")

    return

  ##
  # @if jp
  # @brief Organizationメンバーを更新/削除する
  # @else
  # @brief Update/Remove a member of Organization
  # @endif
  #
  def updateDelegatedPorts(self):
    oldPorts = self._expPorts
    ports = self._rtobj.getProperties().getProperty("conf.default.exported_ports")
    newPorts = ports.split(",")

    
    removedPorts = list(set(oldPorts).difference(set(newPorts)))
    createdPorts = list(set(newPorts).difference(set(oldPorts)))
    
    self._rtcout.RTC_VERBOSE("old    ports: %s", OpenRTM_aist.flatten(oldPorts))
    self._rtcout.RTC_VERBOSE("new    ports: %s", OpenRTM_aist.flatten(newPorts))
    self._rtcout.RTC_VERBOSE("remove ports: %s", OpenRTM_aist.flatten(removedPorts))
    self._rtcout.RTC_VERBOSE("add    ports: %s", OpenRTM_aist.flatten(createdPorts))

    for member in self._rtcMembers:
      self.removePort(member, removedPorts)
      self.addPort(member, createdPorts)

    self._expPorts = newPorts
    return


  class Member:
    def __init__(self, rtobj):
      self._rtobj   = rtobj
      self._profile = rtobj.get_component_profile()
      self._eclist  = rtobj.get_owned_contexts()
      self._config  = rtobj.get_configuration()
      return
            
    def __call__(self, x):
      tmp = x
      tmp.swap(self)
      return self

        
    def swap(self, x):
      rtobj   = x._rtobj
      profile = x._profile
      eclist  = x._eclist
      config  = x._config
      
      x._rtobj   = self._rtobj
      x._profile = self._profile
      x._eclist  = self._eclist
      x._config  = self._config

      self._rtobj   = rtobj
      self._profile = profile
      self._eclist  = eclist
      self._config  = config
      return
            

##
# @if jp
# @namespace RTC
#
# @brief RTコンポーネント
#
# @else
#
# @namespace RTC
#
# @brief RT-Component
#
# @endif
#

##
# @if jp
# @class PeriodicECSharedComposite
# @brief PeriodicECSharedComposite クラス
#
# データフロー型RTComponentの基底クラス。
# 各種データフロー型RTComponentを実装する場合は、本クラスを継承する形で実装
# する。
#
# @since 0.4.0
#
# @else
# @class PeriodicECSharedComposite
# @brief PeriodicECSharedComposite class
#
# This is a base class of the data flow type RT-Component.
# Inherit this class when implementing various data flow type RT-Components.
#
# @since 0.4.0
#
# @endif
#
class PeriodicECSharedComposite(OpenRTM_aist.RTObject_impl):


  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param manager マネージャオブジェクト
  #
  # @else
  # @brief Constructor
  #
  # Constructor
  #
  # @param manager Manager object
  #
  # @endif
  #
  def __init__(self, manager):
    OpenRTM_aist.RTObject_impl.__init__(self,manager)
    self._ref = self._this()
    self._objref = self._ref
    self._org = OpenRTM_aist.PeriodicECOrganization(self)
    OpenRTM_aist.CORBA_SeqUtil.push_back(self._sdoOwnedOrganizations,
                                         self._org.getObjRef())

    self._members = [[]]
    self.bindParameter("members", self._members, "", stringToStrVec)
    self._rtcout = OpenRTM_aist.Manager.instance().getLogbuf("rtobject.periodic_ec_shared")
    self._configsets.addConfigurationSetListener(\
      OpenRTM_aist.ConfigurationSetListenerType.ON_SET_CONFIG_SET,
      setCallback(self._org))

    self._configsets.addConfigurationSetListener(\
      OpenRTM_aist.ConfigurationSetListenerType.ON_ADD_CONFIG_SET,
      addCallback(self._org))

    return


  ##
  # @if jp
  # @brief デストラクタ
  #
  # デストラクタ
  #
  # @else
  # @brief Destructor
  #
  # Destructor
  #
  # @endif
  #
  def __del__(self):
    self._rtcout.RTC_TRACE("destructor of PeriodicECSharedComposite")
    pass

    
  ##
  # @if jp
  # @brief 初期化
  #
  # データフロー型 RTComponent の初期化を実行する。
  # 実際の初期化処理は、各具象クラス内に記述する。
  #
  # @else
  # @brief Initialization
  #
  # Initialization the data flow type RT-Component.
  # Write the actual initialization code in each concrete class.
  #
  # @endif
  #
  def onInitialize(self):
    self._rtcout.RTC_TRACE("onInitialize()")

    active_set = self._properties.getProperty("configuration.active_config",
                                              "default")
    if self._configsets.haveConfig(active_set):
      self._configsets.update(active_set)
    else:
      self._configsets.update("default")

    mgr = OpenRTM_aist.Manager.instance()
    sdos = []
    for member in self._members[0]:
      if member == "":
        continue

      rtc = mgr.getComponent(member)

      if rtc is None:
        print "no RTC found: ", member
        continue

      sdo = rtc.getObjRef()
      if CORBA.is_nil(sdo):
        continue

      OpenRTM_aist.CORBA_SeqUtil.push_back(sdos, sdo)
    
    try:
      self._org.set_members(sdos)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())

    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief 活性化処理用コールバック関数
  # 
  # ComponentAction::on_activated が呼ばれた際に実行されるコールバック
  # 関数。<BR>
  # 本関数は無条件に RTC::RTC_OK を返すようにダミー実装されているので、
  # 各コンポーネントの実際の活性化処理は、本関数をオーバーライドして実装する
  # 必要がある。
  # 
  # @param exec_handle 参加している ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  #
  # @brief Callback function to activate
  # 
  # This is a callback function that is executed when
  # ComponentAction::on_activated was invoked.<BR>
  # As for actual activation of each component, since this function is
  # dummy-implemented to return RTC::RTC_OK unconditionally, you need to
  # implement this function by overriding it.
  # 
  # @param exec_handle ID of the participant ExecutionContext
  #
  # @return The return code of ReturnCode_t type
  # 
  # @endif
  #
  def onActivated(self, exec_handle):
    self._rtcout.RTC_TRACE("onActivated(%d)", exec_handle)
    ecs = self.get_owned_contexts()
    sdos = self._org.get_members()

    for sdo in sdos:
      rtc = sdo._narrow(RTC.RTObject)
      ecs[0].activate_component(rtc)

    len_ = len(self._members[0])

    # since Python 2.5
    # self._rtcout.RTC_DEBUG("%d member RTC%s activated.", (len_,(lambda x:  if x > 1 else "was")(len_)))
    if len_ > 1:
      str_ = "s were"
    else:
      str_ = "was"

    self._rtcout.RTC_DEBUG("%d member RTC%s activated.", (len_, str_))

    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief 非活性化処理用コールバック関数
  # 
  # ComponentAction::on_deactivated が呼ばれた際に実行されるコールバック
  # 関数。<BR>
  # 本関数は無条件に RTC::RTC_OK を返すようにダミー実装されているので、
  # 各コンポーネントの実際の非活性化処理は、本関数をオーバーライドして実装する
  # 必要がある。
  # 
  # @param exec_handle 参加している ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  #
  # @brief Callback function to deactivate
  # 
  # This is a callback function that is executed when
  # ComponentAction::on_deactivated was invoked.<BR>
  # As for actual deactivation of each component, since this function is
  # dummy-implemented to return RTC::RTC_OK unconditionally, you need to
  # implement this function by overriding it.
  # 
  # @param exec_handle ID of the participant ExecutionContext
  #
  # @return The return code of ReturnCode_t type
  # 
  # @endif
  #
  def onDeactivated(self, exec_handle):
    self._rtcout.RTC_TRACE("onDeactivated(%d)", exec_handle)
    ecs = self.get_owned_contexts()
    sdos = self._org.get_members()

    for sdo in sdos:
      rtc = sdo._narrow(RTC.RTObject)
      ecs[0].deactivate_component(rtc)

    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief リセット処理用コールバック関数
  # 
  # ComponentAction::on_reset が呼ばれた際に実行されるコールバック関数。<BR>
  # 本関数は無条件に RTC::RTC_OK を返すようにダミー実装されているので、
  # 各コンポーネントの実際のリセット処理は、本関数をオーバーライドして実装する
  # 必要がある。
  # 
  # @param exec_handle 参加している ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  #
  # @brief Callback function to reset
  # 
  # This is a callback function that is executed when
  # ComponentAction::on_reset was invoked.<BR>
  # As for actual reset of each component, since this function is
  # dummy-implemented to return RTC::RTC_OK unconditionally, you need to
  # implement this function by overriding it.
  # 
  # @param exec_handle ID of the participant ExecutionContext
  #
  # @return The return code of ReturnCode_t type
  # 
  # @endif
  #
  def onReset(self, exec_handle):
    self._rtcout.RTC_TRACE("onReset(%d)", exec_handle)
    ecs = self.get_owned_contexts()
    sdos = self._org.get_members()

    for sdo in sdos:
      rtc = sdo._narrow(RTC.RTObject)
      ecs[0].reset_component(rtc)

    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief [ComponentAction CORBA interface] RTC の終了
  #
  # RTC が破棄される。
  # RTC 固有の終了処理はここで実行する。
  # このオペレーション呼び出しの結果として onFinalize() コールバック関数が
  # 呼び出される。
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [ComponentAction CORBA interface] Finalize RTC
  #
  # The RTC is being destroyed.
  # Any final RTC-specific tear-down logic should be performed here.
  # As a result of this operation, onFinalize() callback function is called.
  #
  # @return The return code of ReturnCode_t type
  #
  # @endif
  #
  def onFinalize(self):
    self._rtcout.RTC_TRACE("onFinalize()")
    self._org.removeAllMembers()
    self._rtcout.RTC_PARANOID("onFinalize() done")
    return RTC.RTC_OK


    
def PeriodicECSharedCompositeInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=periodicecsharedcomposite_spec)
  manager.registerFactory(profile,
                          OpenRTM_aist.PeriodicECSharedComposite,
                          OpenRTM_aist.Delete)

