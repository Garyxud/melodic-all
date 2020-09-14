#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file PortAdmin.py
# @brief RTC's Port administration class
# @date $Date: 2007/09/03 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import traceback
import sys
from omniORB import CORBA

import RTC
import OpenRTM_aist



##
# @if jp
# @class PortAdmin
# @brief PortAdmin クラス
#
# 各種 Port の管理を行うクラス。
# Port の登録/登録解除など各種管理操作を実行するとともに、登録されている
# Port の管理を行うクラス。
#
# @since 0.4.0
#
# @else
# @class PortAdmin
# @brief PortAdmin class
# @endif
class PortAdmin:
  """
  """



  ##
  # @if jp
  # @class comp_op
  # @brief Port 管理用内部クラス
  # @else
  #
  # @endif
  class comp_op:
    def __init__(self, name=None, factory=None):
      if name:
        self._name = name
      if factory:
        self._name = factory.getProfile().name

    def __call__(self, obj):
      name_ = obj.getProfile().name
      return self._name == name_


  ##
  # @if jp
  # @class find_port_name
  # @brief Port 検索用ファンクタ
  # @else
  # @endif
  class find_port_name:
    def __init__(self, name):
      self._name = name

    def __call__(self, p):
      prof = p.get_port_profile()
      name_ = prof.name 
      return self._name == name_


  ##
  # @if jp
  # @brief Port削除用ファンクタ
  # @else
  # @brief Functor to delete the Port
  # @endif
  class del_port:
    def __init__(self, pa):
      self._pa = pa
      return

    def __call__(self, p):
      self._pa.deletePort(p)


  class find_port:
    # find_port(const PortService_ptr& p) : m_port(p) {};
    def __init__(self, p):
      self._port = p

    # bool operator()(const PortService_ptr& p)
    def __call__(self, p):
      return self._port._is_equivalent(p)


  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  # @param orb ORB
  # @param poa POA
  #
  # @else
  # @brief Constructor
  # @endif
  def __init__(self, orb, poa):
    # ORB オブジェクト
    self._orb = orb

    # POA オブジェクト
    self._poa = poa

    # Portのオブジェクトリファレンスのリスト. PortServiceList
    self._portRefs = []

    # サーバントを直接格納するオブジェクトマネージャ
    self._portServants = OpenRTM_aist.ObjectManager(self.comp_op)

    self._rtcout = OpenRTM_aist.Manager.instance().getLogbuf("PortAdmin")

  ##
  # @if jp
  #
  # @brief Port リストの取得
  #
  # registerPort() により登録された Port の リストを取得する。
  #
  # @param self
  #
  # @return Port リスト
  #
  # @else
  #
  # @brief Get PortServiceList
  #
  # This operation returns the pointer to the PortServiceList of Ports regsitered
  # by registerPort().
  #
  # @return PortServiceList+ The pointer points PortServiceList
  #
  # @endif
  def getPortServiceList(self):
    return self._portRefs


  ##
  # @if jp
  #
  # @brief PorProfile リストの取得
  #
  # addPort() により登録された Port の Profile リストを取得する。
  #
  # @return PortProfile リスト
  #
  # @else
  #
  # @brief Get PorProfileList
  #
  # This operation gets the Profile list of Ports registered by 
  # addPort().
  #
  # @return The pointer points PortProfile list
  #
  # @endif
  #
  def getPortProfileList(self):
    ret = []
    for p in self._portRefs:
      ret.append(p.get_port_profile())

    return ret


  ##
  # @if jp
  #
  # @brief Port のオブジェクト参照の取得
  #
  # port_name で指定した Port のオブジェクト参照を返す。
  # port_name で指定する Port はあらかじめ registerPort() で登録されてい
  # なければならない。
  #
  # @param self
  # @param port_name 参照を返すPortの名前
  #
  # @return Port_ptr Portのオブジェクト参照
  #
  # @else
  #
  # @brief Get PortServiceList
  #
  # This operation returns the pointer to the PortServiceList of Ports regsitered
  # by registerPort().
  #
  # @param port_name The name of Port to be returned the reference.
  #
  # @return Port_ptr Port's object reference.
  #
  # @endif
  def getPortRef(self, port_name):
    index = OpenRTM_aist.CORBA_SeqUtil.find(self._portRefs, self.find_port_name(port_name))
    if index >= 0:
      return self._portRefs[index]
    return None


  ##
  # @if jp
  #
  # @brief Port のサーバントのポインタの取得
  #
  # port_name で指定した Port のサーバントのポインタを返す。
  # port_name で指定する Port はあらかじめ registerPort() で登録されてい
  # なければならない。
  #
  # @param self
  # @param port_name 参照を返すPortの名前
  #
  # @return PortBase* Portサーバント基底クラスのポインタ
  #
  # @else
  #
  # @brief Getpointer to the Port's servant
  #
  # This operation returns the pointer to the PortBase servant regsitered
  # by registerPort().
  #
  # @param port_name The name of Port to be returned the servant pointer.
  #
  # @return PortBase* Port's servant's pointer.
  #
  # @endif
  def getPort(self, port_name):
    return self._portServants.find(port_name)


  ##
  # @if jp
  #
  # @brief Port を登録する
  #
  # 引数 port で指定された Port のサーバントを登録する。
  # 登録された Port のサーバントはコンストラクタで与えられたPOA 上で
  # activate され、そのオブジェクト参照はPortのProfileにセットされる。
  #
  # @param self
  # @param port Port サーバント
  #
  # @else
  #
  # @brief Regsiter Port
  #
  # This operation registers the Port's servant given by argument.
  # The given Port's servant will be activated on the POA that is given
  # to the constructor, and the created object reference is set
  # to the Port's profile.
  #
  # @param port The Port's servant.
  #
  # @endif
  # void registerPort(PortBase& port);
  def registerPort(self, port):
    if not self.addPort(port):
      self._rtcout.RTC_ERROR("registerPort() failed.")
    return

  # void registerPort(PortService_ptr port);
  # def registerPortByReference(self, port_ref):
  #   self.addPortByReference(port_ref)
  #   return

  # new interface. since 1.0.0-RELEASE
  # void addPort(PortBase& port);
  def addPort(self, port):
    if isinstance(port, RTC._objref_PortService):
      index = OpenRTM_aist.CORBA_SeqUtil.find(self._portRefs,
                                              self.find_port_name(port.get_port_profile().name))
      if index >= 0:
        return False
      self._portRefs.append(port)
      return True
    else:
      index = OpenRTM_aist.CORBA_SeqUtil.find(self._portRefs,
                                              self.find_port_name(port.getName()))
      if index >= 0:
        return False
      self._portRefs.append(port.getPortRef())
      return self._portServants.registerObject(port)
    return False

  # new interface. since 1.0.0-RELEASE
  # void addPort(PortService_ptr port);
  # def addPortByReference(self, port_ref):
  #   self._portRefs.append(port_ref)
  #   return

  ##
  # @if jp
  #
  # @brief Port の登録を解除する
  #
  # 引数 port で指定された Port の登録を解除する。
  # 削除時に Port は deactivate され、PortのProfileのリファレンスには、
  # nil値が代入される。
  #
  # @param self
  # @param port Port サーバント
  #
  # @else
  #
  # @brief Delete the Port's registration
  #
  # This operation unregisters the Port's registration.
  # When the Port is unregistered, Port is deactivated, and the object
  # reference in the Port's profile is set to nil.
  #
  # @param port The Port's servant.
  #
  # @endif
  def deletePort(self, port):
    if not self.removePort(port):
      self._rtcout.RTC_ERROR("deletePort(PortBase&) failed.")
    return

  # new interface. since 1.0.0-RELEASE
  def removePort(self, port):
    try:
      if isinstance(port,RTC._objref_PortService):
        OpenRTM_aist.CORBA_SeqUtil.erase_if(self._portRefs, self.find_port(port))
        return True

      port.disconnect_all()
      tmp = port.getProfile().name
      OpenRTM_aist.CORBA_SeqUtil.erase_if(self._portRefs, self.find_port_name(tmp))

      self._poa.deactivate_object(self._poa.servant_to_id(port))
      port.setPortRef(RTC.PortService._nil)

      if not self._portServants.unregisterObject(tmp):
        return False
      return True
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      return False


  ##
  # @if jp
  #
  # @brief 名称指定によりPort の登録を解除する
  #
  # 引数で指定された名前を持つ Port の登録を削除する。
  # 削除時に Port は deactivate され、PortのProfileのリファレンスには、
  # nil値が代入される。
  #
  # @param self
  # @param port_name Port の名前
  #
  # @else
  #
  # @brief Delete the Port' registration
  #
  # This operation delete the Port's registration specified by port_ name.
  # When the Port is unregistered, Port is deactivated, and the object
  # reference in the Port's profile is set to nil.
  #
  # @param port_name The Port's name.
  #
  # @endif
  def deletePortByName(self, port_name):
    if not port_name:
      return

    p = self._portServants.find(port_name)
    self.removePort(p)
    return


  ##
  # @if jp
  # @brief 全ての Port のインターフェースを activates する
  # @else
  # @brief Activate all Port interfaces
  # @endif
  # void PortAdmin::activatePorts()
  def activatePorts(self):
    ports = self._portServants.getObjects()
    for port in ports:
      port.activateInterfaces()
      
    return


  ##
  # @if jp
  # @brief 全ての Port のインターフェースを deactivates する
  # @else
  # @brief Deactivate all Port interfaces
  # @endif
  # void PortAdmin::deactivatePorts()
  def deactivatePorts(self):
    ports = self._portServants.getObjects()
    for port in ports:
      port.deactivateInterfaces()

    return


  ##
  # @if jp
  #
  # @brief 全ての Port をdeactivateし登録を削除する
  #
  # 登録されている全てのPortに対して、サーバントのdeactivateを行い、
  # 登録リストから削除する。
  #
  # @param self
  #
  # @else
  #
  # @brief Unregister the Port
  #
  # This operation deactivates the all Port and deletes the all Port's
  # registrations from the list.
  #
  # @endif
  def finalizePorts(self):
    self.deactivatePorts()
    ports = []
    ports = self._portServants.getObjects()
    len_ = len(ports)
    for i in range(len_):
      idx = (len_ - 1) - i
      self.removePort(ports[idx])
