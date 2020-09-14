#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file SdoOrganization.py
# @brief SDO Organization implementation class
# @date $Date: 2007/09/12 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
# 
# Copyright (C) 2006
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import sys
import omniORB.any
from omniORB import CORBA
import threading

import OpenRTM_aist
import SDOPackage, SDOPackage__POA


##
# @if jp
# 
# @class Organization_impl
# @brief SDO Organization 実装クラス
# 
# Organization interface は Resource Data Model で定義されたデータの
# 追加、削除等の操作を行うためのインターフェースである。
# 
# @since 0.4.0
# 
# @else
# 
# @class Organization_impl
# @brief Organization implementation class
# 
# The Organization interface is used to manage the Organization attribute.
# 
# @since 0.4.0
# 
# @endif
class Organization_impl(SDOPackage__POA.Organization):
  """
  """

  ##
  # @if jp
  # 
  # @brief コンストラクタ
  # 
  # コンストラクタ
  # 
  # @else
  # 
  # @endif
  def __init__(self, sdo):
    self._pId         = str(OpenRTM_aist.uuid1())
    self._org_mutex   = threading.RLock()

    self._orgProperty = SDOPackage.OrganizationProperty([])
    self._varOwner    = sdo
    self._memberList  = []
    self._dependency  = SDOPackage.OWN
    self._objref      = self._this()
    self.__rtcout = OpenRTM_aist.Manager.instance().getLogbuf("rtobject.sdo_organization")


  #============================================================
  #
  # <<< CORBA interfaces >>>
  #
  #============================================================
  ##
  # @if jp
  # 
  # @brief [CORBA interface] Organization ID を取得する
  # 
  # Organization の ID を返すオペレーション。
  #
  # @param self
  # 
  # @return Resource Data Model で定義された Organization ID。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Get Organization Id
  # 
  # This operation returns the 'id' of the Organization.
  #
  # @param self
  # 
  # @return The id of the Organization defined in the resource data model.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def get_organization_id(self):
    self.__rtcout.RTC_TRACE("get_organization_id() = %s", self._pId)
    return self._pId


  ##
  # @if jp
  # 
  # @brief [CORBA interface] OrganizationProperty の取得
  # 
  # Organization が所有する OrganizationProperty を返すオペレーション。
  # Organization がプロパティを持たなければ空のリストを返す。
  # 
  # @param self
  # 
  # @return Organization のプロパティのリスト。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Get OrganizationProperty
  # 
  # This operation returns the OrganizationProperty that an Organization
  # has. An empty OrganizationProperty is returned if the Organization does
  # not have any properties.
  # 
  # @param self
  # 
  # @return The list with properties of the organization.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def get_organization_property(self):
    self.__rtcout.RTC_TRACE("get_organization_property()")
    guard = OpenRTM_aist.ScopedLock(self._org_mutex)
    prop = SDOPackage.OrganizationProperty(self._orgProperty.properties)
    return prop


  ##
  # @if jp
  # 
  # @brief [CORBA interface] OrganizationProperty の特定の値の取得
  # 
  # OrganizationProperty の指定された値を返すオペレーション。
  # 引数 "name" で指定されたプロパティの値を返す。
  # 
  # @param self
  # @param name 値を返すプロパティの名前。
  # 
  # @return 引数 "name" で指定されたプロパティの値。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "namne" で指定されたプロパティが
  #            存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Get specified value of OrganizationProperty
  # 
  # This operation returns a value in the OrganizationProperty.
  # The value to be returned is specified by argument "name."
  # 
  # @param self
  # @param name The name of the value to be returned.
  # 
  # @return The value of property which is specified by argument "name".
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception InvalidParameter If there are no Property stored with argument
  #                             "name".
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def get_organization_property_value(self, name):
    self.__rtcout.RTC_TRACE("get_organization_property_value(%s)", name)
    if not name:
      raise SDOPackage.InvalidParameter("Empty name.")

    index = OpenRTM_aist.CORBA_SeqUtil.find(self._orgProperty.properties, self.nv_name(name))

    if index < 0:
      raise SDOPackage.InvalidParameter("Not found.")

    try:
      value = omniORB.any.to_any(self._orgProperty.properties[index].value)
      return value
    except:
      self.__rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("get_organization_property_value()")

    # never reach here
    return None


  ##
  # @if jp
  # 
  # @brief [CORBA interface] OrganizationProperty のセット
  # 
  # ※ SDO Specification の PIM 記述とオペレーション名が異なる。
  # ※ addOrganizationProperty に対応か？<BR>
  # OrganizationProperty を Organization に追加するオペレーション。
  # OrganizationProperty は Organization のプロパティ記述である。
  # 
  # @param self
  # @param org_property セットする OrganizationProperty
  # 
  # @return オペレーションが成功したかどうかを返す。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter "org_property" が null。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Set OrganizationProperty
  # 
  # This operation adds the OrganizationProperty to an Organization. The
  # OrganizationProperty is the property description of an Organization.
  # 
  # @param self
  # @param org_property The type of organization to be added.
  # 
  # @return If the operation was successfully completed.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception InvalidParameter The argument "organizationProperty" is null.
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def add_organization_property(self, org_property):
    self.__rtcout.RTC_TRACE("add_organization_property()")
    if org_property is None:
      raise SDOPackage.InvalidParameter("org_property is Empty.")

    try:
      guard = OpenRTM_aist.ScopedLock(self._org_mutex)
      self._orgProperty = org_property
      return True
    except:
      self.__rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("add_organization_property()")

    return False


  ##
  # @if jp
  # 
  # @brief [CORBA interface] OrganizationProperty の値のセット
  # 
  # OrganizationProperty の NVList に name と value のセットを追加もしくは
  # 更新するオペレーション。name と value は引数 "name" と "value" により
  # 指定する。
  # 
  # @param self
  # @param name 追加・更新されるプロパティの名前。
  # @param value 追加・更新されるプロパティの値。
  # 
  # @return オペレーションが成功したかどうかを返す。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "name" で指定されたプロパティは
  #            存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Set specified value of OrganizationProperty
  # 
  # This operation adds or updates a pair of name and value as a property
  # of Organization to/in NVList of the OrganizationProperty. The name and
  # the value to be added/updated are specified by argument "name" and
  # "value."
  # 
  # @param self
  # @param name The name of the property to be added/updated.
  # @param value The value of the property to be added/updated.
  # 
  # @return If the operation was successfully completed.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InvalidParameter The property that is specified by argument
  #            "name" does not exist.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def set_organization_property_value(self, name, value):
    self.__rtcout.RTC_TRACE("set_organization_property_value(name=%s)", name)
    if not name:
      raise SDOPackage.InvalidParameter("set_organization_property_value(): Enpty name.")

    index = OpenRTM_aist.CORBA_SeqUtil.find(self._orgProperty.properties, self.nv_name(name))

    if index < 0:
      nv = SDOPackage.NameValue(name, value)
      OpenRTM_aist.CORBA_SeqUtil.push_back(self._orgProperty.properties, nv)
    else:
      self._orgProperty.properties[index].value = value

    return True


  ##
  # @if jp
  # 
  # @brief [CORBA interface] OrganizationProperty の削除
  # 
  # OrganizationProperty の NVList から特定のプロパティを削除する。
  # 削除されるプロパティの名前は引数 "name" により指定される。
  # 
  # @param self
  # @param name 削除するプロパティの名前。
  # 
  # @return オペレーションが成功したかどうかを返す。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "name" で指定されたプロパティは
  #            存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Remove specified OrganizationProperty
  # 
  # This operation removes a property of Organization from NVList of the
  # OrganizationProperty. The property to be removed is specified by
  # argument "name."
  # 
  # @param self
  # @param name The name of the property to be removed.
  # 
  # @return If the operation was successfully completed.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InvalidParameter The property that is specified by argument
  #            "name" does not exist.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def remove_organization_property(self, name):
    self.__rtcout.RTC_TRACE("remove_organization_property(%s)", name)
    if not name:
      raise SDOPackage.InvalidParameter("remove_organization_property_value(): Enpty name.")

    index = OpenRTM_aist.CORBA_SeqUtil.find(self._orgProperty.properties, self.nv_name(name))

    if index < 0:
      raise SDOPackage.InvalidParameter("remove_organization_property_value(): Not found.")

    try:
      OpenRTM_aist.CORBA_SeqUtil.erase(self._orgProperty.properties, index)
      return True
    except:
      self.__rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("remove_organization_property_value()")

    return False


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Organization のオーナーを取得する
  # 
  # この Organization のオーナーへの参照を返す。
  # 
  # @param self
  # 
  # @return オーナーオブジェクトへの参照。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Get the owner of the SDO
  # 
  # This operation returns the SDOSystemElement that is owner of
  # the Organization.
  # 
  # @param self
  # 
  # @return Reference of owner object.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def get_owner(self):
    self.__rtcout.RTC_TRACE("get_owner()")
    return self._varOwner


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Organization にオーナーをセットする
  # 
  # Organization に対して SDOSystemElement をオーナーとしてセットする。
  # 引数 "sdo" にセットする SDOSystemElement を指定する。
  # 
  # @param self
  # @param sdo オーナーオブジェクトの参照。
  # 
  # @return オペレーションが成功したかどうかを返す。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "sdo" が nullである、もしくは、
  #                             "sdo" が存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Set the orner of the Organization
  # 
  # This operation sets an SDOSystemElement to the owner of the
  # Organization. The SDOSystemElement to be set is specified by argument
  # "sdo."
  # 
  # @param self
  # @param sdo Reference of owner object.
  # 
  # @return If the operation was successfully completed.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InvalidParameter The argument "sdo" is null, or the object
  #            that is specified by "sdo" in argument "sdo" does not exist.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def set_owner(self, sdo):
    self.__rtcout.RTC_TRACE("set_owner()")
    if CORBA.is_nil(sdo):
      raise SDOPackage.InvalidParameter("set_owner()")

    try:
      self._varOwner = sdo
      return True
    except:
      self.__rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("set_owner()")

    return True


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Organization のメンバーを取得する
  # 
  # Organization のメンバーの SDO のリストを返す。
  # メンバーが存在しなければ空のリストを返す。
  # 
  # @param self
  # 
  # @return Organization に含まれるメンバー SDO のリスト。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Get a menber list of the Organization
  # 
  # This operation returns a list of SDOs that are members of an
  # Organization. An empty list is returned if the Organization does not
  # have any members.
  # 
  # @param self
  # 
  # @return Member SDOs that are contained in the Organization object.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def get_members(self):
    self.__rtcout.RTC_TRACE("get_members()")
    try:
      return self._memberList
    except:
      self.__rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("get_members()")


  ##
  # @if jp
  # 
  # @brief [CORBA interface] SDO の セット
  # 
  # SDO のリストを Organization のメンバーとしてセットする。
  # Organization がすでにメンバーの SDO を管理している場合は、
  # 与えられた SDO のリストに置き換える。
  # 
  # @param self
  # @param sdos メンバーの SDO。
  # 
  # @return オペレーションが成功したかどうかを返す。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "SDOList" が nullである、もしくは
  #            引数に指定された "SDOList" が存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Set SDO's ServiceProfile
  # 
  # This operation assigns a list of SDOs to an Organization as its members.
  # If the Organization has already maintained a member SDO(s) when it is
  # called, the operation replaces the member(s) with specified list of
  # SDOs.
  # 
  # @param self
  # @param sdos Member SDOs to be assigned.
  # 
  # @return If the operation was successfully completed.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InvalidParameter The argument "SDOList" is null, or if the
  #            object that is specified by the argument "sdos" does not
  #            exist.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def set_members(self, sdos):
    self.__rtcout.RTC_TRACE("set_members()")
    if sdos is None:
      raise SDOPackage.InvalidParameter("set_members(): SDOList is empty.")

    try:
      self._memberList = sdos
      return True
    except:
      self.__rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("set_members()")

    return True


  ##
  # @if jp
  # 
  # @brief [CORBA interface] SDO メンバーの追加
  # 
  # Organization にメンバーとして SDO を追加する。
  # 引数 "sdo" に追加するメンバー SDO を指定する。
  # 
  # @param self
  # @param sdo_list Organization に追加される SDO のリスト。
  # 
  # @return オペレーションが成功したかどうかを返す。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "sdo" が nullである。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Add the menebr SDOs
  # 
  # This operation adds a member that is an SDO to the organization.
  # The member to be added is specified by argument "sdo."
  # 
  # @param self
  # @param sdo The member to be added to the organization.
  # 
  # @return If the operation was successfully completed.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InvalidParameter The argument "sdo" is null.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def add_members(self, sdo_list):
    self.__rtcout.RTC_TRACE("add_members()")
    if not sdo_list:
      raise SDOPackage.InvalidParameter("add_members(): SDOList is empty.")

    try:
      OpenRTM_aist.CORBA_SeqUtil.push_back_list(self._memberList, sdo_list)
      return True
    except:
      self.__rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("add_members()")

    return False


  ##
  # @if jp
  # 
  # @brief [CORBA interface] SDO メンバーの削除
  # 
  # Organization から引数で指定された "id" の SDO を削除する。
  # 
  # @param self
  # @param id 削除する SDO の id。
  # 
  # @return オペレーションが成功したかどうかを返す。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "id" が null もしくは存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Remove menber SDO from Organization
  # 
  # This operation removes a member from the organization. The member to be
  # removed is specified by argument "id."
  # 
  # @param self
  # @param id Id of the SDO to be removed from the organization.
  # 
  # @return If the operation was successfully completed.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InvalidParameter The argument "id" is null or does not exist.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def remove_member(self, id):
    self.__rtcout.RTC_TRACE("remove_member(%s)", id)
    if not id:
      self.__rtcout.RTC_ERROR("remove_member(): Enpty name.")
      raise SDOPackage.InvalidParameter("remove_member(): Empty name.")

    index = OpenRTM_aist.CORBA_SeqUtil.find(self._memberList, self.sdo_id(id))

    if index < 0:
      self.__rtcout.RTC_ERROR("remove_member(): Not found.")
      raise SDOPackage.InvalidParameter("remove_member(): Not found.")

    try:
      OpenRTM_aist.CORBA_SeqUtil.erase(self._memberList, index)
      return True
    except:
      self.__rtcout.RTC_ERROR("unknown exception")
      raise SDOPackage.InternalError("remove_member(): Not found.")

    return False


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Organization の DependencyType を取得
  # 
  # Organization の関係を表す "DependencyType" を返す。
  # 
  # @param self
  # 
  # @return Organizaton の依存関係 DependencyType を返す。
  #         DependencyType は OMG SDO 仕様の Section 2.2.2 2-3 ページの
  #         "Data Structures Used by Resource Data Model" を参照。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Get the DependencyType of the Organization
  # 
  # This operation gets the relationship "DependencyType" of the
  # Organization.
  # 
  # @param self
  # 
  # @return The relationship of the Organization as DependencyType.
  #         DependencyType is defined in Section 2.2.2, "Data Structures
  #         Used by Resource Data Model," on page 2-3
  #         of OMG SDO Specification.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def get_dependency(self):
    self.__rtcout.RTC_TRACE("get_dependency()")
    return self._dependency


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Organization の DependencyType をセットする
  # 
  # Organization の依存関係 "DependencyType" をセットする。
  # 引数 "dependencty" により依存関係を与える。
  # 
  # @param self
  # @param dependency Organization の依存関係を表す DependencyType。
  #        DependencyType は OMG SDO 仕様の Section 2.2.2、2-3 ページの
  #        "Data Structures Used by Resource Data Model" を参照。
  # 
  # @return オペレーションが成功したかどうかを返す。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "sProfile" が nullである。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  # 
  # @brief [CORBA interface] Set the DependencyType of the Organization
  # 
  # This operation sets the relationship "DependencyType" of the
  # Organization. The value to be set is specified by argument "dependency."
  # 
  # @param self
  # @param dependency The relationship of the Organization as
  #                   DependencyType. DependencyType is defined in Section
  #                   2.2.2, "Data Structures Used by Resource Data Model,"
  #                   on page 2-3.
  # 
  # @return If the operation was successfully completed.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InvalidParameter The argument "dependency" is null.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def set_dependency(self, dependency):
    self.__rtcout.RTC_TRACE("set_dependency()")
    if dependency is None:
      raise SDOPackage.InvalidParameter("set_dependency(): Empty dependency.")

    try:
      self._dependency = dependency
      return True
    except:
      self.__rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("set_dependency(): Unknown.")

    return False


  def getObjRef(self):
    return self._objref



  # end of CORBA interface definition
  #============================================================


  ##
  # @if jp
  # @class nv_name
  # @brief NVList検索用functor
  # @else
  #
  # @endif
  class nv_name:
    def __init__(self, name):
      self._name = name

    def __call__(self, nv):
      return str(self._name) == str(nv.name)

  ##
  # @if jp
  # @class sdo_id
  # @brief SDO検索用functor
  # @else
  #
  # @endif
  class sdo_id:
    def __init__(self, id_):
      self._id = id_

    def __call__(self, sdo):
      id_ = sdo.get_sdo_id()
      return str(self._id) == str(id_)
    
