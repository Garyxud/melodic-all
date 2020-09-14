#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file SdoConfiguration.py
# @brief RT component base class
# @date $Date: 2007/09/06$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import sys
import copy
import threading

import OpenRTM_aist
##
# @if jp
# @namespace SDOPackage
#
# @brief SDO パッケージ
#
# @else
#
# @namespace SDOPackage
#
# @endif
import SDOPackage, SDOPackage__POA



# SdoConfiguration with SeqEx 159120
# SdoConfiguration with SeqUtil 114504 114224


##
# @if jp
# 
# @brief NVList を Properties へコピーする
# 
# このオペレーションは NVList を Properties へコピーする。
# 
# @param prop NVList の値を格納する Properties
# @param nv コピー元の NVList
# 
# @else
# 
# @brief Copy to Proeprties from NVList
# 
# This operation copies NVList to Properties.
# 
# @param prop Properties to store NVList values
# @param nv NVList that is copies from
# 
# @endif
def toProperties(prop, conf):
  OpenRTM_aist.NVUtil.copyToProperties(prop, conf.configuration_data)


##
# @if jp
# 
# @brief Properties を NVList へコピーする
# 
# このオペレーションは Properties を NVList へコピーする。
# NVList の value は全て CORBA::string 型としてコピーする。
# 
# @param nv Properties の値を格納する NVList
# @param prop コピー元の Properties
# 
# @else
# 
# @brief Copy to NVList from Proeprties
# 
# This operation copies Properties to NVList.
# Created NVList's values are CORBA::string.
# 
# @param nv NVList to store Properties values
# @param prop Properties that is copies from
# 
# @endif
def toConfigurationSet(conf, prop):
  conf.description = prop.getProperty("description")
  conf.id = prop.getName()
  OpenRTM_aist.NVUtil.copyFromProperties(conf.configuration_data, prop)



##
# @if jp
#
# @class Configuration_impl
# @brief SDO Configuration 実装クラス
#
# Configuration interface は Resource Data Model で定義されたデータの
# 追加、削除等の操作を行うためのインターフェースである。
# DeviceProfile, ServiceProfile, ConfigurationProfile および Organization
# の変更を行うためのオペレーションを備えている。SDO の仕様ではアクセス制御
# およびセキュリティに関する詳細については規定していない。
# 
# 複数の設定 (Configuration) を保持することにより、容易かつ素早くある設定
# を反映させることができる。事前に定義された複数の設定を ConfigurationSets
# および configuration profile として保持することができる。ひとつの
# ConfigurationSet は特定の設定に関連付けられた全プロパティ値のリストを、
# ユニークID、詳細とともに持っている。これにより、各設定項目の詳細を記述し
# 区別することができる。Configuration interface のオペレーションはこれら
# ConfiguratioinSets の管理を支援する。
#
#
# - ConfigurationSet: id, description, NVList から構成される1セットの設定
# - ConfigurationSetList: ConfigurationSet のリスト
# - Parameter: name, type, allowed_values から構成されるパラメータ定義。
# - ActiveConfigurationSet: 現在有効なコンフィギュレーションの1セット。
#
# 以下、SDO仕様に明記されていないもしくは解釈がわからないため独自解釈
#
# 以下の関数は ParameterList に対して処理を行う。
# - get_configuration_parameters()
#
# 以下の関数はアクティブなConfigurationSetに対する処理を行う
# - get_configuration_parameter_values()
# - get_configuration_parameter_value()
# - set_configuration_parameter()
#
# 以下の関数はConfigurationSetListに対して処理を行う
# - get_configuration_sets()
# - get_configuration_set()
# - set_configuration_set_values()
# - get_active_configuration_set()
# - add_configuration_set()
# - remove_configuration_set()
# - activate_configuration_set()
#
# @since 0.4.0
#
# @else
#
# @class Configuration_impl
# @brief Configuration implementation class
#
# Configuration interface provides operations to add or remove data
# specified in resource data model. These operations provide functions to
# change DeviceProfile, ServiceProfile, ConfigurationProfile, and
# Organization. This specification does not address access control or
# security aspects. Access to operations that modifies or removes profiles
# should be controlled depending upon the application.
#
# Different configurations can be stored for simple and quick activation.
# Different predefined configurations are stored as different
# ConfigurationSets or configuration profile. A ConfigurationSet stores the
# value of all properties assigned for the particular configuration along
# with its unique id and description to identify and describe the
# configuration respectively. Operations in the configuration interface
# help manage these ConfigurationSets.
#
# @since 0.4.0
#
# @endif
class Configuration_impl(SDOPackage__POA.Configuration):
  """
  """

  ##
  # @if jp
  #
  # @brief コンストラクタ
  # 
  # コンストラクタ
  #
  # @param self
  # @param configAdmin ConfigurationSetList
  # @param sdoServiceAdmin SdoServiceAdmin
  # 
  # @else
  # @brief class constructor
  # @param self
  # @param configAdmin ConfigurationSetList
  # @param sdoServiceAdmin SdoServiceAdmin
  #
  # @endif
  # Configuration_impl(RTC::ConfigAdmin& configAdmin,
  #                    RTC::SdoServiceAdmin& sdoServiceAdmin);
  def __init__(self, configAdmin, sdoServiceAdmin):
    """
     \var self._deviceProfile SDO DeviceProfile with mutex lock
    """
    self._deviceProfile = SDOPackage.DeviceProfile("","","","",[])
    self._dprofile_mutex = threading.RLock()

    """
     \var self._serviceProfiles SDO ServiceProfileList
    """
    self._serviceProfiles = []
    self._sprofile_mutex = threading.RLock()

    self._parameters = []
    self._params_mutex = threading.RLock()

    self._configsets = configAdmin
    self._config_mutex = threading.RLock()

    self._sdoservice = sdoServiceAdmin

    """
     \var self._organizations SDO OrganizationList
    """
    self._organizations = []
    self._org_mutex = threading.RLock()

    self._objref = self._this()
    self._rtcout = OpenRTM_aist.Manager.instance().getLogbuf("rtobject.sdo_config")


  #============================================================
  #
  # <<< CORBA interfaces >>>
  #
  #============================================================

  ##
  # @if jp
  # 
  # @brief [CORBA interface] SDO の DeviceProfile のセット
  #
  # このオペレーションは SDO の DeviceProfile をセットする。SDO が
  # DeviceProfile を保持していない場合は新たな DeviceProfile を生成し、
  # DeviceProfile をすでに保持している場合は既存のものと置き換える。
  #
  # @param self
  # @param dProfile SDO に関連付けられる DeviceProfile。
  #
  # @return オペレーションが成功したかどうかを返す。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InvalidParameter 引数 "dProfile" が null である。
  # @exception InternalError 内部的エラーが発生した。
  # 
  # @else
  #
  # @brief [CORBA interface] Set DeviceProfile of SDO
  #
  # This operation sets the DeviceProfile of an SDO. If the SDO does not
  # have DeviceProfile, the operation will create a new DeviceProfile,
  # otherwise it will replace the existing DeviceProfile.
  #
  # @param self
  # @param dProfile The device profile that is to be assigned to this SDO.
  #
  # @return If the operation was successfully completed.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InvalidParameter The argument "dProfile" is null.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def set_device_profile(self, dProfile):
    self._rtcout.RTC_TRACE("set_device_profile()")
    if dProfile is None:
      raise SDOPackage.InvalidParameter("dProfile is empty.")

    try:
      guard = OpenRTM_aist.ScopedLock(self._dprofile_mutex)
      self._deviceProfile = dProfile
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Unknown Error")

    return True


  ##
  # @if jp
  # 
  # @brief [CORBA interface] SDO の ServiceProfile のセット
  #
  # このオペレーションはこの Configuration interface を所有する対象 SDO の
  # ServiceProfile を追加する。もし引数の ServiceProfile の id が空であれば
  # 新しい ID が生成されその ServiceProfile を格納する。もし id が空で
  # なければ、SDO は同じ id を持つ ServiceProfile を検索する。
  # 同じ id が存在しなければこの ServiceProfile を追加し、id が存在すれば
  # 上書きをする。<br>
  # (注意：最新バージョンではオペレーション名がadd_service_profile変更)
  #
  # @param self
  # @param sProfile 追加する ServiceProfile
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
  # @brief [CORBA interface] Set SDO's ServiceProfile
  #
  # This operation adds ServiceProfile to the target SDO that navigates this
  # Configuration interface. If the id in argument ServiceProfile is null,
  # new id is created and the ServiceProfile is stored. If the id is not
  # null, the target SDO searches for ServiceProfile in it with the same id.
  # It adds the ServiceProfile if not exist, or overwrites if exist.
  #
  # @param self
  # @param sProfile ServiceProfile to be added.
  #
  # @return If the operation was successfully completed.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InvalidParameter The argument "sProfile" is null.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def add_service_profile(self, sProfile):
    self._rtcout.RTC_TRACE("add_service_profile()")
    if sProfile is None:
      raise SDOPackage.InvalidParameter("sProfile is empty.")

    try:
      return self._sdoservice.addSdoServiceConsumer(sProfile)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Configuration.add_service_profile")

    return False


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Organization の追加
  #
  # このオペレーションは Organization object のリファレンスを追加する。
  #
  # @param self
  # @param org 追加する Organization
  #
  # @return オペレーションが成功したかどうかを返す。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InvalidParameter 引数 "organization" が null である。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Add Organization
  #
  # This operation adds reference of an Organization object.
  #
  # @param self
  # @param org Organization to be added.
  #
  # @return If the operation was successfully completed.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InvalidParameter The argument "organization" is null.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def add_organization(self, org):
    self._rtcout.RTC_TRACE("add_organization()")
    if org is None:
      raise SDOPackage.InvalidParameter("org is empty.")

    try:
      OpenRTM_aist.CORBA_SeqUtil.push_back(self._organizations, org)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Configuration.add_organization")

    return True


  ##
  # @if jp
  # 
  # @brief [CORBA interface] ServiceProfile の削除
  #
  # このオペレーションはこの Configuration interface を持つ SDO の
  # Service の ServiceProfile を削除する。削除する ServiceProfile
  # は引数により指定される。
  #
  # @param self
  # @param id_ 削除する ServcieProfile の serviceID。
  #
  # @return オペレーションが成功したかどうかを返す。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "id" が null である。もしくは "id" に
  #            関連付けられた ServiceProfile が存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Remove ServiceProfile
  #
  # This operation removes ServiceProfile object to the SDO that has this
  # Configuration interface. The ServiceProfile object to be removed is
  # specified by argument.
  #
  # @param self
  # @param id_ serviceID of a ServiceProfile to be removed.
  #
  # @return If the operation was successfully completed.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception InvalidParameter The argument "sProfile" is null, or if the
  #          object that is specified by argument "sProfile" does not exist.
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def remove_service_profile(self, id_):
    self._rtcout.RTC_TRACE("remove_service_profile(%s)", id_)
    if id_ is None:
      raise SDOPackage.InvalidParameter("id is empty.")

    try:
      return self._sdoservice.removeSdoServiceConsumer(id_)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Configuration.remove_service_profile")

    return False


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Organization の参照の削除
  #
  # このオペレーションは Organization の参照を削除する。
  #
  # @param self
  # @param organization_id 削除する Organization の一意な id。
  #
  # @return オペレーションが成功したかどうかを返す。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "organization_id" が null である。
  #            もしくは "organization_id" に関連付けられた 
  #            OrganizationProfile が存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Remove the reference of Organization 
  #
  # This operation removes the reference of an Organization object.
  #
  # @param self
  # @param organization_id Unique id of the organization to be removed.
  #
  # @return If the operation was successfully completed.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception InvalidParameter The argument "organizationID" is null,
  #            or the object which is specified by argument "organizationID"
  #            does not exist.
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def remove_organization(self, organization_id):
    self._rtcout.RTC_TRACE("remove_organization(%s)", organization_id)
    if organization_id is None:
      raise SDOPackage.InvalidParameter("organization_id is empty.")

    try:
      guard = OpenRTM_aist.ScopedLock(self._org_mutex)
      OpenRTM_aist.CORBA_SeqUtil.erase_if(self._organizations,
                                          self.org_id(organization_id))
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Configuration.remove_organization")

    return True


  ##
  # @if jp
  # 
  # @brief [CORBA interface] 設定パラメータのリストの取得
  #
  # このオペレーションは configuration parameter のリストを返す。
  # SDO が設定可能なパラメータを持たなければ空のリストを返す。
  #
  # @param self
  #
  # @return 設定を特徴付けるパラメータ定義のリスト。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Getting a list of configuration parameter
  #
  # This operation returns a list of Parameters. An empty list is returned
  # if the SDO does not have any configurable parameter.
  #
  # @param self
  # @return The list with definitions of parameters characterizing the
  #          configuration.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def get_configuration_parameters(self):
    self._rtcout.RTC_TRACE("get_configuration_parameters()")
    try:
      guard = OpenRTM_aist.ScopedLock(self._params_mutex)
      param = copy.copy(self._parameters)
      return param
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Configuration.get_configuration_parameters")

    return []


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Configuration parameter の値のリストの取得
  #
  # このオペレーションは全ての configuration パラメータおよび値を返す。<br>
  # ※本実装では常に空のリストを返す
  #
  # @param self
  #
  # @return 全ての configuration パラメータと値のリスト。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Getting value list of configuration parameter
  #
  # This operation returns all configuration parameters and their values.
  #
  # @param self
  # @return List of all configuration parameters and their values.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def get_configuration_parameter_values(self):
    self._rtcout.RTC_TRACE("get_configuration_parameter_values()")
    guard = OpenRTM_aist.ScopedLock(self._config_mutex)
    nvlist = []
    return nvlist


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Configuration parameter の値の取得
  #
  # このオペレーションは引数 "name" で指定されたパラメータ値を返す。<br>
  # ※本実装では常に None を返す
  #
  # @param self
  # @param name 値を要求するパラメータの名前。
  #
  # @return 指定されたパラメータの値。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "name" が null である。
  #            もしくは "name" に関連付けられたパラメータが存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Getting value of configuration parameter
  #
  # This operation returns a value of parameter that is specified by
  # argument "name."
  #
  # @param self
  # @param Name of the parameter whose value is requested.
  #
  # @return The value of the specified parameter.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception InvalidParameter if the value of the argument "name" is
  #                             empty String, or null, or if the parameter
  #                             that is specified by argument "name"
  #                             does not exist.
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def get_configuration_parameter_value(self, name):
    self._rtcout.RTC_TRACE("get_configuration_parameter_value(%s)", name)
    if not name:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InvalidParameter("Name is empty.")

    return None


  ##
  # @if jp
  # 
  # @brief [CORBA interface] Configuration パラメータの変更
  #
  # このオペレーションは "name" で指定したパラメータの値を "value" に
  # 変更する。<br>
  # ※本実装では常にTrueを返す
  #
  # @param self
  # @param name 変更対象パラメータの名前。
  # @param value 変更対象パラメータの新しい値。
  #
  # @return オペレーションが成功したかどうかを返す。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数( "name"もしくは"value") が null である。
  #            もしくは "name" に関連付けられたパラメータが存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Modify the parameter value
  #
  # This operation sets a parameter to a value that is specified by argument
  # "value." The parameter to be modified is specified by argument " name."
  #
  # @param self
  # @param name The name of parameter to be modified.
  # @param value New value of the specified parameter.
  #
  # @return If the operation was successfully completed.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception InvalidParameter if arguments ("name" and/or "value") is
  #            null, or if the parameter that is specified by the argument
  #            "name" does not exist.
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def set_configuration_parameter(self, name, value):
    self._rtcout.RTC_TRACE("set_configuration_parameter(%s, value)", name)
    if name is None or value is None:
      raise SDOPackage.InvalidParameter("Name/Value is empty.")
    return True


  ##
  # @if jp
  # 
  # @brief [CORBA interface] ConfigurationSet リストの取得 
  #
  # このオペレーションは ConfigurationProfile が持つ ConfigurationSet の
  # リストを返す。 SDO が ConfigurationSet を持たなければ空のリストを返す。
  #
  # @param self
  #
  # @return 保持している ConfigurationSet のリストの現在値。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Getting list of ConfigurationSet
  #
  # This operation returns a list of ConfigurationSets that the
  # ConfigurationProfile has. An empty list is returned if the SDO does not
  # have any ConfigurationSets.
  # This operation returns a list of all ConfigurationSets of the SDO.
  # If no predefined ConfigurationSets exist, then empty list is returned.
  #
  # @param self
  # @return The list of stored configuration with their current values.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def get_configuration_sets(self):
    self._rtcout.RTC_TRACE("get_configuration_sets()")
    try:
      guard = OpenRTM_aist.ScopedLock(self._config_mutex)

      cf = self._configsets.getConfigurationSets()
      len_ = len(cf)

      config_sets = [SDOPackage.ConfigurationSet("","",[]) for i in range(len_)]
      for i in range(len_):
        toConfigurationSet(config_sets[i], cf[i])

      return config_sets

    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Configuration.get_configuration_sets")

    return []

  ##
  # @if jp
  # 
  # @brief [CORBA interface] ConfigurationSet の取得
  #
  # このオペレーションは引数で指定された ConfigurationSet の ID に関連
  # 付けられた ConfigurationSet を返す。
  #
  # @param self
  # @param config_id ConfigurationSet の識別子。
  #
  # @return 引数により指定された ConfigurationSet。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter "config_id" が null か、指定された
  #            ConfigurationSet が存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Getting a ConfigurationSet
  #
  # This operation returns the ConfigurationSet specified by the parameter
  # configurationSetID.
  #
  # @param self
  # @param config_id Identifier of ConfigurationSet requested.
  #
  # @return The configuration set specified by the parameter config_id.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception InvalidParameter If the parameter 'config_id' is null
  #            or if there are no ConfigurationSets stored with such id.
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def get_configuration_set(self, config_id):
    self._rtcout.RTC_TRACE("get_configuration_set(%s)", config_id)
    if not config_id:
      raise SDOPackage.InvalidParameter("ID is empty")

    guard = OpenRTM_aist.ScopedLock(self._config_mutex)

    try:
      if not self._configsets.haveConfig(config_id):
        self._rtcout.RTC_ERROR("No such ConfigurationSet")
        raise SDOPackage.InternalError("No such ConfigurationSet")
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Unknown exception")
      

    configset = self._configsets.getConfigurationSet(config_id)

    try:
      config = SDOPackage.ConfigurationSet("","",[])
      toConfigurationSet(config, configset)
      return config
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Configuration::get_configuration_set()")

    return SDOPackage.ConfigurationSet("","",[])


  ##
  # @if jp
  # 
  # @brief [CORBA interface] ConfigurationSet をセットする
  #
  # このオペレーションは指定された id の ConfigurationSet を更新する。
  #
  # @param self
  # @param configuration_set 変更する ConfigurationSet そのもの。
  #
  # @return ConfigurationSet が正常に更新できた場合は true。
  #         そうでなければ false を返す。
  #
  # @exception InvalidParameter config_id が null か、
  #            指定された id で格納された ConfigurationSetが存在しないか、
  #            指定された configuration_set内の属性の１つが不正。
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Set ConfigurationSet
  #
  # This operation modifies the specified ConfigurationSet of an SDO.
  #
  # @param self
  # @param configuration_set ConfigurationSet to be replaced.
  #
  # @return A flag indicating if the ConfigurationSet was modified 
  #         successfully. "true" - The ConfigurationSet was modified
  #         successfully. "false" - The ConfigurationSet could not be
  #         modified successfully.
  #
  # @exception InvalidParameter if the parameter 'configurationSetID' is
  #            null or if there is no ConfigurationSet stored with such id.
  #            This exception is also raised if one of the attributes
  #            defining ConfigurationSet is not valid.
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def set_configuration_set_values(self, configuration_set):
    self._rtcout.RTC_TRACE("set_configuration_set_values()")
    if not configuration_set or not configuration_set.id:
      raise SDOPackage.InvalidParameter("ID is empty.")

    try:
      conf = OpenRTM_aist.Properties(key=configuration_set.id)
      toProperties(conf, configuration_set)
      # ----------------------------------------------------------------------------
      # Because the format of port-name had been changed from <port_name> 
      # to <instance_name>.<port_name>, the following processing was added. 
      # (since r1648)

      if conf.findNode("exported_ports"):
        exported_ports = conf.getProperty("exported_ports").split(",")
        exported_ports_str = ""
        for i in range(len(exported_ports)):
          keyval = exported_ports[i].split(".")
          if len(keyval) > 2:
            exported_ports_str += keyval[0] + "." + keyval[-1]
          else:
            exported_ports_str += exported_ports[i]

          if i != (len(exported_ports)-1):
            exported_ports_str += ","
            
        conf.setProperty("exported_ports",exported_ports_str)
      #---------------------------------------------------------------------------
      return self._configsets.setConfigurationSetValues(conf)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Configuration::set_configuration_set_values()")

    return True


  ##
  # @if jp
  # 
  # @brief [CORBA interface] アクティブな ConfigurationSet を取得する
  #
  # このオペレーションは当該SDOの現在アクティブな ConfigurationSet を返す。
  # (もしSDOの現在の設定が予め定義された ConfigurationSet により設定されて
  # いるならば。)
  # ConfigurationSet は以下の場合にはアクティブではないものとみなされる。
  #
  # - 現在の設定が予め定義された ConfigurationSet によりセットされていない、
  # - SDO の設定がアクティブになった後に変更された、
  # - SDO を設定する ConfigurationSet が変更された、
  # 
  # これらの場合には、空の ConfigurationSet が返される。
  #
  # @param self
  #
  # @return 現在アクティブな ConfigurationSet。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Get active ConfigurationSet
  #
  # This operation returns the current active ConfigurationSet of an
  # SDO (i.e., if the current configuration of the SDO was set using
  # predefined configuration set).
  # ConfigurationSet cannot be considered active if the:
  #
  # - current configuration of the SDO was not set using any predefined
  #   ConfigurationSet, or
  # - configuration of the SDO was changed after it has been active, or
  # - ConfigurationSet that was used to configure the SDO was modified.
  #
  # Empty ConfigurationSet is returned in these cases.
  #
  # @param self
  # @return The active ConfigurationSet.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def get_active_configuration_set(self):
    self._rtcout.RTC_TRACE("get_active_configuration_set()")
    if not self._configsets.isActive():
      raise SDOPackage.NotAvailable("NotAvailable: Configuration.get_active_configuration_set()")

    try:
      guard = OpenRTM_aist.ScopedLock(self._config_mutex)
      config = SDOPackage.ConfigurationSet("","",[])
      toConfigurationSet(config, self._configsets.getActiveConfigurationSet())
      return config
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Configuration.get_active_configuration_set()")

    return SDOPackage.ConfigurationSet("","",[])


  ##
  # @if jp
  # 
  # @brief [CORBA interface] ConfigurationSet を追加する
  #
  # ConfigurationProfile に ConfigurationSet を追加するオペレーション。
  #
  # @param self
  # @param configuration_set 追加する ConfigurationSet。
  #
  # @return オペレーションが成功したかどうか。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter "configurationSet" が null か、
  #            "configurationSet"で定義された属性の１つが不正か、
  #            指定された configurationSet もIDが既に存在する。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Add ConfigurationSet
  #
  # This operation adds a ConfigurationSet to the ConfigurationProfile.
  #
  # @param self
  # @param configuration_set The ConfigurationSet that is added.
  #
  # @return If the operation was successfully completed.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception InvalidParameter If the argument "configurationSet" is null,
  #            or if one of the attributes defining "configurationSet" is
  #            invalid, or if the specified identifier of the configuration
  #            set already exists.
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def add_configuration_set(self, configuration_set):
    self._rtcout.RTC_TRACE("add_configuration_set()")
    if configuration_set is None:
      raise SDOPackage.InvalidParameter("configuration_set is empty.")

    try:
      guard = OpenRTM_aist.ScopedLock(self._config_mutex)
      config_id = configuration_set.id
      config = OpenRTM_aist.Properties(key=config_id)
      toProperties(config, configuration_set)
      return self._configsets.addConfigurationSet(config)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Configuration::add_configuration_set()")

    return True


  ##
  # @if jp
  # 
  # @brief [CORBA interface] ConfigurationSet を削除する
  #
  # ConfigurationProfile から ConfigurationSet を削除する。
  #
  # @param self
  # @param config_id 削除する ConfigurationSet の id。
  #
  # @return オペレーションが成功したかどうか。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "configurationSetID" が null である、
  #            もしくは、引数で指定された ConfigurationSet が存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Remove ConfigurationSet
  #
  # This operation removes a ConfigurationSet from the ConfigurationProfile.
  #
  # @param self
  # @param config_id The id of ConfigurationSet which is removed.
  #
  # @return If the operation was successfully completed.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception InvalidParameter The arguments "configurationSetID" is null,
  #            or if the object specified by the argument
  #            "configurationSetID" does not exist.
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def remove_configuration_set(self, config_id):
    self._rtcout.RTC_TRACE("remove_configuration_set(%s)", config_id)
    if not config_id:
      raise SDOPackage.InvalidParameter("ID is empty.")
      
    try:
      guard = OpenRTM_aist.ScopedLock(self._config_mutex)
      return self._configsets.removeConfigurationSet(config_id)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("Configuration.remove_configuration_set()")

    return False


  ##
  # @if jp
  # 
  # @brief [CORBA interface] ConfigurationSet のアクティブ化
  #
  # ConfigurationProfile に格納された ConfigurationSet のうち一つを
  # アクティブにする。
  # このオペレーションは特定の ConfigurationSet をアクティブにする。
  # すなわち、SDO のコンフィギュレーション・プロパティがその格納されている
  # ConfigurationSet により設定されるプロパティの値に変更される。
  # 指定された ConfigurationSet の値がアクティブ・コンフィギュレーション
  # にコピーされるということを意味する。
  #
  # @param self
  # @param config_id アクティブ化する ConfigurationSet の id。
  #
  # @return オペレーションが成功したかどうか。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception InvalidParameter 引数 "config_id" が null である、もしくは
  #            引数で指定された ConfigurationSet が存在しない。
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [CORBA interface] Activate ConfigurationSet
  #
  # This operation activates one of the stored ConfigurationSets in the
  # ConfigurationProfile.
  # This operation activates the specified stored ConfigurationSets.
  # This means that the configuration properties of the SDO are changed as
  # the values of these properties specified in the stored ConfigurationSet.
  # In other words, values of the specified ConfigurationSet are now copied
  # to the active configuration.
  #
  # @param self
  # @param Identifier of ConfigurationSet to be activated.
  #
  # @return If the operation was successfully completed.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception InvalidParameter if the argument ("configID") is null or
  #            there is no configuration set with identifier specified by
  #            the argument.
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  def activate_configuration_set(self, config_id):
    self._rtcout.RTC_TRACE("activate_configuration_set(%s)", config_id)
    if not config_id:
      raise SDOPackage.InvalidParameter("ID is empty.")

    if self._configsets.activateConfigurationSet(config_id):
      return True
    else:
      raise SDOPackage.InternalError("Configuration.activate_configuration_set()")

    return False


  #============================================================
  # end of CORBA interface definition
  #============================================================

  ##
  # @if jp
  #
  # @brief オブジェクト　リファレンスを取得する
  # 
  # 対象のオブジェクトリファレンスを取得する
  #
  # @param self
  # 
  # @return オブジェクトリファレンス
  # 
  # @else
  #
  # @endif
  def getObjRef(self):
    return self._objref


  ##
  # @if jp
  #
  # @brief SDO の DeviceProfile を取得する
  # 
  # SDO の DeviceProfile を取得する
  #
  # @param self
  # 
  # @return SDO の DeviceProfile
  # 
  # @else
  #
  # @endif
  def getDeviceProfile(self):
    return self._deviceProfile


  ##
  # @if jp
  #
  # @brief SDO の ServiceProfile のリストを取得する
  # 
  # SDO の ServiceProfile のリストを取得する
  #
  # @param self
  # 
  # @return SDO ServiceProfileリスト
  # 
  # @else
  #
  # @endif
  def getServiceProfiles(self):
    return self._serviceProfiles


  ##
  # @if jp
  #
  # @brief SDO の ServiceProfile を取得する
  # 
  # このオペレーションは引数 "id" で指定されたSDO の ServiceProfileを返す。
  # "id" で指定された ServiceProfileが存在しない場合、
  # ServiceProfileのインスタンスを生成し返す。
  # 
  # @param self
  # @param id ServiceProfile の識別子。
  # 
  # @return 指定された SDO ServiceProfile
  # 
  # @else
  #
  # @endif
  def getServiceProfile(self, id):
    index = OpenRTM_aist.CORBA_SeqUtil.find(self._serviceProfiles,
                                            self.service_id(id))

    if index < 0:
      return SDOPackage.ServiceProfile("","",[],None)

    return self._serviceProfiles[index]


  ##
  # @if jp
  #
  # @brief SDO の Organization リストを取得する
  # 
  # SDO の Organization リストを取得する
  # 
  # @param self
  # 
  # @return SDO の Organization リスト
  # 
  # @else
  #
  # @endif
  def getOrganizations(self):
    return self._organizations


  ##
  # @if jp
  #
  # @brief UUIDを生成する
  # 
  # UUIDを生成する
  # 
  # @param self
  # 
  # @return 生成したUUID
  # 
  # @else
  #
  # @endif
  def getUUID(self):
    return OpenRTM_aist.uuid1()


  # functor for NVList
  ##
  # @if jp
  # @class nv_name
  # @brief  NVList用functor
  # @else
  # @brief  functor for NVList
  # @endif
  class nv_name:
    def __init__(self, name_):
      self._name = str(name_)

    def __call__(self, nv):
      name_ = str(nv.name)
      return self._name == name_


  # functor for ServiceProfile
  ##
  # @if jp
  # @class service_id
  # @brief  ServiceProfile用functor
  # @else
  # @brief  functor for ServiceProfile
  # @endif
  class service_id:
    def __init__(self, id_):
      self._id = str(id_)

    def __call__(self, s):
      id_ = str(s.id)
      return self._id == id_


  # functor for Organization
  ##
  # @if jp
  # @class org_id
  # @brief  Organization用functor
  # @else
  # @brief  functor for Organization
  # @endif
  class org_id:
    def __init__(self, id_):
      self._id = str(id_)

    def __call__(self, o):
      id_ = str(o.get_organization_id())
      return self._id == id_

    
  # functor for ConfigurationSet
  ##
  # @if jp
  # @class config_id
  # @brief  ConfigurationSet用functor
  # @else
  # @brief  functor for ConfigurationSet
  # @endif
  class config_id:
    def __init__(self, id_):
      self._id = str(id_)

    def __call__(self, c):
      id_ = str(c.id)
      return self._id == id_
