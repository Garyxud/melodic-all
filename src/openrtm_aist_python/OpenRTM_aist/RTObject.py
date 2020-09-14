#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file RTObject.py
# @brief RT component base class
# @date $Date: $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.



import string
import sys
import copy

from omniORB import any
from omniORB import CORBA

import OpenRTM__POA
import RTC
import SDOPackage
import OpenRTM_aist

ECOTHER_OFFSET = 1000

default_conf = [
  "implementation_id","",
  "type_name",         "",
  "description",       "",
  "version",           "",
  "vendor",            "",
  "category",          "",
  "activity_type",     "",
  "max_instance",      "",
  "language",          "",
  "lang_type",         "",
  "conf",              "",
  "" ]



##
# @if jp
# @brief RTコンポーネントクラス
#
# 各RTコンポーネントのベースとなるクラス。
# Robotic Technology Component 仕様中の lightweightRTComponentの実装クラス。
# コンポーネントの機能を提供する ComponentAction インターフェースと
# コンポーネントのライフサイクル管理を行うための LightweightRTObject の実装を
# 提供する。
# 実際にユーザがコンポーネントを作成する場合には、Execution Semantics に対応
# した各サブクラスを利用する。<BR>
# (現状の実装では Periodic Sampled Data Processing のみサポートしているため、
#  dataFlowComponent を直接継承している)
#
# @since 0.2.0
#
# @else
#
# @endif
class RTObject_impl(OpenRTM__POA.DataFlowComponent):
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  # @param manager マネージャオブジェクト(デフォルト値:None)
  # @param orb ORB(デフォルト値:None)
  # @param poa POA(デフォルト値:None)
  #
  # @else
  #
  # @brief Consructor
  #
  # @param orb ORB
  # @param poa POA
  #
  # @endif
  def __init__(self, manager=None, orb=None, poa=None):
    if manager:
      self._manager = manager
      self._orb = self._manager.getORB()
      self._poa = self._manager.getPOA()
      self._portAdmin = OpenRTM_aist.PortAdmin(self._manager.getORB(),self._manager.getPOA())
    else:
      self._manager = None
      self._orb = orb
      self._poa = poa
      self._portAdmin = OpenRTM_aist.PortAdmin(self._orb,self._poa)
      
    if self._manager:
      self._rtcout = self._manager.getLogbuf("rtobject")
    else:
      self._rtcout = OpenRTM_aist.Manager.instance().getLogbuf("rtobject")

    self._created = True
    self._properties = OpenRTM_aist.Properties(defaults_str=default_conf)
    self._configsets = OpenRTM_aist.ConfigAdmin(self._properties.getNode("conf"))
    self._profile = RTC.ComponentProfile("","","","","","",[],None,[])

    self._sdoservice = OpenRTM_aist.SdoServiceAdmin(self)
    self._SdoConfigImpl = OpenRTM_aist.Configuration_impl(self._configsets,self._sdoservice)
    self._SdoConfig = self._SdoConfigImpl.getObjRef()
    self._execContexts = []
    self._objref = self._this()
    self._sdoOwnedOrganizations = [] #SDOPackage.OrganizationList()
    self._sdoSvcProfiles        = [] #SDOPackage.ServiceProfileList()
    self._sdoOrganizations      = [] #SDOPackage.OrganizationList()
    self._sdoStatus             = [] #SDOPackage.NVList()
    self._ecMine  = []
    self._ecOther = []
    self._eclist  = []
    self._exiting = False
    self._readAll = False
    self._writeAll = False
    self._readAllCompletion = False
    self._writeAllCompletion = False
    self._inports = []
    self._outports = []
    self._actionListeners = OpenRTM_aist.ComponentActionListeners()
    self._portconnListeners = OpenRTM_aist.PortConnectListeners()
    return


  ##
  # @if jp
  #
  # @brief デストラクタ
  #
  # @param self
  # 
  # @else
  # 
  # @brief destructor
  # 
  # @endif
  def __del__(self):
    return


  #============================================================
  # Overridden functions
  #============================================================

  ##
  # @if jp
  #
  # @brief 初期化処理用コールバック関数
  # 
  # ComponentAction::on_initialize が呼ばれた際に実行されるコールバック
  # 関数。<BR>
  # 本関数は無条件に RTC::RTC_OK を返すようにダミー実装されているので、
  # 各コンポーネントの実際の初期化処理は、本関数をオーバーライドして実装する
  # 必要がある。
  #
  # @param self
  # 
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  # 
  # @endif
  def onInitialize(self):
    self._rtcout.RTC_TRACE("onInitialize()")
    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief 終了処理用コールバック関数
  # 
  # ComponentAction::on_finalize が呼ばれた際に実行されるコールバック
  # 関数。<BR>
  # 本関数は無条件に RTC::RTC_OK を返すようにダミー実装されているので、
  # 各コンポーネントの実際の終了処理は、本関数をオーバーライドして実装する
  # 必要がある。
  #
  # @param self
  # 
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  # 
  # @endif
  def onFinalize(self):
    self._rtcout.RTC_TRACE("onFinalize()")
    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief 開始処理用コールバック関数
  # 
  # ComponentAction::on_startup が呼ばれた際に実行されるコールバック
  # 関数。<BR>
  # 本関数は無条件に RTC::RTC_OK を返すようにダミー実装されているので、
  # 各コンポーネントの実際の開始処理は、本関数をオーバーライドして実装する
  # 必要がある。
  # 
  # @param self
  # @param ec_id 参加している ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  # 
  # @endif
  def onStartup(self, ec_id):
    self._rtcout.RTC_TRACE("onStartup(%d)",ec_id)
    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief 停止処理用コールバック関数
  # 
  # ComponentAction::on_shutdown が呼ばれた際に実行されるコールバック
  # 関数。<BR>
  # 本関数は無条件に RTC::RTC_OK を返すようにダミー実装されているので、
  # 各コンポーネントの実際の停止処理は、本関数をオーバーライドして実装する
  # 必要がある。
  # 
  # @param self
  # @param ec_id 参加している ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  # 
  # @endif
  def onShutdown(self, ec_id):
    self._rtcout.RTC_TRACE("onShutdown(%d)",ec_id)
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
  # @param self
  # @param ec_id 参加している ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  # 
  # @endif
  def onActivated(self, ec_id):
    self._rtcout.RTC_TRACE("onActivated(%d)",ec_id)
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
  # @param self
  # @param ec_id 参加している ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  # 
  # @endif
  def onDeactivated(self, ec_id):
    self._rtcout.RTC_TRACE("onDeactivated(%d)",ec_id)
    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief 周期処理用コールバック関数
  # 
  # DataFlowComponentAction::on_execute が呼ばれた際に実行される
  # コールバック関数。<BR>
  # 本関数は無条件に RTC::RTC_OK を返すようにダミー実装されているので、
  # 各コンポーネントの実際の周期処理は、本関数をオーバーライドして実装する
  # 必要がある。<BR>
  # 本関数は Periodic Sampled Data Processing における Two-Pass Executionの
  # １回目の実行パスとして定期的に呼び出される。
  # 
  # @param self
  # @param ec_id 参加している ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  # 
  # @endif
  def onExecute(self, ec_id):
    self._rtcout.RTC_TRACE("onExecute(%d)",ec_id)
    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief 中断処理用コールバック関数
  # 
  # ComponentAction::on_aborting が呼ばれた際に実行されるコールバック
  # 関数。<BR>
  # 本関数は無条件に RTC::RTC_OK を返すようにダミー実装されているので、
  # 各コンポーネントの実際の中断処理は、本関数をオーバーライドして実装する
  # 必要がある。
  # 
  # @param self
  # @param ec_id 参加している ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  # 
  # @endif
  def onAborting(self, ec_id):
    self._rtcout.RTC_TRACE("onAborting(%d)",ec_id)
    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief エラー処理用コールバック関数
  # 
  # ComponentAction::on_error が呼ばれた際に実行されるコールバック関数。<BR>
  # 本関数は無条件に RTC::RTC_OK を返すようにダミー実装されているので、
  # 各コンポーネントの実際のエラー処理は、本関数をオーバーライドして実装する
  # 必要がある。
  # 
  # @param self
  # @param ec_id 参加している ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  # 
  # @endif
  def onError(self, ec_id):
    self._rtcout.RTC_TRACE("onError(%d)",ec_id)
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
  # @param self
  # @param ec_id 参加している ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  # 
  # @endif
  def onReset(self, ec_id):
    self._rtcout.RTC_TRACE("onReset(%d)",ec_id)
    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief 状態変更処理用コールバック関数
  # 
  # DataFlowComponentAction::on_state_update が呼ばれた際に実行される
  # コールバック関数。<BR>
  # 本関数は無条件に RTC::RTC_OK を返すようにダミー実装されているので、
  # 各コンポーネントの実際の状態変更処理は、本関数をオーバーライドして実装する
  # 必要がある。<BR>
  # 本関数は Periodic Sampled Data Processing における Two-Pass Executionの
  # ２回目の実行パスとして定期的に呼び出される。
  #
  # @param self
  # @param ec_id 参加している ExecutionContext の ID
  # 
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  # 
  # @endif
  def onStateUpdate(self, ec_id):
    self._rtcout.RTC_TRACE("onStateupdate(%d)",ec_id)
    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief 動作周期変更通知用コールバック関数
  # 
  # DataFlowComponentAction::on_rate_changed が呼ばれた際に実行される
  # コールバック関数。<BR>
  # 本関数は無条件に RTC::RTC_OK を返すようにダミー実装されているので、
  # 各コンポーネントの実際の状態変更処理は、本関数をオーバーライドして実装する
  # 必要がある。<BR>
  # 本関数は Periodic Sampled Data Processing において ExecutionContext の
  # 実行が更新された際に呼び出される。
  #
  # @param self
  # @param ec_id 参加している ExecutionContext の ID
  # 
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  # 
  # @endif
  def onRateChanged(self, ec_id):
    self._rtcout.RTC_TRACE("onRatechanged(%d)",ec_id)
    return RTC.RTC_OK 


  #============================================================
  # RTC::LightweightRTObject
  #============================================================

  ##
  # @if jp
  #
  # @brief [CORBA interface] RTCを初期化する
  #
  # このオペレーション呼び出しの結果として、ComponentAction::on_initialize
  # コールバック関数が呼ばれる。
  # 
  # 制約
  # - RTC は Created状態の場合み初期化が行われる。他の状態にいる場合には
  #   ReturnCode_t::PRECONDITION_NOT_MET が返され呼び出しは失敗する。
  # - このオペレーションは RTC のミドルウエアから呼ばれることを想定しており、
  #   アプリケーション開発者は直接このオペレーションを呼ぶことは想定
  #   されていない。
  #
  # @param self
  # 
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  #
  # @brief Initialize the RTC that realizes this interface.
  #
  # The invocation of this operation shall result in the invocation of the
  # callback ComponentAction::on_initialize.
  #
  # Constraints
  # - An RTC may be initialized only while it is in the Created state. Any
  #   attempt to invoke this operation while in another state shall fail
  #   with ReturnCode_t::PRECONDITION_NOT_MET.
  # - Application developers are not expected to call this operation
  #   directly; it exists for use by the RTC infrastructure.
  #
  # @return
  # 
  # @endif
  def initialize(self):
    self._rtcout.RTC_TRACE("initialize()")

    ec_args = self._properties.getProperty("exec_cxt.periodic.type")
    ec_args += "?"
    ec_args += "rate="
    ec_args += self._properties.getProperty("exec_cxt.periodic.rate")

    ec = OpenRTM_aist.Manager.instance().createContext(ec_args)
    if ec is None:
      return RTC.RTC_ERROR

    ec.set_rate(float(self._properties.getProperty("exec_cxt.periodic.rate")))
    self._eclist.append(ec)
    ecv = ec.getObjRef()
    if CORBA.is_nil(ecv):
      return RTC.RTC_ERROR

    ec.bindComponent(self)

    # at least one EC must be attached
    if len(self._ecMine) == 0:
      return RTC.PRECONDITION_NOT_MET

    ret = self.on_initialize()
    if ret is not RTC.RTC_OK:
      return ret
    self._created = False

    # -- entering alive state --
    for i in range(len(self._ecMine)):
      self._rtcout.RTC_DEBUG("EC[%d] starting.", i)
      self._ecMine[i].start()

    # ret must be RTC_OK
    return ret


  ##
  # @if jp
  #
  # @brief [CORBA interface] RTC を終了する
  #
  # このオペレーション呼び出しの結果として ComponentAction::on_finalize()
  # を呼び出す。
  #
  # 制約
  # - RTC が ExecutionContext に所属している間は終了されない。この場合は、
  #   まず最初に ExecutionContextOperations::remove_component によって参加を
  #   解除しなければならない。これ以外の場合は、このオペレーション呼び出しは
  #   いかなる場合も ReturnCode_t::PRECONDITION_NOT_ME で失敗する。
  # - RTC が Created 状態である場合、終了処理は行われない。
  #   この場合、このオペレーション呼び出しはいかなる場合も
  #   ReturnCode_t::PRECONDITION_NOT_MET で失敗する。
  # - このオペレーションはRTCのミドルウエアから呼ばれることを想定しており、
  #   アプリケーション開発者は直接このオペレーションを呼ぶことは想定
  #   されていない。
  #
  # @param self
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  #
  # @brief Finalize the RTC for preparing it for destruction
  # 
  # This invocation of this operation shall result in the invocation of the
  # callback ComponentAction::on_finalize.
  #
  # Constraints
  # - An RTC may not be finalized while it is participating in any execution
  #   context. It must first be removed with 
  #   ExecutionContextOperations::remove_component. Otherwise, this operation
  #   shall fail with ReturnCode_t::PRECONDITION_NOT_MET. 
  # - An RTC may not be finalized while it is in the Created state. Any 
  #   attempt to invoke this operation while in that state shall fail with 
  #   ReturnCode_t::PRECONDITION_NOT_MET.
  # - Application developers are not expected to call this operation directly;
  #  it exists for use by the RTC infrastructure.
  #
  # @return
  # 
  # @endif
  def finalize(self):
    self._rtcout.RTC_TRACE("finalize()")
    if self._created or not self._exiting:
      return RTC.PRECONDITION_NOT_MET

    # Return RTC::PRECONDITION_NOT_MET,
    # When the component is registered in ExecutionContext.
    if len(self._ecOther) != 0:
      for ec in self._ecOther:
        if not CORBA.is_nil(ec):
          return RTC.PRECONDITION_NOT_MET
      
      self._ecOther = []

    ret = self.on_finalize()
    self.shutdown()
    return ret


  ##
  # @if jp
  #
  # @brief [CORBA interface] RTC がオーナーである ExecutionContext を
  #        停止させ、そのコンテンツと共に終了させる
  #
  # この RTC がオーナーであるすべての実行コンテキストを停止する。
  # この RTC が他の実行コンテキストを所有する RTC に属する実行コンテキスト
  # (i.e. 実行コンテキストを所有する RTC はすなわちその実行コンテキストの
  # オーナーである。)に参加している場合、当該 RTC はそれらのコンテキスト上
  # で非活性化されなければならない。
  # RTC が実行中のどの ExecutionContext でも Active 状態ではなくなった後、
  # この RTC とこれに含まれる RTC が終了する。
  # 
  # 制約
  # - RTC が初期化されていなければ、終了させることはできない。
  #   Created 状態にある RTC に exit() を呼び出した場合、
  #   ReturnCode_t::PRECONDITION_NOT_MET で失敗する。
  #
  # @param self
  #
  # @return ReturnCode_t 型のリターンコード
  # 
  # @else
  #
  # @brief Stop the RTC's execution context(s) and finalize it along with its
  #        contents.
  # 
  # Any execution contexts for which the RTC is the owner shall be stopped. 
  # If the RTC participates in any execution contexts belonging to another
  # RTC that contains it, directly or indirectly (i.e. the containing RTC
  # is the owner of the ExecutionContext), it shall be deactivated in those
  # contexts.
  # After the RTC is no longer Active in any Running execution context, it
  # and any RTCs contained transitively within it shall be finalized.
  #
  # Constraints
  # - An RTC cannot be exited if it has not yet been initialized. Any
  #   attempt to exit an RTC that is in the Created state shall fail with
  #   ReturnCode_t::PRECONDITION_NOT_MET.
  #
  # @return
  # 
  # @endif
  def exit(self):
    self._rtcout.RTC_TRACE("exit()")
    if self._created:
      return RTC.PRECONDITION_NOT_MET

    # deactivate myself on owned EC
    OpenRTM_aist.CORBA_SeqUtil.for_each(self._ecMine,
                                        self.deactivate_comps(self._objref))
    # deactivate myself on other EC
    OpenRTM_aist.CORBA_SeqUtil.for_each(self._ecOther,
                                        self.deactivate_comps(self._objref))

    # stop and detach myself from owned EC
    for ec in self._ecMine:
      if not CORBA.is_nil(ec) or not ec._non_existent():
        # ret = ec.stop()
        # ec.remove_component(self._this())
        pass

    # detach myself from other EC
    for ec in self._ecOther:
      if not CORBA.is_nil(ec):
        # ec.stop()
        ec.remove_component(self._this())

    self._exiting = True
    return self.finalize()


  ##
  # @if jp
  #
  # @brief [CORBA interface] RTC が Alive 状態であるかどうか確認する。
  #
  # RTC が指定した ExecutionContext に対して Alive状態であるかどうか確認する。
  # RTC の状態が Active であるか、Inactive であるか、Error であるかは実行中の
  # ExecutionContext に依存する。すなわち、ある ExecutionContext に対しては
  # Active  状態であっても、他の ExecutionContext に対しては Inactive 状態と
  # なる場合もありえる。従って、このオペレーションは指定された
  # ExecutionContext に問い合わせて、この RTC の状態が Active、Inactive、
  # Error の場合には Alive 状態として返す。
  #
  # @param self
  #
  # @param exec_context 取得対象 ExecutionContext ハンドル
  #
  # @return Alive 状態確認結果
  #
  # @else
  #
  # @brief Confirm whether RTC is an Alive state or NOT.
  #
  # A component is alive or not regardless of the execution context from
  # which it is observed. However, whether or not it is Active, Inactive,
  # or in Error is dependent on the execution context(s) in which it is
  # running. That is, it may be Active in one context but Inactive in
  # another. Therefore, this operation shall report whether this RTC is
  # either Active, Inactive or in Error; which of those states a component
  # is in with respect to a particular context may be queried from the
  # context itself.
  #
  # @return Result of Alive state confirmation
  #
  # @endif
  # virtual CORBA::Boolean is_alive(ExecutionContext_ptr exec_context)
  def is_alive(self, exec_context):
    self._rtcout.RTC_TRACE("is_alive()")
    for ec in self._ecMine:
      if exec_context._is_equivalent(ec):
        return True

    for ec in self._ecOther:
      if not CORBA.is_nil(ec):
        if exec_context._is_equivalent(ec):
          return True

    return False


  ##
  # @if jp
  # @brief [CORBA interface] ExecutionContextListを取得する
  #
  # この RTC が所有する ExecutionContext のリストを取得する。
  #
  # @param self
  #
  # @return ExecutionContext リスト
  #
  # @else
  # @brief [CORBA interface] Get ExecutionContextList.
  #
  # This operation returns a list of all execution contexts owned by this RTC.
  #
  # @return ExecutionContext List
  #
  # @endif
  #def get_contexts(self):
  #  execlist = []
  #  OpenRTM_aist.CORBA_SeqUtil.for_each(self._execContexts, self.ec_copy(execlist))
  #  return execlist


  ##
  # @if jp
  # @brief [CORBA interface] ExecutionContextを取得する
  #
  # 指定したハンドルの ExecutionContext を取得する。
  # ハンドルから ExecutionContext へのマッピングは、特定の RTC インスタンスに
  # 固有である。ハンドルはこの RTC を attach_context した際に取得できる。
  #
  # @param self
  # @param ec_id 取得対象 ExecutionContext ハンドル
  #
  # @return ExecutionContext
  #
  # @else
  # @brief [CORBA interface] Get ExecutionContext.
  #
  # Obtain a reference to the execution context represented by the given 
  # handle.
  # The mapping from handle to context is specific to a particular RTC 
  # instance. The given handle must have been obtained by a previous call to 
  # attach_context on this RTC.
  #
  # @param ec_id ExecutionContext handle
  #
  # @return ExecutionContext
  #
  # @endif
  # virtual ExecutionContext_ptr get_context(UniqueId exec_handle)
  def get_context(self, ec_id):
    global ECOTHER_OFFSET

    self._rtcout.RTC_TRACE("get_context(%d)", ec_id)
    # owned EC
    if ec_id < ECOTHER_OFFSET:
      if ec_id < len(self._ecMine):
        return self._ecMine[ec_id]
      else:
        return RTC.ExecutionContext._nil

    # participating EC
    index = ec_id - ECOTHER_OFFSET

    if index < len(self._ecOther):
      if not CORBA.is_nil(self._ecOther[index]):
        return self._ecOther[index]

    return RTC.ExecutionContext._nil


  ##
  # @if jp
  # @brief [CORBA interface] 所有する ExecutionContextListを 取得する
  #
  # この RTC が所有する ExecutionContext のリストを取得する。
  #
  # @return ExecutionContext リスト
  #
  # @else
  # @brief [CORBA interface] Get ExecutionContextList.
  #
  # This operation returns a list of all execution contexts owned by this
  # RTC.
  #
  # @return ExecutionContext List
  #
  # @endif
  # virtual ExecutionContextList* get_owned_contexts()
  def get_owned_contexts(self):
    self._rtcout.RTC_TRACE("get_owned_contexts()")
    execlist = []
    OpenRTM_aist.CORBA_SeqUtil.for_each(self._ecMine, self.ec_copy(execlist))
    return execlist

  ##
  # @if jp
  # @brief [CORBA interface] 参加している ExecutionContextList を取得する
  #
  # この RTC が参加している ExecutionContext のリストを取得する。
  #
  # @return ExecutionContext リスト
  #
  # @else
  # @brief [CORBA interface] Get participating ExecutionContextList.
  #
  # This operation returns a list of all execution contexts in
  # which this RTC participates.
  #
  # @return ExecutionContext List
  #
  # @endif
  # virtual ExecutionContextList* get_participating_contexts()
  def get_participating_contexts(self):
    self._rtcout.RTC_TRACE("get_participating_contexts()")
    execlist = []
    OpenRTM_aist.CORBA_SeqUtil.for_each(self._ecOther, self.ec_copy(execlist))
    return execlist


  #
  # @if jp
  # @brief [CORBA interface] ExecutionContext のハンドルを返す
  #
  # @param ExecutionContext 実行コンテキスト
  #
  # @return ExecutionContextHandle
  #
  # 与えられた実行コンテキストに関連付けられたハンドルを返す。
  #
  # @else
  # @brief [CORBA interface] Return a handle of a ExecutionContext
  #
  # @param ExecutionContext
  #
  # @return ExecutionContextHandle
  #
  # This operation returns a handle that is associated with the given
  # execution context.
  #
  # @endif
  #
  # virtual ExecutionContextHandle_t
  #   get_context_handle(ExecutionContext_ptr cxt)
  def get_context_handle(self, cxt):
    self._rtcout.RTC_TRACE("get_context_handle()")

    num = OpenRTM_aist.CORBA_SeqUtil.find(self._ecMine, self.ec_find(cxt))
    if num != -1:
      return long(num)

    num = OpenRTM_aist.CORBA_SeqUtil.find(self._ecOther, self.ec_find(cxt))
    if num != -1:
      return long(num)

    return long(-1)


  #============================================================
  # RTC::RTObject
  #============================================================

  ##
  # @if jp
  #
  # @brief [RTObject CORBA interface] コンポーネントプロファイルを取得する
  #
  # 当該コンポーネントのプロファイル情報を返す。 
  #
  # @param self
  #
  # @return コンポーネントプロファイル
  #
  # @else
  #
  # @brief [RTObject CORBA interface] Get RTC's profile
  #
  # This operation returns the ComponentProfile of the RTC
  #
  # @return ComponentProfile
  #
  # @endif
  # virtual ComponentProfile* get_component_profile()
  def get_component_profile(self):
    self._rtcout.RTC_TRACE("get_component_profile()")
    try:
      prop_ = RTC.ComponentProfile(self._properties.getProperty("instance_name"),
                                   self._properties.getProperty("type_name"),
                                   self._properties.getProperty("description"),
                                   self._properties.getProperty("version"),
                                   self._properties.getProperty("vendor"),
                                   self._properties.getProperty("category"),
                                   self._portAdmin.getPortProfileList(),
                                   self._profile.parent,
                                   self._profile.properties) 
      OpenRTM_aist.NVUtil.copyFromProperties(self._profile.properties, self._properties)
      return prop_
      # return RTC.ComponentProfile(self._profile.instance_name,
      #               self._profile.type_name,
      #               self._profile.description,
      #               self._profile.version,
      #               self._profile.vendor,
      #               self._profile.category,
      #               self._portAdmin.getPortProfileList(),
      #               self._profile.parent,
      #               self._profile.properties)
    
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())

    assert(False)
    return None


  ##
  # @if jp
  #
  # @brief [RTObject CORBA interface] ポートを取得する
  #
  # 当該コンポーネントが保有するポートの参照を返す。
  #
  # @param self
  #
  # @return ポートリスト
  #
  # @else
  #
  # @brief [RTObject CORBA interface] Get Ports
  #
  # This operation returns a list of the RTCs ports.
  #
  # @return PortList
  #
  # @endif
  # virtual PortServiceList* get_ports()
  def get_ports(self):
    self._rtcout.RTC_TRACE("get_ports()")
    try:
      return self._portAdmin.getPortServiceList()
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())

    assert(False)
    return []



  # RTC::ComponentAction

  ##
  # @if jp
  # @brief [CORBA interface] ExecutionContextをattachする
  #
  # 指定した ExecutionContext にこの RTC を所属させる。この RTC と関連する 
  # ExecutionContext のハンドルを返す。
  # このオペレーションは、ExecutionContextOperations::add_component が呼ばれた
  # 際に呼び出される。返されたハンドルは他のクライアントで使用することを想定
  # していない。
  #
  # @param self
  # @param exec_context 所属先 ExecutionContext
  #
  # @return ExecutionContext ハンドル
  #
  # @else
  # @brief [CORBA interface] Attach ExecutionContext.
  #
  # Inform this RTC that it is participating in the given execution context. 
  # Return a handle that represents the association of this RTC with the 
  # context.
  # This operation is intended to be invoked by 
  # ExecutionContextOperations::add_component. It is not intended for use by 
  # other clients.
  #
  # @param exec_context Prticipating ExecutionContext
  #
  # @return ExecutionContext Handle
  #
  # @endif
  # UniqueId attach_context(ExecutionContext_ptr exec_context)
  def attach_context(self, exec_context):
    global ECOTHER_OFFSET
    self._rtcout.RTC_TRACE("attach_context()")
    # ID: 0 - (offset-1) : owned ec
    # ID: offset -       : participating ec
    # owned       ec index = ID
    # participate ec index = ID - offset
    ecs = exec_context._narrow(RTC.ExecutionContextService)
    if CORBA.is_nil(ecs):
      return -1
    
    # if m_ecOther has nil element, insert attached ec to there.
    for i in range(len(self._ecOther)):
      if CORBA.is_nil(self._ecOther[i]):
        self._ecOther[i] = ecs
        ec_id = i + ECOTHER_OFFSET
        self.onAttachExecutionContext(ec_id)
        return ec_id

    # no space in the list, push back ec to the last.
    OpenRTM_aist.CORBA_SeqUtil.push_back(self._ecOther,ecs)
    ec_id = long(len(self._ecOther) - 1 + ECOTHER_OFFSET)
    self.onAttachExecutionContext(ec_id)
    return ec_id


  # UniqueId bindContext(ExecutionContext_ptr exec_context);
  def bindContext(self, exec_context):
    global ECOTHER_OFFSET
    self._rtcout.RTC_TRACE("bindContext()")
    # ID: 0 - (offset-1) : owned ec
    # ID: offset -       : participating ec
    # owned       ec index = ID
    # participate ec index = ID - offset
    ecs = exec_context._narrow(RTC.ExecutionContextService)

    if CORBA.is_nil(ecs):
      return -1
    
    # if m_ecMine has nil element, insert attached ec to there.
    for i in range(len(self._ecMine)):
      if CORBA.is_nil(self._ecMine[i]):
        self._ecMine[i] = ecs
        self.onAttachExecutionContext(i)
        return i
        #return i + ECOTHER_OFFSET

    # no space in the list, push back ec to the last.
    OpenRTM_aist.CORBA_SeqUtil.push_back(self._ecMine,ecs)
    
    return long(len(self._ecMine) - 1)
    #return long(len(self._ecMine) - 1 + ECOTHER_OFFSET)


  ##
  # @if jp
  # @brief [CORBA interface] ExecutionContextをdetachする
  #
  # 指定した ExecutionContext からこの RTC の所属を解除する。
  # このオペレーションは、ExecutionContextOperations::remove_component が呼ば
  # れた際に呼び出される。返されたハンドルは他のクライアントで使用することを
  # 想定していない。
  # 
  # 制約
  # - 指定された ExecutionContext に RTC がすでに所属していない場合には、
  #   ReturnCode_t::PRECONDITION_NOT_MET が返される。
  # - 指定された ExecutionContext にたしいて対して RTC がActive 状態である場
  #   合には、 ReturnCode_t::PRECONDITION_NOT_MET が返される。
  #
  # @param self
  # @param ec_id 解除対象 ExecutionContextハンドル
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  # @brief [CORBA interface] Attach ExecutionContext.
  #
  # Inform this RTC that it is no longer participating in the given execution 
  # context.
  # This operation is intended to be invoked by 
  # ExecutionContextOperations::remove_component. It is not intended for use 
  # by other clients.
  # Constraints
  # - This operation may not be invoked if this RTC is not already 
  #   participating in the execution context. Such a call shall fail with 
  #   ReturnCode_t::PRECONDITION_NOT_MET.
  # - This operation may not be invoked if this RTC is Active in the indicated
  #   execution context. Otherwise, it shall fail with 
  #   ReturnCode_t::PRECONDITION_NOT_MET.
  #
  # @param ec_id Dettaching ExecutionContext Handle
  #
  # @return
  #
  # @endif
  # ReturnCode_t detach_context(UniqueId exec_handle)
  def detach_context(self, ec_id):
    global ECOTHER_OFFSET
    self._rtcout.RTC_TRACE("detach_context(%d)", ec_id)
    len_ = len(self._ecOther)

    # ID: 0 - (offset-1) : owned ec
    # ID: offset -       : participating ec
    # owned       ec index = ID
    # participate ec index = ID - offset
    if (long(ec_id) < long(ECOTHER_OFFSET)) or \
          (long(ec_id - ECOTHER_OFFSET) > len_):
      return RTC.BAD_PARAMETER
    
    index = long(ec_id - ECOTHER_OFFSET)

    if index < 0 or CORBA.is_nil(self._ecOther[index]):
      return RTC.BAD_PARAMETER
    
    #OpenRTM_aist.CORBA_SeqUtil.erase(self._ecOther, index)
    self._ecOther[index] = RTC.ExecutionContextService._nil
    self.onDetachExecutionContext(ec_id)
    return RTC.RTC_OK


  ##
  # @if jp
  #
  # @brief [ComponentAction CORBA interface] RTC の初期化
  #
  # RTC が初期化され、Alive 状態に遷移する。
  # RTC 固有の初期化処理はここで実行する。
  # このオペレーション呼び出しの結果として onInitialize() コールバック関数が
  # 呼び出される。
  #
  # @param self
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [ComponentAction CORBA interface] Initialize RTC
  #
  # The RTC has been initialized and entered the Alive state.
  # Any RTC-specific initialization logic should be performed here.
  #
  # @return
  #
  # @endif
  def on_initialize(self):
    self._rtcout.RTC_TRACE("on_initialize()")
    ret = RTC.RTC_ERROR
    try:
      self.preOnInitialize(0)
      ret = self.onInitialize()
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      ret = RTC.RTC_ERROR

    active_set = self._properties.getProperty("configuration.active_config",
                                              "default")

    if self._configsets.haveConfig(active_set):
      self._configsets.update(active_set)
    else:
      self._configsets.update("default")

    self.postOnInitialize(0,ret)
    return ret


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
  # @param self
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [ComponentAction CORBA interface] Finalize RTC
  #
  # The RTC is being destroyed.
  # Any final RTC-specific tear-down logic should be performed here.
  #
  # @return
  #
  # @endif
  def on_finalize(self):
    self._rtcout.RTC_TRACE("on_finalize()")
    ret = RTC.RTC_ERROR
    try:
      self.preOnFinalize(0)
      ret = self.onFinalize()
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      ret = RTC.RTC_ERROR
    self.postOnFinalize(0, ret)
    return ret


  ##
  # @if jp
  #
  # @brief [ComponentAction CORBA interface] RTC の開始
  #
  # RTC が所属する ExecutionContext が Stopped 状態から Running 状態へ遷移
  # した場合に呼び出される。
  # このオペレーション呼び出しの結果として onStartup() コールバック関数が
  # 呼び出される。
  #
  # @param self
  # @param ec_id 状態遷移した ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [ComponentAction CORBA interface] StartUp RTC
  #
  # The given execution context, in which the RTC is participating, has 
  # transitioned from Stopped to Running.
  #
  # @param ec_id
  #
  # @return
  #
  # @endif
  def on_startup(self, ec_id):
    self._rtcout.RTC_TRACE("on_startup(%d)", ec_id)
    ret = RTC.RTC_ERROR
    try:
      self.preOnStartup(ec_id)
      ret = self.onStartup(ec_id)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      ret = RTC.RTC_ERROR
    self.postOnStartup(ec_id, ret)
    return ret


  ##
  # @if jp
  #
  # @brief [ComponentAction CORBA interface] RTC の停止
  #
  # RTC が所属する ExecutionContext が Running 状態から Stopped 状態へ遷移
  # した場合に呼び出される。
  # このオペレーション呼び出しの結果として onShutdown() コールバック関数が
  # 呼び出される。
  #
  # @param self
  # @param ec_id 状態遷移した ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [ComponentAction CORBA interface] ShutDown RTC
  #
  # The given execution context, in which the RTC is participating, has 
  # transitioned from Running to Stopped.
  #
  # @param ec_id
  #
  # @return
  #
  # @endif
  def on_shutdown(self, ec_id):
    self._rtcout.RTC_TRACE("on_shutdown(%d)", ec_id)
    ret = RTC.RTC_ERROR
    try:
      self.preOnShutdown(ec_id)
      ret = self.onShutdown(ec_id)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      ret = RTC.RTC_ERROR
    self.postOnShutdown(ec_id, ret)
    return ret


  ##
  # @if jp
  #
  # @brief [ComponentAction CORBA interface] RTC の活性化
  #
  # 所属する ExecutionContext から RTC が活性化された際に呼び出される。
  # このオペレーション呼び出しの結果として onActivated() コールバック関数が
  # 呼び出される。
  #
  # @param self
  # @param ec_id 活性化 ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [ComponentAction CORBA interface] Activate RTC
  #
  # The RTC has been activated in the given execution context.
  #
  # @param ec_id
  #
  # @return
  #
  # @endif
  def on_activated(self, ec_id):
    self._rtcout.RTC_TRACE("on_activated(%d)", ec_id)
    ret = RTC.RTC_ERROR
    try:
      self.preOnActivated(ec_id)
      self._configsets.update()
      ret = self.onActivated(ec_id)
      self._portAdmin.activatePorts()
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      ret = RTC.RTC_ERROR
    self.postOnActivated(ec_id, ret)
    return ret


  ##
  # @if jp
  #
  # @brief [ComponentAction CORBA interface] RTC の非活性化
  #
  # 所属する ExecutionContext から RTC が非活性化された際に呼び出される。
  # このオペレーション呼び出しの結果として onDeactivated() コールバック関数が
  # 呼び出される。
  #
  # @param self
  # @param ec_id 非活性化 ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [ComponentAction CORBA interface] Deactivate RTC
  #
  # The RTC has been deactivated in the given execution context.
  #
  # @param ec_id
  #
  # @return
  #
  # @endif
  def on_deactivated(self, ec_id):
    self._rtcout.RTC_TRACE("on_deactivated(%d)", ec_id)
    ret = RTC.RTC_ERROR
    try:
      self.preOnDeactivated(ec_id)
      self._portAdmin.deactivatePorts()
      ret = self.onDeactivated(ec_id)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      ret = RTC.RTC_ERROR
    self.postOnDeactivated(ec_id, ret)
    return ret


  ##
  # @if jp
  #
  # @brief [ComponentAction CORBA interface] RTC のエラー状態への遷移
  #
  # RTC が所属する ExecutionContext が Active 状態から Error 状態へ遷移した
  # 場合に呼び出される。
  # このオペレーションは RTC が Error 状態に遷移した際に一度だけ呼び出される。
  # このオペレーション呼び出しの結果として onAborting() コールバック関数が
  # 呼び出される。
  #
  # @param self
  # @param ec_id 状態遷移した ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [ComponentAction CORBA interface] Transition Error State
  #
  # The RTC is transitioning from the Active state to the Error state in some
  # execution context.
  # This callback is invoked only a single time for time that the RTC 
  # transitions into the Error state from another state. This behavior is in 
  # contrast to that of on_error.
  #
  # @param ec_id
  #
  # @return
  #
  # @endif
  def on_aborting(self, ec_id):
    self._rtcout.RTC_TRACE("on_aborting(%d)", ec_id)
    ret = RTC.RTC_ERROR
    try:
      self.preOnAborting(ec_id)
      ret = self.onAborting(ec_id)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      ret = RTC.RTC_ERROR
    self.postOnAborting(ec_id, ret)
    return ret


  ##
  # @if jp
  #
  # @brief [ComponentAction CORBA interface] RTC のエラー処理
  #
  # RTC がエラー状態にいる際に呼び出される。
  # RTC がエラー状態の場合に、対象となる ExecutionContext のExecutionKind に
  # 応じたタイミングで呼び出される。例えば、
  # - ExecutionKind が PERIODIC の場合、本オペレーションは
  #   DataFlowComponentAction::on_execute と on_state_update の替わりに、
  #   設定された順番、設定された周期で呼び出される。
  # - ExecutionKind が EVENT_DRIVEN の場合、本オペレーションは
  #   FsmParticipantAction::on_action が呼ばれた際に、替わりに呼び出される。
  # このオペレーション呼び出しの結果として onError() コールバック関数が呼び出
  # される。
  #
  # @param self
  # @param ec_id 対象 ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [ComponentAction CORBA interface] Error Processing of RTC
  #
  # The RTC remains in the Error state.
  # If the RTC is in the Error state relative to some execution context when
  # it would otherwise be invoked from that context (according to the 
  # context’s ExecutionKind), this callback shall be invoked instead. 
  # For example,
  # - If the ExecutionKind is PERIODIC, this operation shall be invoked in 
  #   sorted order at the rate of the context instead of 
  #   DataFlowComponentAction::on_execute and on_state_update.
  # - If the ExecutionKind is EVENT_DRIVEN, this operation shall be invoked 
  #   whenever FsmParticipantAction::on_action would otherwise have been 
  #   invoked.
  #
  # @param ec_id
  #
  # @return
  #
  # @endif
  def on_error(self, ec_id):
    self._rtcout.RTC_TRACE("on_error(%d)", ec_id)
    ret = RTC.RTC_ERROR
    try:
      self.preOnError(ec_id)
      ret = self.onError(ec_id)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      ret = RTC.RTC_ERROR
    self._configsets.update()
    self.postOnError(ec_id, ret)
    return ret


  ##
  # @if jp
  #
  # @brief [ComponentAction CORBA interface] RTC のリセット
  #
  # Error 状態にある RTC のリカバリ処理を実行し、Inactive 状態に復帰させる
  # 場合に呼び出される。
  # RTC のリカバリ処理が成功した場合は Inactive 状態に復帰するが、それ以外の
  # 場合には Error 状態に留まる。
  # このオペレーション呼び出しの結果として onReset() コールバック関数が呼び
  # 出される。
  #
  # @param self
  # @param ec_id リセット対象 ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [ComponentAction CORBA interface] Resetting RTC
  #
  # The RTC is in the Error state. An attempt is being made to recover it such
  # that it can return to the Inactive state.
  # If the RTC was successfully recovered and can safely return to the
  # Inactive state, this method shall complete with ReturnCode_t::OK. Any
  # other result shall indicate that the RTC should remain in the Error state.
  #
  # @param ec_id
  #
  # @return
  #
  # @endif
  def on_reset(self, ec_id):
    self._rtcout.RTC_TRACE("on_reset(%d)", ec_id)
    ret = RTC.RTC_ERROR
    try:
      self.preOnReset(ec_id)
      ret = self.onReset(ec_id)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      ret = RTC.RTC_ERROR
    self.postOnReset(ec_id, ret)
    return ret


  ##
  # @if jp
  #
  # @brief [DataFlowComponentAction CORBA interface] RTC の定常処理(第一周期)
  #
  # 以下の状態が保持されている場合に、設定された周期で定期的に呼び出される。
  # - RTC は Alive 状態である。
  # - 指定された ExecutionContext が Running 状態である。
  # 本オペレーションは、Two-Pass Execution の第一周期で実行される。
  # このオペレーション呼び出しの結果として onExecute() コールバック関数が呼び
  # 出される。
  #
  # 制約
  # - 指定された ExecutionContext の ExecutionKind は、 PERIODIC でなければな
  #   らない
  #
  # @param self
  # @param ec_id 定常処理対象 ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [DataFlowComponentAction CORBA interface] Primary Periodic 
  #        Operation of RTC
  #
  # This operation will be invoked periodically at the rate of the given
  # execution context as long as the following conditions hold:
  # - The RTC is Active.
  # - The given execution context is Running
  # This callback occurs during the first execution pass.
  #
  # Constraints
  # - The execution context of the given context shall be PERIODIC.
  #
  # @param ec_id
  #
  # @return
  #
  # @endif
  def on_execute(self, ec_id):
    self._rtcout.RTC_TRACE("on_execute(%d)", ec_id)
    ret = RTC.RTC_ERROR
    try:
      self.preOnExecute(ec_id)
      if self._readAll:
        self.readAll()
      
      ret = self.onExecute(ec_id)

      if self._writeAll:
        self.writeAll()
      
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      ret = RTC.RTC_ERROR
    self.postOnExecute(ec_id, ret)
    return ret


  ##
  # @if jp
  #
  # @brief [DataFlowComponentAction CORBA interface] RTC の定常処理(第二周期)
  #
  # 以下の状態が保持されている場合に、設定された周期で定期的に呼び出される。
  # - RTC は Alive 状態である。
  # - 指定された ExecutionContext が Running 状態である。
  # 本オペレーションは、Two-Pass Execution の第二周期で実行される。
  # このオペレーション呼び出しの結果として onStateUpdate() コールバック関数が
  # 呼び出される。
  #
  # 制約
  # - 指定された ExecutionContext の ExecutionKind は、 PERIODIC でなければな
  #   らない
  #
  # @param self
  # @param ec_id 定常処理対象 ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [DataFlowComponentAction CORBA interface] Secondary Periodic 
  #        Operation of RTC
  #
  # This operation will be invoked periodically at the rate of the given
  # execution context as long as the following conditions hold:
  # - The RTC is Active.
  # - The given execution context is Running
  # This callback occurs during the second execution pass.
  #
  # Constraints
  # - The execution context of the given context shall be PERIODIC.
  #
  # @param ec_id
  #
  # @return
  #
  # @endif
  def on_state_update(self, ec_id):
    self._rtcout.RTC_TRACE("on_state_update(%d)", ec_id)
    ret = RTC.RTC_ERROR
    try:
      self.preOnStateUpdate(ec_id)
      ret = self.onStateUpdate(ec_id)
      self._configsets.update()
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      ret = RTC.RTC_ERROR
    self.postOnStateUpdate(ec_id, ret)
    return ret


  ##
  # @if jp
  #
  # @brief [DataFlowComponentAction CORBA interface] 実行周期変更通知
  #
  # 本オペレーションは、ExecutionContext の実行周期が変更されたことを通知する
  # 際に呼び出される。
  # このオペレーション呼び出しの結果として onRateChanged() コールバック関数が
  # 呼び出される。
  #
  # 制約
  # - 指定された ExecutionContext の ExecutionKind は、 PERIODIC でなければな
  #   らない
  #
  # @param self
  # @param ec_id 定常処理対象 ExecutionContext の ID
  #
  # @return ReturnCode_t 型のリターンコード
  #
  # @else
  #
  # @brief [DataFlowComponentAction CORBA interface] Notify rate chenged
  #
  # This operation is a notification that the rate of the indicated execution 
  # context has changed.
  #
  # Constraints
  # - The execution context of the given context shall be PERIODIC.
  #
  # @param ec_id
  #
  # @return
  #
  # @endif
  def on_rate_changed(self, ec_id):
    self._rtcout.RTC_TRACE("on_rate_changed(%d)", ec_id)
    ret = RTC.RTC_ERROR
    try:
      self.preOnRateChanged(ec_id)
      ret = self.onRateChanged(ec_id)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      ret = RTC.RTC_ERROR
    self.postOnRateChanged(ec_id, ret)
    return ret


  #============================================================
  # SDOPackage::SdoSystemElement
  #============================================================

  ##
  # @if jp
  # 
  # @brief [SDO interface] Organization リストの取得 
  #
  # SDOSystemElement は0個もしくはそれ以上の Organization を所有することが
  # 出来る。 SDOSystemElement が1つ以上の Organization を所有している場合
  # には、このオペレーションは所有する Organization のリストを返す。
  # もしOrganizationを一つも所有していないければ空のリストを返す。
  #
  # @param self
  #
  # @return 所有している Organization リスト
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  #
  # @else
  #
  # @brief [SDO interface] Getting Organizations
  #
  # SDOSystemElement can be the owner of zero or more organizations.
  # If the SDOSystemElement owns one or more Organizations, this operation
  # returns the list of Organizations that the SDOSystemElement owns.
  # If it does not own any Organization, it returns empty list.
  #
  # @return Owned Organization List
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable if the target SDO is reachable but cannot
  #                         respond.
  # @exception InternalError if the target SDO cannot execute the operation
  #                          completely due to some internal error.
  #
  # @endif
  # virtual SDOPackage::OrganizationList* get_owned_organizations()
  def get_owned_organizations(self):
    self._rtcout.RTC_TRACE("get_owned_organizations()")
    try:
      return self._sdoOwnedOrganizations
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.NotAvailable("NotAvailable: get_owned_organizations")

    return []


  #============================================================
  # SDOPackage::SDO
  #============================================================

  ##
  # @if jp
  # 
  # @brief [SDO interface] SDO ID の取得
  #
  # SDO ID を返すオペレーション。
  # このオペレーションは以下の型の例外を発生させる。
  #
  # @param self
  # 
  # @return    リソースデータモデルで定義されている SDO の ID
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  #
  # @else
  #
  # @brief [SDO interface] Getting SDO ID
  #
  # This operation returns id of the SDO.
  # This operation throws SDOException with one of the following types.
  #
  # @return    id of the SDO defined in the resource data model.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable if the target SDO is reachable but cannot
  #                         respond.
  # @exception InternalError if the target SDO cannot execute the operation
  #                          completely due to some internal error.
  #
  # @endif
  # virtual char* get_sdo_id()
  def get_sdo_id(self):
    self._rtcout.RTC_TRACE("get_sdo_id()")
    try:
      return self._profile.instance_name
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("get_sdo_id()")


  ##
  # @if jp
  # 
  # @brief [SDO interface] SDO タイプの取得
  # 
  # SDO Type を返すオペレーション。
  # このオペレーションは以下の型の例外を発生させる。
  #
  # @param self
  #
  # @return    リソースデータモデルで定義されている SDO の Type
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  #
  # @else
  #
  # @brief [SDO interface] Getting SDO type
  #
  # This operation returns sdoType of the SDO.
  # This operation throws SDOException with one of the following types.
  #
  # @return    Type of the SDO defined in the resource data model.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable if the target SDO is reachable but cannot
  #                         respond.
  # @exception InternalError if the target SDO cannot execute the operation
  #                          completely due to some internal error.
  #
  # @endif
  # virtual char* get_sdo_type()
  def get_sdo_type(self):
    self._rtcout.RTC_TRACE("get_sdo_type()")
    try:
      return self._profile.description
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("get_sdo_type()")
    return ""


  ##
  # @if jp
  # 
  # @brief [SDO interface] SDO DeviceProfile リストの取得 
  #
  # SDO の DeviceProfile を返すオペレーション。 SDO がハードウエアデバイス
  # に関連付けられていない場合には、空の DeviceProfile が返される。
  # このオペレーションは以下の型の例外を発生させる。
  #
  # @param self
  #
  # @return    SDO DeviceProfile
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  #
  # @else
  #
  # @brief [SDO interface] Getting SDO DeviceProfile
  #
  # This operation returns the DeviceProfile of the SDO. If the SDO does not
  # represent any hardware device, then a DeviceProfile with empty values
  # are returned.
  # This operation throws SDOException with one of the following types.
  #
  # @return    The DeviceProfile of the SDO.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable if the target SDO is reachable but cannot
  #                         respond.
  # @exception InternalError if the target SDO cannot execute the operation
  #                          completely due to some internal error.
  #
  # @endif
  # virtual SDOPackage::DeviceProfile* get_device_profile()
  def get_device_profile(self):
    self._rtcout.RTC_TRACE("get_device_profile()")
    try:
      return self._SdoConfigImpl.getDeviceProfile()
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("get_device_profile()")

    return SDOPackage.DeviceProfile("","","","",[])


  ##
  # @if jp
  # 
  # @brief [SDO interface] SDO ServiceProfile の取得 
  #
  # SDO が所有している Service の ServiceProfile を返すオペレーション。
  # SDO がサービスを一つも所有していない場合には、空のリストを返す。
  # このオペレーションは以下の型の例外を発生させる。
  #
  # @param self
  # 
  # @return    SDO が提供する全ての Service の ServiceProfile。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  #
  # @else
  #
  # @brief [SDO interface] Getting SDO ServiceProfile
  # 
  # This operation returns a list of ServiceProfiles that the SDO has.
  # If the SDO does not provide any service, then an empty list is returned.
  # This operation throws SDOException with one of the following types.
  # 
  # @return    List of ServiceProfiles of all the services the SDO is
  #            providing.
  # 
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable if the target SDO is reachable but cannot
  #                         respond.
  # @exception InternalError if the target SDO cannot execute the operation
  #                          completely due to some internal error.
  #
  # @endif
  # virtual SDOPackage::ServiceProfileList* get_service_profiles()
  def get_service_profiles(self):
    self._rtcout.RTC_TRACE("get_service_profiles()")
    self._sdoSvcProfiles = self._SdoConfigImpl.getServiceProfiles()
    try:
      return self._sdoSvcProfiles
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("get_service_profiles()")

    return []


  ##
  # @if jp
  # 
  # @brief [SDO interface] 特定のServiceProfileの取得 
  #
  # 引数 "id" で指定された名前のサービスの ServiceProfile を返す。
  # 
  # @param     self
  # @param     _id SDO Service の ServiceProfile に関連付けられた識別子。
  # 
  # @return    指定された SDO Service の ServiceProfile。
  # 
  # @exception InvalidParameter "id" で指定した ServiceProfile が存在しない。
  #                             "id" が null。
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  #
  # @else
  #
  # @brief [SDO interface] Getting Organizations
  #
  # This operation returns the ServiceProfile that is specified by the
  # argument "id."
  # 
  # @param     _id The identifier referring to one of the ServiceProfiles.
  # 
  # @return    The profile of the specified service.
  # 
  # @exception InvalidParameter if the ServiceProfile that is specified by 
  #                             the argument 'id' does not exist or if 'id'
  #                             is 'null.'
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable If the target SDO is reachable but cannot
  #                         respond.
  # @exception InternalError If the target SDO cannot execute the operation
  #                          completely due to some internal error.
  #
  # @endif
  # virtual SDOPackage::ServiceProfile* get_service_profile(const char* id)
  def get_service_profile(self, _id):
    self._rtcout.RTC_TRACE("get_service_profile(%s)", _id)
    self._sdoSvcProfiles = self._SdoConfigImpl.getServiceProfiles()
    if not _id:
      raise SDOPackage.InvalidParameter("get_service_profile(): Empty name.")

    try:
      index = OpenRTM_aist.CORBA_SeqUtil.find(self._sdoSvcProfiles, self.svc_name(_id))

      if index < 0:
        raise SDOPackage.InvalidParameter("get_service_profile(): Not found")

      return self._sdoSvcProfiles[index]
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("get_service_profile()")

    return SDOPackage.ServiceProfile("", "", [], None)


  ##
  # @if jp
  # 
  # @brief [SDO interface] 指定された SDO Service の取得
  #
  # このオペレーションは引数 "id" で指定された名前によって区別される
  # SDO の Service へのオブジェクト参照を返す。 SDO により提供される
  # Service はそれぞれ一意の識別子により区別される。
  #
  # @param self
  # @param _id SDO Service に関連付けられた識別子。
  #
  # @return 要求された SDO Service への参照。
  #
  # 
  # @exception InvalidParameter "id" で指定した ServiceProfile が存在しない。
  #                             "id" が null。
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  #
  # @else
  #
  # @brief [SDO interface] Getting specified SDO Service's reference
  #
  # This operation returns an object implementing an SDO's service that
  # is identified by the identifier specified as an argument. Different
  # services provided by an SDO are distinguished with different
  # identifiers. See OMG SDO specification Section 2.2.8, "ServiceProfile,"
  # on page 2-12 for more details.
  #
  # @param _id The identifier referring to one of the SDO Service
  # @return The object implementing the requested service.
  # @exception InvalidParameter if argument “id” is null, or if the 
  #                             ServiceProfile that is specified by argument
  #                            “id” does not exist.
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable If the target SDO is reachable but cannot
  #                         respond.
  # @exception InternalError If the target SDO cannot execute the operation
  #                          completely due to some internal error.
  #
  # @endif
  # virtual SDOPackage::SDOService_ptr get_sdo_service(const char* id)
  def get_sdo_service(self, _id):
    self._rtcout.RTC_TRACE("get_sdo_service(%s)", _id)
    self._sdoSvcProfiles = self._SdoConfigImpl.getServiceProfiles()

    if not _id:
      raise SDOPackage.InvalidParameter("get_service(): Empty name.")

    index = OpenRTM_aist.CORBA_SeqUtil.find(self._sdoSvcProfiles, self.svc_name(_id))

    if index < 0:
      raise SDOPackage.InvalidParameter("get_service(): Not found")

    try:
      return self._sdoSvcProfiles[index].service
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("get_service()")
    return SDOPackage.SDOService._nil


  ##
  # @if jp
  # 
  # @brief [SDO interface] Configuration オブジェクトの取得 
  #
  # このオペレーションは Configuration interface への参照を返す。
  # Configuration interface は各 SDO を管理するためのインターフェースの
  # ひとつである。このインターフェースは DeviceProfile, ServiceProfile,
  # Organization で定義された SDO の属性値を設定するために使用される。
  # Configuration インターフェースの詳細については、OMG SDO specification
  # の 2.3.5節, p.2-24 を参照のこと。
  #
  # @param self
  #
  # @return SDO の Configuration インターフェースへの参照
  #
  # @exception InterfaceNotImplemented SDOはConfigurationインターフェースを
  #                                    持たない。
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  #
  # @else
  #
  # @brief [SDO interface] Getting Configuration object
  #
  # This operation returns an object implementing the Configuration
  # interface. The Configuration interface is one of the interfaces that
  # each SDO maintains. The interface is used to configure the attributes
  # defined in DeviceProfile, ServiceProfile, and Organization.
  # See OMG SDO specification Section 2.3.5, "Configuration Interface,"
  # on page 2-24 for more details about the Configuration interface.
  #
  # @return The Configuration interface of an SDO.
  #
  # @exception InterfaceNotImplemented The target SDO has no Configuration
  #                                    interface.
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  # virtual SDOPackage::Configuration_ptr get_configuration()
  def get_configuration(self):
    self._rtcout.RTC_TRACE("get_configuration()")
    if self._SdoConfig is None:
      raise SODPackage.InterfaceNotImplemented("InterfaceNotImplemented: get_configuration")
    try:
      return self._SdoConfig
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("get_configuration()")
    return SDOPackage.Configuration._nil


  ##
  # @if jp
  # 
  # @brief [SDO interface] Monitoring オブジェクトの取得 
  #
  # このオペレーションは Monitoring interface への参照を返す。
  # Monitoring interface は SDO が管理するインターフェースの一つである。
  # このインターフェースは SDO のプロパティをモニタリングするために
  # 使用される。
  # Monitoring interface の詳細については OMG SDO specification の
  # 2.3.7節 "Monitoring Interface" p.2-35 を参照のこと。
  #
  # @param self
  #
  # @return SDO の Monitoring interface への参照
  #
  # @exception InterfaceNotImplemented SDOはConfigurationインターフェースを
  #                                    持たない。
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  #
  # @else
  #
  # @brief [SDO interface] Get Monitoring object
  #
  # This operation returns an object implementing the Monitoring interface.
  # The Monitoring interface is one of the interfaces that each SDO
  # maintains. The interface is used to monitor the properties of an SDO.
  # See OMG SDO specification Section 2.3.7, "Monitoring Interface," on
  # page 2-35 for more details about the Monitoring interface.
  #
  # @return The Monitoring interface of an SDO.
  #
  # @exception InterfaceNotImplemented The target SDO has no Configuration
  #                                    interface.
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  # virtual SDOPackage::Monitoring_ptr get_monitoring()
  def get_monitoring(self):
    self._rtcout.RTC_TRACE("get_monitoring()")
    raise SDOPackage.InterfaceNotImplemented("Exception: get_monitoring")
    return SDOPackage.Monitoring._nil


  ##
  # @if jp
  # 
  # @brief [SDO interface] Organization リストの取得 
  #
  # SDO は0個以上の Organization (組織)に所属することができる。 もし SDO が
  # 1個以上の Organization に所属している場合、このオペレーションは所属する
  # Organization のリストを返す。SDO が どの Organization にも所属していない
  # 場合には、空のリストが返される。
  #
  # @param self
  #
  # @return SDO が所属する Organization のリスト。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [SDO interface] Getting Organizations
  #
  # An SDO belongs to zero or more organizations. If the SDO belongs to one
  # or more organizations, this operation returns the list of organizations
  # that the SDO belongs to. An empty list is returned if the SDO does not
  # belong to any Organizations.
  #
  # @return The list of Organizations that the SDO belong to.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  # @endif
  # virtual SDOPackage::OrganizationList* get_organizations()
  def get_organizations(self):
    self._rtcout.RTC_TRACE("get_organizations()")
    self._sdoOrganizations = self._SdoConfigImpl.getOrganizations()
    try:
      return self._sdoOrganizations
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("get_organizations()")
    return []


  ##
  # @if jp
  # 
  # @brief [SDO interface] SDO Status リストの取得 
  #
  # このオペレーションは SDO のステータスを表す NVList を返す。
  #
  # @param self
  #
  # @return SDO のステータス。
  #
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InternalError 内部的エラーが発生した。
  #
  # @else
  #
  # @brief [SDO interface] Get SDO Status
  #
  # This operation returns an NVlist describing the status of an SDO.
  #
  # @return The actual status of an SDO.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  #
  # @endif
  # virtual SDOPackage::NVList* get_status_list()
  def get_status_list(self):
    self._rtcout.RTC_TRACE("get_status_list()")
    try:
      return self._sdoStatus
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("get_status_list()")
    return []


  ##
  # @if jp
  # 
  # @brief [SDO interface] SDO Status の取得 
  #
  # This operation returns the value of the specified status parameter.
  #
  # @param self
  # @param name SDO のステータスを定義するパラメータ。
  # 
  # @return 指定されたパラメータのステータス値。
  # 
  # @exception SDONotExists ターゲットのSDOが存在しない。(本例外は、CORBA標準
  #                         システム例外のOBJECT_NOT_EXISTにマッピングされる)
  # @exception NotAvailable SDOは存在するが応答がない。
  # @exception InvalidParameter 引数 "name" が null あるいは存在しない。
  # @exception InternalError 内部的エラーが発生した。
  # @else
  #
  # @brief [SDO interface] Get SDO Status
  #
  # @param name One of the parameters defining the "status" of an SDO.
  #
  # @return The value of the specified status parameter.
  #
  # @exception SDONotExists if the target SDO does not exist.(This exception 
  #                         is mapped to CORBA standard system exception
  #                         OBJECT_NOT_EXIST.)
  # @exception NotAvailable The target SDO is reachable but cannot respond.
  # @exception InvalidParameter The parameter defined by "name" is null or
  #                             does not exist.
  # @exception InternalError The target SDO cannot execute the operation
  #                          completely due to some internal error.
  #
  #
  # @endif
  # virtual CORBA::Any* get_status(const char* name)
  def get_status(self, name):
    self._rtcout.RTC_TRACE("get_status(%s)", name)
    index = OpenRTM_aist.CORBA_SeqUtil.find(self._sdoStatus, self.nv_name(name))
    if index < 0:
      raise SDOPackage.InvalidParameter("get_status(): Not found")

    try:
      return any.to_any(self._sdoStatus[index].value)
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
      raise SDOPackage.InternalError("get_status()")
    return any.to_any("")


  #============================================================
  # Local interfaces
  #============================================================

  ##
  # @if jp
  #
  # @brief [local interface] インスタンス名の取得
  # 
  # ComponentProfile に設定されたインスタンス名を返す。
  #
  # @param self
  # 
  # @return インスタンス名
  # 
  # @else
  # 
  # @endif
  # const char* getInstanceName()
  def getInstanceName(self):
    self._rtcout.RTC_TRACE("getInstanceName()")
    return self._profile.instance_name


  ##
  # @if jp
  #
  # @brief [local interface] インスタンス名の設定
  # 
  # ComponentProfile に指定されたインスタンス名を設定する。
  #
  # @param self
  # 
  # @param instance_name インスタンス名
  # 
  # @else
  # 
  # @endif
  # void setInstanceName(const char* instance_name);
  def setInstanceName(self, instance_name):
    self._rtcout.RTC_TRACE("setInstanceName(%s)", instance_name)
    self._properties.setProperty("instance_name",instance_name)
    self._profile.instance_name = self._properties.getProperty("instance_name")


  ##
  # @if jp
  #
  # @brief [local interface] 型名の取得
  # 
  # ComponentProfile に設定された型名を返す。
  #
  # @param self
  # 
  # @return 型名
  # 
  # @else
  # 
  # @endif
  # const char* getTypeName()
  def getTypeName(self):
    self._rtcout.RTC_TRACE("getTypeName()")
    return self._profile.type_name


  ##
  # @if jp
  #
  # @brief [local interface] Description の取得
  # 
  # ComponentProfile に設定された Description を返す。
  #
  # @param self
  # 
  # @return Description
  # 
  # @else
  # 
  # @endif
  # const char* getDescription()
  def getDescription(self):
    self._rtcout.RTC_TRACE("getDescription()")
    return self._profile.description


  ##
  # @if jp
  #
  # @brief [local interface] バージョン情報の取得
  # 
  # ComponentProfile に設定されたバージョン情報を返す。
  #
  # @param self
  # 
  # @return バージョン情報
  # 
  # @else
  # 
  # @endif
  # const char* getVersion()
  def getVersion(self):
    self._rtcout.RTC_TRACE("getVersion()")
    return self._profile.version


  ##
  # @if jp
  #
  # @brief [local interface] ベンダー情報の取得
  # 
  # ComponentProfile に設定されたベンダー情報を返す。
  #
  # @param self
  # 
  # @return ベンダー情報
  # 
  # @else
  # 
  # @endif
  # const char* getVendor()
  def getVendor(self):
    self._rtcout.RTC_TRACE("getVendor()")
    return self._profile.vendor


  ##
  # @if jp
  #
  # @brief [local interface] カテゴリ情報の取得
  # 
  # ComponentProfile に設定されたカテゴリ情報を返す。
  #
  # @param self
  # 
  # @return カテゴリ情報
  # 
  # @else
  # 
  # @endif
  # const char* getCategory()
  def getCategory(self):
    self._rtcout.RTC_TRACE("getCategory()")
    return self._profile.category


  ##
  # @if jp
  #
  # @brief [local interface] Naming Server 情報の取得
  # 
  # 設定された Naming Server 情報を返す。
  #
  # @param self
  # 
  # @return Naming Server リスト
  # 
  # @else
  # 
  # @endif
  # std::vector<std::string> getNamingNames();
  def getNamingNames(self):
    self._rtcout.RTC_TRACE("getNamingNames()")
    return [s.strip() for s in self._properties.getProperty("naming.names").split(",")]


  ##
  # @if jp
  #
  # @brief [local interface] オブジェクトリファレンスの設定
  # 
  # RTC の CORBA オブジェクトリファレンスを設定する。
  # 
  # @param self
  # @param rtobj オブジェクトリファレンス
  # 
  # @else
  # 
  # @endif
  # void setObjRef(const RTObject_ptr rtobj);
  def setObjRef(self, rtobj):
    self._rtcout.RTC_TRACE("setObjRef()")
    self._objref = rtobj
    return


  ##
  # @if jp
  #
  # @brief [local interface] オブジェクトリファレンスの取得
  # 
  # 設定された CORBA オブジェクトリファレンスを取得する。
  # 
  # @param self
  # 
  # @return オブジェクトリファレンス
  # 
  # @else
  # 
  # @endif
  # RTObject_ptr getObjRef() const;
  def getObjRef(self):
    self._rtcout.RTC_TRACE("getObjRef()")
    return self._objref


  ##
  # @if jp
  # 
  # @brief [local interface] RTC のプロパティを設定する
  #
  # RTC が保持すべきプロパティを設定する。与えられるプロパティは、
  # ComponentProfile 等に設定されるべき情報を持たなければならない。
  # このオペレーションは通常 RTC が初期化される際に Manager から
  # 呼ばれることを意図している。
  # 
  # @param self
  # @param prop RTC のプロパティ
  #
  # @else
  #
  # @brief [local interface] Set RTC property
  #
  # This operation sets the properties to the RTC. The given property
  # values should include information for ComponentProfile.
  # Generally, this operation is designed to be called from Manager, when
  # RTC is initialized
  #
  # @param prop Property for RTC.
  #
  # @endif
  # void setProperties(const coil::Properties& prop);
  def setProperties(self, prop):
    self._rtcout.RTC_TRACE("setProperties()")
    self._properties.mergeProperties(prop)
    self._profile.instance_name = self._properties.getProperty("instance_name")
    self._profile.type_name     = self._properties.getProperty("type_name")
    self._profile.description   = self._properties.getProperty("description")
    self._profile.version       = self._properties.getProperty("version")
    self._profile.vendor        = self._properties.getProperty("vendor")
    self._profile.category      = self._properties.getProperty("category")


  ##
  # @if jp
  # 
  # @brief [local interface] RTC のプロパティを取得する
  #
  # RTC が保持しているプロパティを返す。
  # RTCがプロパティを持たない場合は空のプロパティが返される。
  # 
  # @param self
  # 
  # @return RTC のプロパティ
  #
  # @else
  #
  # @brief [local interface] Get RTC property
  #
  # This operation returns the properties of the RTC.
  # Empty property would be returned, if RTC has no property.
  #
  # @return Property for RTC.
  #
  # @endif
  # coil::Properties& getProperties();
  def getProperties(self):
    self._rtcout.RTC_TRACE("getProperties()")
    return self._properties


  ##
  # @if jp
  #
  # @brief コンフィギュレーションパラメータの設定
  # 
  # コンフィギュレーションパラメータと変数をバインドする
  # \<VarType\>としてコンフィギュレーションパラメータのデータ型を指定する。
  #
  # @param self
  # @param param_name コンフィギュレーションパラメータ名
  # @param var コンフィギュレーションパラメータ格納用変数
  # @param def_val コンフィギュレーションパラメータデフォルト値
  # @param trans 文字列変換用関数(デフォルト値:None)
  #
  # @return 設定結果(設定成功:true，設定失敗:false)
  # 
  # @else
  #
  # @endif
  #  template <typename VarType>
  #  bool bindParameter(const char* param_name, VarType& var,
  #                     const char* def_val,
  #                     bool (*trans)(VarType&, const char*) = coil::stringTo)
  def bindParameter(self, param_name, var,
                    def_val, trans=None):
    self._rtcout.RTC_TRACE("bindParameter()")
    if trans is None:
      trans_ = OpenRTM_aist.stringTo
    else:
      trans_ = trans
    self._configsets.bindParameter(param_name, var, def_val, trans_)
    return True


  ##
  # @if jp
  #
  # @brief コンフィギュレーションパラメータの更新(ID指定)
  # 
  # 指定したIDのコンフィギュレーションセットに設定した値で、
  # コンフィギュレーションパラメータの値を更新する
  #
  # @param self
  # @param config_set 設定対象のコンフィギュレーションセットID
  # 
  # @else
  #
  # @endif
  # void updateParameters(const char* config_set);
  def updateParameters(self, config_set):
    self._rtcout.RTC_TRACE("updateParameters(%s)", config_set)
    self._configsets.update(config_set)
    return


  ##
  # @if jp
  # 
  # @brief [local interface] Port を登録する
  #
  # RTC が保持するPortを登録する。
  # Port を外部からアクセス可能にするためには、このオペレーションにより
  # 登録されていなければならない。登録される Port はこの RTC 内部において
  # PortProfile.name により区別される。したがって、Port は RTC 内において、
  # ユニークな PortProfile.name を持たなければならない。
  # 登録された Port は内部で適切にアクティブ化された後、その参照と
  # オブジェクト参照がリスト内に保存される。
  # 
  # @param self
  # @param port RTC に登録する Port
  # @param port_type if port is PortBase, port_type is None,
  #                  if port is PortService, port_type is True
  #
  # @else
  #
  # @brief [local interface] Register Port
  #
  # This operation registers a Port to be held by this RTC.
  # In order to enable access to the Port from outside of RTC, the Port
  # must be registered by this operation. The Port that is registered by
  # this operation would be identified by PortProfile.name in the inside of
  # RTC. Therefore, the Port should have unique PortProfile.name in the RTC.
  # The registering Port would be activated properly, and the reference
  # and the object reference would be stored in lists in RTC.
  #
  # @param port Port which is registered in the RTC
  #
  # @endif
  # void registerPort(PortBase& port);
  def registerPort(self, port):
    self._rtcout.RTC_TRACE("registerPort()")
    if not self.addPort(port):
      self._rtcout.RTC_ERROR("addPort(PortBase&) failed.")
    return

  # void registerPort(PortService_ptr port);
  # def registerPortByReference(self, port_ref):
  #   self._rtcout.RTC_TRACE("registerPortByReference()")
  #   self.addPortByReference(port_ref)
  #   return

  # new interface. since 1.0.0-RELEASE
  # void addPort(PortBase& port);
  def addPort(self, port):
    self._rtcout.RTC_TRACE("addPort()")
    if isinstance(port, OpenRTM_aist.CorbaPort):
      self._rtcout.RTC_TRACE("addPort(CorbaPort)")
      propkey = "port.corbaport."
      prop = self._properties.getNode(propkey)
      if prop:
        self._properties.getNode(propkey).mergeProperties(self._properties.getNode("port.corba"))
      port.init(self._properties.getNode(propkey))
      port.setOwner(self.getObjRef())

    elif isinstance(port, OpenRTM_aist.PortBase):
      self._rtcout.RTC_TRACE("addPort(PortBase)")
      port.setOwner(self.getObjRef())
      port.setPortConnectListenerHolder(self._portconnListeners)
      self.onAddPort(port.getPortProfile())

    elif isinstance(port, RTC._objref_PortService):
      self._rtcout.RTC_TRACE("addPort(PortService)")
    return self._portAdmin.addPort(port)


  # new interface. since 1.0.0-RELEASE
  # void addPort(PortService_ptr port);
  # def addPortByReference(self, port_ref):
  #   self._rtcout.RTC_TRACE("addPortByReference()")
  #   self._portAdmin.registerPortByReference(port_ref)
  #   return
    

  ##
  # @if jp
  # 
  # @brief [local interface] DataInPort を登録する
  #
  # RTC が保持する DataInPort を登録する。
  # Port のプロパティにデータポートであること("port.dataport")、
  # TCPを使用すること("tcp_any")を設定するとともに、 DataInPort の
  # インスタンスを生成し、登録する。
  # 
  # @param self
  # @param name port 名称
  # @param inport 登録対象 DataInPort
  #
  # @else
  #
  # @endif
  def registerInPort(self, name, inport):
    self._rtcout.RTC_TRACE("registerInPort(%s)", name)
    if not self.addInPort(name, inport):
      self._rtcout.RTC_ERROR("addInPort(%s) failed.", name)
    return

  # new interface. since 1.0.0-RELEASE
  def addInPort(self, name, inport):
    self._rtcout.RTC_TRACE("addInPort(%s)", name)

    propkey = "port.inport." + name
    prop_ = copy.copy(self._properties.getNode(propkey))
    prop_.mergeProperties(self._properties.getNode("port.inport.dataport"))

    ret = self.addPort(inport)

    if not ret:
      self._rtcout.RTC_ERROR("addInPort() failed.")
      return ret
      
    inport.init(self._properties.getNode(propkey))
    self._inports.append(inport)
    return ret


  ##
  # @if jp
  # 
  # @brief [local interface] DataOutPort を登録する
  #
  # RTC が保持する DataOutPor tを登録する。
  # Port のプロパティにデータポートであること("port.dataport")、
  # TCPを使用すること("tcp_any")を設定するとともに、 DataOutPort の
  # インスタンスを生成し、登録する。
  # 
  # @param self
  # @param name port 名称
  # @param outport 登録対象 DataInPort
  #
  # @else
  #
  # @endif
  # void registerOutPort(const char* name, OutPortBase& outport);
  def registerOutPort(self, name, outport):
    self._rtcout.RTC_TRACE("registerOutPort(%s)", name)
    if not self.addOutPort(name, outport):
      self._rtcout.RTC_ERROR("addOutPort(%s) failed.", name)
    return

  # new interface. since 1.0.0-RELEASE
  # void addOutPort(const char* name, OutPortBase& outport);
  def addOutPort(self, name, outport):
    self._rtcout.RTC_TRACE("addOutPort(%s)", name)

    propkey = "port.outport." + name
    prop_ = copy.copy(self._properties.getNode(propkey))
    prop_.mergeProperties(self._properties.getNode("port.outport.dataport"))

    ret = self.addPort(outport)

    if not ret:
      self._rtcout.RTC_ERROR("addOutPort() failed.")
      return ret

    outport.init(self._properties.getNode(propkey))
    self._outports.append(outport)
    return ret


  ##
  # @if jp
  # 
  # @brief [local interface] InPort の登録を削除する
  #
  # RTC が保持するInPortの登録を削除する。
  # 
  # @param port 削除対象 Port
  # @return 削除結果(削除成功:true，削除失敗:false)
  #
  # @else
  #
  # @brief [local interface] Unregister InPort
  #
  # This operation unregisters a InPort held by this RTC.
  #
  # @param port Port which is unregistered
  # @return Unregister result (Successful:true, Failed:false)
  #
  # @endif
  #
  # bool removeInPort(InPortBase& port);
  def removeInPort(self, port):
    self._rtcout.RTC_TRACE("removeInPort()")
    ret = self.removePort(inport)

    if ret:
      for inport in self._inports:
        if port == inport:
          try:
            self._inports.remove(port)
          except:
            self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
            
          return True

    return False


  ##
  # @if jp
  # 
  # @brief [local interface] OutPort の登録を削除する
  #
  # RTC が保持するOutPortの登録を削除する。
  # 
  # @param port 削除対象 Port
  # @return 削除結果(削除成功:true，削除失敗:false)
  #
  # @else
  #
  # @brief [local interface] Unregister OutPort
  #
  # This operation unregisters a OutPort held by this RTC.
  #
  # @param port Port which is unregistered
  # @return Unregister result (Successful:true, Failed:false)
  #
  # @endif
  #
  # bool removeOutPort(OutPortBase& port);
  def removeOutPort(self, port):
    self._rtcout.RTC_TRACE("removeOutPort()")
    ret = self.removePort(outport)

    if ret:
      for outport in self._outports:
        if port == outport:
          try:
            self._outports.remove(port)
          except:
            self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())
            
          return True

    return False


  ##
  # @if jp
  # 
  # @brief [local interface] Port の登録を削除する
  #
  # RTC が保持するPortの登録を削除する。
  # 
  # @param self
  # @param port 削除対象 Port
  #
  # @else
  #
  # @brief [local interface] Unregister Port
  #
  # This operation unregisters a Port to be held by this RTC.
  #
  # @param port Port which is unregistered in the RTC
  #
  # @endif
  # void RTObject_impl::deletePort(PortBase& port)
  def deletePort(self, port):
    self._rtcout.RTC_TRACE("deletePort()")
    if not self.removePort(port):
      self._rtcout.RTC_ERROR("removePort() failed.")
    return

  # new interface. since 1.0.0-RELEASE
  def removePort(self, port):
    self._rtcout.RTC_TRACE("removePort()")
    if isinstance(port, OpenRTM_aist.PortBase) or isinstance(port, OpenRTM_aist.CorbaPort):
      self.onRemovePort(port.getPortProfile())
    return self._portAdmin.removePort(port)


  ##
  # @if jp
  # 
  # @brief [local interface] 名前指定により Port の登録を削除する
  #
  # 名称を指定して RTC が保持するPortの登録を削除する。
  # 
  # @param self
  # @param port_name 削除対象 Port 名
  #
  # @else
  #
  # @endif
  def deletePortByName(self, port_name):
    self._rtcout.RTC_TRACE("deletePortByName(%s)", port_name)
    self._portAdmin.deletePortByName(port_name)
    return


  ##
  # @if jp
  #
  # @brief [local interface] 実行コンテキストを取得する
  #
  # get_context() と同じ機能のローカル版。違いはない。
  # この関数は以下の関数内で呼ばれることを前提としている。
  #
  # - onStartup()
  # - onShutdown()
  # - onActivated()
  # - onDeactivated()
  # - onExecute()
  # - onAborting()
  # - onError()
  # - onReset()
  # - onStateUpdate()
  # - onRateChanged()
  # 
  # この関数の引数はこれらの関数の引数 UniquieID exec_handle でなけ
  # ればならない。
  # 
  # @param ec_id 上記関数の第1引数 exec_handle を渡す必要がある。
  # 
  # @else
  # 
  # @brief [local interface] Getting current execution context
  # 
  # This function is the local version of get_context(). completely
  # same as get_context() function. This function is assumed to be
  # called from the following functions.
  # 
  # - onStartup()
  # - onShutdown()
  # - onActivated()
  # - onDeactivated()
  # - onExecute()
  # - onAborting()
  # - onError()
  # - onReset()
  # - onStateUpdate()
  # - onRateChanged()
  # 
  # The argument of this function should be the first argument
  # (UniqueId ec_id) of the above functions.
  # 
  # @param ec_id The above functions' first argument "exec_handle."
  # 
  # @endif
  #
  # ExecutionContext_ptr getExecutionContext(RTC::UniqueId ec_id);
  def getExecutionContext(self, ec_id):
    return self.get_context(ec_id)

  ##
  # @if jp
  # 
  # @brief [local interface] 実行コンテキストの実行レートを取得する
  #
  # 現在実行中の実行コンテキストの実行レートを取得する。実行コンテキ
  # ストのKindがPERIODIC以外の場合の動作は未定義である。この関数は以
  # 下の関数内で呼ばれることを前提としている。
  #
  # - onStartup()
  # - onShutdown()
  # - onActivated()
  # - onDeactivated()
  # - onExecute()
  # - onAborting()
  # - onError()
  # - onReset()
  # - onStateUpdate()
  # - onRateChanged()
  #
  # この関数の引数はこれらの関数の引数 UniquieID exec_handle でなけ
  # ればならない。
  #
  # @param ec_id 上記関数の第1引数 exec_handle を渡す必要がある。
  #
  # @else
  # 
  # @brief [local interface] Getting current context' execution rate
  #
  # This function returns current execution rate in this
  # context. If this context's kind is not PERIODC, behavior is not
  # defined. This function is assumed to be called from the
  # following functions.
  #
  # - onStartup()
  # - onShutdown()
  # - onActivated()
  # - onDeactivated()
  # - onExecute()
  # - onAborting()
  # - onError()
  # - onReset()
  # - onStateUpdate()
  # - onRateChanged()
  #
  # The argument of this function should be the first argument
  # (UniqueId ec_id) of the above functions.
  #
  # @param ec_id The above functions' first argument "exec_handle."
  #
  # @endif
  #
  # double getExecutionRate(RTC::UniqueId ec_id);
  def getExecutionRate(self, ec_id):
    ec = self.getExecutionContext(ec_id)
    if CORBA.is_nil(ec):
      return 0.0

    return ec.get_rate()


  ##
  # @if jp
  # 
  # @brief [local interface] 実行コンテキストの実行レートを設定する
  #
  # 現在実行中の実行コンテキストの実行レートを設定する。実行コンテキ
  # ストのKindがPERIODIC以外の場合の動作は未定義である。この関数は以
  # 下の関数内で呼ばれることを前提としている。
  #
  # - onStartup()
  # - onShutdown()
  # - onActivated()
  # - onDeactivated()
  # - onExecute()
  # - onAborting()
  # - onError()
  # - onReset()
  # - onStateUpdate()
  # - onRateChanged()
  #
  # この関数の引数はこれらの関数の引数 UniquieID exec_handle でなけ
  # ればならない。
  #
  # @param ec_id 上記関数の第1引数 exec_handle を渡す必要がある。
  # @param rate 実行レートを [Hz] で与える
  #
  # @else
  # 
  # @brief [local interface] Setting current context' execution rate
  #
  # This function sets a execution rate in the context. If this
  # context's kind is not PERIODC, behavior is not defined. This
  # function is assumed to be called from the following functions.
  #
  # - onStartup()
  # - onShutdown()
  # - onActivated()
  # - onDeactivated()
  # - onExecute()
  # - onAborting()
  # - onError()
  # - onReset()
  # - onStateUpdate()
  # - onRateChanged()
  #
  # The argument of this function should be the first argument
  # (UniqueId ec_id) of the above functions.
  #
  # @param ec_id The above functions' first argument "exec_handle."
  # @param rate Execution rate in [Hz].
  #
  # @endif
  #
  # ReturnCode_t setExecutionRate(RTC::UniqueId ec_id, double rate);
  def setExecutionRate(self, ec_id, rate):
    ec = self.getExecutionContext(ec_id)
    if CORBA.is_nil(ec):
      return RTC.RTC_ERROR
    ec.set_rate(rate)
    return RTC.RTC_OK


  ##
  # @if jp
  # 
  # @brief [local interface] 実行コンテキストの所有権を調べる
  #
  # 現在実行中の実行コンテキストの所有権を調べる。この関数は以下の関
  # 数内で呼ばれることを前提としている。
  #
  # - onStartup()
  # - onShutdown()
  # - onActivated()
  # - onDeactivated()
  # - onExecute()
  # - onAborting()
  # - onError()
  # - onReset()
  # - onStateUpdate()
  # - onRateChanged()
  #
  # この関数の引数はこれらの関数の引数 UniquieID exec_handle でなけ
  # ればならない。
  #
  # @param ec_id 上記関数の第1引数 exec_handle を渡す必要がある。
  # @return true: 自身の実行コンテキスト、false: 他の実行コンテキスト
  #
  # @else
  # 
  # @brief [local interface] Checking if the current context is own context
  #
  # This function checks if the current context is own execution
  # context. This function is assumed to be called from the
  # following functions.
  #
  # - onStartup()
  # - onShutdown()
  # - onActivated()
  # - onDeactivated()
  # - onExecute()
  # - onAborting()
  # - onError()
  # - onReset()
  # - onStateUpdate()
  # - onRateChanged()
  #
  # The argument of this function should be the first argument
  # (UniqueId ec_id) of the above functions.
  #
  # @param ec_id The above functions' first argument "exec_handle."
  # @return true: Own context, false: other's context
  #
  # @endif
  #
  # bool isOwnExecutionContext(RTC::UniqueId ec_id);
  def isOwnExecutionContext(self, ec_id):
    global ECOTHER_OFFSET
    if ec_id < ECOTHER_OFFSET:
      return True
    return False


  ##
  # @if jp
  # 
  # @brief [local interface] 状態を Inactive に遷移させる
  #
  # 状態を Active から Inactive に遷移させる。この関数は以下の関
  # 数内で呼ばれることを前提としている。
  #
  # - onActivated()
  # - onExecute()
  # - onStateUpdate()
  #
  # この関数の引数は上記の関数の引数 UniquieID exec_handle でなけ
  # ればならない。
  #
  # @param ec_id 上記関数の第1引数 exec_handle を渡す必要がある。
  # @return リターンコード
  #
  # @else
  # 
  # @brief [local interface] Make transition to Inactive state
  #
  # This function makes transition from Active to Inactive
  # state. This function is assumed to be called from the following
  # functions.
  #
  # - onActivated()
  # - onExecute()
  # - onStateUpdate()
  #
  # The argument of this function should be the first argument
  # (UniqueId ec_id) of the above function.
  #
  # @param ec_id The above functions' first argument "exec_handle."
  # @return Return code
  #
  # @endif
  #
  # ReturnCode_t deactivate(RTC::UniqueId ec_id);
  def deactivate(self, ec_id):
    ec = self.getExecutionContext(ec_id)
    if CORBA.is_nil(ec):
      return RTC.RTC_ERROR
    return ec.deactivate_component(self.getObjRef())


  ##
  # @if jp
  # 
  # @brief [local interface] 状態を Active に遷移させる
  #
  # 状態を Inactive から Active に遷移させる。この関数は以下の関
  # 数内で呼ばれることを前提としている。
  #
  # - onStartup()
  # - onDeactivated()
  #
  # この関数の引数は上記の関数の引数 UniquieID exec_handle でなけ
  # ればならない。
  #
  # @param ec_id 上記関数の第1引数 exec_handle を渡す必要がある。
  # @return リターンコード
  #
  # @else
  # 
  # @brief [local interface] Make transition to Active state
  #
  # This function makes transition from Inactive to Active
  # state. This function is assumed to be called from the following
  # functions.
  #
  # - onStartup()
  # - onDeactivated()
  #
  # The argument of this function should be the first argument
  # (UniqueId ec_id) of the above function.
  #
  # @param ec_id The above functions' first argument "exec_handle."
  # @return Return code
  #
  # @endif
  #
  # ReturnCode_t activate(RTC::UniqueId ec_id);
  def activate(self, ec_id):
    ec = self.getExecutionContext(ec_id)
    if CORBA.is_nil(ec):
      return RTC.RTC_ERROR
    return ec.activate_component(self.getObjRef())


  ##
  # @if jp
  # 
  # @brief [local interface] 状態をリセットし Inactive に遷移させる
  #
  # 状態を Error から Inactive に遷移させる。この関数は以下の関
  # 数内で呼ばれることを前提としている。
  #
  # - onError()
  #
  # この関数の引数は上記の関数の引数 UniquieID exec_handle でなけ
  # ればならない。
  #
  # @param ec_id 上記関数の第1引数 exec_handle を渡す必要がある。
  # @return リターンコード
  #
  # @else
  # 
  # @brief [local interface] Resetting and go to Inactive state
  #
  # This function reset RTC and makes transition from Error to Inactive
  # state. This function is assumed to be called from the following
  # functions.
  #
  # - onError()
  #
  # The argument of this function should be the first argument
  # (UniqueId ec_id) of the above function.
  #
  # @param ec_id The above functions' first argument "exec_handle."
  # @return Return code
  #
  # @endif
  #
  # ReturnCode_t reset(RTC::UniqueId ec_id);
  def reset(self, ec_id):
    ec = self.getExecutionContext(ec_id)
    if CORBA.is_nil(ec):
      return RTC.RTC_ERROR
    return ec.reset_component(self.getObjRef())
    

  ##
  # @if jp
  # @brief [local interface] SDO service provider をセットする
  # @else
  # @brief [local interface] Set a SDO service provider
  # @endif
  #
  # bool addSdoServiceProvider(const SDOPackage::ServiceProfile& prof,
  #                            SdoServiceProviderBase* provider);
  def addSdoServiceProvider(self, prof, provider):
    return self._sdoservice.addSdoServiceProvider(prof, provider)


  ##
  # @if jp
  # @brief [local interface] SDO service provider を削除する
  # @else
  # @brief [local interface] Remove a SDO service provider
  # @endif
  #
  # bool removeSdoServiceProvider(const char* id);
  def removeSdoServiceProvider(self, id):
    return self._sdoservice.removeSdoServiceProvider(id)


  ##
  # @if jp
  # @brief [local interface] SDO service consumer をセットする
  # @else
  # @brief [local interface] Set a SDO service consumer
  # @endif
  #
  # bool addSdoServiceConsumer(const SDOPackage::ServiceProfile& prof);
  def addSdoServiceConsumer(self, prof):
    return self._sdoservice.addSdoServiceConsumer(prof)


  ##
  # @if jp
  # @brief [local interface] SDO service consumer を削除する
  # @else
  # @brief [local interface] Remove a SDO service consumer
  # @endif
  #
  # bool removeSdoServiceConsumer(const char* id);
  def removeSdoServiceConsumer(self, id):
    return self._sdoservice.removeSdoServiceConsumer(id)


  ##
  # @if jp
  #
  # @brief 全 InPort のデータを読み込む。
  #
  # RTC が保持する全ての InPort のデータを読み込む。
  #
  # @return 読み込み結果(全ポートの読み込み成功:true，失敗:false)
  #
  # @else
  #
  # @brief Readout the value from All InPorts.
  #
  # This operation read the value from all InPort
  # registered in the RTC.
  #
  # @return result (Successful:true, Failed:false)
  #
  # @endif
  #
  # bool readAll();
  def readAll(self):
    self._rtcout.RTC_TRACE("readAll()")
    ret = True
    for inport in self._inports:
      if not inport.read():
        self._rtcout.RTC_DEBUG("The error occurred in readAll().")
        ret = False
        if not self._readAllCompletion:
          return False

    return ret


  ##
  # @if jp
  #
  # @brief 全 OutPort のwrite()メソッドをコールする。
  #
  # RTC が保持する全ての OutPort のwrite()メソッドをコールする。
  #
  # @return 読み込み結果(全ポートへの書き込み成功:true，失敗:false)
  #
  # @else
  #
  # @brief The write() method of all OutPort is called. 
  #
  # This operation call the write() method of all OutPort
  # registered in the RTC.
  #
  # @return result (Successful:true, Failed:false)
  #
  # @endif
  #
  # bool writeAll();
  def writeAll(self):
    self._rtcout.RTC_TRACE("writeAll()")
    ret = True
    for outport in self._outports:
      if not outport.write():
        self._rtcout.RTC_DEBUG("The error occurred in writeAll().")
        ret = False
        if not self._writeAllCompletion:
          return False

    return ret


  ##
  # @if jp
  #
  # @brief onExecute()実行前でのreadAll()メソッドの呼出を有効または無効にする。
  #
  # このメソッドをパラメータをtrueとして呼ぶ事により、onExecute()実行前に
  # readAll()が呼出されるようになる。
  # パラメータがfalseの場合は、readAll()呼出を無効にする。
  #
  # @param read(default:true) 
  #        (readAll()メソッド呼出あり:true, readAll()メソッド呼出なし:false)
  #
  # @param completion(default:false) 
  #    readAll()にて、どれかの一つのInPortのread()が失敗しても全てのInPortのread()を呼び出す:true,
  #    readAll()にて、どれかの一つのInPortのread()が失敗した場合、すぐにfalseで抜ける:false
  #
  # @else
  #
  # @brief Set whether to execute the readAll() method. 
  #
  # Set whether to execute the readAll() method. 
  #
  # @param read(default:true)
  #        (readAll() is called:true, readAll() isn't called:false)
  #
  # @param completion(default:false)
  #     All InPort::read() calls are completed.:true,
  #     If one InPort::read() is False, return false.:false
  #
  # @param completion(default:false)
  #
  # @endif
  #
  # void setReadAll(bool read=true, bool completion=false);
  def setReadAll(self, read=True, completion=False):
    self._readAll = read
    self._readAllCompletion = completion


  ##
  # @if jp
  #
  # @brief onExecute()実行後にwriteAll()メソッドの呼出を有効または無効にする。
  #
  # このメソッドをパラメータをtrueとして呼ぶ事により、onExecute()実行後に
  # writeAll()が呼出されるようになる。
  # パラメータがfalseの場合は、writeAll()呼出を無効にする。
  #
  # @param write(default:true) 
  #        (writeAll()メソッド呼出あり:true, writeAll()メソッド呼出なし:false)
  #
  # @param completion(default:false) 
  #    writeAll()にて、どれかの一つのOutPortのwrite()が失敗しても全てのOutPortのwrite()を呼び出しを行う:true,
  #    writeAll()にて、どれかの一つのOutPortのwrite()が失敗した場合、すぐにfalseで抜ける:false
  #
  # @else
  #
  # @brief Set whether to execute the writeAll() method. 
  #
  # Set whether to execute the writeAll() method. 
  #
  # @param write(default:true)
  #        (writeAll() is called:true, writeAll() isn't called:false)
  #
  # @param completion(default:false)
  #     All OutPort::write() calls are completed.:true,
  #     If one OutPort::write() is False, return false.:false
  #
  # @endif
  #
  # void setWriteAll(bool write=true, bool completion=false);
  def setWriteAll(self, write=True, completion=False):
    self._writeAll = write
    self._writeAllCompletion = completion


  ##
  # @if jp
  #
  # @brief 全 Port の登録を削除する
  #
  # RTC が保持する全ての Port を削除する。
  # 
  # @param self
  #
  # @else
  #
  # @brief Unregister the All Portse
  #
  # This operation deactivates the all Port and deletes the all Port's
  # registrations in the RTC..
  #
  # @endif
  def finalizePorts(self):
    self._rtcout.RTC_TRACE("finalizePorts()")
    self._portAdmin.finalizePorts()
    self._inports = []
    self._outports = []
    return


  def finalizeContexts(self):
    self._rtcout.RTC_TRACE("finalizeContexts()")
    len_ = len(self._eclist)
    for i in range(len_):
      idx = (len_ - 1) - i
      self._eclist[idx].stop()
      try:
        self._poa.deactivate_object(self._poa.servant_to_id(self._eclist[idx]))
      except:
        self._rtcout.RTC_TRACE(OpenRTM_aist.Logger.print_exception())
      del self._eclist[idx]

    if self._eclist:
      self._eclist = []
    return


  ##
  # @if jp
  # @brief PreComponentActionListener リスナを追加する
  #
  # ComponentAction 実装関数の呼び出し直前のイベントに関連する各種リ
  # スナを設定する。
  #
  # 設定できるリスナのタイプとコールバックイベントは以下の通り
  #
  # - PRE_ON_INITIALIZE:    onInitialize 直前
  # - PRE_ON_FINALIZE:      onFinalize 直前
  # - PRE_ON_STARTUP:       onStartup 直前
  # - PRE_ON_SHUTDOWN:      onShutdown 直前
  # - PRE_ON_ACTIVATED:     onActivated 直前
  # - PRE_ON_DEACTIVATED:   onDeactivated 直前
  # - PRE_ON_ABORTING:       onAborted 直前
  # - PRE_ON_ERROR:         onError 直前
  # - PRE_ON_RESET:         onReset 直前
  # - PRE_ON_EXECUTE:       onExecute 直前
  # - PRE_ON_STATE_UPDATE:  onStateUpdate 直前
  #
  # リスナは PreComponentActionListener を継承し、以下のシグニチャを持つ
  # operator() を実装している必要がある。
  #
  # PreComponentActionListener::operator()(UniqueId ec_id)
  #
  # デフォルトでは、この関数に与えたリスナオブジェクトの所有権は
  # RTObjectに移り、RTObject解体時もしくは、
  # removePreComponentActionListener() により削除時に自動的に解体される。
  # リスナオブジェクトの所有権を呼び出し側で維持したい場合は、第3引
  # 数に false を指定し、自動的な解体を抑制することができる。
  #
  # @param listener_type リスナタイプ
  # @param memfunc 関数オブジェクト
  # @param autoclean リスナオブジェクトの自動的解体を行うかどうかのフラグ
  #
  # @else
  # @brief Adding PreComponentAction type listener
  #
  # This operation adds certain listeners related to ComponentActions
  # pre events.
  # The following listener types are available.
  #
  # - PRE_ON_INITIALIZE:    before onInitialize
  # - PRE_ON_FINALIZE:      before onFinalize
  # - PRE_ON_STARTUP:       before onStartup
  # - PRE_ON_SHUTDOWN:      before onShutdown
  # - PRE_ON_ACTIVATED:     before onActivated
  # - PRE_ON_DEACTIVATED:   before onDeactivated
  # - PRE_ON_ABORTING:       before onAborted
  # - PRE_ON_ERROR:         before onError
  # - PRE_ON_RESET:         before onReset
  # - PRE_ON_EXECUTE:       before onExecute
  # - PRE_ON_STATE_UPDATE:  before onStateUpdate
  #
  # Listeners should have the following function operator().
  #
  # PreComponentActionListener::operator()(UniqueId ec_id)
  #
  # The ownership of the given listener object is transferred to
  # this RTObject object in default.  The given listener object will
  # be destroied automatically in the RTObject's dtor or if the
  # listener is deleted by removePreComponentActionListener() function.
  # If you want to keep ownership of the listener object, give
  # "false" value to 3rd argument to inhibit automatic destruction.
  #
  # @param listener_type A listener type
  # @param memfunc  member function object
  # @param autoclean A flag for automatic listener destruction
  #
  # @endif
  #
  # template <class Listener>
  # PreComponentActionListener*
  # addPreComponentActionListener(PreCompActionListenerType listener_type,
  #                               void (Listener::*memfunc)(UniqueId ec_id),
  #                               bool autoclean = true)
  def addPreComponentActionListener(self, listener_type,
                                    memfunc, autoclean = True):
    class Noname(OpenRTM_aist.PreComponentActionListener):
      def __init__(self, memfunc):
        self._memfunc = memfunc

      def __call__(self, ec_id):
        self._memfunc(ec_id)
        return

    listener = Noname(memfunc)
    self._actionListeners.preaction_[listener_type].addListener(listener, autoclean)
    return listener


  ##
  # @if jp
  # @brief PreComponentActionListener リスナを削除する
  #
  # 設定した各種リスナを削除する。
  # 
  # @param listener_type リスナタイプ
  # @param listener リスナオブジェクトへのポインタ
  #
  # @else
  # @brief Removing PreComponentAction type listener
  #
  # This operation removes a specified listener.
  #     
  # @param listener_type A listener type
  # @param listener A pointer to a listener object
  #
  # @endif
  #
  # void 
  # removePreComponentActionListener(PreComponentActionListenerType listener_type,
  #                                  PreComponentActionListener* listener);
  def removePreComponentActionListener(self, listener_type, listener):
    self._actionListeners.preaction_[listener_type].removeListener(listener)
    return


  ##
  # @if jp
  # @brief PostComponentActionListener リスナを追加する
  #
  # ComponentAction 実装関数の呼び出し直後のイベントに関連する各種リ
  # スナを設定する。
  #
  # 設定できるリスナのタイプとコールバックイベントは以下の通り
  #
  # - POST_ON_INITIALIZE:    onInitialize 直後
  # - POST_ON_FINALIZE:      onFinalize 直後
  # - POST_ON_STARTUP:       onStartup 直後
  # - POST_ON_SHUTDOWN:      onShutdown 直後
  # - POST_ON_ACTIVATED:     onActivated 直後
  # - POST_ON_DEACTIVATED:   onDeactivated 直後
  # - POST_ON_ABORTING:       onAborted 直後
  # - POST_ON_ERROR:         onError 直後
  # - POST_ON_RESET:         onReset 直後
  # - POST_ON_EXECUTE:       onExecute 直後
  # - POST_ON_STATE_UPDATE:  onStateUpdate 直後
  #
  # リスナは PostComponentActionListener を継承し、以下のシグニチャを持つ
  # operator() を実装している必要がある。
  #
  # PostComponentActionListener::operator()(UniqueId ec_id, ReturnCode_t ret)
  #
  # デフォルトでは、この関数に与えたリスナオブジェクトの所有権は
  # RTObjectに移り、RTObject解体時もしくは、
  # removePostComponentActionListener() により削除時に自動的に解体される。
  # リスナオブジェクトの所有権を呼び出し側で維持したい場合は、第3引
  # 数に false を指定し、自動的な解体を抑制することができる。
  #
  # @param listener_type リスナタイプ
  # @param memfunc 関数オブジェクト
  # @param autoclean リスナオブジェクトの自動的解体を行うかどうかのフラグ
  #
  # @else
  # @brief Adding PostComponentAction type listener
  #
  # This operation adds certain listeners related to ComponentActions
  # post events.
  # The following listener types are available.
  #
  # - POST_ON_INITIALIZE:    after onInitialize
  # - POST_ON_FINALIZE:      after onFinalize
  # - POST_ON_STARTUP:       after onStartup
  # - POST_ON_SHUTDOWN:      after onShutdown
  # - POST_ON_ACTIVATED:     after onActivated
  # - POST_ON_DEACTIVATED:   after onDeactivated
  # - POST_ON_ABORTING:       after onAborted
  # - POST_ON_ERROR:         after onError
  # - POST_ON_RESET:         after onReset
  # - POST_ON_EXECUTE:       after onExecute
  # - POST_ON_STATE_UPDATE:  after onStateUpdate
  #
  # Listeners should have the following function operator().
  #
  # PostComponentActionListener::operator()(UniqueId ec_id, ReturnCode_t ret)
  #
  # The ownership of the given listener object is transferred to
  # this RTObject object in default.  The given listener object will
  # be destroied automatically in the RTObject's dtor or if the
  # listener is deleted by removePostComponentActionListener() function.
  # If you want to keep ownership of the listener object, give
  # "false" value to 3rd argument to inhibit automatic destruction.
  #
  # @param listener_type A listener type
  # @param memfunc  member function object
  # @param autoclean A flag for automatic listener destruction
  #
  # @endif
  #
  # template <class Listener>
  # PostComponentActionListener*
  # addPostComponentActionListener(PostCompActionListenerType listener_type,
  #                                void (Listener::*memfunc)(UniqueId ec_id,
  #                                                          ReturnCode_t ret),
  #                                bool autoclean = true)
  def addPostComponentActionListener(self, listener_type,
                                     memfunc, autoclean = True):
    class Noname(OpenRTM_aist.PostComponentActionListener):
      def __init__(self, memfunc):
        self._memfunc = memfunc
        return
      def __call__(self, ec_id, ret):
        self._memfunc(ec_id, ret)
        return
      
    listener = Noname(memfunc)
    self._actionListeners.postaction_[listener_type].addListener(listener, autoclean)
    return listener


  ##
  # @if jp
  # @brief PostComponentActionListener リスナを削除する
  #
  # 設定した各種リスナを削除する。
  # 
  # @param listener_type リスナタイプ
  # @param listener リスナオブジェクトへのポインタ
  #
  # @else
  # @brief Removing PostComponentAction type listener
  #
  # This operation removes a specified listener.
  #     
  # @param listener_type A listener type
  # @param listener A pointer to a listener object
  #
  # @endif
  ##
  # void 
  # removePostComponentActionListener(PostComponentActionListenerType listener_type,
  #                                   PostComponentActionListener* listener);
  def removePostComponentActionListener(self, listener_type, listener):
    self._actionListeners.postaction_[listener_type].removeListener(listener)
    return


  ##
  # @if jp
  # @brief PortActionListener リスナを追加する
  #
  # Portの追加、削除時にコールバックされる各種リスナを設定する。
  #
  # 設定できるリスナのタイプとコールバックイベントは以下の通り
  #
  # - ADD_PORT:    Port追加時
  # - REMOVE_PORT: Port削除時
  #
  # リスナは PortActionListener を継承し、以下のシグニチャを持つ
  # operator() を実装している必要がある。
  #
  # PortActionListener::operator()(PortProfile& pprof)
  #
  # デフォルトでは、この関数に与えたリスナオブジェクトの所有権は
  # RTObjectに移り、RTObject解体時もしくは、
  # removePortActionListener() により削除時に自動的に解体される。
  # リスナオブジェクトの所有権を呼び出し側で維持したい場合は、第3引
  # 数に false を指定し、自動的な解体を抑制することができる。
  #
  # @param listener_type リスナタイプ
  # @param memfunc 関数オブジェクト
  # @param autoclean リスナオブジェクトの自動的解体を行うかどうかのフラグ
  #
  # @else
  # @brief Adding PortAction type listener
  #
  # This operation adds certain listeners related to ComponentActions
  # post events.
  # The following listener types are available.
  #
  # - ADD_PORT:    At adding Port
  # - REMOVE_PORT: At removing Port
  #
  # Listeners should have the following function operator().
  #
  # PortActionListener::operator()(RTC::PortProfile pprof)
  #
  # The ownership of the given listener object is transferred to
  # this RTObject object in default.  The given listener object will
  # be destroied automatically in the RTObject's dtor or if the
  # listener is deleted by removePortActionListener() function.
  # If you want to keep ownership of the listener object, give
  # "false" value to 3rd argument to inhibit automatic destruction.
  #
  # @param listener_type A listener type
  # @param memfunc  member function object
  # @param autoclean A flag for automatic listener destruction
  #
  # @endif
  #
  # template <class Listener>
  # PortActionListener*
  # addPortActionListener(PortActionListenerType listener_type,
  #                       void (Listener::*memfunc)(const RTC::PortProfile&),
  #                       bool autoclean=true)
  def addPortActionListener(self, listener_type,
                            memfunc, autoclean = True):
    class Noname(OpenRTM_aist.PortActionListener):
      def __init__(self, memfunc):
        self._memfunc = memfunc
        return

      def __call__(self, pprofile):
        self._memfunc(pprofile)
        return

    listener = Noname(memfunc)
    self._actionListeners.portaction_[listener_type].addListener(listener, autoclean)
    return listener


  ##
  # @if jp
  # @brief PortActionListener リスナを削除する
  #
  # 設定した各種リスナを削除する。
  # 
  # @param listener_type リスナタイプ
  # @param listener リスナオブジェクトへのポインタ
  #
  # @else
  # @brief Removing PortAction type listener
  #
  # This operation removes a specified listener.
  #     
  # @param listener_type A listener type
  # @param listener A pointer to a listener object
  #
  # @endif
  # void 
  # removePortActionListener(PortActionListenerType listener_type,
  #                          PortActionListener* listener);
  def removePortActionListener(self, listener_type, listener):
    self._actionListeners.portaction_[listener_type].removeListener(listener)
    return


  ##
  # @if jp
  # @brief ExecutionContextActionListener リスナを追加する
  #
  # ExecutionContextの追加、削除時にコールバックされる各種リスナを設定する。
  #
  # 設定できるリスナのタイプとコールバックイベントは以下の通り
  #
  # - ATTACH_EC:    ExecutionContext アタッチ時
  # - DETACH_EC:    ExecutionContext デタッチ時
  #
  # リスナは ExecutionContextActionListener を継承し、以下のシグニチャを持つ
  # operator() を実装している必要がある。
  #
  # ExecutionContextActionListener::operator()(UniqueId　ec_id)
  #
  # デフォルトでは、この関数に与えたリスナオブジェクトの所有権は
  # RTObjectに移り、RTObject解体時もしくは、
  # removeExecutionContextActionListener() により削除時に自動的に解体される。
  # リスナオブジェクトの所有権を呼び出し側で維持したい場合は、第3引
  # 数に false を指定し、自動的な解体を抑制することができる。
  #
  # @param listener_type リスナタイプ
  # @param memfunc 関数オブジェクト
  # @param autoclean リスナオブジェクトの自動的解体を行うかどうかのフラグ
  #
  # @else
  # @brief Adding ExecutionContextAction type listener
  #
  # This operation adds certain listeners related to ComponentActions
  # post events.
  # The following listener types are available.
  #
  # - ADD_PORT:    At adding ExecutionContext
  # - REMOVE_PORT: At removing ExecutionContext
  #
  # Listeners should have the following function operator().
  #
  # ExecutionContextActionListener::operator()(UniqueId ec_id)
  #
  # The ownership of the given listener object is transferred to
  # this RTObject object in default.  The given listener object will
  # be destroied automatically in the RTObject's dtor or if the
  # listener is deleted by removeExecutionContextActionListener() function.
  # If you want to keep ownership of the listener object, give
  # "false" value to 3rd argument to inhibit automatic destruction.
  #
  # @param listener_type A listener type
  # @param memfunc  member function object
  # @param autoclean A flag for automatic listener destruction
  #
  # @endif
  #
  # template <class Listener>
  # ECActionListener*
  # addExecutionContextActionListener(ECActionListenerType listener_type,
  #                                   void (Listener::*memfunc)(UniqueId),
  #                                   bool autoclean = true);
  def addExecutionContextActionListener(self, listener_type,
                                        memfunc, autoclean = True):
    class Noname(OpenRTM_aist.ExecutionContextActionListener):
      def __init__(self, memfunc):
        self._memfunc = memfunc
        return

      def __call__(self, ec_id):
        self._memfunc(ec_id)
        return

    listener = Noname(memfunc)
    self._actionListeners.ecaction_[listener_type].addListener(listener, autoclean)
    return listener
    

  ##
  # @if jp
  # @brief ExecutionContextActionListener リスナを削除する
  #
  # 設定した各種リスナを削除する。
  # 
  # @param listener_type リスナタイプ
  # @param listener リスナオブジェクトへのポインタ
  #
  # @else
  # @brief Removing ExecutionContextAction type listener
  #
  # This operation removes a specified listener.
  #     
  # @param listener_type A listener type
  # @param listener A pointer to a listener object
  #
  # @endif
  #
  # void 
  # removeExecutionContextActionListener(ECActionListenerType listener_type,
  #                                      ECActionListener* listener);
  def removeExecutionContextActionListener(self, listener_type, listener):
    self._actionListeners.ecaction_[listener_type].removeListener(listener)
    return


  ##
  # @if jp
  # @brief PortConnectListener リスナを追加する
  #
  # Portの接続時や接続解除時に呼び出される各種リスナを設定する。
  #
  # 設定できるリスナのタイプとコールバックイベントは以下の通り
  #
  # - ON_NOTIFY_CONNECT: notify_connect() 関数内呼び出し直後
  # - ON_NOTIFY_DISCONNECT: notify_disconnect() 呼び出し直後
  # - ON_UNSUBSCRIBE_INTERFACES: notify_disconnect() 内のIF購読解除時
  #
  # リスナは PortConnectListener を継承し、以下のシグニチャを持つ
  # operator() を実装している必要がある。
  #
  # PortConnectListener::operator()(const char*, ConnectorProfile)
  #
  # デフォルトでは、この関数に与えたリスナオブジェクトの所有権は
  # RTObjectに移り、RTObject解体時もしくは、
  # removePortConnectListener() により削除時に自動的に解体される。
  # リスナオブジェクトの所有権を呼び出し側で維持したい場合は、第3引
  # 数に false を指定し、自動的な解体を抑制することができる。
  #
  # @param listener_type リスナタイプ
  # @param memfunc 関数オブジェクト
  # @param autoclean リスナオブジェクトの自動的解体を行うかどうかのフラグ
  #
  # @else
  # @brief Adding PortConnect type listener
  #
  # This operation adds certain listeners related to Port's connect actions.
  # The following listener types are available.
  #
  # - ON_NOTIFY_CONNECT: right after entering into notify_connect()
  # - ON_NOTIFY_DISCONNECT: right after entering into notify_disconnect()
  # - ON_UNSUBSCRIBE_INTERFACES: unsubscribing IF in notify_disconnect()
  #
  # Listeners should have the following function operator().
  #
  # PortConnectListener::operator()(const char*, ConnectorProfile)
  #
  # The ownership of the given listener object is transferred to
  # this RTObject object in default.  The given listener object will
  # be destroied automatically in the RTObject's dtor or if the
  # listener is deleted by removePortConnectListener() function.
  # If you want to keep ownership of the listener object, give
  # "false" value to 3rd argument to inhibit automatic destruction.
  #
  # @param listener_type A listener type
  # @param memfunc  member function object
  # @param autoclean A flag for automatic listener destruction
  #
  # @endif
  #
  # template <class Listener>
  # PortConnectListener*
  # addPortConnectListener(PortConnectListenerType listener_type,
  #                        void (Listener::*memfunc)(const char*,
  #                                                  ConnectorProfile&),
  #                        bool autoclean = true)
  def addPortConnectListener(self, listener_type,
                             memfunc, autoclean = True):
    class Noname(OpenRTM_aist.PortConnectListener):
      def __init__(self, memfunc):
        self._memfunc = memfunc
        return

      def __call__(self, portname, cprofile):
        self._memfunc(portname, cprofile)
        return

    listener = Noname(memfunc)
    self._portconnListeners.portconnect_[listener_type].addListener(listener, autoclean)
    return listener
    

  ##
  # @if jp
  # @brief PortConnectListener リスナを削除する
  #
  # 設定した各種リスナを削除する。
  # 
  # @param listener_type リスナタイプ
  # @param listener リスナオブジェクトへのポインタ
  #
  # @else
  # @brief Removing PortConnect type listener
  #
  # This operation removes a specified listener.
  #     
  # @param listener_type A listener type
  # @param listener A pointer to a listener object
  #
  # @endif
  #
  # void 
  # removePortConnectListener(PortConnectListenerType listener_type,
  #                           PortConnectListener* listener);
  def removePortConnectListener(self, listener_type, listener):
    self._portconnListeners.portconnect_[listener_type].removeListener(listener)
    return


  ##
  # @if jp
  # @brief PortConnectRetListener リスナを追加する
  #
  # Portの接続時や接続解除時に呼び出される各種リスナを設定する。
  #
  # 設定できるリスナのタイプとコールバックイベントは以下の通り
  #
  # - ON_CONNECT_NEXTPORT: notify_connect() 中のカスケード呼び出し直後
  # - ON_SUBSCRIBE_INTERFACES: notify_connect() 中のインターフェース購読直後
  # - ON_CONNECTED: nofity_connect() 接続処理完了時に呼び出される
  # - ON_DISCONNECT_NEXT: notify_disconnect() 中にカスケード呼び出し直後
  # - ON_DISCONNECTED: notify_disconnect() リターン時
  #
  # リスナは PortConnectRetListener を継承し、以下のシグニチャを持つ
  # operator() を実装している必要がある。
  #
  # PortConnectRetListener::operator()(const char*, ConnectorProfile)
  #
  # デフォルトでは、この関数に与えたリスナオブジェクトの所有権は
  # RTObjectに移り、RTObject解体時もしくは、
  # removePortConnectRetListener() により削除時に自動的に解体される。
  # リスナオブジェクトの所有権を呼び出し側で維持したい場合は、第3引
  # 数に false を指定し、自動的な解体を抑制することができる。
  #
  # @param listener_type リスナタイプ
  # @param memfunc 関数オブジェクト
  # @param autoclean リスナオブジェクトの自動的解体を行うかどうかのフラグ
  #
  # @else
  # @brief Adding PortConnectRet type listener
  #
  # This operation adds certain listeners related to Port's connect actions.
  # The following listener types are available.
  #
  # - ON_CONNECT_NEXTPORT: after cascade-call in notify_connect()
  # - ON_SUBSCRIBE_INTERFACES: after IF subscribing in notify_connect()
  # - ON_CONNECTED: completed nofity_connect() connection process
  # - ON_DISCONNECT_NEXT: after cascade-call in notify_disconnect()
  # - ON_DISCONNECTED: completed notify_disconnect() disconnection process
  #
  # Listeners should have the following function operator().
  #
  # PortConnectRetListener::operator()(const char*, ConnectorProfile)
  #
  # The ownership of the given listener object is transferred to
  # this RTObject object in default.  The given listener object will
  # be destroied automatically in the RTObject's dtor or if the
  # listener is deleted by removePortConnectRetListener() function.
  # If you want to keep ownership of the listener object, give
  # "false" value to 3rd argument to inhibit automatic destruction.
  #
  # @param listener_type A listener type
  # @param memfunc  member function object
  # @param autoclean A flag for automatic listener destruction
  #
  # @endif
  #
  # template <class Listener>
  # PortConnectRetListener*
  # addPortConnectRetListener(PortConnectRetListenerType listener_type,
  #                           void (Listener::*memfunc)(const char*,
  #                                                     ConnectorProfile&,
  #                                                     ReturnCode_t))
  def addPortConnectRetListener(self, listener_type,
                                memfunc, autoclean = True):
    class Noname(OpenRTM_aist.PortConnectRetListener):
      def __init__(self, memfunc):
        self._memfunc = memfunc
        return

      def __call__(self, portname, cprofile, ret):
        self._memfunc(portname, cprofile, ret)
        return

    listener = Noname(memfunc)
    self._portconnListeners.portconnret_[listener_type].addListener(listener, autoclean)
    return listener
    

  ##
  # @if jp
  # @brief PortConnectRetListener リスナを削除する
  #
  # 設定した各種リスナを削除する。
  # 
  # @param listener_type リスナタイプ
  # @param listener リスナオブジェクトへのポインタ
  #
  # @else
  # @brief Removing PortConnectRet type listener
  #
  # This operation removes a specified listener.
  #     
  # @param listener_type A listener type
  # @param listener A pointer to a listener object
  #
  # @endif
  #
  # void 
  # removePortConnectRetListener(PortConnectRetListenerType listener_type,
  #                              PortConnectRetListener* listener);
  def removePortConnectRetListener(self, listener_type, listener):
    self._portconnListeners.portconnret_[listener_type].removeListener(listener)
    return


  ##
  # @if jp
  #
  # @brief ConfigurationParamListener を追加する
  #
  # update(const char* config_set, const char* config_param) が呼ばれた際に
  # コールされるリスナ ConfigurationParamListener を追加する。
  # type には現在のところ ON_UPDATE_CONFIG_PARAM のみが入る。
  #
  # @param type ConfigurationParamListenerType型の値。
  #             ON_UPDATE_CONFIG_PARAM がある。
  #
  # @param memfunc 関数オブジェクト
  # @param autoclean リスナオブジェクトを自動で削除するかどうかのフラグ
  # 
  # @else
  #
  # @brief Adding ConfigurationParamListener 
  # 
  # This function adds a listener object which is called when
  # update(const char* config_set, const char* config_param) is
  # called. In the type argument, currently only
  # ON_UPDATE_CONFIG_PARAM is allowed.
  #
  # @param type ConfigurationParamListenerType value
  #             ON_UPDATE_CONFIG_PARAM is only allowed.
  #
  # @param memfunc  member function object
  # @param autoclean a flag whether if the listener object autocleaned.
  #
  # @endif
  #
  # template <class Listener>
  # ConfigurationParamListener*
  # addConfigurationParamListener(ConfigurationParamListenerType listener_type,
  #                               void (Listener::*memfunc)(const char*,
  #                                                         const char*),
  #                               bool autoclean = true)
  def addConfigurationParamListener(self, type,
                                    memfunc, autoclean = True):
    class Noname(OpenRTM_aist.ConfigurationParamListener):
      def __init__(self, memfunc):
        self._memfunc = memfunc
        return

      def __call__(self, config_set_name, config_param_name):
        self._memfunc(config_set_name, config_param_name)
        return

    listener = Noname(memfunc)
    self._configsets.addConfigurationParamListener(type, listener, autoclean)
    return listener


  ##
  # @if jp
  #
  # @brief ConfigurationParamListener を削除する
  #
  # addConfigurationParamListener で追加されたリスナオブジェクトを削除する。
  #
  # @param type ConfigurationParamListenerType型の値。
  #             ON_UPDATE_CONFIG_PARAM がある。
  # @param listener 与えたリスナオブジェクトへのポインタ
  # 
  # @else
  #
  # @brief Removing ConfigurationParamListener 
  # 
  # This function removes a listener object which is added by
  # addConfigurationParamListener() function.
  #
  # @param type ConfigurationParamListenerType value
  #             ON_UPDATE_CONFIG_PARAM is only allowed.
  # @param listener a pointer to ConfigurationParamListener listener object.
  #
  # @endif
  #
  # void removeConfigurationParamListener(ConfigurationParamListenerType type,
  #                                       ConfigurationParamListener* listener);
  def removeConfigurationParamListener(self, type, listener):
    self._configsets.removeConfigurationParamListener(type, listener)
    return
    

  ##
  # @if jp
  #
  # @brief ConfigurationSetListener を追加する
  #
  # ConfigurationSet が更新されたときなどに呼ばれるリスナ
  # ConfigurationSetListener を追加する。設定可能なイベントは以下の
  # 2種類がある。
  #
  # - ON_SET_CONFIG_SET: setConfigurationSetValues() で
  #                      ConfigurationSet に値が設定された場合。
  # - ON_ADD_CONFIG_SET: addConfigurationSet() で新しい
  #                      ConfigurationSet が追加された場合。
  #
  # @param type ConfigurationSetListenerType型の値。
  # @param memfunc 関数オブジェクト
  # @param autoclean リスナオブジェクトを自動で削除するかどうかのフラグ
  # 
  # @else
  #
  # @brief Adding ConfigurationSetListener 
  # 
  # This function add a listener object which is called when
  # ConfigurationSet is updated. Available events are the followings.
  #
  # @param type ConfigurationSetListenerType value
  # @param memfunc  member function object
  # @param autoclean a flag whether if the listener object autocleaned.
  #
  # @endif
  #
  # template <class Listener>
  # ConfigurationSetListener*
  # addConfigurationSetListener(ConfigurationSetListenerType listener_type,
  #                             void (Listener::*memfunc)
  #                             (const coil::Properties& config_set))
  def addConfigurationSetListener(self, listener_type,
                                  memfunc, autoclean = True):
    class Noname(OpenRTM_aist.ConfigurationSetListener):
      def __init__(self, memfunc):
        self._memfunc = memfunc
        return

      def __call__(self, config_set):
        self._memfunc(config_set)
        return

    listener = Noname(memfunc)
    self._configsets.addConfigurationSetListener(listener_type, listener, autoclean)
    return listener


  ##
  # @if jp
  #
  # @brief ConfigurationSetListener を削除する
  #
  # addConfigurationSetListener で追加されたリスナオブジェクトを削除する。
  #
  # @param type ConfigurationSetListenerType型の値。
  # @param listener 与えたリスナオブジェクトへのポインタ
  # 
  # @else
  #
  # @brief Removing ConfigurationSetListener 
  # 
  # This function removes a listener object which is added by
  # addConfigurationSetListener() function.
  #
  # @param type ConfigurationSetListenerType value
  # @param listener a pointer to ConfigurationSetListener listener object.
  #
  # @endif
  #
  # void removeConfigurationSetListener(ConfigurationSetListenerType type,
  #                                     ConfigurationSetListener* listener);
  def removeConfigurationSetListener(self, type, listener):
    self._configsets.removeConfigurationSetListener(type, listener)
    return


  ##
  # @if jp
  #
  # @brief ConfigurationSetNameListener を追加する
  #
  # ConfigurationSetName が更新されたときなどに呼ばれるリスナ
  # ConfigurationSetNameListener を追加する。設定可能なイベントは以下の
  # 3種類がある。
  #
  # - ON_UPDATE_CONFIG_SET: ある ConfigurationSet がアップデートされた
  # - ON_REMOVE_CONFIG_SET: ある ConfigurationSet が削除された
  # - ON_ACTIVATE_CONFIG_SET: ある ConfigurationSet がアクティブ化された
  #
  # @param type ConfigurationSetNameListenerType型の値。
  # @param memfunc 関数オブジェクト
  # @param autoclean リスナオブジェクトを自動で削除するかどうかのフラグ
  # 
  # @else
  #
  # @brief Adding ConfigurationSetNameListener 
  # 
  # This function add a listener object which is called when
  # ConfigurationSetName is updated. Available events are the followings.
  #
  # - ON_UPDATE_CONFIG_SET: A ConfigurationSet has been updated.
  # - ON_REMOVE_CONFIG_SET: A ConfigurationSet has been deleted.
  # - ON_ACTIVATE_CONFIG_SET: A ConfigurationSet has been activated.
  #
  # @param type ConfigurationSetNameListenerType value
  # @param memfunc  member function object
  # @param autoclean a flag whether if the listener object autocleaned.
  #
  # @endif
  #
  # template <class Listener>
  # ConfigurationSetNameListener*
  # addConfigurationSetNameListener(ConfigurationSetNameListenerType type,
  #                                 void (Listener::*memfunc)(const char*))
  def addConfigurationSetNameListener(self, type, memfunc, autoclean = True):
    class Noname(OpenRTM_aist.ConfigurationSetNameListener):
      def __init__(self, memfunc):
        self._memfunc = memfunc
        return

      def __call__(self, config_set_name):
        self._memfunc(config_set_name)
        return

    listener = Noname(memfunc)
    self._configsets.addConfigurationSetNameListener(type, listener, autoclean)
    return listener


  ##
  # @if jp
  #
  # @brief ConfigurationSetNameListener を削除する
  #
  # addConfigurationSetNameListener で追加されたリスナオブジェクトを
  # 削除する。
  #
  # @param type ConfigurationSetNameListenerType型の値。
  #             ON_UPDATE_CONFIG_PARAM がある。
  # @param listener 与えたリスナオブジェクトへのポインタ
  # 
  # @else
  #
  # @brief Removing ConfigurationSetNameListener 
  # 
  # This function removes a listener object which is added by
  # addConfigurationSetNameListener() function.
  #
  # @param type ConfigurationSetNameListenerType value
  #             ON_UPDATE_CONFIG_PARAM is only allowed.
  # @param listener a pointer to ConfigurationSetNameListener
  #             listener object.
  #
  # @endif
  # void
  # removeConfigurationSetNameListener(ConfigurationSetNameListenerType type,
  #                                    ConfigurationSetNameListener* listener);
  def removeConfigurationSetNameListener(self, type, listener):
    self._configsets.removeConfigurationSetNameListener(type, listener)
    return


  ##
  # @if jp
  #
  # @brief RTC を終了する
  #
  # RTC の終了処理を実行する。
  # 保持している全 Port の登録を解除するとともに、該当する CORBA オブジェクト
  # を非活性化し、RTC を終了する。
  # 
  # @param self
  #
  # @else
  #
  # @endif
  def shutdown(self):
    self._rtcout.RTC_TRACE("shutdown()")
    try:
      self.finalizePorts()
      self.finalizeContexts()
      self._poa.deactivate_object(self._poa.servant_to_id(self._SdoConfigImpl))
      self._poa.deactivate_object(self._poa.servant_to_id(self))
    except:
      self._rtcout.RTC_ERROR(OpenRTM_aist.Logger.print_exception())

    if self._manager:
      self._rtcout.RTC_DEBUG("Cleanup on Manager")
      self._manager.notifyFinalized(self)

    return

  # inline void preOnInitialize(UniqueId ec_id)
  def preOnInitialize(self, ec_id):
    self._actionListeners.preaction_[OpenRTM_aist.PreComponentActionListenerType.PRE_ON_INITIALIZE].notify(ec_id)
    return

  # inline void preOnFinalize(UniqueId ec_id)
  def preOnFinalize(self, ec_id):
    self._actionListeners.preaction_[OpenRTM_aist.PreComponentActionListenerType.PRE_ON_FINALIZE].notify(ec_id)
    return

  # inline void preOnStartup(UniqueId ec_id)
  def preOnStartup(self, ec_id):
    self._actionListeners.preaction_[OpenRTM_aist.PreComponentActionListenerType.PRE_ON_STARTUP].notify(ec_id)
    return

  # inline void preOnShutdown(UniqueId ec_id)
  def preOnShutdown(self, ec_id):
    self._actionListeners.preaction_[OpenRTM_aist.PreComponentActionListenerType.PRE_ON_SHUTDOWN].notify(ec_id)
    return

  # inline void preOnActivated(UniqueId ec_id)
  def preOnActivated(self, ec_id):
    self._actionListeners.preaction_[OpenRTM_aist.PreComponentActionListenerType.PRE_ON_ACTIVATED].notify(ec_id)
    return

  # inline void preOnDeactivated(UniqueId ec_id)
  def preOnDeactivated(self, ec_id):
    self._actionListeners.preaction_[OpenRTM_aist.PreComponentActionListenerType.PRE_ON_DEACTIVATED].notify(ec_id)
    return

  # inline void preOnAborting(UniqueId ec_id)
  def preOnAborting(self, ec_id):
    self._actionListeners.preaction_[OpenRTM_aist.PreComponentActionListenerType.PRE_ON_ABORTING].notify(ec_id)
    return

  # inline void preOnError(UniqueId ec_id)
  def preOnError(self, ec_id):
    self._actionListeners.preaction_[OpenRTM_aist.PreComponentActionListenerType.PRE_ON_ERROR].notify(ec_id)
    return

  # inline void preOnReset(UniqueId ec_id)
  def preOnReset(self, ec_id):
    self._actionListeners.preaction_[OpenRTM_aist.PreComponentActionListenerType.PRE_ON_RESET].notify(ec_id)
    return

  # inline void preOnExecute(UniqueId ec_id)
  def preOnExecute(self, ec_id):
    self._actionListeners.preaction_[OpenRTM_aist.PreComponentActionListenerType.PRE_ON_EXECUTE].notify(ec_id)
    return

  # inline void preOnStateUpdate(UniqueId ec_id)
  def preOnStateUpdate(self, ec_id):
    self._actionListeners.preaction_[OpenRTM_aist.PreComponentActionListenerType.PRE_ON_STATE_UPDATE].notify(ec_id)
    return
    

  # inline void preOnRateChanged(UniqueId ec_id)
  def preOnRateChanged(self, ec_id):
    self._actionListeners.preaction_[OpenRTM_aist.PreComponentActionListenerType.PRE_ON_RATE_CHANGED].notify(ec_id)
    return
    

  # inline void postOnInitialize(UniqueId ec_id, ReturnCode_t ret)
  def postOnInitialize(self, ec_id, ret):
    self._actionListeners.postaction_[OpenRTM_aist.PostComponentActionListenerType.POST_ON_INITIALIZE].notify(ec_id, ret)
    return
    

  # inline void postOnFinalize(UniqueId ec_id, ReturnCode_t ret)
  def postOnFinalize(self, ec_id, ret):
    self._actionListeners.postaction_[OpenRTM_aist.PostComponentActionListenerType.POST_ON_FINALIZE].notify(ec_id, ret)
    return
    

  # inline void postOnStartup(UniqueId ec_id, ReturnCode_t ret)
  def postOnStartup(self, ec_id, ret):
    self._actionListeners.postaction_[OpenRTM_aist.PostComponentActionListenerType.POST_ON_STARTUP].notify(ec_id, ret)
    return
    

  # inline void postOnShutdown(UniqueId ec_id, ReturnCode_t ret)
  def postOnShutdown(self, ec_id, ret):
    self._actionListeners.postaction_[OpenRTM_aist.PostComponentActionListenerType.POST_ON_SHUTDOWN].notify(ec_id, ret)
    return
    

  # inline void postOnActivated(UniqueId ec_id, ReturnCode_t ret)
  def postOnActivated(self, ec_id, ret):
    self._actionListeners.postaction_[OpenRTM_aist.PostComponentActionListenerType.POST_ON_ACTIVATED].notify(ec_id, ret)
    return
    

  # inline void postOnDeactivated(UniqueId ec_id, ReturnCode_t ret)
  def postOnDeactivated(self, ec_id, ret):
    self._actionListeners.postaction_[OpenRTM_aist.PostComponentActionListenerType.POST_ON_DEACTIVATED].notify(ec_id, ret)
    return
    

  # inline void postOnAborting(UniqueId ec_id, ReturnCode_t ret)
  def postOnAborting(self, ec_id, ret):
    self._actionListeners.postaction_[OpenRTM_aist.PostComponentActionListenerType.POST_ON_ABORTING].notify(ec_id, ret)
    return
    

  # inline void postOnError(UniqueId ec_id, ReturnCode_t ret)
  def postOnError(self, ec_id, ret):
    self._actionListeners.postaction_[OpenRTM_aist.PostComponentActionListenerType.POST_ON_ERROR].notify(ec_id, ret)
    return
    

  # inline void postOnReset(UniqueId ec_id, ReturnCode_t ret)
  def postOnReset(self, ec_id, ret):
    self._actionListeners.postaction_[OpenRTM_aist.PostComponentActionListenerType.POST_ON_RESET].notify(ec_id, ret)
    return
    

  # inline void postOnExecute(UniqueId ec_id, ReturnCode_t ret)
  def postOnExecute(self, ec_id, ret):
    self._actionListeners.postaction_[OpenRTM_aist.PostComponentActionListenerType.POST_ON_EXECUTE].notify(ec_id, ret)
    return
    

  # inline void postOnStateUpdate(UniqueId ec_id, ReturnCode_t ret)
  def postOnStateUpdate(self, ec_id, ret):
    self._actionListeners.postaction_[OpenRTM_aist.PostComponentActionListenerType.POST_ON_STATE_UPDATE].notify(ec_id, ret)
    return
    

  # inline void postOnRateChanged(UniqueId ec_id, ReturnCode_t ret)
  def postOnRateChanged(self, ec_id, ret):
    self._actionListeners.postaction_[OpenRTM_aist.PostComponentActionListenerType.POST_ON_RATE_CHANGED].notify(ec_id, ret)
    return
    

  # inline void onAddPort(const PortProfile& pprof)
  def onAddPort(self, pprof):
    self._actionListeners.portaction_[OpenRTM_aist.PortActionListenerType.ADD_PORT].notify(pprof)
    return
    
    
  # inline void onRemovePort(const PortProfile& pprof)
  def onRemovePort(self, pprof):
    self._actionListeners.portaction_[OpenRTM_aist.PortActionListenerType.REMOVE_PORT].notify(pprof)
    return
    
    
  # inline void onAttachExecutionContext(UniqueId ec_id)
  def onAttachExecutionContext(self, ec_id):
    self._actionListeners.ecaction_[OpenRTM_aist.ExecutionContextActionListenerType.EC_ATTACHED].notify(ec_id)
    return
    
    
  # inline void onDetachExecutionContext(UniqueId ec_id)
  def onDetachExecutionContext(self, ec_id):
    self._actionListeners.ecaction_[OpenRTM_aist.ExecutionContextActionListenerType.EC_DETACHED].notify(ec_id)
    return

    
  ##
  # @if jp
  # @class svc_name
  # @brief SDOService のプロファイルリストからidでサーチするための
  # ファンクタクラス
  # @else
  #
  # @endif
  class svc_name:
    def __init__(self, _id):
      self._id= _id

    def __call__(self, prof):
      return self._id == prof.id


  #------------------------------------------------------------
  # Functor
  #------------------------------------------------------------

  ##
  # @if jp
  # @class nv_name
  # @brief NVList 検索用ファンクタ
  # @else
  #
  # @endif
  class nv_name:
    def __init__(self, _name):
      self._name = _name

    def __call__(self, nv):
      return self._name == nv.name


  ##
  # @if jp
  # @class ec_find
  # @brief ExecutionContext 検索用ファンクタ
  # @else
  #
  # @endif
  class ec_find:
    def __init__(self, _ec):
      self._ec = _ec

    def __call__(self, ecs):
      try:
        if not CORBA.is_nil(ecs):
          ec = ecs._narrow(RTC.ExecutionContext)
          return self._ec._is_equivalent(ec)
      except:
        print OpenRTM_aist.Logger.print_exception()
        return False

      return False


  ##
  # @if jp
  # @class ec_copy
  # @brief ExecutionContext Copy用ファンクタ
  # @else
  #
  # @endif
  class ec_copy:
    def __init__(self, eclist):
      self._eclist = eclist

    def __call__(self, ecs):
      if not CORBA.is_nil(ecs):
        self._eclist.append(ecs)


  ##
  # @if jp
  # @class deactivate_comps
  # @brief RTC 非活性化用ファンクタ
  # @else
  #
  # @endif
  class deactivate_comps:
    def __init__(self, comp):
      self._comp = comp

    def __call__(self, ec):
      try:
        if not CORBA.is_nil(ec) and not ec._non_existent():
          ec.deactivate_component(self._comp)
          ec.stop()
      except:
        print OpenRTM_aist.Logger.print_exception()


# RtcBase = RTObject_impl
