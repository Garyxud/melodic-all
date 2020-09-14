#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file Factory.py
# @brief RTComponent factory class
# @date $Date: 2006/11/06 01:28:36 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2003-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import OpenRTM_aist


##
# @if jp
#
# @brief brief RTコンポーネント破棄用関数
#
# RTコンポーネントのインスタンスを破棄するための関数。
# 引数にて指定したRTコンポーネントのインスタンスを、終了処理を呼び出して
# 破棄する。
#
# @param rtc 破棄対象RTコンポーネントのインスタンス
#
# @else
#
# @endif
def Delete(rtc):
  del rtc



##
# @if jp
#
# @class FactoryBase
# @brief FactoryBase 基底クラス
# 
# RTコンポーネント生成用ファクトリの基底クラス。
# 実際の各種ファクトリクラスを実装する場合は、本クラスを継承する形で実装する。
# 実際の生成、削除処理は具象サブクラスにて実装する必要がある。
#
# @since 0.2.0
#
# @else
#
# @class FactoryBase
# @brief FactoryBase base class
#
# RTComponent factory base class.
#
# @since 0.2.0
#
# @endif
class FactoryBase:
  """
  """

  ##
  # @if jp
  #
  # @brief コンストラクタ
  #
  # コンストラクタ。
  #
  # @param self
  # @param profile コンポーネントのプロファイル
  #
  # @else
  #
  # @brief Constructor.
  #
  # Constructor.
  #
  # @param profile component profile
  #
  # @endif
  def __init__(self, profile):
    ## self._Profile Component profile
    self._Profile = profile
    ## self._Number Number of current component instances.
    self._Number = -1
    
    pass


  ##
  # @if jp
  #
  # @brief コンポーネントの生成(サブクラス実装用)
  #
  # RTComponent のインスタンスを生成するための関数。<BR>
  # 実際の初期化処理は、各具象クラス内にて記述する。
  #
  # @param self
  # @param mgr マネージャオブジェクト
  #
  # @return 生成したコンポーネント
  #
  # @else
  #
  # @brief Create component
  #
  # @param mgr pointer to RtcManager
  #
  # @endif
  def create(self, mgr):
    pass


  ##
  # @if jp
  #
  # @brief コンポーネントの破棄(サブクラス実装用)
  #
  # RTComponent のインスタンスを破棄するための関数。<BR>
  # 実際の初期化処理は、各具象クラス内にて記述する。
  #
  # @param self
  # @param comp 破棄対象 RTコンポーネント
  #
  # @else
  #
  # @brief Destroy component
  #
  # @param comp pointer to RtcBase
  #
  # @endif
  def destroy(self, comp):
    pass


  ##
  # @if jp
  #
  # @brief コンポーネントプロファイルの取得
  #
  # コンポーネントのプロファイルを取得する
  #
  # @param self
  #
  # @return コンポーネントのプロファイル
  #
  # @else
  #
  # @brief Get component profile
  #
  # Get component profile.
  #
  # @endif
  def profile(self):
    return self._Profile


  ##
  # @if jp
  #
  # @brief 現在のインスタンス数の取得
  #
  # コンポーネントの現在のインスタンス数を取得する。
  #
  # @param self
  #
  # @return コンポーネントのインスタンス数
  #
  # @else
  #
  # @brief Get number of component instances
  #
  # Get number of current component instances.
  #
  # @endif
  def number(self):
    return self._Number




##
# @if jp
# @class FactoryPython
# @brief FactoryPython クラス
# 
# Python用コンポーネントファクトリクラス。
#
# @since 0.4.1
#
#
# @else
#
# @class FactoryPython
# @brief FactoryPython class
#
# RTComponent factory class for Python.
#
# @endif
class FactoryPython(FactoryBase):
  """
  """

  ##
  # @if jp
  #
  # @brief コンストラクタ
  #
  # コンストラクタ。
  # 生成対象コンポーネントのプロファイル、コンポーネント生成用関数、
  # コンポーネント破棄用関数、コンポーネント生成時の命名ポリシーを引数に取り、
  # Python で実装されたコンポーネントのファクトリクラスを生成する。
  #
  # @param self
  # @param profile コンポーネントのプロファイル
  # @param new_func コンポーネント生成用関数
  # @param delete_func コンポーネント破棄用関数
  # @param policy コンポーネント生成時の命名ポリシー(デフォルト値:None)
  #
  # @else
  #
  # @brief Constructor.
  #
  # Constructor.
  # Create component factory class with three arguments:
  # component profile, function pointer to object create function and
  # object delete function.
  #
  # @param profile Component profile
  # @param new_func Pointer to component create function
  # @param delete_func Pointer to component delete function
  # @param policy Pointer to component delete function
  #
  # @endif
  def __init__(self, profile, new_func, delete_func, policy=None):
    FactoryBase.__init__(self, profile)
    
    if policy is None:
      self._policy = OpenRTM_aist.DefaultNumberingPolicy()
    else:
      self._policy = policy

    self._New = new_func
    
    self._Delete = delete_func


  ##
  # @if jp
  #
  # @brief コンポーネントの生成
  #
  # RTComponent のインスタンスを生成する。
  #
  # @param self
  # @param mgr マネージャオブジェクト
  #
  # @return 生成したコンポーネント
  #
  # @else
  #
  # @brief Create component
  #
  # Create component implemented in Python.
  #
  # @param mgr
  #
  # @endif
  def create(self, mgr):
    try:
      rtobj = self._New(mgr)
      if rtobj == 0:
        return None

      self._Number += 1
      
      rtobj.setProperties(self.profile())
      
      instance_name = rtobj.getTypeName()
      instance_name += self._policy.onCreate(rtobj)
      rtobj.setInstanceName(instance_name)

      return rtobj
    except:
      print OpenRTM_aist.Logger.print_exception()
      return None


  ##
  # @if jp
  #
  # @brief コンポーネントの破棄
  #
  # RTComponent のインスタンスを破棄する。
  #
  # @param self
  # @param comp 破棄対象 RTComponent
  #
  # @else
  #
  # @brief Destroy component
  #
  # Destroy component instance
  #
  # @param comp
  #
  # @endif
  def destroy(self, comp):
    self._Number -= 1
    self._policy.onDelete(comp)
    self._Delete(comp)
