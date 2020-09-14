#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file ECFactory.py
# @brief ExecutionContext Factory class
# @date $Date: 2007/04/13 16:06:22 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
# 
# Copyright (C) 2007-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import string

import OpenRTM_aist


##
# @if jp
#
# @brief ExecutionContext破棄用関数
# 
# ExecutionContextのインスタンスを破棄するための関数。
#
# \param ec 破棄対象ExecutionContextのインスタンス
#
# @else
#
# @endif
def ECDelete(ec):
  del ec


##
# @if jp
# @class ECFactoryBase
# @brief ECFactoryBase 抽象クラス
# 
# ExecutionContext生成用Factoryの抽象クラス。
# 各ExecutionContextを生成するための具象Factoryクラスは、
# 以下の関数の実装を提供しなければならない。
#
# publicインターフェースとして以下のものを提供する。
# - name()   : 生成対象ExecutionContext名称の取得
# - create() : ExecutionContextインスタンスの生成
# - destroy(): ExecutionContextインスタンスの破棄
#
# @since 0.4.0
#
# @else
#
# @endif
class ECFactoryBase :
  """
  """

  ##
  # @if jp
  #
  # @brief 生成対象ExecutionContext名称取得用関数(サブクラス実装用)
  # 
  # 生成対象ExecutionContextの名称を取得するための関数。<BR>
  # この関数は具象サブクラスで実装する必要がある。
  #
  # @param self
  #
  # @return 生成対象ExecutionContext名称
  # 
  # @else
  # 
  # This method should be implemented in subclasses
  #
  # @endif
  def name(self):
    pass


  ##
  # @if jp
  #
  # @brief ExecutionContext生成用関数(サブクラス実装用)
  # 
  # ExecutionContextのインスタンスを生成するための関数。<BR>
  # この関数は具象サブクラスで実装する必要がある。
  #
  # @param self
  #
  # @return 生成したExecutionContextインスタンス
  # 
  # @else
  #
  # @endif
  def create(self):
    pass

  ##
  # @if jp
  #
  # @brief ExecutionContext破棄用関数(サブクラス実装用)
  # 
  # ExecutionContextのインスタンスを破棄するための関数。<BR>
  # この関数は具象サブクラスで実装する必要がある。
  #
  # @param self
  # @param comp 破棄対象のExecutionContextインスタンス
  # 
  # @else
  #
  # @endif
  def destroy(self, comp):
    pass



##
# @if jp
# @class ECFactoryPython
# @brief ECFactoryPython クラス
# 
# Python言語用ExecutionContextインスタンスを生成するFactoryクラス。
#
# @since 0.4.1
#
# @else
#
# @endif
class ECFactoryPython(ECFactoryBase):
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
  # @param name 生成対象ExecutionContext名称
  # @param new_func ExecutionContext生成用関数
  # @param delete_func ExecutionContext破棄用関数
  # 
  # @else
  #
  # @endif
  def __init__(self, name, new_func, delete_func):
    self._name   = name
    self._New    = new_func
    self._Delete = delete_func
    
    return


  ##
  # @if jp
  #
  # @brief 生成対象ExecutionContext名称を取得
  # 
  # 生成対象のExecutionContext名称を取得する。
  #
  # @param self
  #
  # @return 生成対象ExecutionContext名称
  # 
  # @else
  #
  # @endif
  def name(self):
    return self._name

  ##
  # @if jp
  #
  # @brief 生成対象ExecutionContextインスタンスを生成
  # 
  # 生成対象のExecutionContextクラスのインスタンスを生成する。
  #
  # @param self
  #
  # @return 生成したExecutionContextインスタンス
  # 
  # @else
  #
  # @endif
  def create(self):
    return self._New()

  ##
  # @if jp
  #
  # @brief 対象ExecutionContextインスタンスを破棄
  # 
  # 対象ExecutionContextクラスのインスタンスを破棄する。
  #
  # @param self
  # @param ec 破棄対象ExecutionContextインスタンス
  # 
  # @else
  #
  # @endif
  def destroy(self, ec):
    self._Delete(ec)
    
