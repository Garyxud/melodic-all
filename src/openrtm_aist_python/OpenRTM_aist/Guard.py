#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file  Guard.py
# @brief RT-Middleware mutx guard class
# @date  $Date$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#


##
# @if jp
# @class ScopedLock
# @brief ScopedLock クラス
#
# 排他処理用ロッククラス。
#
# @since 0.4.0
#
# @else
#
# @endif
class ScopedLock:
  """
  """

  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  # @param mutex ロック用ミューテックス
  #
  # @else
  #
  # @endif
  def __init__(self, mutex):
    self.mutex = mutex
    self.mutex.acquire()


  ##
  # @if jp
  # @brief デストラクタ
  #
  # デストラクタ
  #
  # @param self
  #
  # @else
  #
  # @endif
  def __del__(self):
    self.mutex.release()

