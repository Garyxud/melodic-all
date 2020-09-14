#!/usr/bin/env python
# -*- coding: euc-jp -*-


##
#
# @file CorbaConsumer.py
# @brief CORBA Consumer class
# @date $Date: 2007/09/20 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
# 
# Copyright (C) 2006-2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


from omniORB import CORBA

##
# @if jp
# @class CorbaConsumerBase
#
# @brief オブジェクトリファレンスを保持するプレースホルダ基底クラス
#
# 通信手段として CORBA を選択した場合のコンシューマ実装のための基底クラス
#
# @since 0.4.0
#
# @else
# @class ConsumerBase
# @brief Placeholder base class to hold remote object reference.
# @endif
class CorbaConsumerBase:
  """
  """



  ##
  # @if jp
  #
  # @brief コンストラクタ
  # 
  # @param self 
  # @param consumer コピー元のCorbaConsumerBaseオブジェクト
  #
  # @else
  #
  # @brief Consructor
  #
  # @param self 
  #
  # @endif
  def __init__(self, consumer=None):
    if consumer:
      self._objref = consumer._objref
    else:
      self._objref = None


  ##
  # @if jp
  # 
  # @brief 代入演算子
  # 
  # @param self 
  # @param consumer 代入元
  # 
  # @return 代入結果
  # 
  # @else
  # 
  # @brief Assignment operator
  # 
  # @param self 
  # @param consumer Copy source.
  # 
  # @endif
  def equal(self, consumer):
    self._objref = consumer._objref
    return self


  ##
  # @if jp
  #
  # @brief CORBAオブジェクトをセットする
  #
  # 与えられたオブジェクトリファレンスは、ConsumerBase オブジェクト内に
  # CORBA::Object_var 型として保持される。 
  #
  # @param self
  # @param obj CORBA オブジェクトのリファレンス
  #
  # @return obj が nil リファレンスの場合 false を返す。
  #
  # @else
  #
  # @brief Set CORBA Object
  #
  # The given CORBA Object is held as CORBA::Object_var type
  #
  # @param self
  # @param obj Object reference of CORBA object
  #
  # @return If obj is nil reference, it returns false.
  #
  # @endif
  def setObject(self, obj):
    if CORBA.is_nil(obj):
      return False

    self._objref = obj
    return True


  ##
  # @if jp
  #
  # @brief CORBAオブジェクトを取得する
  #
  # ConsumerBase オブジェクト内に CORBA::Object_var 型として保持されている
  # オブジェクトリファレンスを取得する。 
  #
  # @param self
  #
  # @return obj CORBA オブジェクトのリファレンス
  #
  # @else
  #
  # @brief Get CORBA Object
  #
  # @param self
  #
  # @return Object reference of CORBA object
  #
  # @endif
  def getObject(self):
    return self._objref


  ##
  # @if jp
  #
  # @brief CORBAオブジェクトの設定をクリアする
  #
  # 設定されている CORBA オブジェクトをクリアする。
  # CORBAオブジェクトそのものに対しては何も操作しない。
  #
  # @param self
  #
  # @else
  #
  # @endif
  def releaseObject(self):
    self._objref = CORBA.Object._nil



##
# @if jp
#
# @class CorbaConsumer
# @brief オブジェクトリファレンスを保持するプレースホルダクラス
# 
# 引数で与えられた型のCORBAオブジェクトを保持する。
# オブジェクトがセットされたときに、与えられた型で narrow されるので、
# _ptr() で取得するリファレンスは、narrow 済みのリファレンスである。
#
# @since 0.4.0
#
# @else
#
# @class Consumer.CorbaConsumer
# @brief Placeholder class to hold remote object reference.
#
# This class holds a type of object that given by parameter.
# For internal use, _ptr type and _var type should be given as parameter.
#
# @since 0.4.0
#
# @endif
class CorbaConsumer(CorbaConsumerBase):
  """
  """



  ##
  # @if jp
  #
  # @brief コンストラクタ
  #
  # @param self
  # @param interfaceType このホルダが保持するオブジェクトの型
  #                      (デフォルト値;None)
  # @param consumer このホルダが保持するオブジェクト(デフォルト値;None)
  #
  # @else
  #
  # @brief Consructor
  #
  # @endif
  def __init__(self, interfaceType=None, consumer=None):
    if interfaceType:
      self._interfaceType = interfaceType
    else:
      self._interfaceType = None

    if consumer:
      CorbaConsumerBase.__init__(self, consumer)
      self._var = consumer._var
    else:
      CorbaConsumerBase.__init__(self)
      self._var = None


  ##
  # @if jp
  # 
  # @brief 代入演算子
  # 
  # @param self 
  # @param consumer 代入元
  # 
  # @return 代入結果
  # 
  # @else
  # 
  # @brief Assignment operator
  # 
  # @param self 
  # @param consumer Copy source.
  # 
  # @endif
  def equal(self, consumer):
    self._var = consumer._var


  def __del__(self):
    self.releaseObject()


  ##
  # @if jp
  # @brief オブジェクトをセットする
  #
  # ConsumerBase のオーバーライド。CORBA::Object_var にオブジェクトをセット
  # するとともに、パラメータの型で narrow したオブジェクトを保持する。
  #
  # @param self
  # @param obj CORBA Objecct
  #
  # @return オブジェクト設定結果
  #         設定対象オブジェクトが null の場合は false が返ってくる
  # 
  # @else
  # @brief Set Object
  #
  # Override function of ConsumerBase. This operation set an Object to 
  # CORBA:Object_var in the class, and this object is narrowed to
  # given parameter and stored in.
  #
  # @param self
  # @param obj CORBA Objecct
  #
  # @endif
  def setObject(self, obj):
    if not CorbaConsumerBase.setObject(self, obj):
      self.releaseObject()
      return False

    if self._interfaceType:
      self._var = obj._narrow(self._interfaceType)
    else:
      self._var = self._objref

    if not CORBA.is_nil(self._var):
      return True

    self.releaseObject()
    return False


  ##
  # @if jp
  # @brief ObjectType 型のオブジェクトのリファレンスを取得
  #
  # ObjectType に narrow済みのオブジェクトのリファレンスを取得する。
  # オブジェクトリファレンスを使用するには、setObject() でセット済みで
  # なければならない。
  # オブジェクトがセットされていなければ　nil オブジェクトリファレンスが
  # 返される。
  #
  # @param self
  #
  # @return ObjectType に narrow 済みのオブジェクトのリファレンス
  # 
  # @else
  # @brief Get Object reference narrowed as ObjectType
  #
  # This operation returns object reference narrowed as ObjectType.
  # To use the returned object reference, reference have to be set by
  # setObject().
  # If object is not set, this operation returns nil object reference.
  #
  # @return The object reference narrowed as ObjectType
  #
  # @endif
  def _ptr(self):
    return self._var


  ##
  # @if jp
  #
  # @brief CORBAオブジェクトの設定をクリアする
  #
  # 設定されている CORBA オブジェクトをクリアする。
  # CORBAオブジェクトそのものに対しては何も操作しない。
  #
  # @param self
  #
  # @else
  #
  # @endif
  def releaseObject(self):
    CorbaConsumerBase.releaseObject(self)
    self._var = CORBA.Object._nil
