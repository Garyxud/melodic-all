#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file SdoService.py
# @brief SDO Service administration class
# @date $Date: 2007/09/12 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
# 
# Copyright (C) 2006-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
# 

import SDOPackage, SDOPackage__POA

##
# @if jp
#
# @class SDOServiceProfile
# @brief SDO Service Profileクラス
#
# SDO Service Profile は SDO Service の情報を保持するためのクラスである。
#
# @since 0.4.0
#
# @else
#
# @class SDOServiceProfile
# @brief SDO Service Profile class
#
# @since 0.4.0
#
# @endif
class SDOServiceProfile:
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
  # @param id_ Service のID(デフォルト値:None)
  # @param type_ Service の型(デフォルト値:None)
  #
  # @else
  #
  # @endif
  def __init__(self, id_=None, type_=None):
    if id_ is None:
      self.id = ""
    else:
      self.id = id_

    if type_ is None:
      self.type = ""
    else:
      self.type = type_
      
    self.interfaceType = ""
    self.idlDefinition = ""
    self.properties = []
    self.serviceRef = None


  ##
  # @if jp
  #
  # @brief Service Profileを取得する
  # 
  # Service Profileを取得する
  #
  # @param self
  # 
  # @return Service Profile
  # 
  # @else
  #
  # @endif
  def getProfile(self):
    return self


  ##
  # @if jp
  # @brief ServiceProfile.id をセットする
  # 
  # SDO Service のIDをセットする
  # 
  # @param self
  # @param id_ Service のID
  # 
  # @else
  # @brief Setting ServiceProfile.id
  # @endif
  def setName(self, id_):
    self.id = id_


  ##
  # @if jp
  # @brief ServiceProfile.id を取得
  # 
  # SDO Service のIDを取得する
  # 
  # @param self
  # 
  # @return Service のID
  # 
  # @else
  # @brief Getting ServiceProfile.id
  # @endif
  def getName(self):
    return self.id


  ##
  # @if jp
  # @brief SDO ServiceProfile.interfaceType をセットする
  # 
  # SDO Service のinterfaceTypeをセットする
  # 
  # @param self
  # @param interfaceType Service のinterfaceType
  # 
  # @else
  # @brief Setting SDOServiceProfile.interfaceType
  # @endif
  def setInterfaceType(self, interfaceType):
    self.interfaceType = interfaceType
    


  # @if jp
  # @brief SDO ServiceProfile.interfaceType を取得する
  # 
  # SDO Service のinterfaceTypeを取得する
  # 
  # @param self
  # 
  # @return Service のinterfaceType
  # 
  # @else
  # @brief Getting SDOServiceProfile.interfaceType
  # @endif
  def getInterfaceType(self):
    return self.interfaceType


  ##
  # @if jp
  # @brief SDO ServiceProfile.idlDefinition をセットする
  # 
  # SDO Service のidlDefinitionをセットする
  # 
  # @param self
  # @param idlDefinition Service のidlDefinition
  # 
  # @else
  # @brief Setting SDOServiceProfile.idlDefnition
  # @endif
  def setIdlDefinition(self, idlDefinition):
    self.idlDefinition = idlDefinition


  ##
  # @if jp
  # @brief SDO ServiceProfile.idlDefinition を取得する
  # 
  # SDO Service のidlDefinitionを取得する
  # 
  # @param self
  # 
  # @return Service のidlDefinition
  # 
  # @else
  # @brief Getting SDO ServiceProfile.idlDefnition
  # @endif
  def getIdlDefinition(self):
    return self.idlDefinition


  ##
  # @if jp
  # @brief SDO ServiceProfile.properties をセットする
  # 
  # SDO Service のpropertiesをセットする
  # 
  # @param self
  # @param properties Service のproperties
  # 
  # @else
  # @brief Setting SDO ServiceProfile.properties
  # @endif
  def setProperties(self, properties):
    self.properties = properties


  ##
  # @if jp
  # @brief SDO ServiceProfile.properties を取得する
  # 
  # SDO Service のpropertiesを取得する
  # 
  # @param self
  # 
  # @return Service のproperties
  # 
  # @else
  # @brief Getting SDO ServiceProfile.properties
  # @endif
  def getProperties(self):
    return self.properties


  # bool addProperty(char name, CORBA::Any data);


  ##
  # @if jp
  # @brief SDO ServiceProfile.serviceRef をセットする
  # 
  # SDO Service のserviceRefをセットする
  # 
  # @param self
  # @param serviceRef Serviceへの参照
  # 
  # @else
  # @brief Setting SDO ServiceProfile.serviceRef
  # @endif
  def setServiceRef(self, serviceRef):
    self.serviceRef = serviceRef


  ##
  # @if jp
  # @brief SDO ServiceProfile.serviceRef を取得する
  # 
  # SDO Service への参照を取得する
  # 
  # @param self
  # 
  # @return Serviceへの参照
  # 
  # @else
  # @brief Getting SDO ServiceProfile.serviceRef
  # @endif
  def getServiceRef(self):
    return self.serviceRef
  
