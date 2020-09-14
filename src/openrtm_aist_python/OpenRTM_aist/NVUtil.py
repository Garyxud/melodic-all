#!/usr/bin/env python
# -*- coding: euc-jp -*- 

##
# @file NVUtil.py
# @brief NameValue and NVList utility functions
# @date $Date: 2007/09/11$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
# 
# Copyright (C) 2006-2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import sys
import traceback
from omniORB import any

import OpenRTM_aist
import SDOPackage, SDOPackage__POA


##
# @if jp
#
# @brief NameValue を生成する
#
# このオペレーションはNameValueを作成する。
#
# @param name NameValue の name
# @param value NameValue の value
#
# @return NameValue
#
# @else
#
# @brief Create NameVale
#
# This operation creates NameVale.
#
# @param name name of NameValue
# @param value value of NameValue
#
# @return NameValue
#
# @endif
def newNV(name, value):
  try:
    any_val = any.to_any(value)
  except:
    print "ERROR  NVUtil.newNV : Can't convert to any. ", type(value)
    raise

    
  nv = SDOPackage.NameValue(name, any_val)
  return nv


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
# void copyFromProperties(SDOPackage::NVList& nv, const coil::Properties& prop);
def copyFromProperties(nv, prop):
  keys = prop.propertyNames()
  keys_len = len(keys)
  nv_len = len(nv)
  if nv_len > 0:
    for i in range(nv_len):
      del nv[-1]

  for i in range(keys_len):
    nv.append(newNV(keys[i], prop.getProperty(keys[i])))


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
# void copyToProperties(coil::Properties& prop, const SDOPackage::NVList& nv);
def copyToProperties(prop, nvlist):
  for nv in nvlist:
    try:
      val = str(any.from_any(nv.value, keep_structs=True))
      prop.setProperty(str(nv.name),val)
    except:
      print OpenRTM_aist.Logger.print_exception()
      pass



##
# @if jp
# @class to_prop
# @brief NVList → Properties 変換用ファンクタ
# @endif
class to_prop:
  def __init__(self):
    self._prop = OpenRTM_aist.Properties()
    
  def __call__(self, nv):
    self._prop.setProperty(nv.name, nv.value)



##
# @if jp
#
# @brief NVList を Properties へ変換する
#
# このオペレーションは NVList を Properties へ変換する。
#
# @param nv 変換元の NVList
#
# @return 変換結果Property
#
# @else
#
# @endif
# coil::Properties toProperties(const SDOPackage::NVList& nv);
def toProperties(nv):
  p = OpenRTM_aist.CORBA_SeqUtil.for_each(nv, to_prop())
  return p._prop



##
# @if jp
# @class nv_find
# @brief NVList 検索用ファンクタ
# @endif
class nv_find:
  """
  """

  def __init__(self, name):
    self._name = name

  def __call__(self, nv):
    return str(self._name) == str(nv.name)


##
# @if jp
#
# @brief NVList から name で指定された value を返す
#
# このオペレーションは name で指定された value を Any 型で返す。
# 指定した名称の要素が存在しない場合は例外を発生させる。
#
# @param nv 検索対象の NVList
# @param name 検索する名前
#
# @return 検索結果
#
# @else
#
# @brief Get value in NVList specified by name
#
# This operation returns Any type of value specified by name.
# Created NVList's values are CORBA::string.
#
# @param nv NVList to be searched
# @param prop name to seartch in NVList
#
# @endif
def find(nv, name):
  index = OpenRTM_aist.CORBA_SeqUtil.find(nv, nv_find(name))

  if index < 0:
    raise "Not found."

  return nv[index].value


##
# @if jp
#
# @brief name で指定された要素のインデックスを返す
#
# このオペレーションは name で指定された要素が格納されている位置の
# インデックスを返す。
#
# @param nv 検索対象の NVList
# @param name 検索する名前
#
# @return 検索対象のインデックス
#
# @else
#
# @endif
def find_index(nv, name):
  return OpenRTM_aist.CORBA_SeqUtil.find(nv, nv_find(name))


##
# @if jp
#
# @brief 指定された name の value の型が string であるか検証する
#
# このオペレーションは name で指定された value の型が CORBA::string
# かどうかを bool 値で返す。
#
# @param nv 検索対象の NVList
# @param name 検索する名前
#
# @return string検証結果(string:true、それ以外:false)
#
# @else
#
# @endif
def isString(nv, name):
  try:
    value = find(nv, name)
    val = any.from_any(value, keep_structs=True)
    return type(val) == str
  except:
    return False


##
# @if jp
#
# @brief 指定された name の value の型が指定した文字列と一致するか検証する
#
# このオペレーションは name で指定された value の型が CORBA::string
# かどうかを判断し、  CORBA::string である場合には指定した文字列と一致するか
# をbool 値で返す。
#
# @param nv 検索対象の NVList
# @param name 検索する名前
# @param value 比較対象文字列
#
# @return 検証結果(文字列と一致:true、非一致:false)
#
# @else
#
# @endif
def isStringValue(nv, name, value):
  if isString(nv, name):
    if toString(nv, name) == value:
      return True
  return False


##
# @if jp
#
# @brief 指定された name の NVList を string として返す。
#
# このオペレーションは name で指定された NVList の値を string で返す。
# もし、name で指定した value の値が CORBA::string でなければ、
# 空の文字列のstringを返す。
#
# @param nv 検索対象の NVList
# @param name 検索する名前
#
# @return name に対応する値のstring型の値
#
# @else
#
# @brief Get string value in NVList specified by name
#
# This operation returns string value in NVList specified by name.
# If the value in NVList specified by name is not CORBA::string type
# this operation returns empty string value.
#
# @param nv NVList to be searched
# @param name name to to serach
#
# @return string value named by name
#
# @endif
def toString(nv, name=None):
  if not name:
    str_ = [""]
    return dump_to_stream(str_, nv)

  str_value = ""
  try:
    ret_value = find(nv, name)
    val = any.from_any(ret_value, keep_structs=True)
    if type(val) == str:
      str_value = val
  except:
    print OpenRTM_aist.Logger.print_exception()
    pass
  
  return str_value


##
# @if jp
#
# @brief 指定された文字列を NVList の要素に追加する。
#
# このオペレーションは name で指定された要素に value で指定された文字列を
# 追加する。
# name で指定した要素に既に value の値が設定されている場合には何もしない。
# name で指定した要素に value の値が設定されていない場合は、 ｢,｣区切りで
# value の値を追加する。
# 指定された値を設定する。
# name で指定した要素が存在しない場合は、 NVList の最後に新たな要素を追加し、
# 指定された値を設定する。
#
# @param nv 検索対象の NVList
# @param name 追加対象要素名
# @param value 追加する文字列
#
# @return 追加操作結果
#
# @else
#
# @endif
def appendStringValue(nv, name, value):
  index = find_index(nv, name)
  if index >= 0:
    tmp_str = nv[index].value.value()
    values = OpenRTM_aist.split(tmp_str,",")
    find_flag = False
    for val in values:
      if val == value:
        find_flag = True

    if not find_flag:
      tmp_str += ", "
      tmp_str += value
      nv[index].value = any.to_any(tmp_str)
  else:
    OpenRTM_aist.CORBA_SeqUtil.push_back(nv, newNV(name, value))

  return True


##
# @if jp
#
# @brief NVList に要素を追加する。
#
# このオペレーションは dest で指定された NVList に src で指定された要素を
# 追加する。
#
# @param dest 追加される NVList
# @param src 追加する NVList
#
# @else
#
# @endif
def append(dest, src):
  for i in range(len(src)):
    OpenRTM_aist.CORBA_SeqUtil.push_back(dest, src[i])


##
# @if jp
# @brief NVList に設定されている内容を文字列として出力する。
# @else
# @brief Print information configured in NVList as a string type
# @endif
# std::ostream& dump_to_stream(std::ostream& out, const SDOPackage::NVList& nv)
def dump_to_stream(out, nv):
  for i in range(len(nv)):
    val = any.from_any(nv[i].value, keep_structs=True)
    if type(val) == str:
	    out[0] += (nv[i].name + ": " + str(nv[i].value) + "\n")
    else:
	    out[0] += (nv[i].name + ": not a string value \n")

  return out[0]


##
# @if jp
#
# @brief NVList に設定されている内容を文字列として出力する。
#
# 指定された NVList に設定された内容を文字列として出力する。
# なお、設定されている要素が文字列型以外の場合には、その旨(文字列ではない)を
# 出力する。
#
# @param nv 出力対象 NVList
#
# @else
#
# @endif
def dump(nv):
  out = [""]
  print dump_to_stream(out, nv)
