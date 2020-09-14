#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file StringUtil.py
# @brief String operation utility
# @date $Date: $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2003-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.



import string

##
# @if jp
# @brief 文字列がエスケープされているか判断する
#
# 指定された文字がエスケープされているかどうかを判断する。
#
# @param _str エスケープされているかどうか判断する文字を含む文字列
# @param pos エスケープされているかどうか判断する文字の位置
#
# @return 指定した文字がエスケープされていれば true, それ以外は false
#
# @else
# @brief Whether the character is escaped or not
#
# This operation returns true if the specified character is escaped, and
# if the specified character is not escaped, it returns false
#
# @param str The string thath includes the character to be investigated.
# @param pos The position of the character to be investigated.
#
# @return true: the character is escaped, false: the character is not escaped.
#
# @endif
def isEscaped(_str, pos):
  pos -= 1

  i = 0
  while pos >= 0 and _str[pos] == "\\":
    i += 1
    pos -= 1

  return i % 2 == 1


##
# @if jp
# @class escape_functor
# @brief  文字列エスケープ処理用functor
# @else
#
# @endif
class escape_functor:
  def __init__(self):
    self._str = ""

  def __call__(self,c):
    if   c == '\t':
      self._str += "\\t"
    elif c == '\n':
      self._str += "\\n"
    elif c == '\f':
      self._str += "\\f"
    elif c == '\r':
      self._str += "\\r"
    elif c == '\\':
      self._str += "\\\\"
    else:
      self._str += c


##
# @if jp
# @class unescape_functor
# @brief  文字列アンエスケープ処理用functor
# @else
#
# @endif
class unescape_functor:
  def __init__(self):
    self.count = 0
    self._str = ""

  def __call__(self,c):
    if c == "\\":
      self.count += 1
      if not (self.count % 2):
        self._str += c
    else:
      if self.count > 0 and (self.count % 2):
        self.count = 0
        if c == 't':
          self._str+='\t'
        elif c == 'n':
          self._str+='\n'
        elif c == 'f':
          self._str+='\f'
        elif c == 'r':
          self._str+='\r'
        elif c == '\"':
          self._str+='\"'
        elif c == '\'':
          self._str+='\''
        else:
          self._str+=c
      else:
        self.count = 0
        self._str+=c


##
# @if jp
# @class unique_strvec
# @brief  重複文字削除処理用functor
# @else
#
# @endif
class unique_strvec:
  def __init__(self):
    self._str = []

  def __call__(self,s):
    if self._str.count(s) == 0:
      return self._str.append(s)


##
# @if jp
# @brief  インスタンス生成用functor
# @else
#
# @endif
def for_each(_str, instance):
  for i in _str:
    instance(i)

  return instance


##
# @if jp
# @brief 文字列をエスケープする
# 
# 次の文字をエスケープシーケンスに変換する。<br>
# HT -> "\t" <br>
# LF -> "\n" <br>
# CR -> "\r" <br>
# FF -> "\f" <br>
# シングルクオート、ダブルクオートについてはとくに処理はしない。
# 
# @else
# 
# @brief Escape string
# 
# The following characters are converted. <br>
# HT -> "\t" <br>
# LF -> "\n" <br>
# CR -> "\r" <br>
# FF -> "\f" <br>
# Single quote and dobule quote are not processed.
# 
# @endif
def escape(_str):
  return for_each(_str, escape_functor())._str


##
# @if jp
# @brief 文字列のエスケープを戻す
# 
# 次のエスケープシーケンスを文字に変換する。<br>
# "\t" -> HT <br>
# "\n" -> LF <br>
# "\r" -> CR <br>
# "\f" -> FF <br>
# "\"" -> "  <br>
# "\'" -> '  <br>
# 
# @else
# 
# @brief Unescape string
# 
# The following characters are converted. <br>
# "\t" -> HT <br>
# "\n" -> LF <br>
# "\r" -> CR <br>
# "\f" -> FF <br>
# "\'" -> '  <br>
# "\"" -> "  <br>
# @endif
def unescape(_str):
  return for_each(_str, unescape_functor())._str


##
# @if jp
# @brief 文字列の空白文字を削除する
#
# 与えられた文字列の空白文字を削除する。
# 空白文字として扱うのは' '(スペース)と'\\t'(タブ)。
#
# @param str(list) 空白文字削除処理文字列のリスト
#
# @else
# @brief Erase blank characters of string
#
# Erase blank characters that exist at the head of the given string.
# Space ' 'and tab '\\t' are supported as the blank character.
#
# @param str The target blank characters of string for the erase
#
# @endif
#
def eraseBlank(str):
    if len(str) == 0:
        return
    str[0] = str[0].strip(" ")
    l_str = str[0].split(" ")
    tmp_str = ""
    for s in l_str:
        if s:
            tmp_str+=s.strip(" ")

    tmp_str = tmp_str.strip('\t')
    l_str = tmp_str.split('\t')
    tmp_str = ""
    for s in l_str:
        if s:
            tmp_str+=s.strip('\t')

    str[0] = tmp_str


##
# @if jp
# @brief 文字列の先頭の空白文字を削除する
#
# 与えられた文字列の先頭に存在する空白文字を削除する。
# 空白文字として扱うのは' '(スペース)と'\\t'(タブ)。
#
# @param _str 先頭空白文字削除処理文字列
#
# @else
# @brief Erase the head blank characters of string
# @endif
def eraseHeadBlank(_str):
  _str[0] = _str[0].lstrip('\t ')


##
# @if jp
# @brief 文字列の末尾の空白文字を削除する
#
# 与えられた文字列の末尾に存在する空白文字を削除する。
# 空白文字として扱うのは' '(スペース)と'\\t'(タブ)。
#
# @param _str 末尾空白文字削除処理文字列
#
# @else
# @brief Erase the tail blank characters of string
# @endif
def eraseTailBlank(_str):
  #_str[0] = _str[0].rstrip('\t ')
  if _str[0] == "":
    return

  while (_str[0][-1] == " " or _str[0][-1] == '\t') and not isEscaped(_str[0], len(_str[0]) - 1):
    _str[0] = _str[0][:-1]


#
# @if jp
# @brief 文字列を正規化する
# @else
# @brief Erase the head/tail blank and replace upper case to lower case
# @endif
#
def normalize(_str):
  _str[0] = _str[0].strip().lower()
  return _str[0]


##
# @if jp
# @brief 文字列を置き換える
#
# 与えられた文字列に対して、指定した文字の置き換えを行う。
#
# @param str 置き換え処理対象文字列
# @param _from 置換元文字
# @param _to 置換先文字
#
# @else
# @brief Replace string
# @endif
def replaceString(str, _from, _to):
  str[0] = str[0].replace(_from, _to)


##
# @if jp
# @brief 文字列を分割文字で分割する
# 
# 設定された文字列を与えられたデリミタで分割する。
#
# @param input 分割対象文字列
# @param delimiter 分割文字列(デリミタ)
#
# @return 文字列分割結果リスト
#
# @else
# @brief Split string by delimiter
# @endif
def split(input, delimiter):
  if not input:
    return []

  del_result = input.split(delimiter)

  len_ = len(del_result)

  result = []
  for i in range(len_):
    if del_result[i] == "" or del_result[i] == " ":
      continue
      
    str_ = [del_result[i]]
    eraseHeadBlank(str_)
    eraseTailBlank(str_)
    result.append(str_[0])
    
  return result


##
# @if jp
# @brief 与えられた文字列をbool値に変換する
# 
# 指定された文字列を、true表現文字列、false表現文字列と比較し、その結果を
# bool値として返す。
# 比較の結果、true表現文字列、false表現文字列のどちらとも一致しない場合は、
# 与えられたデフォルト値を返す。
#
# @param _str 判断対象文字列
# @param yes true表現文字列
# @param no false表現文字列
# @param default_value デフォルト値(デフォルト値:None)
# @else
# @brief Convert given string to bool value
# @endif
def toBool(_str, yes, no, default_value=None):
  if default_value is None:
    default_value = True

  _str = _str.upper()
  yes  = yes.upper()
  no   = no.upper()

  if _str.find(yes) != -1:
    return True
  elif (_str.find(no)) != -1:
    return False
  else:
    return default_value

##
# @if jp
# @brief 文字列リスト中にある文字列が含まれるかどうか
# 
# 第1引数にカンマ区切りのリストを、第2引数に探索対象文字列を指定し、
# その文字列が第1引数の中に含まれるかを判断する。
#
# @param list 対象リスト
# @param value 探索文字列
# @return true: 含まれる、false: 含まれない
#
# @else
# @brief Include if a string is included in string list
# 
# if the second argument is included in the comma separated string
# list of the first argument, This operation returns "true value".
#
# @param list The target comma separated string
# @param value The searched string
# @return true: included, false: not included
#
# @endif
#
#  bool includes(const vstring& list, std::string value,
#                bool ignore_case = true);
def includes(_list, value, ignore_case = True):
  if not (type(_list) == list or type(_list) == str):
    return False

  if type(_list) == str:
    _list = _list.split(",")

  tmp_list = _list
  if ignore_case:
    value = value.lower()
    tmp_list = map((lambda x: x.lower()),_list)
    
  if tmp_list.count(value) > 0:
    return True

  return False
    


##
# @if jp
# @brief 与えられた文字列が絶対パスかどうかを判断する
#
# 与えられた文字列が絶対パス表現であるかどうかを判断する。
# 文字列が以下の場合には絶対パスとして判断する。
#  - 先頭文字が'/' (UNIXの場合)
#  - 先頭３文字がアルファベット＋'/'＋'\\' (Windowsの場合)
#  - 先頭２文字が'\\\\' (Windowsネットワークパスの場合)
#
# @param str 判定対象文字列
#
# @return 絶対パス判定結果
#
# @else
# @brief Investigate whether the given string is absolute path or not
# @endif
def isAbsolutePath(str):
  if str[0] == "/":
    return True
  if str[0].isalpha() and str[1] == ":" and str[2] == "\\":
    return True
  if str[0] == "\\" and str[1] == "\\":
    return True

  return False


##
# @if jp
# @brief 与えられた文字列がURLかどうかを判断する
#
# 与えられた文字列がURL表現かどうかを判断する。
# 与えられた文字列中に、'://'という文字列が含まれている場合には
# URL表現として判断する。
#
# @param str 判定対象文字列
#
# @return URL判定結果
#
# @else
# @brief Investigate whether the given string is URL or not
# @endif
def isURL(str):
  pos = 0
  if str == "":
    return False

  pos = str.find(":")
  if pos != 0 and pos != -1 and str[pos+1] == "/" and str[pos+2] == "/":
    return True

  return False


##
# @if jp
# @brief 与えられたオブジェクトを文字列に変換
#
# 引数で指定されたオブジェクトを文字列に変換する。
#
# @param n 変換対象オブジェクト
#
# @return 文字列変換結果
#
# @else
# @brief Convert the given object to st::string.
# @endif
def otos(n):
  if type(n) == int or type(n) == str or type(n) == long or type(n) == float:
    return str(n)



##
# @if jp
# @brief 与えられた文字列をリストに変換
#
# 引数で指定された文字列を｢,｣で分割し、リストに変換する。
#
# @param _type 変換結果リスト
# @param _str 変換元文字列
#
# @return リスト変換処理結果
#
# @else
# 
# @endif
def _stringToList(_type, _str):
  list_ = split(_str,",")
  len_ = len(list_)

  if len(_type[0]) < len(list_):
    sub = len(list_) - len(_type[0])
    for i in range(sub):
      _type[0].append(_type[0][0])
  elif len(_type[0]) > len(list_):
    sub = len(_type[0]) - len(list_)
    for i in range(sub):
      del _type[0][-1]

  for i in range(len_):
    str_ = [list_[i]]
    eraseHeadBlank(str_)
    eraseTailBlank(str_)
    list_[i] = str_[0]

  for i in range(len(list_)):
    if type(_type[0][i]) == int:
      _type[0][i] = int(list_[i])
    elif type(_type[0][i]) == long:
      _type[0][i] = long(list_[i])
    elif type(_type[0][i]) == float:
      _type[0][i] = float(list_[i])
    elif type(_type[0][i]) == str:
      _type[0][i] = str(list_[i])
    else:
      return False

  return True


##
# @if jp
# @brief 与えられた文字列をオブジェクトに変換
#
# 引数で与えられた文字列を指定されたオブジェクトに変換する。
#
# @param _type 変換先オブジェクト
# @param _str 変換元文字列
#
# @return 変換処理実行結果
#
# @else
# @brief Convert the given object to st::string.
# @endif
def stringTo(_type, _str):
  if not _str:
    return False

  if type(_type[0]) == int:
    _type[0] = int(_str)
    return True
  elif type(_type[0]) == long:
    _type[0] = long(_str)
    return True
  elif type(_type[0]) == float:
    _type[0] = float(_str)
    return True
  elif type(_type[0]) == list:
    return _stringToList(_type, _str)
  elif type(_type[0]) == str:
    _type[0] = str(_str)
    return True
  
  return False


##
# @if jp
# @brief 与えられた文字列リストから重複を削除
#
# 引数で与えられた文字列リストから重複を削除したリストを作成する。
#
# @param sv 確認元文字列リスト
#
# @return 重複削除処理結果リスト
#
# @else
#
# @endif
def unique_sv(sv):
  return for_each(sv, unique_strvec())._str


##
# @if jp
# @brief 与えられた文字列リストからCSVを生成
#
# 引数で与えられた文字列リストの各要素を並べたCSVを生成する。
# 文字列リストが空の場合には空白文字を返す。
#
# @param sv CSV変換対象文字列リスト
#
# @return CSV変換結果文字列
#
# @else
#
# @endif
def flatten(sv):
  if len(sv) == 0:
    return ""

  _str = ", ".join(sv)

  return _str


##
# @if jp
# @brief 与えられた文字列リストを引数リストに変換
#
# 引数で与えられた文字列リストの各要素末尾に'\\0'を加え、
# 引数リストに変換する。<br>
# ※本モジュールでは引数をそのまま返す
#
# @param args 変換対象文字列リスト
#
# @return 引数変換結果文字列
#
# @else
#
# @endif
def toArgv(args):
  return args
