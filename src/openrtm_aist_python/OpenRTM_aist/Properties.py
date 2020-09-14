#!/usr/bin/env python
# -*- coding: euc-jp -*-
  

##
# @file Properties.py
# @brief Property list class (derived from Java Properties)
# @date $Date: $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import sys
import string

import OpenRTM_aist


##
# @if jp
#
# @class Properties
# @brief プロパティセットを表現するクラス
#
# Properties クラスは、不変のプロパティセットを表す。 Properties をストリーム
# に保管したり、ストリームからロードしたりすることができる。
# プロパティリストの各キー、およびそれに対応する値は文字列となっている。
#
# プロパティリストには、その「デフォルト値」として別のプロパティリストを持つ
# ことができる。元のプロパティリストでプロパティキーが見つからないと、この
# 2番目のプロパティリストが検索される。 
#
# プロパティの取得には getProperty() 、プロパティのセットには setProperty() と
# いったメソッドを使用することが推奨される。
#
# プロパティをストリームに保存するとき、またはストリームからロードするとき
# に、ISO 8859-1 文字エンコーディングが使用される。このエンコーディングに
# 直接表示できない文字は、扱うことができない。
#
# このクラスは、Java の Properties クラス (java.util.Properties) とほぼ同様の
# メソッドを持つ。また、入出力されるファイルは Java の Properties クラスが
# 出力するものと互換性があるが、Unicode を含むものは扱うことができない。
#
# @since 0.4.0
#
# @else
#
# @class Properties
#
# The Properties class represents a persistent set of properties. The
# Properties can be saved to a stream or loaded from a stream. Each key and
# its corresponding value in the property list is a string. 
#
# A property list can contain another property list as its "defaults"; this
# second property list is searched if the property key is not found in the
# original property list. 
#
# Because Properties inherits from Hashtable, the put and putAll methods can
# be applied to a Properties object. Their use is strongly discouraged as 
# they allow the caller to insert entries whose keys or values are not 
# Strings. The setProperty method should be used instead. If the store or 
# save method is called on a "compromised" Properties object that contains a 
# non-String key or value, the call will fail. 
#
# The load and store methods load and store properties in a simple
# line-oriented format specified below. This format uses the ISO 8859-1
# character encoding. Characters that cannot be directly represented in this
# encoding can be written using Unicode escapes ; only a single 'u' character
# is allowed in an escape sequence. The native2ascii tool can be used to
# convert property files to and from other character encodings. 
#
# This class has almost same methods of Java's Properties class. Input and 
# Output stream of this properties are compatible each other except Unicode
# encoded property file.
#
# @endif
class Properties:
  """
  """

  ##
  # @if jp
  #
  # @brief コンストラクタ
  #
  # 以下の順に引数をチェックし、インスタンスの生成を行う。
  #
  # 引数 prop に値が設定されている場合、
  # 引数に与えられた Properties のキー、値およびデフォルト値が
  # 全てそのままコピーされる。
  #
  # 引数 key に値が設定されている場合、
  # key と value のみを与えて Property のルートノードを作成する。
  # 値は全てデフォルト値として設定される。
  #
  # 引数 defaults_map に値が設定されている場合、
  # defaults_map に設定された内容をデフォルト値にもつ Properties を作成する。
  # 値は全てデフォルト値として設定される。
  # 
  # 引数 defaults_str に値が設定されている場合、
  # 指定されたデフォルト値を持つ空のプロパティリストを作成する。
  # 値は全てデフォルト値として設定される。
  # デフォルト値は char* の配列により与えられ、key と value の対になって
  # おり、リストの終端は配列の数を表す引数 num か、空文字の key で与えらられ
  # なければならない。
  # 以下に例を示す。
  #
  # <pre>
  # const char* defaults = {
  #     "key1", "value1",
  #     "key2", "value2",
  #     "key3", "value3",
  #     "key4", "value4",
  #     "key5", "value5",
  #     "" };
  # Properties p(defaults);
  # // もしくは
  # Properties p(defaults, 10);
  # </pre>
  # 
  # @param self
  # @param key プロパティのキー(デフォルト値:None)
  # @param value プロパティの値(デフォルト値:None)
  # @param defaults_map デフォルト値として指定されるmap(デフォルト値:None)
  # @param defaults_str デフォルト値を指定する配列(デフォルト値:None)
  # @param num デフォルト値を設定する要素数(デフォルト値:None)
  # @param prop デフォルト値として指定されるproperty(デフォルト値:None)
  # 
  # @else
  #
  # @brief Constructor
  #
  # All of given Properties's keys, values and default values are copied to
  # new Properties.
  #
  # Creates a root node of Property with root's key and value.
  #
  # Creates an Properties with default value of std::string map.
  #
  # Creates an empty property list with the specified defaults.
  # The default values are given by array of char*, which should be pairs
  # of "key" and "value". The end of list is specified by argument "num",
  # which specifies number of array or null character of key.
  # The following is an example.
  #
  # const char* defaults = {
  #     "key1", "value1",
  #     "key2", "value2",
  #     "key3", "value3",
  #     "key4", "value4",
  #     "key5", "value5",
  #     "" };
  # Properties p(defaults);
  # // or
  # Properties p(defaults, 10);
  #
  # @endif
  def __init__(self, key=None, value=None, defaults_map=None, defaults_str=None, num=None, prop=None):
    self.default_value = ""
    self.root = None
    self.empty = ""
    self.leaf = []

    # Properties::Properties(const Properties& prop)
    if prop:
      self.name          = prop.name
      self.value         = prop.value
      self.default_value = prop.default_value

      keys = prop.propertyNames()
      for _key in keys:
        node = None
        node = prop.getNode(_key)
        if node:
          self.setDefault(_key, node.default_value)
          self.setProperty(_key, node.value)
          
      return

    # Properties::Properties(const char* key, const char* value)
    if key:
      self.name = key
      if value is None:
        self.value = ""
      else:
        self.value = value
      return

    self.name  = ""
    self.value = ""

    # Properties::Properties(std::map<std::string, std::string>& defaults)
    if defaults_map:
      #for i in range(len(defaults_map.items())):
      #  self.setDefault(defaults_map.keys()[i], defaults_map.values()[i])
      for key, value in defaults_map.items():
        self.setDefault(key, value)
      return

    if defaults_str:
      if num is None:
        _num = sys.maxint
      else:
        _num = num
      self.setDefaults(defaults_str, _num)
      return


  ##
  # @if jp
  # @brief 代入演算子
  # 
  # 左辺値の Properties のキー、値およびデフォルト値は全て削除され、
  # 右辺値の Properties のキー、値およびデフォルト値が全てそのまま
  # コピーされる。
  # 
  # @param self
  # @param prop OpenRTM_aist.Properties
  # 
  # @else
  # @brief Assignment operator
  # @param self
  # @param prop OpenRTM_aist.Properties
  # @endif
  def assigmentOperator(self, prop):
    self.clear()
    self.name = prop.name
    self.value = prop.value
    self.default_value = prop.default_value

    keys = prop.propertyNames()

    for key in keys:
      node = None
      node = prop.getNode(key)
      if node:
        self.setDefault(key, node.default_value)
        self.setProperty(key, node.value)

    return self


  ##
  # @if jp
  #
  # @brief デストラクタ
  #
  # @param self
  #
  # @else
  #
  # @brief Destructor
  #
  # @endif
  def __del__(self):
    self.clear()
    if self.root:
      self.root.removeNode(self.name)
    return

  #============================================================
  # public functions
  #============================================================


  ##
  # @if jp
  # @brief Name の取得
  #
  # プロパティの名称を取得する。
  #
  # @param self
  #
  # @return プロパティ名
  #
  # @else
  #
  # @endif
  def getName(self):
    return self.name


  ##
  # @if jp
  # @brief 値の取得
  #
  # プロパティの値を取得する。
  #
  # @param self
  #
  # @return プロパティ値
  #
  # @else
  #
  # @endif
  def getValue(self):
    return self.value


  ##
  # @if jp
  # @brief デフォルト値の取得
  #
  # プロパティのデフォルト値を取得する。
  #
  # @param self
  #
  # @return プロパティデフォルト値
  #
  # @else
  #
  # @endif
  def getDefaultValue(self):
    return self.default_value


  ##
  # @if jp
  # @brief 子要素の取得
  #
  # プロパティの子要素を取得する。
  #
  # @param self
  #
  # @return 子要素
  #
  # @else
  #
  # @endif
  def getLeaf(self):
    return self.leaf


  ##
  # @if jp
  # @brief ルート要素の取得
  #
  # プロパティのルート要素を取得する。
  #
  # @param self
  #
  # @return ルート要素
  #
  # @else
  #
  # @endif
  def getRoot(self):
    return self.root


  ##
  # @if jp
  #
  # @brief 指定されたキーを持つプロパティを、プロパティリストから探す
  #
  # 指定されたキーを持つプロパティを、プロパティリストから探す。
  # そのキーがプロパティリストにない場合は、デフォルト値の引数が返される。 
  #
  # @param self
  # @param key プロパティキー
  # @param default デフォルト値(デフォルト値:None)
  #
  # @return 指定されたキー値を持つこのプロパティリストの値
  #
  # @else
  #
  # @brief Searches for the property with the specified key in this property
  #
  # Searches for the property with the specified key in this property list.
  # The method returns the default value argument if the property is not 
  # found.
  #
  # @param key the property key
  # @param defaultValue a default value. 
  #
  # @return the value in this property list with the specified key value.
  #
  # @endif
  def getProperty(self, key, default=None):
    if default is None:
      keys = []
      #keys = string.split(key, ".")
      self.split(key, ".", keys)

      node = None
      node = self._getNode(keys, 0, self)
      if node:
        if node.value:
          return node.value
        else:
          return node.default_value
      return self.empty

    else:
      value = self.getProperty(key)
      if value:
        return value
      else:
        return default


  ##
  # @if jp
  # @brief 指定されたキーに対してデフォルト値を取得する
  #
  # 指定されたキーを持つプロパティのデフォルト値を返す。
  # 指定されたキーを持つプロパティが存在しない場合には空文字を返す。
  #
  # @param self
  # @param key プロパティキー
  #
  # @return 指定されたキー値を持つプロパティのデフォルト値
  #
  # @else
  # @brief Set value as the default value to specified key's property
  # @endif
  def getDefault(self, key):
    keys = []
    #keys = string.split(key, ".")
    self.split(key, ".", keys)
    node = None
    node = self._getNode(keys, 0, self)
    if node:
      return node.default_value

    return self.empty


  ##
  # @if jp
  #
  # @brief Properties に value を key について登録する
  #
  # Properties に value を key について登録する。
  # すでに key に対する値を持っている場合、戻り値に古い値を返す。
  #
  # @param self
  # @param key プロパティリストに配置されるキー
  # @param value key に対応する値(デフォルト値:None)
  #
  # @return プロパティリストの指定されたキーの前の値。それがない場合は null
  #
  # @else
  #
  # @brief Sets a value associated with key in the property list
  #
  # This method sets the "value" associated with "key" in the property list.
  # If the property list has a value of "key", old value is returned.
  #
  # @param key the key to be placed into this property list.
  # @param value the value corresponding to key. 
  #
  # @return the previous value of the specified key in this property list,
  #         or null if it did not have one.
  #
  #@endif
  def setProperty(self, key, value=None):
    if value is not None:
      keys = []
      #keys = string.split(key, ".")
      self.split(key, ".", keys)
      curr = self
      for _key in keys:
        next = curr.hasKey(_key)
        if next is None:
          next = OpenRTM_aist.Properties(key=_key)
          next.root = curr
          curr.leaf.append(next)
        curr = next
      retval = curr.value
      curr.value = value
      return retval

    else:
      self.setProperty(key, self.getProperty(key))
      prop = self.getNode(key)
      return prop.value


  ##
  # @if jp
  # @brief デフォルト値を登録する
  #
  # key で指定される要素にデフォルト値を登録する。
  #
  # @param self
  # @param key デフォルト値を登録するプロパティのキー
  # @param value 登録されるデフォルト値
  #
  # @return 指定されたデフォルト値
  #
  # @else
  # @brief Sets a default value associated with key in the property list
  # @endif
  def setDefault(self, key, value):
    keys = []
    self.split(key, ".", keys)
    #keys = string.split(key, ".")

    curr = self
    for _key in keys:
      next = curr.hasKey(_key)
      if next is None:
        next = OpenRTM_aist.Properties(key=_key)
        next.root = curr
        curr.leaf.append(next)
      curr = next
    if value != "" and value[-1] == "\n":
      value = value[0:len(value)-1]
    curr.default_value = value
    return value


  ##
  # @if jp
  # @brief Properties にデフォルト値をまとめて登録する
  #
  # 配列で指定された要素にデフォルト値をまとめて登録する。
  # デフォルト値は char* の配列により与えられ、key と value の対になって
  # おり、リストの終端は配列の数を表す引数 num か、空文字の key で与えらられ
  # なければならない。
  # 
  # @param self
  # @param defaults デフォルト値を指定する配列
  # @param num デフォルト値を設定する要素数(デフォルト値:None)
  # 
  # @else
  # @brief Sets a default value associated with key in the property list
  # @endif
  def setDefaults(self, defaults, num = None):
    if num is None:
      num = sys.maxint

    i = 0
    len_ = len(defaults)
    while 1:
      if i > num or i > (len_ - 1) or defaults[i] == "":
        break

      key = [defaults[i]]
      value = [defaults[i+1]]

      OpenRTM_aist.eraseHeadBlank(key)
      OpenRTM_aist.eraseTailBlank(key)

      OpenRTM_aist.eraseHeadBlank(value)
      OpenRTM_aist.eraseTailBlank(value)

      self.setDefault(key[0], value[0])

      i +=2



  #============================================================
  # load and save functions
  #============================================================

  ##
  # @if jp
  #
  # @brief 指定された出力ストリームに、プロパティリストを出力する
  #
  # 指定された出力ストリームに、プロパティリストを出力する。
  # このメソッドは主にデバッグに用いられる。
  #
  # @param self
  # @param out 出力ストリーム
  #
  # @else
  #
  # @brief Prints this property list out to the specified output stream
  #
  # Prints this property list out to the specified output stream.
  # This method is useful for debugging.
  #
  # @param out an output stream.
  #
  # @endif
  def list(self, out):
    self._store(out, "", self)
    return


  ##
  # @if jp
  #
  # @brief 入力ストリームからキーと要素が対になったプロパティリストを読み込む
  #
  # 入力ストリームからキーと要素が対になったプロパティリストを読み込む。
  # ストリームは、ISO 8859-1 文字エンコーディングを使用しているとみなされる。
  # 各プロパティは、入力ストリームに行単位で登録されているものとみなされ、
  # 各行は行区切り文字 (\\n、\\r、または \\r\\n) で終わる。
  # 入力ストリームから読み込んだ行は、入力ストリームでファイルの終わりに
  # 達するまで処理される。
  #
  # 空白文字だけの行、または最初の非空白文字が ASCII 文字 # または ! である
  # 行は無視される。つまり、# または ! はコメント行を示す。
  #
  # 空白行またはコメント行以外のすべての行は、テーブルに追加されるプロパティ
  # を記述する。ただし、行の終わりが \ の場合は、次の行があれば継続行として
  # 扱われる (下記を参照)。 キーは、最初の非空白文字から、最初の ASCII 文字
  # =、:、または空白文字の直前までの、行内のすべての文字から構成される。
  #
  # キーの終わりを示す文字は、前に \ を付けることによりキーに含めることも
  # できる。キーの後ろの空白はすべてスキップされる。
  # キーの後ろの最初の非空白文字が = または : である場合は、これらのキーは
  # 無視され、そのあとの空白文字もすべてスキップされる。
  # 行内のそれ以外の文字はすべて、関連した要素文字列の一部となる。
  # 要素文字列内では、ASCII エスケープシーケンス \\t、\\n、\\r、\\\\、\\"、
  # \\'、\\ (円記号とスペース)、および \\uxxxx は認識され、単独の文字に変換
  # される。
  # また、行の最後の文字が \ である場合は、次の行は現在の行の継続として
  # 扱われる。その場合、\ と行区切り文字が破棄され、継続行の先頭に空白が
  # あればそれもすべて破棄され、要素文字列の一部にはならない。 
  #
  # たとえば、次の 4 行はそれぞれキー Truth と関連した要素値 Beauty を表す。
  # 
  # Truth = Beauty <BR>
  # Truth:Beauty <BR>
  # Truth\\t\\t\\t:Beauty <BR>
  #
  # また、次の 3 行は 1 つのプロパティを表す。 
  #
  # fruits\\t\\t\\t\\tapple, banana, pear, \ <BR>
  #                                  cantaloupe, watermelon, \ <BR>
  #                                  kiwi, mango <BR>
  # キーは fruits で、次の要素に関連付けれられる。 
  # "apple, banana, pear, cantaloupe, watermelon, kiwi, mango"
  # 最終的な結果でコンマのあとに必ずスペースが表示されるように、
  # 各 \ の前にスペースがある。行の終わりを示す \ と、継続行の先頭にある
  # 空白は破棄され、他の文字に置換されない。 
  # また、次の 3 番目の例では、キーが cheeses で、関連した要素が空の文字列
  # であることを表す。 
  #
  # cheeses <BR>
  # キーは、cheeses で、関連要素は空の文字列であることを指定している。 
  #
  # @param self
  # @param inStream 入力ストリーム 
  #
  # @else
  #
  # @brief Loads property list consists of key:value from input stream
  #
  # Reads a property list (key and element pairs) from the input stream.
  # The stream is assumed to be using the ISO 8859-1 character encoding; that
  # is each byte is one Latin1 character. Characters not in Latin1, and
  # certain special characters, can be represented in keys and elements using
  # escape sequences similar to those used for character and string literals
  # The differences from the character escape sequences used for characters
  # and strings are: 
  # - Octal escapes are not recognized. 
  # - The character sequence \b does not represent a backspace character. 
  # - The method does not treat a backslash character, \, before a non-valid
  #   escape character as an error; the backslash is silently dropped. For
  #   example, in a Java string the sequence "\z" would cause a compile time
  #   error. In contrast, this method silently drops the backslash. 
  #   Therefore, this method treats the two character sequence "\b" as 
  #   equivalent to the single character 'b'. 
  # - Escapes are not necessary for single and double quotes; however, by the
  #   rule above, single and double quote characters preceded by a backslash
  #   still yield single and double quote characters, respectively. 
  # An IllegalArgumentException is thrown if a malformed Unicode escape
  # appears in the input. 
  #
  # This method processes input in terms of lines. A natural line of input is
  # terminated either by a set of line terminator characters
  # (\n or \r or \r\n) or by the end of the file. A natural line may be 
  # either a blank line, a comment line, or hold some part of a key-element 
  # pair. The logical line holding all the data for a key-element pair may 
  # be spread out across several adjacent natural lines by escaping the line 
  # terminator sequence with a backslash character, \. Note that a comment 
  # line cannot be extended in this manner; every natural line that is a 
  # comment must have its own comment indicator, as described below. If a 
  # logical line is continued over several natural lines, the continuation 
  # lines receive further processing, also described below. Lines are read 
  # from the input stream until end of file is reached. 
  #
  # A natural line that contains only white space characters is considered
  # blank and is ignored. A comment line has an ASCII '#' or '!' as its first
  # non-white space character; comment lines are also ignored and do not
  # encode key-element information. In addition to line terminators, this
  # method considers the characters space (' ', '\u0020'), tab 
  # ('\t', '\u0009'), and form feed ('\f', '\u000C') to be white space. 
  #
  # If a logical line is spread across several natural lines, the backslash
  # escaping the line terminator sequence, the line terminator sequence, and
  # any white space at the start the following line have no affect on the key
  # or element values. The remainder of the discussion of key and element
  # parsing will assume all the characters constituting the key and element
  # appear on a single natural line after line continuation characters have
  # been removed. Note that it is not sufficient to only examine the 
  # character preceding a line terminator sequence to see if the line 
  # terminator is escaped; there must be an odd number of contiguous 
  # backslashes for the line terminator to be escaped. Since the input is 
  # processed from left to right, a non-zero even number of 2n contiguous 
  # backslashes before a line terminator (or elsewhere) encodes n 
  # backslashes after escape processing. 
  #
  # The key contains all of the characters in the line starting with the 
  # first non-white space character and up to, but not including, the first
  # unescaped '=', ':', or white space character other than a line 
  # terminator. All of these key termination characters may be included in 
  # the key by escaping them with a preceding backslash character; 
  # for example,
  #
  # \:\=
  #
  # would be the two-character key ":=". Line terminator characters can be
  # included using \r and \n escape sequences. Any white space after the key
  # is skipped; if the first non-white space character after the key is '=' 
  # or ':', then it is ignored and any white space characters after it are 
  # also skipped. All remaining characters on the line become part of the
  # associated element string; if there are no remaining characters, the
  # element is the empty string "". Once the raw character sequences
  # constituting the key and element are identified, escape processing is
  # performed as described above. 
  #
  # As an example, each of the following three lines specifies the key 
  # "Truth" and the associated element value "Beauty": 
  #
  # Truth = Beauty <BR>
  #        Truth:Beauty <BR>
  # Truth                  :Beauty <BR>
  #  As another example, the following three lines specify a single 
  # property: 
  #
  # fruits                           apple, banana, pear, \ <BR>
  #                                  cantaloupe, watermelon, \ <BR>
  #                                  kiwi, mango <BR>
  # The key is "fruits" and the associated element is: 
  # "apple, banana, pear, cantaloupe, watermelon, kiwi, mango"Note that a
  # space appears before each \ so that a space will appear after each comma
  # in the final result; the \, line terminator, and leading white space on
  # the continuation line are merely discarded and are not replaced by one or
  # more other characters. 
  # As a third example, the line: 
  #
  # cheeses <BR>
  # specifies that the key is "cheeses" and the associated element is the
  # empty string "".
  #
  # @param inStream the input stream.
  #
  # @endif
  def load(self, inStream):
    pline = ""
    for readStr in inStream:
      if not readStr:
        continue
      
      tmp = [readStr]
      OpenRTM_aist.eraseHeadBlank(tmp)
      _str = tmp[0]
      
      if _str[0] == "#" or _str[0] == "!" or _str[0] == "\n":
        continue

      _str = _str.rstrip('\r\n')

      if _str[len(_str)-1] == "\\" and not OpenRTM_aist.isEscaped(_str, len(_str)-1):
        #_str = _str[0:len(_str)-1]
        tmp = [_str[0:len(_str)-1]]
        OpenRTM_aist.eraseTailBlank(tmp)
        #pline += _str
        pline += tmp[0]
        continue
      pline += _str
      if pline == "":
        continue

      key = []
      value = []
      self.splitKeyValue(pline, key, value)
      key[0] = OpenRTM_aist.unescape(key)
      OpenRTM_aist.eraseHeadBlank(key)
      OpenRTM_aist.eraseTailBlank(key)

      value[0] = OpenRTM_aist.unescape(value)
      OpenRTM_aist.eraseHeadBlank(value)
      OpenRTM_aist.eraseTailBlank(value)

      self.setProperty(key[0], value[0])
      pline = ""


  ##
  # @if jp
  #
  # @brief プロパティリストを指定されたストリームに保存する
  #
  # プロパティリストを指定されたストリームに保存する。
  # このメソッドは Java Properties との互換性のために定義されている。
  # (内部的には store メソッドを利用している。)
  #
  # @param self
  # @param out 出力ストリーム
  # @param header プロパティリストの記述 
  #
  # @else
  #
  # @brief Save the properties list to the stream
  #
  # Deprecated. 
  #
  # @param out The output stream
  # @param header A description of the property list
  #
  # @endif
  def save(self, out, header):
    self.store(out, header)
    return


  ##
  # @if jp
  #
  # @brief プロパティリストを出力ストリームへ保存する
  #
  # Properties テーブル内のプロパティリスト (キーと要素のペア) を、load
  # メソッドを使って Properties テーブルにロードするのに適切なフォーマットで
  # 出力ストリームに書き込む。 
  #
  # Properties テーブル内のプロパティリスト (キーと要素のペア) を、load
  # メソッドを使って Properties テーブルにロードするのに適切なフォーマットで
  # 出力ストリームに書き込む。ストリームは、ISO 8859-1 文字
  # エンコーディングを使用して書き込まれる。 
  # Properties テーブル (存在する場合) のデフォルトテーブルからの
  # プロパティは、このメソッドによっては書き込まれない。 
  #
  # header 引数が null でない場合は、ASCII 文字の #、header の文字列、
  # および行区切り文字が最初に出力ストリームに書き込まれます。このため、
  # header は識別コメントとして使うことができる。 
  #
  # 次に、ASCII 文字の #、現在の日時 (Date の toString メソッドによって
  # 現在時刻が生成されるのと同様)、および Writer によって生成される行区切り
  # からなるコメント行が書き込まれる。 
  #
  # 続いて、 Properties テーブル内のすべてのエントリが 1 行ずつ書き出される。
  # 各エントリのキー文字列、ASCII 文字の=、関連した要素文字列が書き込まれる。
  # 要素文字列の各文字は、エスケープシーケンスとして描画する必要があるか
  # どうか確認される。ASCII 文字の \、タブ、改行、および復帰はそれぞれ \\\\、
  # \\t、\\n、および \\r として書き込まれる。\\u0020 より小さい文字および
  # \\u007E より大きい文字は、対応する 16 進値 xxxx を使って \\uxxxx として
  # 書き込まれる。埋め込み空白文字でも後書き空白文字でもない先行空白文字は、
  # 前に \ を付けて書き込まれる。キーと値の文字 #、!、=、および : は、
  # 必ず正しくロードされるように、前にスラッシュを付けて書き込まれる。 
  #
  # エントリが書き込まれたあとで、出力ストリームがフラッシュされる。
  # 出力ストリームはこのメソッドから復帰したあとも開いたままとなる。 
  #
  # @param self
  # @param out 出力ストリーム
  # @param header プロパティリストの記述 
  #
  # @else
  #
  # @brief Stores property list to the output stream
  #
  # Writes this property list (key and element pairs) in this Properties 
  # table to the output stream in a format suitable for loading into a 
  # Properties table using the load method. The stream is written using the 
  # ISO 8859-1 character encoding. 
  #
  # Properties from the defaults table of this Properties table (if any) are
  # not written out by this method. 
  #
  # If the comments argument is not null, then an ASCII # character, the
  # comments string, and a line separator are first written to the output
  # stream. Thus, the comments can serve as an identifying comment. 
  #
  # Next, a comment line is always written, consisting of an ASCII #
  # character, the current date and time (as if produced by the toString
  # method of Date for the current time), and a line separator as generated
  # by the Writer. 
  #
  # Then every entry in this Properties table is written out, one per line.
  # For each entry the key string is written, then an ASCII =, then the
  # associated element string. Each character of the key and element strings
  # is examined to see whether it should be rendered as an escape sequence.
  # The ASCII characters \, tab, form feed, newline, and carriage return are
  # written as \\, \t, \f \n, and \r, respectively. Characters less than
  # \u0020 and characters greater than \u007E are written as \uxxxx for the
  # appropriate hexadecimal value xxxx. For the key, all space characters are
  # written with a preceding \ character. For the element, leading space
  # characters, but not embedded or trailing space characters, are written
  # with a preceding \ character. The key and element characters #, !, =, and
  # : are written with a preceding backslash to ensure that they are properly
  # loaded. 
  #
  # After the entries have been written, the output stream is flushed. The
  # output stream remains open after this method returns. 
  #
  # @param out an output stream.
  # @param header a description of the property list.
  #
  # @endif
  def store(self, out, header):
    out.write("#"+header+"\n")
    self._store(out, "", self)


  #============================================================
  # other util functions
  #============================================================

  ##
  # @if jp
  #
  # @brief プロパティのキーのリストを vector で返す
  #
  # メインプロパティリストに同じ名前のキーが見つからない場合は、デフォルトの
  # プロパティリストにある個別のキーを含む、このプロパティリストにあるすべて
  # のキーのリストを返す。 
  #
  # @param self
  #
  # @return プロパティリストにあるすべてのキーのリスト。
  #         デフォルトのプロパティリストにあるキーを含む
  #
  # @else
  #
  # @brief Returns an vector of all the keys in this property
  #
  # Returns an enumeration of all the keys in this property list, including
  # distinct keys in the default property list if a key of the same name has
  # not already been found from the main properties list.
  #
  # @return an vector of all the keys in this property list, including the
  #         keys in the default property list.
  #
  # @endif
  def propertyNames(self):
    names = []
    for leaf in self.leaf:
      self._propertyNames(names, leaf.name, leaf)
    return names


  ##
  # @if jp
  # @brief プロパティの数を取得する
  #
  # 設定済みのプロパティ数を取得する。
  #
  # @param self
  #
  # @return プロパティ数
  #
  # @else
  # @brief Get number of Properties
  # @endif
  def size(self):
    return len(self.propertyNames())


  ##
  # @if jp
  # @brief ノードを検索する
  # @else
  # @brief Find node of properties
  # @endif
  # Properties* const Properties::findNode(const std::string& key) const
  def findNode(self, key):
    if not key:
      return None

    keys = []
    self.split(key, '.', keys)
    return self._getNode(keys, 0, self)


  ##
  # @if jp
  # @brief ノードを取得する
  #
  # 指定したキーを持つノードを取得する。
  #
  # @param self
  # @param key 取得対象ノードのキー
  #
  # @return 対象ノード
  #
  # @else
  # @brief Get node of Properties
  # @endif
  def getNode(self, key):
    if not key:
      return self

    leaf = self.findNode(key)
    if leaf:
      return leaf

    self.createNode(key)
    return self.findNode(key)


  ##
  # @if jp
  # @brief 新規ノードを生成する
  #
  # 指定したキーを持つ新規ノードを生成する。
  # 既に同一キーを持つノードが登録済みの場合にはエラーを返す。
  #
  # @param self
  # @param key 新規ノードのキー
  #
  # @return 新規ノード生成結果
  #         指定したキーを持つノードが既に存在する場合にはfalse
  #
  # @else
  #
  # @endif
  def createNode(self, key):
    if not key:
      return False

    if self.findNode(key):
      return False
    
    self.setProperty(key,"")
    return True


  ##
  # @if jp
  # @brief ノードを削除する
  #
  # 指定した名称を持つプロパティを削除する。
  # 削除したプロパティを返す。
  #
  # @param self
  # @param leaf_name 削除対象プロパティ名称
  #
  # @return 削除したプロパティ
  #
  # @else
  # @brief Get node of Properties
  # @endif
  def removeNode(self, leaf_name):
    len_ = len(self.leaf)
    for i in range(len_):
      idx = (len_ - 1) - i
      if self.leaf[idx].name == leaf_name:
        prop = self.leaf[idx]
        del self.leaf[idx]
        return prop
    return None


  ##
  # @if jp
  # @brief 子ノードにkeyがあるかどうか
  #
  # 指定したキーを持つ子ノードが存在するかどうか確認する。
  # 存在する場合、子ノードを返す。
  #
  # @param self
  # @param key 確認対象のキー
  #
  # @return 子ノード
  #
  # @else
  # @brief If key exists in the children
  # @endif
  def hasKey(self, key):
    for leaf in self.leaf:
      if leaf.name == key:
        return leaf

    return None


  ##
  # @if jp
  # @brief 子ノードを全て削除する
  #
  # @param self
  #
  # @else
  # @brief If key exists in the children
  # @endif
  def clear(self):
    len_ = len(self.leaf)
    for i in range(len_):
      if self.leaf[-1]:
        del self.leaf[-1]

    return


  ##
  # @if jp
  # @brief Propertyをマージする
  #
  # 現在のプロパティに設定したプロパティをマージする。
  #
  # @param self
  # @param prop マージするプロパティ
  #
  # @return プロパティマージ結果
  #
  # @else
  # @brief Merge properties
  # @endif
  def mergeProperties(self, prop):
    keys = prop.propertyNames()

    for i in range(prop.size()):
      self.setProperty(keys[i], prop.getProperty(keys[i]))

    return self


  ##
  # @if jp
  # @brief 文字列をキーと値のペアに分割する
  #
  # 与えられた文字列を、設定されたデリミタでキーと値のペアに分割する。
  # まず最初に与えられた文字列に':'もしくは'='が含まれるかを検索し、
  # どちらかの文字が含まれている場合にはそれをデリミタとして使用する。
  # 両方とも含まれていない場合には、' '(スペース)を用いて分割を試みる。
  # 全てのデリミタ候補が含まれていない場合には、与えられた文字列をキーとして
  # 設定し、値に空の文字列を設定する。
  # どのデリミタ候補についてもエスケープされている(直前に'\'が設定されている)
  # 場合には、デリミタとして使用しない。
  #
  # @param self
  # @param _str 分割対象文字列
  # @param key 分割結果キー
  # @param value 分割結果値
  #
  # @else
  #
  # @endif
  def splitKeyValue(self, _str, key, value):
    i = 0
    length = len(_str)

    while i < length:
      if (_str[i] == ":" or _str[i] == "=") and not OpenRTM_aist.isEscaped(_str, i):
        key.append(_str[0:i])
        value.append(_str[i+1:])
        return
      i += 1

    # If no ':' or '=' exist, ' ' would be delimiter.
    i = 0
    while i < length:
      if (_str[i] == " ") and not OpenRTM_aist.isEscaped(_str, i):
        key.append(_str[0:i])
        value.append(_str[i+1:])
        return
      i += 1

    key.append(_str)
    value.append("")
    return


  ##
  # @if jp
  # @brief 文字列を分割する
  #
  # 与えられた文字列を、与えられたデリミタで分割する。
  # 与えられた文字列が空の場合は、エラーを返す。
  # 与えられたデリミタがエスケープされている(直前に'\'が設定されている)場合
  # には、デリミタとして使用しない。
  #
  # @param self
  # @param _str 分割対象文字列
  # @param delim デリミタ
  # @param value 分割結果値リスト
  #
  # @return 分割処理結果
  #
  # @else
  #
  # @endif
  def split(self, _str, delim, value):
    if _str == "":
      return False

    begin_it = end_it = 0

    length = len(_str)

    while end_it < length:
      if _str[end_it] == delim and not OpenRTM_aist.isEscaped(_str, end_it):
        value.append(_str[begin_it:end_it])
        begin_it = end_it + 1
      end_it += 1

    value.append(_str[begin_it:end_it])
    return True


  ##
  # @if jp
  # @brief プロパティを取得する
  #
  # キーリストで指定されたプロパティを取得する。
  # キーリストでは、指定するキーのプロパティでの階層関係をリスト形式で表現
  # する。
  # 指定したキーリストに該当するプロパティが存在しない場合はNoneを返す。
  #
  # @param self
  # @param keys 取得対象プロパティのキーのリスト表現
  # @param index キーリストの階層数
  # @param curr 検索対象プロパティ
  #
  # @return 検索対象プロパティ
  #
  # @else
  #
  # @endif
  def _getNode(self, keys, index, curr):
    next = curr.hasKey(keys[index])
    if next is None:
      return None

    if index < (len(keys) - 1):
      index+=1
      return next._getNode(keys, index, next)
    else:
      return next

    return None


  ##
  # @if jp
  # @brief プロパティの名称リストを取得する
  #
  # プロパティの名称を'.'区切りで表現したリストを取得する。
  #
  # @param self
  # @param names プロパティの名称リスト
  # @param curr_name 現在のプロパティ名
  # @param curr 対象プロパティ
  #
  # @else
  #
  # @endif
  def _propertyNames(self, names, curr_name, curr):
    if len(curr.leaf) > 0:
      for i in range(len(curr.leaf)):
        next_name = curr_name+"."+curr.leaf[i].name
        self._propertyNames(names, next_name, curr.leaf[i])
    else:
      names.append(curr_name)

    return


  ##
  # @if jp
  # @brief プロパティの名称リストを保存する
  #
  # プロパティの名称を'.'区切りで表現したリストを保存する。
  #
  # @param self
  # @param out プロパティの名称リスト保存先の出力ストリーム
  # @param curr_name 現在のプロパティ名
  # @param curr 対象プロパティ
  #
  # @else
  #
  # @endif
  def _store(self, out, curr_name, curr):
    if len(curr.leaf) > 0:
      for i in range(len(curr.leaf)):
        if curr_name == "":
          next_name = curr.leaf[i].name
        else:
          next_name = curr_name+"."+curr.leaf[i].name
        self._store(out, next_name, curr.leaf[i])
        
    else:
      val = curr.value
      if val == "":
        val = curr.default_value
      out.write(curr_name+": "+val+"\n")

    return


  ##
  # @if jp
  # @brief インデントを生成する
  #
  # 指定された数字に従って生成したインデントを返す。
  # 返されるインデントは、指定数字×2つの空白。
  #
  # @param self
  # @param index インデント数の指定
  #
  # @return 生成されたインデント
  #
  # @else
  #
  # @endif
  def indent(self, index):
    space = ""

    for i in range(index-1):
      space += "  "

    return space


  ##
  # @if jp
  # @brief プロパティの内容を保存する
  #
  # プロパティに設定された内容を保存する。
  # 保存時にはプロパティ階層の深さを表す数字が付加される。
  # 値が設定されていないプロパティについては、デフォルト値が出力される。
  #
  # @param self
  # @param out プロパティ内容保存先の出力ストリーム
  # @param curr 対象プロパティ
  # @param index 現在のプロパティ階層
  #
  # @else
  #
  # @endif
  def _dump(self, out, curr, index):
    if index != 0:
      #ut.write(self.indent(index)+"- "+curr.name)
      out[0]+=self.indent(index)+"- "+curr.name

    if curr.leaf == []:
      if curr.value == "":
        #out.write(": "+curr.default_value+"\n")
        out[0]+=": "+curr.default_value+"\n"
      else:
        #out.write(": "+curr.value+"\n")
        out[0]+=": "+str(curr.value)+"\n"
      return out[0]

    if index != 0:
      #out.write("\n")
      out[0]+="\n"

    for i in range(len(curr.leaf)):
      self._dump(out, curr.leaf[i], index + 1)

    return out[0]


  ##
  # @if jp
  # @brief プロパティの内容を出力する
  #
  # プロパティに設定された内容を出力する。<br>
  # friend std::ostream& operator<<(std::ostream& lhs, const Properties& rhs);
  # の代わりに、print objにて呼び出し可能とするためのメソッド。
  #
  # @param self
  #
  # @return 設定プロパティ文字列表示
  #
  # @else
  #
  # @endif
  def __str__(self): 
    string=[""]
    return self._dump(string, self, 0)

  
