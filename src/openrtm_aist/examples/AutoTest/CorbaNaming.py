#!/usr/bin/env python
# -*- coding: euc-jp -*-


##
# \file CorbaNaming.py
# \brief CORBA naming service helper class
# \author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import omniORB.CORBA as CORBA
import CosNaming
import string

##
# @if jp
# @class CorbaNaming
# @brief CORBA Naming Service ヘルパークラス
#
# このクラスは、CosNaming::NamingContext に対するラッパークラスである。
# CosNaming::NamingContext が持つオペレーションとほぼ同じ機能の
# オペレーションを提供するとともに、ネームコンポーネント CosNaming::Name
# の代わりに文字列による名前表現を受け付けるオペレーションも提供する。
#
# オブジェクトは生成時、あるいは生成直後に CORBA ネームサーバに接続し
# 以後、このネームサーバのルートコンテキストに対して種々のオペレーション
# を処理する。
# 深い階層のネーミングコンテキストの作成やオブジェクトのバインドにおいて、
# 途中のコンテキストが存在しない場合でも、強制的にコンテキストをバインド
# し目的のコンテキストやオブジェクトのバインドを行うこともできる。
#
# @since 0.4.0
#
# @else
# @class CorbaNaming
# @brief CORBA Naming Service helper class
#
# This class is a wrapper class of CosNaming::NamingContext.
# Almost the same operations which CosNaming::NamingContext has are
# provided, and some operation allows string naming representation of
# context and object instead of CosNaming::Name.
#
# The object of the class would connect to a CORBA naming server at
# the instantiation or immediately after instantiation.
# After that the object invokes operations to the root context of it.
# This class realizes forced binding to deep NamingContext, without binding
# intermediate NamingContexts explicitly.
#
# @since 0.4.0
#
# @endif
class CorbaNaming:
  """
  """



  ##
  # @if jp
  #
  # @brief コンストラクタ
  #
  # @param self
  # @param orb ORB
  # @param name_server ネームサーバの名称(デフォルト値:None)
  #
  # @else
  #
  # @brief Consructor
  #
  # @endif
  def __init__(self, orb, name_server=None):
    self._orb = orb
    self._nameServer = ""
    self._rootContext = CosNaming.NamingContext._nil
    self._blLength = 100

    if name_server:
      self._nameServer = "corbaloc::" + name_server + "/NameService"
      try:
        obj = orb.string_to_object(self._nameServer)
        self._rootContext = obj._narrow(CosNaming.NamingContext)
        if CORBA.is_nil(self._rootContext):
          print "CorbaNaming: Failed to narrow the root naming context."

      except CORBA.ORB.InvalidName:
        print "Service required is invalid [does not exist]."

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


  ##
  # @if jp
  #
  # @brief ネーミングサービスの初期化
  # 
  # 指定されたネームサーバ上のネーミングサービスを初期化します。
  # 
  # @param self
  # @param name_server ネームサーバの名称
  # 
  # @else
  # 
  # @endif
  def init(self, name_server):
    self._nameServer = "corbaloc::" + name_server + "/NameService"
    obj = self._orb.string_to_object(self._nameServer)
    self._rootContext = obj._narrow(CosNaming.NamingContext)
    if CORBA.is_nil(self._rootContext):
      raise MemoryError

    return


  ##
  # @if jp
  #
  # @brief Object を bind する
  #
  # CosNaming::bind() とほぼ同等の働きをするが、常に与えられたネームサーバの
  # ルートコンテキストに対してbind()が呼び出される点が異なる。
  #
  # Name <name> と Object <obj> を当該 NamingContext 上にバインドする。
  # c_n が n 番目の NameComponent をあらわすとすると、
  # name が n 個の NameComponent から成るとき、以下のように扱われる。
  #
  # cxt->bind(<c_1, c_2, ... c_n>, obj) は以下の操作と同等である。
  # cxt->resolve(<c_1, ... c_(n-1)>)->bind(<c_n>, obj)
  #
  # すなわち、1番目からn-1番目のコンテキストを解決し、n-1番目のコンテキスト
  # 上に name <n> として　obj を bind する。
  # 名前解決に参加する <c_1, ... c_(n-1)> の NemingContext は、
  # bindContext() や rebindContext() で既にバインド済みでなければならない。
  # もし <c_1, ... c_(n-1)> の NamingContext が存在しない場合には、
  # NotFound 例外が発生する。
  #
  # ただし、強制バインドフラグ force が true の時は、<c_1, ... c_(n-1)>
  # が存在しない場合にも、再帰的にコンテキストをバインドしながら、
  # 最終的に obj を名前 name <c_n> にバインドする。
  #
  # いずれの場合でも、n-1番目のコンテキスト上に name<n> のオブジェクト
  # (Object あるいは コンテキスト) がバインドされていれば
  # AlreadyBound 例外が発生する。
  #
  # @param self
  # @param name_list オブジェクトに付ける名前の NameComponent
  # @param obj 関連付けられる Object
  # @param force trueの場合、途中のコンテキストを強制的にバインドする
  #              (デフォルト値:None)
  #
  # @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
  # @exception CannotProceed 何らかの理由で処理を継続できない。
  # @exception InvalidName 引数 name_list の名前が不正。
  # @exception AlreadyBound name <c_n> の Object がすでにバインドされている。
  #
  # @else
  #
  # @brief
  #
  # @endif
  def bind(self, name_list, obj, force=None):
    if force is None :
      force = True

    try:
      self._rootContext.bind(name_list, obj)
    except CosNaming.NamingContext.NotFound:
      if force:
        self.bindRecursive(self._rootContext, name_list, obj)
      else:
        raise
    except CosNaming.NamingContext.CannotProceed, err:
      if force:
        self.bindRecursive(err.cxt, err.rest_of_name, obj)
      else:
        raise
    except CosNaming.NamingContext.AlreadyBound:
      self._rootContext.rebind(name_list, obj)


  ##
  # @if jp
  #
  # @brief Object を bind する
  #
  # Object を bind する際に与える名前が文字列表現であること以外は、bind()
  # と同じである。bind(toName(string_name), obj) と等価。
  #
  # @param self
  # @param string_name オブジェクトに付ける名前の文字列表現
  # @param obj 関連付けられるオブジェクト
  # @param force trueの場合、途中のコンテキストを強制的にバインドする
  #              (デフォルト値:true)
  #
  # @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
  # @exception CannotProceed 何らかの理由で処理を継続できない。
  # @exception InvalidName 引数 string_name の名前が不正。
  # @exception AlreadyBound name <n> の Object がすでにバインドされている。
  #
  # @else
  #
  # @brief
  #
  # @endif
  def bindByString(self, string_name, obj, force=True):
    self.bind(self.toName(string_name), obj, force)


  ##
  # @if jp
  #
  # @brief 途中のコンテキストを bind しながら Object を bind する
  #
  # context で与えられた NamingContext に対して、name で指定された
  # ネームコンポーネント <c_1, ... c_(n-1)> を NamingContext として
  # 解決しながら、名前 <c_n> に対して obj を bind する。
  # もし、<c_1, ... c_(n-1)> に対応する NamingContext がない場合には
  # 新たな NamingContext をバインドする。
  #
  # 最終的に <c_1, c_2, ..., c_(n-1)> に対応する NamingContext が生成
  # または解決された上で、CosNaming::bind(<c_n>, object) が呼び出される。
  # このとき、すでにバインディングが存在すれば AlreadyBound例外が発生する。
  #
  # 途中のコンテキストを解決する過程で、解決しようとするコンテキストと
  # 同じ名前の NamingContext ではない Binding が存在する場合、
  # CannotProceed 例外が発生し処理を中止する。
  #
  # @param self
  # @param context bind を開始する　NamingContext
  # @param name_list オブジェクトに付ける名前のネームコンポーネント
  # @param obj 関連付けられるオブジェクト
  #
  # @exception CannotProceed <c_1, ..., c_(n-1)> に対応する NamingContext 
  #            のうちひとつが、すでに NamingContext 以外の object にバインド
  #            されており、処理を継続できない。
  # @exception InvalidName 名前 name_list が不正
  # @exception AlreadyBound name <c_n> にすでに何らかの object がバインド
  #            されている。
  # @else
  #
  # @brief
  #
  # @endif
  def bindRecursive(self, context, name_list, obj):
    length = len(name_list)
    cxt = context
    for i in range(length):
      if i == length -1:
        try:
          cxt.bind(self.subName(name_list, i, i), obj)
        except CosNaming.NamingContext.AlreadyBound:
          cxt.rebind(self.subName(name_list, i, i), obj)
        return
      else:
        if self.objIsNamingContext(cxt):
          cxt = self.bindOrResolveContext(cxt,self.subName(name_list, i, i))
        else:
          raise CosNaming.NamingContext.CannotProceed(cxt, self.subName(name_list, i))
    return


  ##
  # @if jp
  #
  # @brief Object を rebind する
  #
  # name_list で指定された Binding がすでに存在する場合を除いて bind() と同じ
  # である。バインディングがすでに存在する場合には、新しいバインディングに
  # 置き換えられる。
  #
  # @param self
  # @param name_list オブジェクトに付ける名前の NameComponent
  # @param obj 関連付けられるオブジェクト
  # @param force trueの場合、途中のコンテキストを強制的にバインドする
  #              (デフォルト値:true)
  #
  # @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
  # @exception CannotProceed 何らかの理由で処理を継続できない。
  # @exception InvalidName 名前 name_list が不正
  #
  # @else
  #
  # @brief
  #
  # @endif
  def rebind(self, name_list, obj, force=True):
    if force is None:
      force = True
      
    try:
      self._rootContext.rebind(name_list, obj)

    except CosNaming.NamingContext.NotFound:
      if force:
        self.rebindRecursive(self._rootContext, name_list, obj)
      else:
        raise

    except CosNaming.NamingContext.CannotProceed, err:
      if force:
        self.rebindRecursive(err.cxt, err,rest_of_name, obj)
      else:
        raise
      
    return


  ##
  # @if jp
  #
  # @brief Object を rebind する
  #
  # Object を rebind する際に与える名前が文字列表現であること以外は rebind()
  # と同じである。rebind(toName(string_name), obj) と等価。
  #
  # @param self
  # @param string_name オブジェクトに付ける名前の文字列表現
  # @param obj 関連付けられるオブジェクト
  # @param force trueの場合、途中のコンテキストを強制的にバインドする
  #              (デフォルト値:true)
  #
  # @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
  # @exception CannotProceed 何らかの理由で処理を継続できない。
  # @exception InvalidName 引数 string_name の名前が不正。
  #
  # @else
  #
  # @brief
  #
  # @endif
  def rebindByString(self, string_name, obj, force=True):
    self.rebind(self.toName(string_name), obj, force)

    return


  ##
  # @if jp
  #
  # @brief 途中のコンテキストを bind しながら Object を rebind する
  #
  # name_list <c_n> で指定された NamingContext もしくは Object がすでに存在する
  # 場合を除いて bindRecursive() と同じである。
  #
  # name_list <c_n> で指定されたバインディングがすでに存在する場合には、
  # 新しいバインディングに置き換えられる。
  #
  # @param self
  # @param context bind を開始する　NamingContext
  # @param name_list オブジェクトに付ける名前の NameComponent
  # @param obj 関連付けられるオブジェクト
  #
  # @exception CannotProceed 途中のコンテキストが解決できない。
  # @exception InvalidName 与えられた name_list が不正。
  #
  # @else
  #
  # @brief
  #
  # @endif
  def rebindRecursive(self, context, name_list, obj):
    length = len(name_list)
    for i in range(length):
      if i == length - 1:
        context.rebind(self.subName(name_list, i, i), obj)
        return
      else:
        if self.objIsNamingContext(context):
          try:
            context = context.bind_new_context(self.subName(name_list, i, i))
          except CosNaming.NamingContext.AlreadyBound:
            obj_ = context.resolve(self.subName(name_list, i, i))
            context = obj_._narrow(CosNaming.NamingContext)
        else:
          raise CosNaming.NamingContext.CannotProceed(context, self.subName(name_list, i))
    return


  ##
  # @if jp
  #
  # @brief NamingContext を bind する
  #
  # bind 対象として指定された引数 name が文字列の場合は bindByString() と、
  # それ以外の場合は bind() と同じである。
  #
  # @param self
  # @param name オブジェクトに付ける名前
  # @param name_cxt 関連付けられる NamingContext
  # @param force trueの場合、途中のコンテキストを強制的にバインドする
  #              (デフォルト値:True)
  #
  # @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
  # @exception CannotProceed 何らかの理由で処理を継続できない。
  # @exception InvalidName 引数 name の名前が不正。
  # @exception AlreadyBound name <c_n> の Object がすでにバインドされている。
  #
  # @else
  #
  # @brief
  #
  # @endif
  def bindContext(self, name, name_cxt, force=True):
    if isinstance(name, basestring):
      self.bind(self.toName(name), name_cxt, force)
    else:
      self.bind(name, name_cxt, force)
    return


  ##
  # @if jp
  #
  # @brief NamingContext を bind する
  #
  # bind されるオブジェクトが NamingContext であることを除いて
  # bindRecursive() と同じである。
  #
  # @param self
  # @param context bind を開始する　NamingContext
  # @param name_list オブジェクトに付ける名前のネームコンポーネント
  # @param name_cxt 関連付けられる NamingContext
  #
  # @else
  #
  # @brief
  #
  # @endif
  def bindContextRecursive(self, context, name_list, name_cxt):
    self.bindRecursive(context, name_list, name_cxt)
    return


  ##
  # @if jp
  #
  # @brief NamingContext を rebind する
  #
  # bind 対象として指定された引数 name が文字列の場合は rebindByString() と、
  # それ以外の場合は rebind() と同じである。
  # どちらの場合もバインディングがすでに存在する場合には、
  # 新しいバインディングに置き換えられる。
  #
  # @param self
  # @param name オブジェクトに付ける名前のネームコンポーネント
  # @param name_cxt 関連付けられる NamingContext
  # @param force trueの場合、途中のコンテキストを強制的にバインドする
  #              (デフォルト値:true)
  #
  # @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
  # @exception CannotProceed 何らかの理由で処理を継続できない。
  # @exception InvalidName 引数 name の名前が不正。
  #
  # @else
  #
  # @endif
  def rebindContext(self, name, name_cxt, force=True):
    if isinstance(name, basestring):
      self.rebind(self.toName(name), name_cxt, force)
    else:
      self.rebind(name, name_cxt, force)
    return


  ##
  # @if jp
  #
  # @brief 途中のコンテキストを再帰的に rebind し NamingContext を rebind する    #
  # bind されるオブジェクトが NamingContext であることを除いて
  # rebindRecursive() と同じである。
  #
  # @param self
  # @param context bind を開始する　NamingContext
  # @param name_list オブジェクトに付ける名前の NameComponent
  # @param name_cxt 関連付けられる NamingContext
  #
  # @else
  #
  # @brief
  #
  # @endif
  def rebindContextRecursive(self, context, name_list, name_cxt):
    self.rebindRecursive(context, name_list, name_cxt)
    return


  ##
  # @if jp
  #
  # @brief Object を name から解決する
  #
  # name に bind されているオブジェクト参照を返す。
  # ネームコンポーネント <c_1, c_2, ... c_n> は再帰的に解決される。
  # 
  # 引数 name に与えられた値が文字列の場合にはまず最初に toName() によって
  # NameComponent に変換される。
  # 
  # CosNaming::resolve() とほぼ同等の働きをするが、常に与えられた
  # ネームサーバのルートコンテキストに対して resolve() が呼び出される点が
  # 異なる。
  #
  # @param self
  # @param name 解決すべきオブジェクトの名前のネームコンポーネント
  #
  # @return 解決されたオブジェクト参照
  #
  # @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
  # @exception CannotProceed 何らかの理由で処理を継続できない。
  # @exception InvalidName 引数 name の名前が不正。
  #
  # @else
  #
  # @endif
  def resolve(self, name):
    if isinstance(name, basestring):
      name_ = self.toName(name)
    else:
      name_ = name
      
    try:
      obj = self._rootContext.resolve(name_)
      return obj
    except CosNaming.NamingContext.NotFound, ex:
      return None


  ##
  # @if jp
  #
  # @brief 指定された名前のオブジェクトの bind を解除する
  #
  # name に bind されているオブジェクト参照を解除する。
  # ネームコンポーネント <c_1, c_2, ... c_n> は再帰的に解決される。
  # 
  # 引数 name に与えられた値が文字列の場合にはまず最初に toName() によって
  # NameComponent に変換される。
  # 
  # CosNaming::unbind() とほぼ同等の働きをするが、常に与えられた
  # ネームサーバのルートコンテキストに対して unbind() が呼び出される点が
  # 異なる。
  #
  # @param self
  # @param name 削除するオブジェクトのネームコンポーネント
  #
  # @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
  # @exception CannotProceed 何らかの理由で処理を継続できない。
  # @exception InvalidName 引数 name の名前が不正。
  #
  # @else
  #
  # @endif
  # void unbind(const CosNaming::Name& name)
  #   throw(NotFound, CannotProceed, InvalidName);
  def unbind(self, name):
    if isinstance(name, basestring):
      name_ = self.toName(name)
    else:
      name_ = name

    self._rootContext.unbind(name_)
    return


  ##
  # @if jp
  #
  # @brief 新しいコンテキストを生成する
  #
  # 与えられたネームサーバ上で生成された NamingContext を返す。
  # 返された NamingContext は bind されていない。
  # 
  # @param self
  # 
  # @return 生成された新しい NamingContext
  #
  # @else
  #
  # @endif
  def newContext(self):
    return self._rootContext.new_context()


  ##
  # @if jp
  #
  # @brief 新しいコンテキストを bind する
  #
  # 与えられた name に対して新しいコンテキストをバインドする。
  # 生成された　NamingContext はネームサーバ上で生成されたものである。
  # 
  # 引数 name に与えられた値が文字列の場合にはまず最初に toName() によって
  # NameComponent に変換される。
  # 
  # @param self
  # @param name NamingContextに付ける名前のネームコンポーネント
  # @param force trueの場合、途中のコンテキストを強制的にバインドする
  #              (デフォルト値:true)
  #
  # @return 生成された新しい NamingContext
  #
  # @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
  # @exception CannotProceed 何らかの理由で処理を継続できない。
  # @exception InvalidName 引数 name の名前が不正。
  # @exception AlreadyBound name <n> の Object がすでにバインドされている。
  #
  # @else
  #
  # @endif
  def bindNewContext(self, name, force=True):
    if force is None:
      force = True
      
    if isinstance(name, basestring):
      name_ = self.toName(name)
    else:
      name_ = name

    try:
      return self._rootContext.bind_new_context(name_)
    except CosNaming.NamingContext.NotFound:
      if force:
        self.bindRecursive(self._rootContext, name_, self.newContext())
      else:
        raise
    except CosNaming.NamingContext.CannotProceed, err:
      if force:
        self.bindRecursive(err.cxt, err.rest_of_name, self.newContext())
      else:
        raise
    return None


  ##
  # @if jp
  #
  # @brief NamingContext を非アクティブ化する
  #
  # context で指定された NamingContext を非アクティブ化する。
  # context に他のコンテキストがバインドされている場合は NotEmpty 例外が
  # 発生する。
  # 
  # @param self
  # @param context 非アクティブ化する NamingContext
  #
  # @exception NotEmpty 対象context に他のコンテキストがバインドされている。
  #
  # @else
  #
  # @else
  #
  # @brief Destroy the naming context
  #
  # Delete the specified naming context.
  # any bindings should be <unbind> in which the given context is bound to
  # some names before invoking <destroy> operation on it. 
  #
  # @param context NamingContext which is destroied.
  #     
  # @exception NotEmpty 
  #
  # @else
  #
  # @endif
  def destroy(self, context):
    context.destroy()


  ##
  # @if jp
  # @brief NamingContext を再帰的に下って非アクティブ化する
  #
  # context で与えられた NamingContext に対して、name で指定された
  # ネームコンポーネント <c_1, ... c_(n-1)> を NamingContext として
  # 解決しながら、名前 <c_n> に対して 非アクティブ化を行う。
  #
  # @param self
  # @param context 非アクティブ化する NamingContext
  #
  # @exception NotEmpty 対象context に他のコンテキストがバインドされている。
  # @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
  # @exception CannotProceed 何らかの理由で処理を継続できない。
  # @exception InvalidName 引数 name の名前が不正。
  #
  # @else
  # @brief Destroy the naming context recursively
  # @endif
  def destroyRecursive(self, context):
    cont = True
    bl = []
    bi = 0
    bl, bi = context.list(self._blLength)
    while cont:
      for i in range(len(bl)):
        if bl[i].binding_type == CosNaming.ncontext:
          obj = context.resolve(bl[i].binding_name)
          next_context = obj._narrow(CosNaming.NamingContext)

          self.destroyRecursive(next_context)
          context.unbind(bl[i].binding_name)
          next_context.destroy()
        elif bl[i].binding_type == CosNaming.nobject:
          context.unbind(bl[i].binding_name)
        else:
          assert(0)
      if CORBA.is_nil(bi):
        cont = False
      else:
        bi.next_n(self._blLength, bl)

    if not (CORBA.is_nil(bi)):
      bi.destroy()
    return


  ##
  # @if jp
  # @brief すべての Binding を削除する
  #
  # 登録されている全てのBinding を削除する。
  #
  # @param self
  #
  # @else
  # @brief Destroy all binding
  # @endif
  def clearAll(self):
    self.destroyRecursive(self._rootContext)
    return


  ##
  # @if jp
  # @brief 与えられた NamingContext の Binding を取得する
  #
  # 指定された NamingContext の Binding を取得する。
  #
  # @param self
  # @param name_cxt Binding 取得対象 NamingContext
  # @param how_many Binding を取得する階層の深さ
  # @param rbl 取得した Binding を保持するホルダ
  # @param rbi 取得した Binding をたどるためのイテレータ
  #
  # @else
  # @endif
  def list(self, name_cxt, how_many, rbl, rbi):
    bl, bi = name_cxt.list(how_many)

    for i in bl:
      rbl.append(bl)

    rbi.append(bi)
  

  #============================================================
  # interface of NamingContext
  #============================================================

  ##
  # @if jp
  # @brief 与えられた NameComponent の文字列表現を返す
  #
  # 指定された NameComponent を文字に変換する。
  #
  # @param self
  # @param name_list 変換対象 NameComponent
  #
  # @return 文字列変換結果
  #
  # @exception InvalidName 引数 name_list の名前が不正。
  #
  # @else
  # @brief Get string representation of given NameComponent
  # @endif
  def toString(self, name_list):
    if len(name_list) == 0:
      raise CosNaming.NamingContext.InvalidName

    slen = self.getNameLength(name_list)
    string_name = [""]
    self.nameToString(name_list, string_name, slen)

    return string_name


  ##
  # @if jp
  # @brief 与えられた文字列表現を NameComponent に分解する
  #
  # 指定された文字列を NameComponent に変換する。
  #
  # @param self
  # @param sname 変換対象文字列
  #
  # @return NameComponent 変換結果
  #
  # @exception InvalidName 引数 sname が不正。
  #
  # @else
  # @brief Get NameComponent from gien string name representation
  # @endif
  def toName(self, sname):
    if not sname:
      raise CosNaming.NamingContext.InvalidName

    string_name = sname
    name_comps = []

    nc_length = 0
    nc_length = self.split(string_name, "/", name_comps)
    if not (nc_length > 0):
      raise CosNaming.NamingContext.InvalidName

    name_list = [CosNaming.NameComponent("","") for i in range(nc_length)]

    for i in range(nc_length):
      pos = string.rfind(name_comps[i][0:],".")
      if pos == -1:
        name_list[i].id   = name_comps[i]
        name_list[i].kind = ""
      else:
        name_list[i].id   = name_comps[i][0:pos]
        name_list[i].kind = name_comps[i][(pos+1):]

    return name_list


  ##
  # @if jp
  # @brief 与えられた addr と string_name から URL表現を取得する
  #
  # 指定されたアドレスと名称をURLに変換する。
  #
  # @param self
  # @param addr 変換対象アドレス
  # @param string_name 変換対象名称
  #
  # @return URL 変換結果
  #
  # @exception InvalidAddress 引数 addr が不正。
  # @exception InvalidName 引数 string_name が不正。
  #
  # @else
  # @brief Get URL representation from given addr and string_name
  # @endif
  def toUrl(self, addr, string_name):
    return self._rootContext.to_url(addr, string_name)


  ##
  # @if jp
  # @brief 与えられた文字列表現を resolve しオブジェクトを返す
  #
  # 指定された文字列表現をresolveし，オブジェクトを取得する。
  #
  # @param self
  # @param string_name 取得対象オブジェクト文字列表現
  #
  # @return 解決されたオブジェクト
  #
  # @exception NotFound 途中の <c_1, c_2, ..., c_(n-1)> が存在しない。
  # @exception CannotProceed 何らかの理由で処理を継続できない。
  # @exception InvalidName 引数 name の名前が不正。
  # @exception AlreadyBound name <n> の Object がすでにバインドされている。
  #
  # @else
  # @brief Resolve from name of string representation and get object 
  # @endif
  def resolveStr(self, string_name):
    return self.resolve(self.toName(string_name))


  #============================================================
  # Find functions
  #============================================================

  ##
  # @if jp
  #
  # @brief オブジェクトの名前をバインドまたは解決する
  #
  # 指定されたコンテキストに対してオブジェクトを NameComponent で指定された
  # 位置にバインドする。
  # 同一箇所に既に他の要素がバインド済みの場合は、既存のバインド済み要素を
  # 取得する。
  #
  # @param self
  # @param context bind もしくは resole 対象コンテキスト
  # @param name_list オブジェクトに付ける名前の NameComponent
  # @param obj 関連付けられる Object
  #
  # @return NameComponent で指定された位置にバインドされているオブジェクト
  #
  # @else
  # @brief Bind of resolve the given name component
  # @endif
  def bindOrResolve(self, context, name_list, obj):
    try:
      context.bind_context(name_list, obj)
      return obj
    except CosNaming.NamingContext.AlreadyBound:
      obj = context.resolve(name_list)
      return obj
    return CORBA.Object._nil


  ##
  # @if jp
  #
  # @brief コンテキストの名前をバインドまたは解決する
  #
  # 指定されたコンテキストに対して Contextを NameComponent で指定された位置に
  # バインドする。
  # Context が空の場合は新規コンテキストを生成してバインドする。
  # 同一箇所に既に他の要素がバインド済みの場合は、既存のバインド済み要素を
  # 取得する。
  #
  # @param self
  # @param context bind もしくは resole 対象コンテキスト
  # @param name_list コンテキストに付ける名前の NameComponent
  # @param new_context 関連付けられる Context(デフォルト値:None)
  #
  # @return NameComponent で指定された位置にバインドされているContext
  #
  # @else
  # @brief Bind of resolve the given name component
  # @endif
  def bindOrResolveContext(self, context, name_list, new_context=None):
    if new_context is None:
      new_cxt = self.newContext()
    else:
      new_cxt = new_context

    obj = self.bindOrResolve(context, name_list, new_cxt)
    return obj._narrow(CosNaming.NamingContext)


  ##
  # @if jp
  # @brief ネームサーバの名前を取得する
  #
  # 設定したネームサーバの名前を取得する。
  #
  # @param self
  #
  # @return ネームサーバの名前
  #
  # @else
  # @brief Get the name of naming server
  # @endif
  def getNameServer(self):
    return self._nameServer


  ##
  # @if jp
  # @brief ルートコンテキストを取得する
  #
  # 設定したネームサーバのルートコンテキストを取得する。
  #
  # @param self
  #
  # @return ネームサーバのルートコンテキスト
  #
  # @else
  # @brief Get the root context
  # @endif
  def getRootContext(self):
    return self._rootContext


  ##
  # @if jp
  # @brief オブジェクトがネーミングコンテキストか判別する
  #
  # 指定した要素がネーミングコンテキストか判別する
  #
  # @param self
  # @param obj 判別対象要素
  #
  # @return 判別結果(ネーミングコンテキスト:true、それ以外:false)
  #
  # @else
  # @brief Whether the object is NamingContext
  # @endif
  def objIsNamingContext(self, obj):
    nc = obj._narrow(CosNaming.NamingContext)
    if CORBA.is_nil(nc):
      return False
    else:
      return True


  ##
  # @if jp
  # @brief 与えられた名前がネーミングコンテキストかどうか判別する
  #
  # NameComponent もしくは文字列で指定した要素がネーミングコンテキストか
  # 判別する
  #
  # @param self
  # @param name_list 判別対象
  #
  # @return 判別結果(ネーミングコンテキスト:true、それ以外:false)
  #
  # @else
  # @brief Whether the given name component is NamingContext
  # @endif
  def nameIsNamingContext(self, name_list):
    return self.objIsNamingContext(self.resolve(name_list))


  ##
  # @if jp
  # @brief ネームコンポーネントの部分を返す
  #
  # 指定された範囲のネームコンポーネントを取得する。
  # 終了位置が指定されていない場合は、最後の要素を除いたネームコンポーネント
  # を返す。
  #
  # @param self
  # @param name_list 検索対象NameComponent
  # @param begin 取得範囲開始位置
  # @param end 取得範囲終了位置(デフォルト値:None)
  #
  # @return NameComponent 取得結果
  #
  # @else
  # @brief Get subset of given name component
  # @endif
  def subName(self, name_list, begin, end = None):
    if end is None or end < 0:
      end = len(name_list) - 1

    sub_len = end - (begin -1)
    objId = ""
    kind  = ""
    
    sub_name = []
    for i in range(sub_len):
      sub_name.append(name_list[begin + i])

    return sub_name


  ##
  # @if jp
  # @brief ネームコンポーネントの文字列表現を取得する
  #
  # 指定した範囲のネームコンポーネントの文字列表現を取得する。
  # 文字列表現は、NameComponentの構成が{Nc[0],Nc[1],Nc[2]･･･}の場合、
  #   Nc[0]id.Nc[0].kind/Nc[1]id.Nc[1].kind/Nc[2].id/Nc[2].kind･･･
  # という形式で取得できる。
  # 取得した文字列の長さが指定した長さ以上の場合は、
  # 指定した長さで切り捨てられる。
  #
  # @param self
  # @param name_list 取得対象NameComponent
  # @param string_name 取得結果文字列
  # @param slen 取得対象文字列最大値
  #
  # @else
  # @brief Get string representation of name component
  # @endif
  def nameToString(self, name_list, string_name, slen):
    for i in range(len(name_list)):
      for id_ in name_list[i].id:
        if id_ == "/" or id_ == "." or id_ == "\\":
          string_name[0] += "\\"
        string_name[0] += id_

      if name_list[i].id == "" or name_list[i].kind != "":
        string_name[0] += "."

      for kind_ in name_list[i].kind:
        if kind_ == "/" or kind_ == "." or kind_ == "\\":
          string_name[0] += "\\"
        string_name[0] += kind_

      string_name[0] += "/"


  ##
  # @if jp
  # @brief ネームコンポーネントの文字列表現時の文字長を取得する
  #
  # 指定したネームコンポーネントを文字列で表現した場合の長さを取得する。
  # 文字列表現は、NameComponentの構成が{Nc[0],Nc[1],Nc[2]・・・}の場合、
  #   Nc[0]id.Nc[0].kind/Nc[1]id.Nc[1].kind/Nc[2].id/Nc[2].kind・・・
  # という形式で取得できる。
  #
  # @param self
  # @param name_list 取得対象NameComponent
  #
  # @return 指定したネームコンポーネントの文字列長さ
  #
  # @else
  # @brief Get string length of the name component's string representation
  # @endif
  def getNameLength(self, name_list):
    slen = 0

    for i in range(len(name_list)):
      for id_ in name_list[i].id:
        if id_ == "/" or id_ == "." or id_ == "\\":
          slen += 1
        slen += 1
      if name_list[i].id == "" or name_list[i].kind == "":
        slen += 1

      for kind_ in name_list[i].kind:
        if kind_ == "/" or kind_ == "." or kind_ == "\\":
          slen += 1
        slen += 1

      slen += 1

    return slen


  ##
  # @if jp
  # @brief 文字列の分割
  #
  # 文字列を指定したデリミタで分割する。
  #
  # @param self
  # @param input 分割対象文字列
  # @param delimiter 分割用デリミタ
  # @param results 分割結果
  #
  # @return 分割した文字列の要素数
  #
  # @else
  # @brief Split of string
  # @endif
  def split(self, input, delimiter, results):
    delim_size = len(delimiter)
    found_pos = begin_pos = pre_pos = substr_size = 0

    if input[0:delim_size] == delimiter:
      begin_pos = pre_pos = delim_size

    while 1:
      found_pos = string.find(input[begin_pos:],delimiter)
      
      if found_pos == -1:
        results.append(input[pre_pos:])
        break

      if found_pos > 0 and input[found_pos - 1] == "\\":
        begin_pos += found_pos + delim_size
      else:
        substr_size = found_pos + (begin_pos - pre_pos)
        if substr_size > 0:
          results.append(input[pre_pos:(pre_pos+substr_size)])
        begin_pos += found_pos + delim_size
        pre_pos   = begin_pos

    return len(results)
