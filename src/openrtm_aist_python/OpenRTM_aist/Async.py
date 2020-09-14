#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file Async.py
# @brief Asynchronous function invocation helper class
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2009
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#
#

import threading
import OpenRTM_aist


class Async_t(OpenRTM_aist.Task):

  def __init__(self, obj, func, *args):
    OpenRTM_aist.Task.__init__(self)
    self._obj        = obj
    self._func       = func
    self._finished   = False
    self._args       = args
    self._mutex      = threading.RLock()

  def invoke(self):
    self.activate()


  def finished(self):
    guard = OpenRTM_aist.ScopedLock(self._mutex)
    return self._finished


  def svc(self):
    if len(self._args) > 0:
      self._func(self._obj, self._args)
    else:
      self._func(self._obj)

    guard = OpenRTM_aist.ScopedLock(self._mutex)
    self._finished = True
    return 0


class Async_ref_t(OpenRTM_aist.Task):

  def __init__(self, obj, func, *args):
    OpenRTM_aist.Task.__init__(self)
    self._obj        = obj
    self._func       = func
    self._args       = args
    self._finished   = False
    

  def invoke(self):
    self.activate()


  def finished(self):
    return self._finished
  

  def svc(self):
    if len(self._args) > 0:
      self._func(self._obj, self._args)
    else:
      self._func(self._obj)

    self._finished = True
    return 0
  
  
##
# @if jp
# @brief 非同期メンバー関数呼び出しヘルパー関数
#
# メンバー関数を非同期に呼ぶためのヘルパー関数
# 例
#
#  class A
#  {
#  public:
#    // 時間のかかる関数
#    void hoge() {
#      for (int i(0); i < 5; ++i) {
#        std::cout << "hoge" << std::endl;
#        sleep(1);
#      }
#    }
#    // 時間のかかる関数
#    void munya(const char* msg) {
#      for (int i(0); i < 10; ++i) {
#        std::cout << "message is: " << msg << std::endl;
#        sleep(1);
#      }
#    }
#    int add_one(int val) {
#      return val + 1;
#    }
#  };
# この様なクラスのオブジェクトに対して、
#
#  A a;
#  Async* invoker0(AsyncInvoker(&a,
#                               std::mem_fun(&A::hoge)));
#  Async* invoker1(AsyncInvoker(&a,
#                               std::bind2nd(std::mem_fun(&A::munya),
#                                            "ほげ")));
#  invoker0->invoke(); // すぐに戻る
#  invoker1->invoke(); // すぐに戻る
#
#  delete invoker0; // 必ず削除すること
#  delete invoker1; // 必ず削除すること
#
# のように非同期の呼び出しができる。
# 呼び出しの戻り値を取得したい場合は、自前の関数オブジェクトを用意する。
#
#  class add_one_functor
#  {
#    int m_val, m_ret;
#  public:
#    add_one_functor(int val) : m_val(val), m_ret(0) {}
#    void operaotr(A* obj) {
#      m_ret = obj->add_one(m_val);
#    }
#    int get_ret() {
#      return m_ret;
#    }
#  };
#
# 上記の関数オブジェクトのインスタンスを作成し、そのポインタを渡す。
#
#  add_one_functor aof(100);
#  Async* invoker2(AsyncInvoker(&a, &aof));
#  invoker2->invoke();
#  invoker2->wait();
#  std::cout << "result: " << aof.get_ret() << std::endl;
#  delete invoker2;
#
# 通常、AsyncInvoker が返すオブジェクトは明示的に削除しなければ
# ならないが、第三引数に true を渡すことで、非同期実行が終了すると同時に
# 自動的にインスタンスが削除される。
#
# // invoker3 は削除 (delete invoker3) してはいけない
# Async* invoker3(AsyncInvoker(&a, std::mem_fun(&A::hoge), true));
#
# // インスタンス生成と同時に実行することもできる。
# AsyncInvoker(&a, std::mem_fun(&A::hoge))->invoke();
#
# @else
#
# @endif
#
#def Async_tInvoker(func, auto_delete = False):
def Async_tInvoker(obj, func, *args):
  return Async_t(obj, func, *args)


def Async_ref_tInvoker(obj, func, *args):
  return Async_ref_t(obj, func, *args)
