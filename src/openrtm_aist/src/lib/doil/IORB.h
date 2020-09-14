// -*- C++ -*-
/*!
 * @file IORB.h
 * @brief Generic ORB interface
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef DOIL_IORB_H
#define DOIL_IORB_H

#include <coil/Properties.h>
#include <doil/doil.h>
#include <doil/ServantFactory.h>
#include <doil/ImplBase.h>

namespace doil
{

  /*!
   * @if jp
   * @class 汎用 ORB インターフェース
   * 
   * このインターフェースは汎用のORBのためのインターフェースである。利
   * 用したい分散オブジェクトミドルウエアに対して、このインターフェース
   * を継承したORBマネージャクラスを実装する。ORBインターフェースははモ
   * ジュール境界になるため、一切の例外を発生させてはならない。すべての
   * エラーはエラーコードとして呼び出し側に返すこと。
   *
   * @else
   * @class generic ORB interface
   * @endif
   */
  class IORB
  {
  public:
    /*!
     * @if jp
     * @brief ORBManager に ORB を登録する
     *
     * ORBManager に IORB を継承した ORB オブジェクトを登録する。
     *
     * @else
     * @brief Register an ORB to the ORBManager
     *
     * This operation will register an ORB that inherits IORB
     * interface to the ORBManager.
     *
     * @endif
     */
    virtual ~IORB() throw() {};

    /*!
     * @if jp
     * @brief ORB の名前を取得する
     *
     * ORB の名前を返す。
     *
     * @else
     * @brief Getting ORB's name
     *
     * This operation will return ORB's name.
     *
     * @endif
     */
    virtual const char* name() throw() = 0;

    /*!
     * @if jp
     * @brief ORB の shutdown する
     *
     * ORB を終了する。
     *
     * @else
     * @brief Shutdown ORB
     *
     * This operation will shutdown ORB
     *
     * @endif
     */
    virtual void shutdown() throw() = 0;

    /*!
     * @if jp
     * @brief Servant の Factory を登録する
     *
     * Servant の Factory を登録する。
     *
     * @return OK:             正常終了
     *         ALREADY_EXISTS: 与えられたファクトリはすでに登録済み
     *         INVALID_ARGS:   引数の一つ以上が不正
     *         UNKNOWN:        不明なエラー
     *
     * @else
     * @brief Register servant's factory
     *
     * This operation will register servant's factory.
     *
     * @return OK:             Normal return
     *         ALREADY_EXISTS: Given factory already exists.
     *         INVALID_ARGS:   One or more given arguments are invalid.
     *         UNKNOWN:        Unknown error
     *
     * @endif
     */
    virtual ReturnCode_t registerFactory(const char* id,
                                         ServantNewFunc new_func,
                                         ServantDeleteFunc delete_func)
      throw() = 0;
    
    /*!
     * @if jp
     * @brief オブジェクトをactivateする
     *
     * 与えられたオブジェクトをアクティブ化する。
     * 与えられたオブジェクトのIDと合致するサーバントのファクトリが
     * 登録済みである必要がある。もし、オブジェクトのIDと合致するファクトリ
     * が存在しない場合、INVALID_ARGS エラーが返される。
     *
     * @return OK:             正常終了
     *         ALREADY_EXISTS: 与えられたオブジェクトはすでに存在する
     *         INVALID_ARGS:   引数が不正
     *         UNKNOWN:        不明なエラー
     *
     * @else
     * @brief Activate object
     *
     * This operation will activate given object
     * The servant that has same ID with the given object's ID should 
     * exist in this ORB servant's map. INVALID_ARGS error code will return
     * if the factory does not exist.
     *
     * @return OK:             Normal return
     *         ALREADY_EXISTS: Given object already exists.
     *         INVALID_ARGS:   The given arguments is invalid.
     *         UNKNOWN:        Unknown error
     *
     * @endif
     */
    virtual ReturnCode_t activateObject(ImplBase* impl)
      throw() = 0;

    /*!
     * @if jp
     * @brief オブジェクトをactivateする
     *
     * 与えられたオブジェクトを与えられたサーバントと共にアクティブ化する。
     * オブジェクトをアクティブ化するサーバントは存在する必要はない。
     * ただし、与えられたサーバントのIDと与えられたオブジェクトのIDは
     * 一致している必要がある。一致しない場合は INVALID_ARGS エラーが返される。
     *
     * @return OK:             正常終了
     *         ALREADY_EXISTS: 与えられたオブジェクトはすでに存在する
     *         INVALID_ARGS:   引数が不正
     *         UNKNOWN:        不明なエラー
     *
     * @else
     * @brief Activate object
     *
     * This operation will activate given object
     * The servant's factory that activate the given object does not need to
     * exists in this ORB manager. However, the given servant's ID and
     * the given object's ID have to be same. INVALID_ARGS error will be
     * returned if these IDs are not same. 
     *
     * @return OK:             Normal return
     *         ALREADY_EXISTS: Given object already exists.
     *         INVALID_ARGS:   The given arguments is invalid.
     *         UNKNOWN:        Unknown error
     *
     * @endif
     */
    virtual ReturnCode_t activateObject(ImplBase* impl,
                                        ServantBase* servant)
      throw() = 0;

    /*!
     * @if jp
     * @brief オブジェクトをdeactivateする
     *
     * 与えられたオブジェクトを非アクティブ化する。
     * オブジェクトがオブジェクトマップにない場合は NOT_FOUND エラーが返される。
     *
     * @return OK:             正常終了
     *         NOT_FOUND:      与えられたオブジェクトはマップ上に存在しない
     *         INVALID_ARGS:   引数が不正
     *         UNKNOWN:        不明なエラー
     *
     * @else
     * @brief Deactivate object
     *
     * This operation will deactivate the given object.
     * NOT_FOUND error will be returned if the given object does not exist
     * in the ative object map.
     *
     * @return OK:             Normal return
     *         NOT_FOUND:      Given object does not exist in the map.
     *         INVALID_ARGS:   The given arguments is invalid.
     *         UNKNOWN:        Unknown error
     *
     * @endif
     */
    virtual ReturnCode_t deactivateObject(ImplBase* impl)
      throw() = 0;

    /*!
     * @if jp
     * @brief オブジェクトをdeactivateする
     *
     * 与えられた名前のオブジェクトを非アクティブ化する。オブジェクト名
     * がオブジェクトマップにない場合は NOT_FOUND エラーが返される。
     *
     * @return OK:             正常終了
     *         NOT_FOUND:      与えられたオブジェクトはマップ上に存在しない
     *         INVALID_ARGS:   引数が不正
     *         UNKNOWN:        不明なエラー
     *
     * @else
     * @brief Deactivate object
     *
     * This operation will deactivate the given name's object.
     * NOT_FOUND error will be returned if the given object does not exist
     * in the ative object map.
     *
     * @return OK:             Normal return
     *         NOT_FOUND:      Given object does not exist in the map.
     *         INVALID_ARGS:   The given arguments is invalid.
     *         UNKNOWN:        Unknown error
     *
     * @endif
     */
    virtual ReturnCode_t deactivateObject(const char* name)
      throw() = 0;

    /*!
     * @if jp
     * @brief Implオブジェクトを名前で取得する
     *
     * 与えられた名前のオブジェクトを取得する。
     * オブジェクトがマップにない場合は NOT_FOUND エラーが返される。
     *
     * @return OK:             正常終了
     *         NOT_FOUND:      オブジェクトはマップ上に存在しない
     *         INVALID_ARGS:   引数が不正
     *         UNKNOWN:        不明なエラー
     *
     * @else
     * @brief Getting object by name
     *
     * This operation will return a object by given name.  NOT_FOUND
     * error will be returned if the given name's object does not
     * exist in the ative object map.
     *
     * @return OK:             Normal return
     *         NOT_FOUND:      The object does not exist in the map.
     *         INVALID_ARGS:   The given arguments is invalid.
     *         UNKNOWN:        Unknown error
     *
     * @endif
     */
    virtual doil::ImplBase* getImpl(const char* name)
      throw() = 0;

    /*!
     * @if jp
     * @brief ImplオブジェクトをServantで取得する
     *
     * 与えられたオブジェクトに対応するサーバントを取得する。
     * オブジェクトがマップにない場合は NOT_FOUND エラーが返される。
     *
     * @return OK:             正常終了
     *         NOT_FOUND:      オブジェクトはマップ上に存在しない
     *         INVALID_ARGS:   引数が不正
     *         UNKNOWN:        不明なエラー
     *
     * @else
     * @brief Getting impl object by servant
     *
     * This operation will return an impl object by given servant.
     * NOT_FOUND error will be returned if the given servant's impl object
     * does not exist in the ative object map.
     *
     * @return OK:             Normal return
     *         NOT_FOUND:      The object does not exist in the map.
     *         INVALID_ARGS:   The given arguments is invalid.
     *         UNKNOWN:        Unknown error
     *
     * @endif
     */
    virtual doil::ImplBase* toImpl(doil::ServantBase* servant)
      throw() = 0;

    /*!
     * @if jp
     * @brief Servantオブジェクトを取得する
     *
     * 与えられた名前のServantオブジェクトを取得する。
     * オブジェクトがマップにない場合は NOT_FOUND エラーが返される。
     *
     * @return OK:             正常終了
     *         NOT_FOUND:      オブジェクトはマップ上に存在しない
     *         INVALID_ARGS:   引数が不正
     *         UNKNOWN:        不明なエラー
     *
     * @else
     * @brief Getting servant object by name
     *
     * This operation will return an servant object by given name.
     * NOT_FOUND error will be returned if the given name's servant object
     * does not exist in the ative object map.
     *
     * @return OK:             Normal return
     *         NOT_FOUND:      The object does not exist in the map.
     *         INVALID_ARGS:   The given arguments is invalid.
     *         UNKNOWN:        Unknown error
     *
     * @endif
     */
    virtual doil::ServantBase* getServant(const char* name)
      throw() = 0;

    /*!
     * @if jp
     * @brief Servantオブジェクトを取得する
     *
     * 与えられたimplオブジェクトのServantオブジェクトを取得する。
     * オブジェクトがマップにない場合は NOT_FOUND エラーが返される。
     *
     * @return OK:             正常終了
     *         NOT_FOUND:      オブジェクトはマップ上に存在しない
     *         INVALID_ARGS:   引数が不正
     *         UNKNOWN:        不明なエラー
     *
     * @else
     * @brief Getting servant object by impl object
     *
     * This operation will return an servant object by given impl object.
     * NOT_FOUND error will be returned if the given name's servant object
     * does not exist in the ative object map.
     *
     * @return OK:             Normal return
     *         NOT_FOUND:      The object does not exist in the map.
     *         INVALID_ARGS:   The given arguments is invalid.
     *         UNKNOWN:        Unknown error
     *
     * @endif
     */
    virtual doil::ServantBase* toServant(doil::ImplBase* impl)
      throw() = 0;

  };
}; // namespace doil
#endif // DOIL_IORB_H
