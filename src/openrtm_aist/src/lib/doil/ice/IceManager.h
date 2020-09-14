// -*- C++ -*-
/*!
 * @file IceManager.h
 * @brief Ice manager for doil
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
#ifndef DOIL_ICE_ICEMANAGER_H
#define DOIL_ICE_ICEMANAGER_H

#include <coil/Properties.h>
#include <coil/Mutex.h>
#include <coil/Guard.h>
#include <doil/IORB.h>
#include <doil/ServantFactory.h>
#include <doil/ice/Ice.h>
#include <doil/ice/IceServantBase.h>
#include <doil/ObjectManager.h>

namespace doil
{
namespace Ice
{
  /*!
   * @if jp
   * @class IceManager クラス
   *
   * Ice ORB を IORB でインターフェースする Singleton クラス。
   *
   * @else
   * @class IceManager class
   *
   * This class is a singleton class that interfaces IORB.
   *
   * @endif
   */
  class IceManager
    : public doil::IORB
  {
  public:
    /*!
     * @if jp
     * @brief 初期化関数
     * @else
     * @brief initializer
     * @endif
     */
    static IceManager* init(coil::Properties prop)
      throw();

    /*!
     * @if jp
     * @brief インスタンス取得関数
     * @else
     * @brief getting instance
     * @endif
     */
    static IceManager& instance()
      throw();

    //------------------------------------------------------------
    // IORB interfaces
    //------------------------------------------------------------
    /*!
     * @if jp
     * @brief ORB の名前を取得する
     *
     * ORB の名前を返す。
     * この関数は必ず "corba" を返す。
     *
     * @else
     * @brief Getting ORB's name
     *
     * This operation will return ORB's name.
     *
     * @endif
     */
    virtual const char* name()
      throw();

    virtual void run()
      throw();
    virtual void shutdown()
      throw();

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
    virtual ReturnCode_t
    registerFactory(const char* id,
                    doil::ServantNewFunc new_func,
                    doil::ServantDeleteFunc delete_func)
      throw();

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
    virtual ReturnCode_t activateObject(doil::ImplBase* impl)
      throw();

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
    virtual ReturnCode_t activateObject(doil::ImplBase* impl,
                                        doil::ServantBase* servant)
      throw();

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
    virtual ReturnCode_t deactivateObject(doil::ImplBase* impl)
      throw();

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
      throw();

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
    virtual ImplBase* getImpl(const char* name)
      throw();

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
    virtual ImplBase* toImpl(doil::ServantBase* servant)
      throw();

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
    virtual ServantBase* getServant(const char* name)
      throw();

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
    virtual ServantBase* toServant(doil::ImplBase* lobj)
      throw();

    //------------------------------------------------------------
    // IceManager interfaces
    //------------------------------------------------------------
    /*!
     * @if jp
     * @brief Objectを関連付けられたImplに変換する
     *
     * @else
     * @brief Convert Ice ObjectPrx to Impl related to it.
     *
     * @endif
     */
    doil::ImplBase* toImpl(::Ice::ObjectPrx obj)
      throw();

    /*!
     * @if jp
     * @brief 名前からオブジェクト参照を取得する
     *
     * @else
     * @brief Getting object reference from the given name
     *
     * @endif
     */
    ::Ice::ObjectPrx getReference(const char* name)
      throw();

    /*!
     * @if jp
     * @brief Implオブジェクトからオブジェクト参照へ変換する
     *
     * @else
     * @brief Converting Impl object to object reference
     *
     * @endif
     */
    ::Ice::ObjectPrx toReference(doil::ImplBase* impl)
      throw();

    /*!
     * @if jp
     * @brief Servantオブジェクトからオブジェクト参照へ変換する
     *
     * @else
     * @brief Converting Servant object to object reference
     *
     * @endif
     */
    ::Ice::ObjectPrx toReference(doil::ServantBase* servant)
      throw();

    /*!
     * @if jp
     * @brief ORBのポインタを取得する
     *
     * @else
     * @brief Getting ORB pointer
     *
     * @endif
     */
    doil::ServantBase* toServant(::Ice::ObjectPrx obj)
      throw();

    //------------------------------------------------------------
    // Ice functions
    //------------------------------------------------------------
    /*!
     * @if jp
     * @brief ORBのポインタを取得する
     *
     * @else
     * @brief Getting ORB pointer
     *
     * @endif
     */
    ::Ice::CommunicatorPtr getORB()
      throw();

    /*!
     * @if jp
     * @brief デフォルトPOAのポインタを取得する
     *
     * @else
     * @brief Getting default POA pointer
     *
     * @endif
     */
    ::Ice::ObjectAdapterPtr getAdapter()
      throw();

    //------------------------------------------------------------
    // protected functions:
    //------------------------------------------------------------
  protected:
    /*!
     * @if jp
     * @brief Ice ORB の初期化処理
     *
     * 引数により与えられた設定を元にORBを初期化する。
     *
     * @return ORB 初期化処理結果(初期化成功:true、初期化失敗:false)
     *
     * @else
     * @brief Ice ORB initialization
     *
     * Initialize ORB based on the configuration given by arguments.
     *
     * @return ORB initialization result (Successful:true, Failed:false)
     *
     * @endif
     */
    void initIce(coil::Properties prop);

    /*!
     * @if jp
     * @brief ORB のコマンドラインオプション作成
     *
     * コンフィギュレーション情報に設定された内容から
     * ORB の起動時オプションを作成する。
     *
     * @return ORB 起動時オプション
     *
     * @else
     * @brief Create ORB command options
     *
     * Create ORB launch options from configuration information
     * that has been set.
     *
     * @return ORB launch options
     *
     * @endif
     */
    std::string createIceOptions();
    
    //------------------------------------------------------------
    // private functions:
    //------------------------------------------------------------
  private:
    // private constructor
    IceManager(){}
    // private big three
    IceManager(const IceManager& cmgr);
    IceManager& operator=(const IceManager& rhs);
    ~IceManager() throw() {}

    //------------------------------------------------------------
    // private data:
    //------------------------------------------------------------
  private:
    // static data member for singleton
    static IceManager* manager;
    static coil::Mutex mutex;

    // Ice communicator pointer
    ::Ice::CommunicatorPtr m_ice;

    // Ice object adapter pointer
    ::Ice::ObjectAdapterPtr m_adapter;

    // configuration properties
    coil::Properties m_config;


    // Entry class for active object map
    class Entry
    {
    public:
      Entry(ImplBase* impl, IceServantBase* servant,
            ::Ice::ObjectPrx objref)
        : impl_(impl), servant_(servant), objref_(objref)
      {
      }
      virtual ~Entry()
      {
      }
      ImplBase* impl_;
      IceServantBase* servant_;
      ::Ice::ObjectPrx objref_;
    };

    // Predicate class for active object map
    class EntryPred
    {
    public:
      EntryPred(const char* name)
        : m_name(name)
      {
      }
      EntryPred(Entry* entry)
        : m_name(entry->impl_->name())
      {
      }
      bool operator()(Entry* entry)
      {
        return m_name == entry->impl_->name();
      }
      std::string m_name;
    };
    
    // Active object map
    typedef ObjectManager<const char*, Entry, EntryPred> ObjectMap;
    ObjectMap m_map;

    // Predicate functor for Factory map
    class FactoryPred
    {
    public:
      FactoryPred(const char* id) : m_id(id) {}
      FactoryPred(ServantFactory* factory) : m_id(factory->id()) {}
      bool operator()(ServantFactory* factory)
      {
        return m_id == factory->id();
      }
      std::string m_id;
    };

    // Servant factory map
    ObjectManager<const char*, ServantFactory, FactoryPred> m_factory;


    class find_by_obj
    {
    public:
      find_by_obj(::Ice::ObjectPrx obj) : result_(NULL), m_obj(obj) {};
      void operator()(IceManager::Entry* entry)
      {
        if (m_obj == (entry->objref_))
          {
            result_ = entry;
          }
      }
    public:
      Entry* result_;
    private:
      ::Ice::ObjectPrx m_obj;
    };


  };
};
};

extern "C"
{
  void DoilIceInit(coil::Properties& prop);
}
#endif // RTM_Ice_IceMANAGER_H
