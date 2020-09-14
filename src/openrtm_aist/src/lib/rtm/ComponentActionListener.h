// -*- C++ -*-
/*!
 * @file ComponentActionListener.h
 * @brief component action listener class
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2011
 *     Noriaki Ando
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef RTC_COMPONENTACTIONLISTENER_H
#define RTC_COMPONENTACTIONLISTENER_H

#include <vector>
#include <utility>
#include <coil/Mutex.h>
#include <coil/Guard.h>
#include <rtm/RTC.h>
#include <rtm/idl/RTCSkel.h>
#include <rtm/ConnectorBase.h>

namespace RTC
{
  typedef ExecutionContextHandle_t UniqueId;
  //============================================================
  /*!
   * @if jp
   * @brief PreComponentActionListener のタイプ
   *
   * - PRE_ON_INITIALIZE:    onInitialize 直前
   * - PRE_ON_FINALIZE:      onFinalize 直前
   * - PRE_ON_STARTUP:       onStartup 直前
   * - PRE_ON_SHUTDOWN:      onShutdown 直前
   * - PRE_ON_ACTIVATED:     onActivated 直前
   * - PRE_ON_DEACTIVATED:   onDeactivated 直前
   * - PRE_ON_ABORTING:      onAborted 直前
   * - PRE_ON_ERROR:         onError 直前
   * - PRE_ON_RESET:         onReset 直前
   * - PRE_ON_EXECUTE:       onExecute 直前
   * - PRE_ON_STATE_UPDATE:  onStateUpdate 直前
   * - PRE_ON_RATE_CHANGED:  onRateChanged 直前
   *
   * @else
   * @brief The types of ConnectorDataListener
   * 
   * @endif
   */

  enum PreComponentActionListenerType
    {
      PRE_ON_INITIALIZE,
      PRE_ON_FINALIZE,
      PRE_ON_STARTUP,
      PRE_ON_SHUTDOWN,
      PRE_ON_ACTIVATED,
      PRE_ON_DEACTIVATED,
      PRE_ON_ABORTING,
      PRE_ON_ERROR,
      PRE_ON_RESET,
      PRE_ON_EXECUTE,
      PRE_ON_STATE_UPDATE,
      PRE_ON_RATE_CHANGED,
      PRE_COMPONENT_ACTION_LISTENER_NUM
    };

  /*!
   * @if jp
   * @class PreComponentActionListener クラス
   * @brief PreComponentActionListener クラス
   *
   * OMG RTC仕様で定義されている以下のコンポーネントアクショントについ
   * て、
   *
   * - on_initialize()
   * - on_finalize()
   * - on_startup()
   * - on_shutdown()
   * - on_activated
   * - on_deactivated()
   * - on_aborted()
   * - on_error()
   * - on_reset()
   * - on_execute()
   * - on_state_update()
   * - on_rate_changed()
   *
   * 各アクションに対応するユーザーコードが呼ばれる直前のタイミング
   * でコールされるリスナクラスの基底クラス。
   *
   * - PRE_ON_INITIALIZE:
   * - PRE_ON_FINALIZE:
   * - PRE_ON_STARTUP:
   * - PRE_ON_SHUTDOWN:
   * - PRE_ON_ACTIVATED:
   * - PRE_ON_DEACTIVATED:
   * - PRE_ON_ABORTING:
   * - PRE_ON_ERROR:
   * - PRE_ON_RESET:
   * - PRE_IN_EXECUTE:
   * - PRE_ON_STATE_UPDATE:
   * - PRE_ON_RATE_CHANGED:
   *
   * @else
   * @class PreComponentActionListener class
   * @brief PreComponentActionListener class
   *
   * This class is abstract base class for listener classes that
   * provides callbacks for various events in rtobject.
   *
   * @endif
   */
  class PreComponentActionListener
  {
  public:
    /*!
     * @if jp
     *
     * @brief PreComponentActionListenerType を文字列に変換
     *
     * PreComponentActionListenerType を文字列に変換する
     *
     * @param type 変換対象 PreComponentActionListenerType
     *
     * @return 文字列変換結果
     *
     * @else
     *
     * @brief Convert PreComponentActionListenerType into the string.
     *
     * Convert PreComponentActionListenerType into the string.
     *
     * @param type The target PreComponentActionListenerType for transformation
     *
     * @return Trnasformation result of string representation
     *
     * @endif
     */
    static const char* toString(PreComponentActionListenerType type)
    {
      static const char* typeString[] =
        {
          "PRE_ON_INITIALIZE",
          "PRE_ON_FINALIZE",
          "PRE_ON_STARTUP",
          "PRE_ON_SHUTDOWN",
          "PRE_ON_ACTIVATED",
          "PRE_ON_DEACTIVATED",
          "PRE_ON_ABORTING",
          "PRE_ON_ERROR",
          "PRE_ON_RESET",
          "PRE_ON_EXECUTE",
          "PRE_ON_STATE_UPDATE",
          "PRE_ON_RATE_CHANGED",
          "PRE_COMPONENT_ACTION_LISTENER_NUM"
        };
      if (type < PRE_COMPONENT_ACTION_LISTENER_NUM) { return typeString[type]; }
      return "";
    }

    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~PreComponentActionListener();

    /*!
     * @if jp
     *
     * @brief 仮想コールバック関数
     *
     * PreComponentActionListener のコールバック関数
     *
     * @else
     *
     * @brief Virtual Callback function
     *
     * This is a the Callback function for PreComponentActionListener.
     *
     * @endif
     */
    virtual void operator()(UniqueId ec_id) = 0;
  };


  //============================================================
  /*!
   * @if jp
   * @brief PostCompoenntActionListener のタイプ
   *
   * - POST_ON_INITIALIZE:
   * - POST_ON_FINALIZE:
   * - POST_ON_STARTUP:
   * - POST_ON_SHUTDOWN:
   * - POST_ON_ACTIVATED:
   * - POST_ON_DEACTIVATED:
   * - POST_ON_ABORTING:
   * - POST_ON_ERROR:
   * - POST_ON_RESET:
   * - POST_ON_EXECUTE:
   * - POST_ON_STATE_UPDATE:
   * - POST_ON_RATE_CHANGED:
   *
   * @else
   * @brief The types of ConnectorDataListener
   * 
   * @endif
   */
  enum PostComponentActionListenerType
    {
      POST_ON_INITIALIZE,
      POST_ON_FINALIZE,
      POST_ON_STARTUP,
      POST_ON_SHUTDOWN,
      POST_ON_ACTIVATED,
      POST_ON_DEACTIVATED,
      POST_ON_ABORTING,
      POST_ON_ERROR,
      POST_ON_RESET,
      POST_ON_EXECUTE,
      POST_ON_STATE_UPDATE,
      POST_ON_RATE_CHANGED,
      POST_COMPONENT_ACTION_LISTENER_NUM
    };


  /*!
   * @if jp
   * @class PostComponentActionListener クラス
   * @brief PostComponentActionListener クラス
   *
   * OMG RTC仕様で定義されている以下のコンポーネントアクショントについ
   * て、
   *
   * - on_initialize()
   * - on_finalize()
   * - on_startup()
   * - on_shutdown()
   * - on_activated
   * - on_deactivated()
   * - on_aborted()
   * - on_error()
   * - on_reset()
   * - on_execute()
   * - on_state_update()
   * - on_rate_changed()
   *
   * 各アクションに対応するユーザーコードが呼ばれる直前のタイミング
   * でコールされるリスなクラスの基底クラス。
   *
   * - POST_ON_INITIALIZE:
   * - POST_ON_FINALIZE:
   * - POST_ON_STARTUP:
   * - POST_ON_SHUTDOWN:
   * - POST_ON_ACTIVATED:
   * - POST_ON_DEACTIVATED:
   * - POST_ON_ABORTING:
   * - POST_ON_ERROR:
   * - POST_ON_RESET:
   * - POST_ON_EXECUTE:
   * - POST_ON_STATE_UPDATE:
   * - POST_ON_RATE_CHANGED:
   *
   * @else
   * @class PostComponentActionListener class
   * @brief PostComponentActionListener class
   *
   * This class is abstract base class for listener classes that
   * provides callbacks for various events in rtobject.
   *
   * @endif
   */
  class PostComponentActionListener
  {
  public:
    /*!
     * @if jp
     *
     * @brief PostComponentActionListenerType を文字列に変換
     *
     * PostComponentActionListenerType を文字列に変換する
     *
     * @param type 変換対象 PostComponentActionListenerType
     *
     * @return 文字列変換結果
     *
     * @else
     *
     * @brief Convert PostComponentActionListenerType into the string.
     *
     * Convert PostComponentActionListenerType into the string.
     *
     * @param type The target PostComponentActionListenerType for transformation
     *
     * @return Trnasformation result of string representation
     *
     * @endif
     */
    static const char* toString(PostComponentActionListenerType type)
    {
      static const char* typeString[] =
        {
          "POST_ON_INITIALIZE",
          "POST_ON_FINALIZE",
          "POST_ON_STARTUP",
          "POST_ON_SHUTDOWN",
          "POST_ON_ACTIVATED",
          "POST_ON_DEACTIVATED",
          "POST_ON_ABORTING",
          "POST_ON_ERROR",
          "POST_ON_RESET",
          "POST_ON_EXECUTE",
          "POST_ON_STATE_UPDATE",
          "POST_ON_RATE_CHANGED",
          "POST_COMPONENT_ACTION_LISTENER_NUM"
        };
      if (type < POST_COMPONENT_ACTION_LISTENER_NUM)
        {
          return typeString[type];
        }
      return "";
    }

    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~PostComponentActionListener();

    /*!
     * @if jp
     *
     * @brief 仮想コールバック関数
     *
     * PostComponentActionListener のコールバック関数
     *
     * @else
     *
     * @brief Virtual Callback function
     *
     * This is a the Callback function for PostComponentActionListener.
     *
     * @endif
     */
    virtual void operator()(UniqueId ec_id,
                            ReturnCode_t ret) = 0;
  };

  //============================================================
  /*!
   * @if jp
   * @brief PortActionListener のタイプ
   *
   * - ADD_PORT:             Port 追加時
   * - REMOVE_PORT:          Port 削除時
   *
   * @else
   * @brief The types of PortActionListener
   * 
   * @endif
   */

  enum PortActionListenerType
    {
      ADD_PORT,
      REMOVE_PORT,
      PORT_ACTION_LISTENER_NUM
    };

  /*!
   * @if jp
   * @class PortActionListener クラス
   * @brief PortActionListener クラス
   *
   * 各アクションに対応するユーザーコードが呼ばれる直前のタイミング
   * でコールされるリスなクラスの基底クラス。
   *
   * - ADD_PORT:
   * - REMOVE_PORT:
   *
   * @else
   * @class PortActionListener class
   * @brief PortActionListener class
   *
   * This class is abstract base class for listener classes that
   * provides callbacks for various events in rtobject.
   *
   * @endif
   */
  class PortActionListener
  {
  public:
    /*!
     * @if jp
     *
     * @brief PreComponentActionListenerType を文字列に変換
     *
     * PreComponentActionListenerType を文字列に変換する
     *
     * @param type 変換対象 PreComponentActionListenerType
     *
     * @return 文字列変換結果
     *
     * @else
     *
     * @brief Convert PreComponentActionListenerType into the string.
     *
     * Convert PreComponentActionListenerType into the string.
     *
     * @param type The target PreComponentActionListenerType for transformation
     *
     * @return Trnasformation result of string representation
     *
     * @endif
     */
    static const char* toString(PortActionListenerType type)
    {
      static const char* typeString[] =
        {
          "ADD_PORT",
          "REMOVE_PORT",
          "PORT_ACTION_LISTENER_NUM"
        };
      if (type < PORT_ACTION_LISTENER_NUM) { return typeString[type]; }
      return "";
    }

    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~PortActionListener();

    /*!
     * @if jp
     *
     * @brief 仮想コールバック関数
     *
     * PortActionListener のコールバック関数
     *
     * @else
     *
     * @brief Virtual Callback function
     *
     * This is a the Callback function for PortActionListener
     *
     * @endif
     */
    virtual void operator()(const ::RTC::PortProfile& pprof) = 0;
  };


  //============================================================
  /*!
   * @if jp
   * @brief ExecutionContextActionListener のタイプ
   *
   * - ADD_PORT:             ExecutionContext 追加時
   * - REMOVE_PORT:          ExecutionContext 削除時
   *
   * @else
   * @brief The types of ExecutionContextActionListener
   * 
   * @endif
   */

  enum ExecutionContextActionListenerType
    {
      EC_ATTACHED,
      EC_DETACHED,
      EC_ACTION_LISTENER_NUM
    };

  /*!
   * @if jp
   * @class ExecutionContextActionListener クラス
   * @brief ExecutionContextActionListener クラス
   *
   * 各アクションに対応するユーザーコードが呼ばれる直前のタイミング
   * でコールされるリスなクラスの基底クラス。
   *
   * - ADD_PORT:
   * - REMOVE_PORT:
   *
   * @else
   * @class ExecutionContextActionListener class
   * @brief ExecutionContextActionListener class
   *
   * This class is abstract base class for listener classes that
   * provides callbacks for various events in rtobject.
   *
   * @endif
   */
  class ExecutionContextActionListener
  {
  public:
    /*!
     * @if jp
     *
     * @brief PreComponentActionListenerType を文字列に変換
     *
     * PreComponentActionListenerType を文字列に変換する
     *
     * @param type 変換対象 PreComponentActionListenerType
     *
     * @return 文字列変換結果
     *
     * @else
     *
     * @brief Convert PreComponentActionListenerType into the string.
     *
     * Convert PreComponentActionListenerType into the string.
     *
     * @param type The target PreComponentActionListenerType for transformation
     *
     * @return Trnasformation result of string representation
     *
     * @endif
     */
    static const char* toString(ExecutionContextActionListenerType type)
    {
      static const char* typeString[] =
        {
          "ATTACH_EC",
          "DETACH_ECT",
          "EC_ACTION_LISTENER_NUM"
        };
      if (type < EC_ACTION_LISTENER_NUM) { return typeString[type]; }
      return "";
    }

    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~ExecutionContextActionListener();

    /*!
     * @if jp
     *
     * @brief 仮想コールバック関数
     *
     * ExecutionContextActionListener のコールバック関数
     *
     * @else
     *
     * @brief Virtual Callback function
     *
     * This is a the Callback function for ExecutionContextActionListener
     *
     * @endif
     */
    virtual void operator()(UniqueId ec_id) = 0;
  };



  //============================================================
  /*!
   * @if jp
   * @class PreComponentActionListenerHolder 
   * @brief PreComponentActionListener ホルダクラス
   *
   * 複数の PreComponentActionListener を保持し管理するクラス。
   *
   * @else
   * @class PreComponentActionListenerHolder
   * @brief PreComponentActionListener holder class
   *
   * This class manages one ore more instances of
   * PreComponentActionListener class.
   *
   * @endif
   */
  class PreComponentActionListenerHolder
  {
    typedef std::pair<PreComponentActionListener*, bool> Entry;
    typedef coil::Guard<coil::Mutex> Guard;
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     * @else
     * @brief Constructor
     * @endif
     */
    PreComponentActionListenerHolder();
    
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~PreComponentActionListenerHolder();
    
    /*!
     * @if jp
     *
     * @brief リスナーの追加
     *
     * リスナーを追加する。
     *
     * @param listener 追加するリスナ
     * @param autoclean true:デストラクタで削除する,
     *                  false:デストラクタで削除しない
     * @else
     *
     * @brief Add the listener.
     *
     * This method adds the listener. 
     *
     * @param listener Added listener
     * @param autoclean true:The listener is deleted at the destructor.,
     *                  false:The listener is not deleted at the destructor. 
     * @endif
     */
    void addListener(PreComponentActionListener* listener, bool autoclean);
    
    /*!
     * @if jp
     *
     * @brief リスナーの削除
     *
     * リスナを削除する。
     *
     * @param listener 削除するリスナ
     * @else
     *
     * @brief Remove the listener. 
     *
     * This method removes the listener. 
     *
     * @param listener Removed listener
     * @endif
     */
    void removeListener(PreComponentActionListener* listener);

    /*!
     * @if jp
     *
     * @brief リスナーへ通知する
     *
     * 登録されているリスナのコールバックメソッドを呼び出す。
     *
     * @param info ConnectorInfo
     * @else
     *
     * @brief Notify listeners. 
     *
     * This calls the Callback method of the registered listener. 
     *
     * @param info ConnectorInfo
     * @endif
     */
    void notify(UniqueId ec_id);
      
  private:
    std::vector<Entry> m_listeners;
    coil::Mutex m_mutex;
  };


  /*!
   * @if jp
   * @class PostComponentActionListenerHolder
   * @brief PostComponentActionListener ホルダクラス
   *
   * 複数の PostComponentActionListener を保持し管理するクラス。
   *
   * @else
   * @class PostComponentActionListenerHolder
   * @brief PostComponentActionListener holder class
   *
   * This class manages one ore more instances of
   * PostComponentActionListener class.
   *
   * @endif
   */
  class PostComponentActionListenerHolder
  {
    typedef std::pair<PostComponentActionListener*, bool> Entry;
    typedef coil::Guard<coil::Mutex> Guard;
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     * @else
     * @brief Constructor
     * @endif
     */
    PostComponentActionListenerHolder();
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~PostComponentActionListenerHolder();
    
    /*!
     * @if jp
     *
     * @brief リスナーの追加
     *
     * リスナーを追加する。
     *
     * @param listener 追加するリスナ
     * @param autoclean true:デストラクタで削除する,
     *                  false:デストラクタで削除しない
     * @else
     *
     * @brief Add the listener.
     *
     * This method adds the listener. 
     *
     * @param listener Added listener
     * @param autoclean true:The listener is deleted at the destructor.,
     *                  false:The listener is not deleted at the destructor. 
     * @endif
     */
    void addListener(PostComponentActionListener* listener, bool autoclean);
    
    /*!
     * @if jp
     *
     * @brief リスナーの削除
     *
     * リスナを削除する。
     *
     * @param listener 削除するリスナ
     * @else
     *
     * @brief Remove the listener. 
     *
     * This method removes the listener. 
     *
     * @param listener Removed listener
     * @endif
     */
    void removeListener(PostComponentActionListener* listener);
    
    /*!
     * @if jp
     *
     * @brief リスナーへ通知する
     *
     * 登録されているリスナのコールバックメソッドを呼び出す。
     *
     * @param info ConnectorInfo
     * @param cdrdata データ
     * @else
     *
     * @brief Notify listeners. 
     *
     * This calls the Callback method of the registered listener. 
     *
     * @param info ConnectorInfo
     * @param cdrdata Data
     * @endif
     */
    void notify(UniqueId ec_id, ReturnCode_t ret);
    
  private:
    std::vector<Entry> m_listeners;
    coil::Mutex m_mutex;
  };


  //============================================================
  /*!
   * @if jp
   * @class PortActionListenerHolder
   * @brief PortActionListener ホルダクラス
   *
   * 複数の PortActionListener を保持し管理するクラス。
   *
   * @else
   * @class PortActionListenerHolder
   * @brief PortActionListener holder class
   *
   * This class manages one ore more instances of
   * PortActionListener class.
   *
   * @endif
   */
  class PortActionListenerHolder
  {
    typedef std::pair<PortActionListener*, bool> Entry;
    typedef coil::Guard<coil::Mutex> Guard;
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     * @else
     * @brief Constructor
     * @endif
     */
    PortActionListenerHolder();
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~PortActionListenerHolder();
    
    /*!
     * @if jp
     *
     * @brief リスナーの追加
     *
     * リスナーを追加する。
     *
     * @param listener 追加するリスナ
     * @param autoclean true:デストラクタで削除する,
     *                  false:デストラクタで削除しない
     * @else
     *
     * @brief Add the listener.
     *
     * This method adds the listener. 
     *
     * @param listener Added listener
     * @param autoclean true:The listener is deleted at the destructor.,
     *                  false:The listener is not deleted at the destructor. 
     * @endif
     */
    void addListener(PortActionListener* listener, bool autoclean);
    
    /*!
     * @if jp
     *
     * @brief リスナーの削除
     *
     * リスナを削除する。
     *
     * @param listener 削除するリスナ
     * @else
     *
     * @brief Remove the listener. 
     *
     * This method removes the listener. 
     *
     * @param listener Removed listener
     * @endif
     */
    void removeListener(PortActionListener* listener);
    
    /*!
     * @if jp
     *
     * @brief リスナーへ通知する
     *
     * 登録されているリスナのコールバックメソッドを呼び出す。
     *
     * @param info ConnectorInfo
     * @param cdrdata データ
     * @else
     *
     * @brief Notify listeners. 
     *
     * This calls the Callback method of the registered listener. 
     *
     * @param info ConnectorInfo
     * @param cdrdata Data
     * @endif
     */
    void notify(const RTC::PortProfile& pprofile);
    
  private:
    std::vector<Entry> m_listeners;
    coil::Mutex m_mutex;
  };

  /*!
   * @if jp
   * @class ExecutionContextActionListenerHolder
   * @brief ExecutionContextActionListener ホルダクラス
   *
   * 複数の ExecutionContextActionListener を保持し管理するクラス。
   *
   * @else
   * @class ExecutionContextActionListenerHolder
   * @brief ExecutionContextActionListener holder class
   *
   * This class manages one ore more instances of
   * ExecutionContextActionListener class.
   *
   * @endif
   */
  class ExecutionContextActionListenerHolder
  {
    typedef std::pair<ExecutionContextActionListener*, bool> Entry;
    typedef coil::Guard<coil::Mutex> Guard;
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     * @else
     * @brief Constructor
     * @endif
     */
    ExecutionContextActionListenerHolder();
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~ExecutionContextActionListenerHolder();
    
    /*!
     * @if jp
     *
     * @brief リスナーの追加
     *
     * リスナーを追加する。
     *
     * @param listener 追加するリスナ
     * @param autoclean true:デストラクタで削除する,
     *                  false:デストラクタで削除しない
     * @else
     *
     * @brief Add the listener.
     *
     * This method adds the listener. 
     *
     * @param listener Added listener
     * @param autoclean true:The listener is deleted at the destructor.,
     *                  false:The listener is not deleted at the destructor. 
     * @endif
     */
    void addListener(ExecutionContextActionListener* listener, bool autoclean);
    
    /*!
     * @if jp
     *
     * @brief リスナーの削除
     *
     * リスナを削除する。
     *
     * @param listener 削除するリスナ
     * @else
     *
     * @brief Remove the listener. 
     *
     * This method removes the listener. 
     *
     * @param listener Removed listener
     * @endif
     */
    void removeListener(ExecutionContextActionListener* listener);
    
    /*!
     * @if jp
     *
     * @brief リスナーへ通知する
     *
     * 登録されているリスナのコールバックメソッドを呼び出す。
     *
     * @param info ConnectorInfo
     * @param cdrdata データ
     * @else
     *
     * @brief Notify listeners. 
     *
     * This calls the Callback method of the registered listener. 
     *
     * @param info ConnectorInfo
     * @param cdrdata Data
     * @endif
     */
    void notify(UniqueId ec_id);
    
  private:
    std::vector<Entry> m_listeners;
    coil::Mutex m_mutex;
  };


  /*!
   * @if jp
   * @class ComponentActionListeners
   * @brief ComponentActionListeners クラス
   *
   *
   * @else
   * @class ComponentActionListeners
   * @brief ComponentActionListeners class
   *
   *
   * @endif
   */
  class ComponentActionListeners
  {
  public:
    /*!
     * @if jp
     * @brief PreComponentActionListenerTypeリスナ配列
     * PreComponentActionListenerTypeリスナを格納
     * @else
     * @brief PreComponentActionListenerType listener array
     * The PreComponentActionListenerType listener is stored. 
     * @endif
     */
    PreComponentActionListenerHolder 
    preaction_[PRE_COMPONENT_ACTION_LISTENER_NUM];
    /*!
     * @if jp
     * @brief PostComponentActionTypeリスナ配列
     * PostComponentActionTypeリスナを格納
     * @else
     * @brief PostComponentActionType listener array
     * The PostComponentActionType listener is stored.
     * @endif
     */
    PostComponentActionListenerHolder 
    postaction_[POST_COMPONENT_ACTION_LISTENER_NUM];
    /*!
     * @if jp
     * @brief PortActionTypeリスナ配列
     * PortActionTypeリスナを格納
     * @else
     * @brief PortActionType listener array
     * The PortActionType listener is stored.
     * @endif
     */
    PortActionListenerHolder
    portaction_[PORT_ACTION_LISTENER_NUM];
    /*!
     * @if jp
     * @brief ExecutionContextActionTypeリスナ配列
     * ExecutionContextActionTypeリスナを格納
     * @else
     * @brief ExecutionContextActionType listener array
     * The ExecutionContextActionType listener is stored.
     * @endif
     */
   ExecutionContextActionListenerHolder
    ecaction_[EC_ACTION_LISTENER_NUM];
  };


}; // namespace RTC

#endif // RTC_COMPONENTACTIONLISTENER_H
