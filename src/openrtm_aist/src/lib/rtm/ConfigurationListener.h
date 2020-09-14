// -*- C++ -*-
/*!
 * @file ConfigurationListener.h
 * @brief Configuration related event listener classes
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

#ifndef RTC_CONFIGURATIONLISTENER_H
#define RTC_CONFIGURATIONLISTENER_H

#include <vector>
#include <coil/Properties.h>
#include <coil/Mutex.h>
#include <coil/Guard.h>

namespace RTC
{
  //============================================================
  /*!
   * @if jp
   * @brief ConfigurationParamListener のタイプ
   *
   * - ON_UPDATE_CONFIG_PARAM,
   *
   * @else
   * @brief The types of ConnectorDataListener
   * 
   * - ON_UPDATE_CONFIG_PARAM,
   *
   * @endif
   */
  enum ConfigurationParamListenerType
    {
      ON_UPDATE_CONFIG_PARAM,
      CONFIG_PARAM_LISTENER_NUM
    };


  /*!
   * @if jp
   * @class ConfigurationParamListener クラス
   * @brief ConfigurationParamListener クラス
   *
   * Configuration パラメータの変更に関するリスナクラス。
   * 以下のイベントに対してコールバックされる。
   *
   * - ON_UPDATE_CONFIG_PARAM
   *
   * @else
   * @class ConfigurationParamListener class
   * @brief ConfigurationParamListener class
   *
   * This class is abstract base class for listener classes that
   * provides callbacks for various events for Configuration parameter.
   * The listener will be called on the following event.
   *
   * - ON_UPDATE_CONFIG_PARAM
   *
   * @endif
   */
  class ConfigurationParamListener
  {
  public:
    /*!
     * @if jp
     *
     * @brief ConfigurationParamListenerType を文字列に変換
     *
     * ConfigurationParamListenerType を文字列に変換する
     *
     * @param type 変換対象 ConfigurationParamListenerType
     *
     * @return 文字列変換結果
     *
     * @else
     *
     * @brief Convert ConfigurationParamListenerType into the string.
     *
     * Convert ConfigurationParamListenerType into the string.
     *
     * @param type The target ConfigurationParamListenerType for transformation
     *
     * @return Trnasformation result of string representation
     *
     * @endif
     */
    static const char* toString(ConfigurationParamListenerType type)
    {
      static const char* typeString[] =
        {
          "ON_UPDATE_CONFIG_PARAM",
          "CONFIG_PARAM_LISTENER_NUM"
        };
      if (type < CONFIG_PARAM_LISTENER_NUM)
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
    virtual ~ConfigurationParamListener();

    /*!
     * @if jp
     *
     * @brief 仮想コールバック関数
     *
     * ConfigurationParamListener のコールバック関数
     *
     * @else
     *
     * @brief Virtual Callback function
     *
     * This is a the Callback function for ConfigurationParamListener.
     *
     * @endif
     */
    virtual void operator()(const char* config_set_name,
                            const char* config_param_name) = 0;
  };


  //============================================================
  /*!
   * @if jp
   * @brief ConfigurationSetListener のタイプ
   *
   * - ON_SET_CONFIG_SET: ConfigurationSet 単位で値がセットされた
   * - ON_ADD_CONFIG_SET: ConfigurationSet が追加された
   *
   * @else
   * @brief The types of ConfigurationSetListener
   * 
   * - ON_SET_CONFIG_SET: Value list has been set as a configuration set 
   * - ON_ADD_CONFIG_SET: A new configuration set has been added
   *
   * @endif
   */

  enum ConfigurationSetListenerType
    {
      ON_SET_CONFIG_SET,
      ON_ADD_CONFIG_SET,
      CONFIG_SET_LISTENER_NUM
    };

  /*!
   * @if jp
   * @class ConfigurationSetListener クラス
   * @brief ConfigurationSetListener クラス
   *
   * Configurationセットが変更されたり追加された場合に呼び出されるリスナクラス。
   * 以下のConfigurationセットに関連するイベントに対するリスナ。
   *
   * - ON_SET_CONFIG_SET: ConfigurationSet 単位で値がセットされた
   * - ON_ADD_CONFIG_SET: ConfigurationSet が追加された
   *
   * @else
   * @class ConfigurationSetListener class
   * @brief ConfigurationSetListener class
   *
   * This class is abstract base class for listener classes that
   * provides callbacks for configuration set's related events.
   *
   * - ON_SET_CONFIG_SET: Value list has been set as a configuration set 
   * - ON_ADD_CONFIG_SET: A new configuration set has been added
   *
   * @endif
   */
  class ConfigurationSetListener
  {
  public:
    /*!
     * @if jp
     *
     * @brief ConfigurationSetNameListenerType を文字列に変換
     *
     * ConfigurationSetNameListenerType を文字列に変換する
     *
     * @param type 変換対象 ConfigurationSetNameListenerType
     *
     * @return 文字列変換結果
     *
     * @else
     *
     * @brief Convert ConfigurationSetNameListenerType into the string.
     *
     * Convert ConfigurationSetNameListenerType into the string.
     *
     * @param type The target ConfigurationSetNameListenerType for
     *             transformation
     *
     * @return Trnasformation result of string representation
     *
     * @endif
     */
    static const char* toString(ConfigurationSetListenerType type)
    {
      static const char* typeString[] =
        {
          "ON_SET_CONFIG_SET",
          "ON_ADD_CONFIG_SET",
          "CONFIG_SET_LISTENER_NUM"
        };
      if (type < CONFIG_SET_LISTENER_NUM) { return typeString[type]; }
      return "";
    }

    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~ConfigurationSetListener();

    /*!
     * @if jp
     *
     * @brief 仮想コールバック関数
     *
     * ConfigurationSetListener のコールバック関数
     *
     * @else
     *
     * @brief Virtual Callback function
     *
     * This is a the Callback function for ConfigurationSetListener
     *
     * @endif
     */
    virtual void operator()(const coil::Properties& config_set) = 0;
  };


  //============================================================
  /*!
   * @if jp
   * @brief ConfigurationSetNameListenerType
   *
   *
   * @else
   * @brief The types of ConfigurationSetNameListener
   * 
   * @endif
   */
  enum ConfigurationSetNameListenerType
    {
      ON_UPDATE_CONFIG_SET,
      ON_REMOVE_CONFIG_SET,
      ON_ACTIVATE_CONFIG_SET,
      CONFIG_SET_NAME_LISTENER_NUM
    };

  /*!
   * @if jp
   * @class ConfigurationSetNameListener クラス
   * @brief ConfigurationSetNameListener クラス
   *
   * ConfigurationSetに関するイベントに関するリスナークラス。
   *
   * - ON_UPDATE_CONFIG_SET:
   * - ON_REMOVE_CONFIG_SET:
   * - ON_ACTIVATE_CONFIG_SET:
   *
   * @else
   * @class ConfigurationSetNameListener class
   * @brief ConfigurationSetNameListener class
   *
   * This class is abstract base class for listener classes that
   * provides callbacks for various events for ConfigurationSet.
   *
   * - ON_UPDATE_CONFIG_SET:
   * - ON_REMOVE_CONFIG_SET:
   * - ON_ACTIVATE_CONFIG_SET:
   *
   * @endif
   */
  class ConfigurationSetNameListener
  {
  public:
    /*!
     * @if jp
     *
     * @brief ConfigurationSetNameListenerType を文字列に変換
     *
     * ConfigurationSetNameListenerType を文字列に変換する
     *
     * @param type 変換対象 ConfigurationSetNameListenerType
     *
     * @return 文字列変換結果
     *
     * @else
     *
     * @brief Convert ConfigurationSetNameListenerType into the string.
     *
     * Convert ConfigurationSetNameListenerType into the string.
     *
     * @param type The target ConfigurationSetNameListenerType for
     *             transformation
     *
     * @return Trnasformation result of string representation
     *
     * @endif
     */
    static const char* toString(ConfigurationSetNameListenerType type)
    {
      static const char* typeString[] =
        {
          "ON_UPDATE_CONFIG_SET",
          "ON_REMOVE_CONFIG_SET",
          "ON_ACTIVATE_CONFIG_SET",
          "CONFIG_SET_NAME_LISTENER_NUM"
        };
      if (type < CONFIG_SET_NAME_LISTENER_NUM) { return typeString[type]; }
      return "";
    }

    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~ConfigurationSetNameListener();

    /*!
     * @if jp
     *
     * @brief 仮想コールバック関数
     *
     * ConfigurationSetNameListener のコールバック関数
     *
     * @else
     *
     * @brief Virtual Callback function
     *
     * This is a the Callback function for ConfigurationSetNameListener.
     *
     * @endif
     */
    virtual void operator()(const char* config_set_name) = 0;
  };


  /*!
   * @if jp
   * @class ConfigurationParamListenerHolder
   * @brief ConfigurationParamListener ホルダクラス
   *
   * 複数の ConfigurationParamListener を保持し管理するクラス。
   *
   * @else
   * @class ConfigurationParamListenerHolder
   * @brief ConfigurationParamListener holder class
   *
   * This class manages one ore more instances of
   * ConfigurationParamListener class.
   *
   * @endif
   */
  class ConfigurationParamListenerHolder
  {
    typedef std::pair<ConfigurationParamListener*, bool> Entry;
    typedef coil::Guard<coil::Mutex> Guard;
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     * @else
     * @brief Constructor
     * @endif
     */
    ConfigurationParamListenerHolder();
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~ConfigurationParamListenerHolder();
    
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
    void addListener(ConfigurationParamListener* listener, bool autoclean);
    
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
    void removeListener(ConfigurationParamListener* listener);
    
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
    void notify(const char* config_set_name, const char* config_param_name);
    
  private:
    std::vector<Entry> m_listeners;
    coil::Mutex m_mutex;
  };


  //============================================================
  /*!
   * @if jp
   * @class ConfigurationSetListenerHolder
   * @brief ConfigurationSetListener ホルダクラス
   *
   * 複数の ConfigurationSetListener を保持し管理するクラス。
   *
   * @else
   * @class ConfigurationSetListenerHolder
   * @brief ConfigurationSetListener holder class
   *
   * This class manages one ore more instances of
   * ConfigurationSetListener class.
   *
   * @endif
   */
  class ConfigurationSetListenerHolder
  {
    typedef std::pair<ConfigurationSetListener*, bool> Entry;
    typedef coil::Guard<coil::Mutex> Guard;
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     * @else
     * @brief Constructor
     * @endif
     */
    ConfigurationSetListenerHolder();
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~ConfigurationSetListenerHolder();
    
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
    void addListener(ConfigurationSetListener* listener, bool autoclean);
    
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
    void removeListener(ConfigurationSetListener* listener);
    
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
    void notify(const coil::Properties& config_set);
    
  private:
    std::vector<Entry> m_listeners;
    coil::Mutex m_mutex;
  };


  //============================================================
  /*!
   * @if jp
   * @class ConfigurationSetNameListenerHolder 
   * @brief ConfigurationSetNameListener ホルダクラス
   *
   * 複数の ConfigurationSetNameListener を保持し管理するクラス。
   *
   * @else
   * @class ConfigurationSetNameListenerHolder
   * @brief ConfigurationSetNameListener holder class
   *
   * This class manages one ore more instances of
   * ConfigurationSetNameListener class.
   *
   * @endif
   */
  class ConfigurationSetNameListenerHolder
  {
    typedef std::pair<ConfigurationSetNameListener*, bool> Entry;
    typedef coil::Guard<coil::Mutex> Guard;
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     * @else
     * @brief Constructor
     * @endif
     */
    ConfigurationSetNameListenerHolder();
    
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~ConfigurationSetNameListenerHolder();
    
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
    void addListener(ConfigurationSetNameListener* listener, bool autoclean);
    
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
    void removeListener(ConfigurationSetNameListener* listener);

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
    void notify(const char* config_set_name);
      
  private:
    std::vector<Entry> m_listeners;
    coil::Mutex m_mutex;
  };

  //------------------------------------------------------------
  /*!
   * @if jp
   * @class ConfigurationActionListeners
   * @brief ConfigurationActionListeners クラス
   *
   *
   * @else
   * @class ConfigurationActionListeners
   * @brief ConfigurationActionListeners class
   *
   *
   * @endif
   */
  class ConfigurationListeners
  {
  public:
    /*!
     * @if jp
     * @brief ConfigurationParamTypeリスナ配列
     * ConfigurationParamTypeリスナを格納
     * @else
     * @brief ConfigurationParamType listener array
     * The ConfigurationParamType listener is stored.
     * @endif
     */
    ConfigurationParamListenerHolder 
    configparam_[CONFIG_PARAM_LISTENER_NUM];
    /*!
     * @if jp
     * @brief ConfigurationSetTypeリスナ配列
     * ConfigurationSetTypeリスナを格納
     * @else
     * @brief ConfigurationSetType listener array
     * The ConfigurationSetType listener is stored.
     * @endif
     */
    ConfigurationSetListenerHolder
    configset_[CONFIG_SET_LISTENER_NUM];
    /*!
     * @if jp
     * @brief ConfigurationSetNameListenerTypeリスナ配列
     * ConfigurationSetNameListenerTypeリスナを格納
     * @else
     * @brief ConfigurationSetNameListenerType listener array
     * The ConfigurationSetNameListenerType listener is stored. 
     * @endif
     */
    ConfigurationSetNameListenerHolder 
    configsetname_[CONFIG_SET_NAME_LISTENER_NUM];
  };


}; // namespace RTC

#endif // RTC_CONFIGURATIONLISTENER_H
