// -*- C++ -*-
/*!
 * @file  OutPortCorbaCdrProvider.h
 * @brief OutPortCorbaCdrProvider class
 * @date  $Date: 2008-01-14 07:52:40 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006-2010
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id: OutPortCorbaCdrProvider.h 1244 2009-03-13 07:25:42Z n-ando $
 *
 */

#ifndef RTC_OUTPORTCORBACDRPROVIDER_H
#define RTC_OUTPORTCORBACDRPROVIDER_H

#include <rtm/idl/DataPortSkel.h>
#include <rtm/BufferBase.h>
#include <rtm/OutPortProvider.h>
#include <rtm/CORBA_SeqUtil.h>
#include <rtm/Manager.h>
#include <rtm/ConnectorListener.h>
#include <rtm/ConnectorBase.h>

#ifdef WIN32
#pragma warning( disable : 4290 )
#endif

namespace RTC
{
  /*!
   * @if jp
   * @class OutPortCorbaCdrProvider
   * @brief OutPortCorbaCdrProvider クラス
   *
   * OutPortProvider 
   *
   * データ転送に CORBA の OpenRTM::OutPortCdr インターフェースを利用し
   * た、pull 型データフロー型を実現する OutPort プロバイダクラス。
   *
   * @since 0.4.0
   *
   * @else
   * @class OutPortCorbaCdrProvider
   * @brief OutPortCorbaCdrProvider class
   *
   * The OutPort provider class which uses the OpenRTM::OutPortCdr
   * interface in CORBA for data transfer and realizes a pull-type
   * dataflow.
   *
   * @since 0.4.0
   *
   * @endif
   */
  class OutPortCorbaCdrProvider
    : public OutPortProvider,
      public virtual ::POA_OpenRTM::OutPortCdr,
      public virtual PortableServer::RefCountServantBase
  {
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     *
     * コンストラクタ
     *
     * @else
     * @brief Constructor
     *
     * Constructor
     *
     * @endif
     */
    OutPortCorbaCdrProvider(void);

    /*!
     * @if jp
     * @brief デストラクタ
     *
     * デストラクタ
     *
     * @else
     * @brief Destructor
     *
     * Destructor
     *
     * @endif
     */
    virtual ~OutPortCorbaCdrProvider(void);

    /*!
     * @if jp
     * @brief 設定初期化
     *
     * OutPortCorbaCdrProvider の各種設定を行う。与えられた
     * Propertiesから必要な情報を取得して各種設定を行う。この init() 関
     * 数は、OutPortProvider生成直後および、接続時にそれぞれ呼ばれる可
     * 能性がある。したがって、この関数は複数回呼ばれることを想定して記
     * 述されるべきである。
     * 
     * @param prop 設定情報
     *
     * @else
     *
     * @brief Initializing configuration
     *
     * This operation would be called to configure in initialization.
     * In the concrete class, configuration should be performed
     * getting appropriate information from the given Properties data.
     * This function might be called right after instantiation and
     * connection sequence respectivly.  Therefore, this function
     * should be implemented assuming multiple call.
     *
     * @param prop Configuration information
     *
     * @endif
     */
    virtual void init(coil::Properties& prop);

    /*!
     * @if jp
     * @brief バッファをセットする
     *
     * OutPortProvider がデータを取り出すバッファをセットする。
     * すでにセットされたバッファがある場合、以前のバッファへの
     * ポインタに対して上書きされる。
     * OutPortProviderはバッファの所有権を仮定していないので、
     * バッファの削除はユーザの責任で行わなければならない。
     *
     * @param buffer OutPortProviderがデータを取り出すバッファへのポインタ
     *
     * @else
     * @brief Setting outside buffer's pointer
     *
     * A pointer to a buffer from which OutPortProvider retrieve data.
     * If already buffer is set, previous buffer's pointer will be
     * overwritten by the given pointer to a buffer.  Since
     * OutPortProvider does not assume ownership of the buffer
     * pointer, destructor of the buffer should be done by user.
     * 
     * @param buffer A pointer to a data buffer to be used by OutPortProvider
     *
     * @endif
     */
    virtual void setBuffer(CdrBufferBase* buffer);

    /*!
     * @if jp
     * @brief リスナを設定する。
     *
     * OutPort はデータ送信処理における各種イベントに対して特定のリスナ
     * オブジェクトをコールするコールバック機構を提供する。詳細は
     * ConnectorListener.h の ConnectorDataListener, ConnectorListener
     * 等を参照のこと。OutPortCorbaCdrProvider では、以下のコールバック
     * が提供される。
     * 
     * - ON_BUFFER_READ
     * - ON_SEND
     * - ON_BUFFER_EMPTY
     * - ON_BUFFER_READ_TIMEOUT
     * - ON_SENDER_EMPTY
     * - ON_SENDER_TIMEOUT
     * - ON_SENDER_ERROR
     *
     * @param info 接続情報
     * @param listeners リスナオブジェクト
     *
     * @else
     * @brief Set the listener. 
     *
     * OutPort provides callback functionality that calls specific
     * listener objects according to the events in the data publishing
     * process. For details, see documentation of
     * ConnectorDataListener class and ConnectorListener class in
     * ConnectorListener.h. In this OutPortCorbaCdrProvider provides
     * the following callbacks.
     * 
     * - ON_BUFFER_READ
     * - ON_SEND
     * - ON_BUFFER_EMPTY
     * - ON_BUFFER_READ_TIMEOUT
     * - ON_SENDER_EMPTY
     * - ON_SENDER_TIMEOUT
     * - ON_SENDER_ERROR
     *
     * @param info Connector information
     * @param listeners Listener objects
     *
     * @endif
     */
    virtual void setListener(ConnectorInfo& info,
                             ConnectorListeners* listeners);

    /*!
     * @if jp
     * @brief Connectorを設定する。
     *
     * OutPort は接続確立時に OutPortConnector オブジェクトを生成し、生
     * 成したオブジェクトのポインタと共にこの関数を呼び出す。所有権は
     * OutPort が保持するので OutPortProvider は OutPortConnector を削
     * 除してはいけない。
     *
     * @param connector OutPortConnector
     *
     * @else
     * @brief set Connector
     *
     * OutPort creates OutPortConnector object when it establishes
     * connection between OutPort and InPort, and it calls this
     * function with a pointer to the connector object. Since the
     * OutPort has the ownership of this connector, OutPortProvider
     * should not delete it.
     *
     * @param connector OutPortConnector
     *
     * @endif
     */
    virtual void setConnector(OutPortConnector* connector);

    /*!
     * @if jp
     * @brief [CORBA interface] バッファからデータを取得する
     *
     * 設定された内部バッファからデータを取得する。
     *
     * @return 取得データ
     *
     * @else
     * @brief [CORBA interface] Get data from the buffer
     *
     * Get data from the internal buffer.
     *
     * @return Data got from the buffer.
     *
     * @endif
     */
    virtual ::OpenRTM::PortStatus get(::OpenRTM::CdrData_out data)
      throw (CORBA::SystemException);

    
  private:
    /*!
     * @if jp
     * @brief リターンコード変換
     * @else
     * @brief Return codes conversion
     * @endif
     */
    ::OpenRTM::PortStatus convertReturn(BufferStatus::Enum status,
                                        const cdrMemoryStream& data);


    /*!
     * @if jp
     * @brief ON_BUFFER_READ のリスナへ通知する。 
     * @param data cdrMemoryStream
     * @else
     * @brief Notify an ON_BUFFER_READ event to listeners
     * @param data cdrMemoryStream
     * @endif
     */
    inline void onBufferRead(const cdrMemoryStream& data)
    {
      m_listeners->
        connectorData_[ON_BUFFER_READ].notify(m_profile, data);
    }

    /*!
     * @if jp
     * @brief ON_SEND のリスナへ通知する。 
     * @param data cdrMemoryStream
     * @else
     * @brief Notify an ON_SEND event to listeners
     * @param data cdrMemoryStream
     * @endif
     */
    inline void onSend(const cdrMemoryStream& data)
    {
      m_listeners->
        connectorData_[ON_SEND].notify(m_profile, data);
    }

    /*!
     * @if jp
     * @brief ON_BUFFER_EMPTYのリスナへ通知する。 
     * @else
     * @brief Notify an ON_BUFFER_EMPTY event to listeners
     * @endif
     */
    inline void onBufferEmpty()
    {
      m_listeners->
        connector_[ON_BUFFER_EMPTY].notify(m_profile);
    }

    /*!
     * @if jp
     * @brief ON_BUFFER_READ_TIMEOUT のリスナへ通知する。 
     * @else
     * @brief Notify an ON_BUFFER_READ_TIMEOUT event to listeners
     * @endif
     */
    inline void onBufferReadTimeout()
    {
      m_listeners->
        connector_[ON_BUFFER_READ_TIMEOUT].notify(m_profile);
    }

    /*!
     * @if jp
     * @brief ON_SENDER_EMPTYのリスナへ通知する。 
     * @else
     * @brief Notify an ON_SENDER_EMPTY event to listeners
     * @endif
     */
    inline void onSenderEmpty()
    {
      m_listeners->
        connector_[ON_SENDER_EMPTY].notify(m_profile);
    }

    /*!
     * @if jp
     * @brief ON_SENDER_TIMEOUT のリスナへ通知する。 
     * @else
     * @brief Notify an ON_SENDER_TIMEOUT event to listeners
     * @endif
     */
    inline void onSenderTimeout()
    {
      m_listeners->
        connector_[ON_SENDER_TIMEOUT].notify(m_profile);
    }

    /*!
     * @if jp
     * @brief ON_SENDER_ERRORのリスナへ通知する。 
     * @else
     * @brief Notify an ON_SENDER_ERROR event to listeners
     * @endif
     */
    inline void onSenderError()
    {
      m_listeners->
        connector_[ON_SENDER_ERROR].notify(m_profile);
    }
    
  private:
    CdrBufferBase* m_buffer;
    ::OpenRTM::OutPortCdr_var m_objref;
    ConnectorListeners* m_listeners;
    ConnectorInfo m_profile;
    OutPortConnector* m_connector;
  };  // class OutPortCorbaCdrProvider
};     // namespace RTC

extern "C"
{
  /*!
   * @if jp
   * @brief モジュール初期化関数
   *
   * OutPortCorbaCdrProvider のファクトリを登録する初期化関数。
   *
   * @else
   * @brief Module initialization
   *
   * This initialization function registers OutPortCorbaCdrProvider's factory.
   *
   * @endif
   */
  void OutPortCorbaCdrProviderInit(void);
};

#ifdef WIN32
#pragma warning( default : 4290 )
#endif

#endif // RTC_OUTPORTCORBACDRPROVIDER_H
