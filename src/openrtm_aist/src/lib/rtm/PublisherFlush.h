// -*- C++ -*-
/*!
 * @file  PublisherFlush.h
 * @brief PublisherFlush class
 * @date  $Date: 2007-12-31 03:08:06 $
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
 * $Id$
 *
 */

#ifndef RTC_PUBLISHERFLUSH_H
#define RTC_PUBLISHERFLUSH_H

#include <coil/Condition.h>
#include <rtm/PublisherBase.h>
#include <rtm/SystemLogger.h>
#include <rtm/ConnectorBase.h>
#include <rtm/ConnectorListener.h>

namespace coil
{
  class Properties;
};

namespace RTC
{
  class InPortConsumer;

  /*!
   * @if jp
   * @class PublisherFlush
   * @brief PublisherFlush クラス
   *
   * Flush 型 Publisher クラス
   * バッファ内に格納されている未送信データを送信する。
   * データ送出を待つコンシューマを、送出する側と同じスレッドで動作させる。
   *
   * @else
   * @class PublisherFlush
   * @brief PublisherFlush class
   *
   * This is a Publisher class of Flush type.
   * This class sends unsend data that has been stored in the buffer.
   * This executes Consumer that waits for the data send timing in the same 
   * thread as its send side.
   *
   * @endif
   */
  class PublisherFlush
    : public PublisherBase
  {
  public:
    typedef coil::Mutex Mutex;
    typedef coil::Condition<Mutex> Condition;
    typedef coil::Guard<coil::Mutex> Guard;
    DATAPORTSTATUS_ENUM

    /*!
     * @if jp
     * @brief コンストラクタ
     *
     * コンストラクタ
     *
     * @else
     * @brief Constructor
     *
     * Consrtuctor.
     *
     * @endif
     */
    PublisherFlush();
    
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
    virtual ~PublisherFlush(void);

    /*!
     * @if jp
     * @brief 初期化
     *
     * このクラスのオブジェクトを使用するのに先立ち、必ずこの関数を呼び
     * 出す必要がある。ただし、この PublisherFlush は現状で初期化するパ
     * ラメータを持たない。
     *    
     * @param property 本Publisherの駆動制御情報を設定したPropertyオブジェクト
     * @return ReturnCode PORT_OK 正常終了
     *                    INVALID_ARGS Properties が不正な値を含む
     *
     * @else
     * @brief initialization
     *
     * This function have to be called before using this class object.
     * However, this PublisherFlush class has no parameters to be initialized.
     *
     * @param property Property objects that includes the control information
     *                 of this Publisher
     * @return ReturnCode PORT_OK normal return
     *                    INVALID_ARGS Properties with invalid values.
     * @endif
     */
    virtual ReturnCode init(coil::Properties& prop);

    /*!
     * @if jp
     * @brief InPortコンシューマのセット
     *
     * この関数では、この Publisher に関連付けられるコンシューマをセットする。
     * コンシューマオブジェクトがヌルポインタの場合、INVALID_ARGSが返される。
     * それ以外の場合は、PORT_OK が返される。
     *
     * @param consumer Consumer へのポインタ
     * @return ReturnCode PORT_OK 正常終了
     *                    INVALID_ARGS 引数に不正な値が含まれている
     *
     * @else
     * @brief Store InPort consumer
     *
     * This operation sets a consumer that is associated with this
     * object. If the consumer object is NULL, INVALID_ARGS will be
     * returned.
     *
     * @param consumer A pointer to a consumer object.
     * @return ReturnCode PORT_OK normal return
     *                    INVALID_ARGS given argument has invalid value
     *
     * @endif
     */
    virtual ReturnCode setConsumer(InPortConsumer* consumer);

    /*!
     * @if jp
     * @brief バッファのセット
     * 
     * PublisherFlushでは、バッファを使用しないため、いかなる場合も
     * PORT_OK を返す。
     *
     * @param buffer CDRバッファ
     * @return PORT_OK 正常終了
     *
     * @else
     * @brief Setting buffer pointer
     *
     * Since PublisherFlush does not use any buffers, This function
     * always returns PORT_OK.
     *
     * @param buffer CDR buffer
     * @return PORT_OK
     *
     * @endif
     */
    virtual ReturnCode setBuffer(CdrBufferBase* buffer);

    /*!
     * @if jp
     * @brief リスナを設定する。
     *
     * Publisher に対してリスナオブジェクト ConnectorListeners を設定する。
     * 各種リスナオブジェクトを含む ConnectorListeners をセットすることで、
     * バッファの読み書き、データの送信時等にこれらのリスナをコールする。
     * ConnectorListeners オブジェクトの所有権はポートまたは RTObject が持ち
     * Publisher 削除時に ConnectorListeners は削除されることはない。
     * ConnectorListeners がヌルポインタの場合 INVALID_ARGS を返す。
     *
     * @param info ConnectorProfile をローカル化したオブジェクト ConnectorInfo
     * @param listeners リスナを多数保持する ConnectorListeners オブジェクト
     * @return PORT_OK      正常終了
     *         INVALID_ARGS 不正な引数
     * @else
     * @brief Set the listener. 
     *
     * This function sets ConnectorListeners listener object to the
     * Publisher. By setting ConnectorListeners containing various
     * listeners objects, these listeners are called at the time of
     * reading and writing of a buffer, and transmission of data
     * etc. Since the ownership of the ConnectorListeners object is
     * owned by Port or RTObject, the Publisher never deletes the
     * ConnectorListeners object. If the given ConnectorListeners'
     * pointer is NULL, this function returns INVALID_ARGS.
     *
     * @param info ConnectorInfo that is localized object of ConnectorProfile
     * @param listeners ConnectorListeners that holds various listeners
     * @return PORT_OK      Normal return
     *         INVALID_ARGS Invalid arguments
     * @endif
     */
    virtual ::RTC::DataPortStatus::Enum
    setListener(ConnectorInfo& profile,
                RTC::ConnectorListeners* listeners);

    /*!
     * @if jp
     * @brief データを書き込む
     *
     * Publisher が保持するコンシューマに対してデータを書き込む。コン
     * シューマ、リスナ等が適切に設定されていない等、Publisher オブジェ
     * クトが正しく初期化されていない場合、この関数を呼び出すとエラーコー
     * ド PRECONDITION_NOT_MET が返され、コンシューマへの書き込み等の操
     * 作は一切行われない。
     *
     * コンシューマへの書き込みに対して、コンシューマがフル状態、コン
     * シューマのエラー、コンシューマへの書き込みがタイムアウトした場合
     * にはそれぞれ、エラーコード SEND_FULL, SEND_ERROR, SEND_TIMEOUT
     * が返される。
     *
     * これら以外のエラーの場合、PORT_ERROR が返される。
     * 
     *
     * @param data 書き込むデータ 
     * @param sec タイムアウト時間
     * @param nsec タイムアウト時間
     *
     * @return PORT_OK             正常終了
     *         PRECONDITION_NO_MET consumer, buffer, listener等が適切に設定
     *                             されていない等、このオブジェクトの事前条件
     *                             を満たさない場合。
     *         SEND_FULL           送信先がフル状態
     *         SEND_TIMEOUT        送信先がタイムアウトした
     *         CONNECTION_LOST     接続が切断されたことを検知した。
     *
     * @else
     * @brief Write data 
     *
     * This function writes data into the consumer associated with
     * this Publisher. If this function is called without initializing
     * correctly such as a consumer, listeners, etc., error code
     * PRECONDITION_NOT_MET will be returned and no operation of the
     * writing to the consumer etc. will be performed.
     *
     * When publisher writes data to the buffer, if the consumer
     * returns full-status, returns error, is returned with timeout,
     * error codes BUFFER_FULL, BUFFER_ERROR and BUFFER_TIMEOUT will
     * be returned respectively.
     *
     * In other cases, PROT_ERROR will be returned.
     *
     * @param data Data to be wrote to the buffer
     * @param sec Timeout time in unit seconds
     * @param nsec Timeout time in unit nano-seconds
     * @return PORT_OK             Normal return
     *         PRECONDITION_NO_MET Precondition does not met. A consumer,
     *                             a buffer, listenes are not set properly.
     *         SEND_FULL           Data was sent but full-status returned
     *         SEND_TIMEOUT        Data was sent but timeout occurred
     *         CONNECTION_LOST     detected that the connection has been lost
     *
     * @endif
     */
    virtual ReturnCode write(const cdrMemoryStream& data,
                             unsigned long sec,
                             unsigned long usec);
    /*!
     * @if jp
     *
     * @brief アクティブ化確認
     * 
     * Publisher はデータポートと同期して activate/deactivate される。
     * activate() / deactivate() 関数によって、アクティブ状態と非アクティ
     * ブ状態が切り替わる。この関数により、現在アクティブ状態か、非アク
     * ティブ状態かを確認することができる。
     *
     * @return 状態確認結果(アクティブ状態:true、非アクティブ状態:false)
     *
     * @else
     *
     * @brief If publisher is active state
     * 
     * A Publisher can be activated/deactivated synchronized with the
     * data port.  The active state and the non-active state are made
     * transition by the "activate()" and the "deactivate()" functions
     * respectively. This function confirms if the publisher is in
     * active state.
     *
     * @return Result of state confirmation
     *         (Active state:true, Inactive state:false)
     *
     * @endif
     */
    virtual bool isActive();

    /*!
     * @if jp
     * @brief アクティブ化する
     *
     * Publisher をアクティブ化する。この関数を呼び出すことにより、
     * Publisherが持つ、データを送信するスレッドが動作を開始する。初期
     * 化が行われていないなどにより、事前条件を満たさない場合、エラーコー
     * ド PRECONDITION_NOT_MET を返す。
     *
     * @return PORT_OK 正常終了
     *         PRECONDITION_NOT_MET 事前条件を満たさない
     *
     * @else
     * @brief activation
     *
     * This function activates the publisher. By calling this
     * function, this publisher starts the thread that pushes data to
     * InPort. If precondition such as initialization process and so
     * on is not met, the error code PRECONDITION_NOT_MET is returned.
     *
     * @return PORT_OK normal return
     *         PRECONDITION_NOT_MET precondition is not met
     *
     * @endif
     */
    virtual ReturnCode activate();

    /*!
     * @if jp
     * @brief 非アクティブ化する
     *
     * Publisher を非アクティブ化する。この関数を呼び出すことにより、
     * Publisherが持つ、データを送信するスレッドが動作を停止する。初期
     * 化が行われていないなどにより、事前条件を満たさない場合、エラーコー
     * ド PRECONDITION_NOT_MET を返す。
     *
     * @return PORT_OK 正常終了
     *         PRECONDITION_NOT_MET 事前条件を満たさない
     *
     * @else
     * @brief deactivation
     *
     * This function deactivates the publisher. By calling this
     * function, this publisher stops the thread that pushes data to
     * InPort. If precondition such as initialization process and so
     * on is not met, the error code PRECONDITION_NOT_MET is returned.
     *
     * @return PORT_OK normal return
     *         PRECONDITION_NOT_MET precondition is not met
     *
     * @endif
     */
    virtual ReturnCode deactivate();

  protected:
    /*!
     * @if jp
     * @brief ON_SENDのリスナへ通知する。 
     * @param data cdrMemoryStream
     * @else
     * @brief Notify an ON_SEND event to listners
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
     * @brief ON_RECEIVEDのリスナへ通知する。 
     * @param data cdrMemoryStream
     * @else
     * @brief Notify an ON_RECEIVED event to listeners
     * @param data cdrMemoryStream
     * @endif
     */
    inline void onReceived(const cdrMemoryStream& data)
    {
      m_listeners->
        connectorData_[ON_RECEIVED].notify(m_profile, data);
    }

    /*!
     * @if jp
     * @brief ON_RECEIVER_FULLのリスナへ通知する。 
     * @param data cdrMemoryStream
     * @else
     * @brief Notify an ON_RECEIVER_FULL event to listeners
     * @param data cdrMemoryStream
     * @endif
     */
    inline void onReceiverFull(const cdrMemoryStream& data)
    {
      m_listeners->
        connectorData_[ON_RECEIVER_FULL].notify(m_profile, data);
    }

    /*!
     * @if jp
     * @brief ON_RECEIVER_TIMEOUTのリスナへ通知する。 
     * @param data cdrMemoryStream
     * @else
     * @brief Notify an ON_RECEIVER_TIMEOUT event to listeners
     * @param data cdrMemoryStream
     * @endif
     */
    inline void onReceiverTimeout(const cdrMemoryStream& data)
    {
      m_listeners->
        connectorData_[ON_RECEIVER_TIMEOUT].notify(m_profile, data);
    }

    /*!
     * @if jp
     * @brief ON_RECEIVER_ERRORのリスナへ通知する。 
     * @param data cdrMemoryStream
     * @else
     * @brief Notify an ON_RECEIVER_ERROR event to listeners
     * @param data cdrMemoryStream
     * @endif
     */
    inline void onReceiverError(const cdrMemoryStream& data)
    {
      m_listeners->
        connectorData_[ON_RECEIVER_ERROR].notify(m_profile, data);
    }
    
  private:
    Logger rtclog;
    InPortConsumer* m_consumer;
    ConnectorInfo m_profile;
    ConnectorListeners* m_listeners;
    ReturnCode m_retcode;
    Mutex m_retmutex;
    bool m_active;
  };

};     // namespace RTC

extern "C"
{
  void DLL_EXPORT PublisherFlushInit();
};

#endif // RTC_PUBLISHERFLUSH_H

