// -*- C++ -*-
/*!
 * @file PublisherBase.h
 * @brief Publisher base class
 * @date $Date: 2007-12-31 03:08:06 $
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

#ifndef RTC_PUBLISHERBASE_H
#define RTC_PUBLISHERBASE_H

#include <coil/Properties.h>
#include <coil/Factory.h>
#include <rtm/RTC.h>
#include <rtm/CdrBufferBase.h>
#include <rtm/DataPortStatus.h>

namespace coil
{
  class Properties;
}
namespace RTC
{
  class InPortConsumer;
  class ConnectorListeners;
  class ConnectorInfo;

  /*!
   * @if jp
   *
   * @class PublisherBase
   *
   * @brief Publisher 基底クラス
   * 
   * データ送出タイミングを管理して送出を駆動するPublisher* の基底クラス。
   * 各種 Publisher はこのクラスを継承して詳細を実装する。
   *
   * @since 0.4.0
   *
   * @else
   *
   * @class PublisherBase
   *
   * @brief Base class of Publisher.
   *
   * This is a base class of Publisher*. This class manages data send timing.
   * Variation of Publisher* which implements details of Publisher inherits
   * this PublisherBase class.
   *
   * @endif
   */
  class PublisherBase
    : public DataPortStatus
  {
  public:
    DATAPORTSTATUS_ENUM
    /*!
     * @if jp
     *
     * @brief デストラクタ
     *
     * @else
     *
     * @brief Destructor
     *
     * @endif
     */
    virtual ~PublisherBase(void){};

    /*!
     * @if jp
     * @brief 設定初期化
     *
     * InPortConsumerの各種設定を行う。実装クラスでは、与えられた
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
    virtual ReturnCode init(coil::Properties& prop) = 0;

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
    virtual ReturnCode setConsumer(InPortConsumer* consumer) = 0;

    /*!
     * @if jp
     * @brief バッファのセット
     *
     * この関数では、この Publisher に関連付けられるバッファをセットする。
     * バッファオブジェクトがヌルポインタの場合、INVALID_ARGSが返される。
     * それ以外の場合は、PORT_OK が返される。
     *
     * @param buffer CDR buffer へのポインタ
     * @return ReturnCode PORT_OK 正常終了
     *                    INVALID_ARGS 引数に不正な値が含まれている
     *
     * @else
     * @brief Setting buffer pointer
     *
     * This operation sets a buffer that is associated with this
     * object. If the buffer object is NULL, INVALID_ARGS will be
     * returned.
     *
     * @param buffer A pointer to a CDR buffer object.
     * @return ReturnCode PORT_OK normal return
     *                    INVALID_ARGS given argument has invalid value
     *
     * @endif
     */
    virtual ReturnCode setBuffer(BufferBase<cdrMemoryStream>* buffer) = 0;

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
    virtual ReturnCode setListener(ConnectorInfo& info,
                                   ConnectorListeners* listeners) = 0;

    /*!
     * @if jp
     * @brief データを書き込む
     *
     * Publisher に対してデータを書き込む。コンシューマ、リスナ等が適切
     * に設定されていない等、Publisher オブジェクトが正しく初期化されて
     * いない場合、この関数を呼び出すとエラーコード
     * PRECONDITION_NOT_MET が返され、書き込み等の操作は一切行われない。
     *
     * 書き込みに対して、コンシューマがフル状態、コンシューマのエラー、
     * コンシューマへの書き込みがタイムアウトした場合にはそれぞれ、エラー
     * コード SEND_FULL, SEND_ERROR, SEND_TIMEOUT が返される。
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
     * This function writes data into the Publisher. If this function
     * is called without initializing correctly such as a consumer,
     * listeners, etc., error code PRECONDITION_NOT_MET will be
     * returned and no operation of the writing etc. will be
     * performed.
     *
     * When publisher writes data, if the consumer returns
     * full-status, returns error, is returned with timeout, error
     * codes BUFFER_FULL, BUFFER_ERROR and BUFFER_TIMEOUT will be
     * returned respectively.
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
                             unsigned long usec) = 0;

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
    virtual bool isActive() = 0;

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
    virtual ReturnCode activate() = 0;

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
    virtual ReturnCode deactivate() = 0;
  
    /*!
     * @if jp
     *
     * @brief Publisher を破棄する。
     *
     * 当該 Publisher を破棄する。
     * 当該 Publisher が不要になった場合に PublisherFactory から呼び出される。
     *
     * @else
     *
     * @brief Release the Publisher
     *
     * Release this Publisher.
     * When Publisher becomes unnecessary, this is invoked from
     * PublisherFactory. 
     *
     * @endif
     */
    virtual void release(){}
  };

  typedef coil::GlobalFactory<PublisherBase> PublisherFactory;

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  EXTERN template class DLL_PLUGIN coil::GlobalFactory<PublisherBase>;
#endif
};
#endif // RTC_PUBLISHERBASE_H
