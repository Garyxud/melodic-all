// -*- C++ -*-
/*!
 * @file ConnectorListener.h
 * @brief connector listener class
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2009
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

#ifndef RTC_CONNECTORLISTENER_H
#define RTC_CONNECTORLISTENER_H

#include <vector>
#include <utility>
#include <coil/Mutex.h>
#include <coil/Guard.h>
#include <rtm/RTC.h>
#include <rtm/ConnectorBase.h>

class cdrMemoryStream;

namespace RTC
{
  class ConnectorInfo;

  /*!
   * @if jp
   * @brief ConnectorDataListener のタイプ
   *
   * - ON_BUFFER_WRITE:          バッファ書き込み時
   * - ON_BUFFER_FULL:           バッファフル時
   * - ON_BUFFER_WRITE_TIMEOUT:  バッファ書き込みタイムアウト時
   * - ON_BUFFER_OVERWRITE:      バッファ上書き時
   * - ON_BUFFER_READ:           バッファ読み出し時
   * - ON_SEND:                  InProtへの送信時
   * - ON_RECEIVED:              InProtへの送信完了時
   * - ON_RECEIVER_FULL:         InProt側バッファフル時
   * - ON_RECEIVER_TIMEOUT:      InProt側バッファタイムアウト時
   * - ON_RECEIVER_ERROR:        InProt側エラー時
   *
   * @else
   * @brief The types of ConnectorDataListener
   * 
   * - ON_BUFFER_WRITE:          At the time of buffer write
   * - ON_BUFFER_FULL:           At the time of buffer full
   * - ON_BUFFER_WRITE_TIMEOUT:  At the time of buffer write timeout
   * - ON_BUFFER_OVERWRITE:      At the time of buffer overwrite
   * - ON_BUFFER_READ:           At the time of buffer read
   * - ON_SEND:                  At the time of sending to InPort
   * - ON_RECEIVED:              At the time of finishing sending to InPort
   * - ON_RECEIVER_FULL:         At the time of bufferfull of InPort
   * - ON_RECEIVER_TIMEOUT:      At the time of timeout of InPort
   * - ON_RECEIVER_ERROR:        At the time of error of InPort
   *
   * @endif
   */
  enum ConnectorDataListenerType
    {
      ON_BUFFER_WRITE = 0, 
      ON_BUFFER_FULL, 
      ON_BUFFER_WRITE_TIMEOUT, 
      ON_BUFFER_OVERWRITE, 
      ON_BUFFER_READ, 
      ON_SEND, 
      ON_RECEIVED,
      ON_RECEIVER_FULL, 
      ON_RECEIVER_TIMEOUT, 
      ON_RECEIVER_ERROR,
      CONNECTOR_DATA_LISTENER_NUM
    };

  /*!
   * @if jp
   * @class ConnectorDataListener クラス
   * @brief ConnectorDataListener クラス
   *
   * データポートの Connector において発生する各種イベントに対するコー
   * ルバックを実現するリスナクラスの基底クラス。
   *
   * コアロジックがOutPortに対してデータ書き込み、InPort側でデータが取
   * 得されるまでの間で発生する各種イベントをフックするコールバックを設
   * 定することができる。なお、リスナークラスは2種類存在し、バッファフ
   * ルや送信時のコールバックで、その時点で有効なデータをファンクタの引
   * 数として受け取る ConnectorDataListener であり、もう一方はデータエ
   * ンプティやバッファ読み込み時のタイムアウトなどデータが取得できない
   * 場合などにコールされるファンクタの引数に何もとらならい
   * ConnecotorListener がある。
   *
   * データポートには、接続時にデータの送受信方法についてデータフロー型、
   * サブスクリプション型等を設定することができる。
   * ConnectorDaataListener/ConnectorListener はともに、様々なイベント
   * に対するコールバックを設定することができるが、これらデータフロー型
   * およびサブスクリプション型の設定に応じて、利用可能なもの利用不可能
   * なものや、呼び出されるタイミングが異なる。
   * 以下に、インターフェースがCORBA CDR型の場合のコールバック一覧を示す。
   *
   * OutPort:
   * -  Push型: Subscription Typeによりさらにイベントの種類が分かれる。
   *   - Flush: Flush型にはバッファがないため ON_BUFFER 系のイベントは発生しない
   *     - ON_SEND
   *     - ON_RECEIVED
   *     - ON_RECEIVER_FULL
   *     - ON_RECEIVER_TIMEOUT
   *     - ON_RECEIVER_ERROR
   *     - ON_CONNECT
   *     - ON_DISCONNECT
   *     .
   *   - New型
   *     - ON_BUFFER_WRITE
   *     - ON_BUFFER_FULL
   *     - ON_BUFFER_WRITE_TIMEOUT
   *     - ON_BUFFER_OVERWRITE
   *     - ON_BUFFER_READ
   *     - ON_SEND
   *     - ON_RECEIVED
   *     - ON_RECEIVER_FULL
   *     - ON_RECEIVER_TIMEOUT
   *     - ON_RECEIVER_ERROR
   *     - ON_SENDER_ERROR
   *     - ON_CONNECT
   *     - ON_DISCONNECT
   *     .
   *   - Periodic型
   *     - ON_BUFFER_WRITE
   *     - ON_BUFFER_FULL
   *     - ON_BUFFER_WRITE_TIMEOUT
   *     - ON_BUFFER_READ
   *     - ON_SEND
   *     - ON_RECEIVED
   *     - ON_RECEIVER_FULL
   *     - ON_RECEIVER_TIMEOUT
   *     - ON_RECEIVER_ERROR
   *     - ON_BUFFER_EMPTY
   *     - ON_SENDER_EMPTY
   *     - ON_SENDER_ERROR
   *     - ON_CONNECT
   *     - ON_DISCONNECT
   *     .
   *   .
   * - Pull型
   *   - ON_BUFFER_READ
   *   - ON_SEND
   *   - ON_BUFFER_EMPTY
   *   - ON_BUFFER_READ_TIMEOUT
   *   - ON_SENDER_EMPTY
   *   - ON_SENDER_TIMEOUT
   *   - ON_SENDER_ERROR
   *   - ON_CONNECT
   *   - ON_DISCONNECT
   *
   * InPort:
   * - Push型:
   *     - ON_BUFFER_WRITE
   *     - ON_BUFFER_FULL
   *     - ON_BUFFER_WRITE_TIMEOUT
   *     - ON_BUFFER_WRITE_OVERWRITE
   *     - ON_RECEIVED
   *     - ON_RECEIVER_FULL
   *     - ON_RECEIVER_TIMEOUT
   *     - ON_RECEIVER_ERROR
   *     - ON_CONNECT
   *     - ON_DISCONNECT
   *     .
   * - Pull型
   *     - ON_CONNECT
   *     - ON_DISCONNECT
   *
   * @else
   * @class ConnectorDataListener class
   * @brief ConnectorDataListener class
   *
   * This class is abstract base class for listener classes that
   * provides callbacks for various events in the data port's
   * connectors.
   *
   * @endif
   */
  class ConnectorDataListener
  {
  public:
    /*!
     * @if jp
     *
     * @brief ConnectorDataListenerType を文字列に変換
     *
     * ConnectorDataListenerType を文字列に変換する
     *
     * @param type 変換対象 ConnectorDataListenerType
     *
     * @return 文字列変換結果
     *
     * @else
     *
     * @brief Convert ConnectorDataListenerType into the string.
     *
     * Convert ConnectorDataListenerType into the string.
     *
     * @param type The target ConnectorDataListenerType for transformation
     *
     * @return Trnasformation result of string representation
     *
     * @endif
     */
    static const char* toString(ConnectorDataListenerType type)
    {
      static const char* typeString[] =
        {
          "ON_BUFFER_WRITE",
          "ON_BUFFER_FULL",
          "ON_BUFFER_WRITE_TIMEOUT",
          "ON_BUFFER_OVERWRITE",
          "ON_BUFFER_READ", 
          "ON_SEND", 
          "ON_RECEIVED",
          "ON_RECEIVER_FULL", 
          "ON_RECEIVER_TIMEOUT", 
          "ON_RECEIVER_ERROR",
          "CONNECTOR_DATA_LISTENER_NUM"
        };
      if (type < CONNECTOR_DATA_LISTENER_NUM) { return typeString[type]; }
      return "";
    }

    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~ConnectorDataListener();

    /*!
     * @if jp
     *
     * @brief 仮想コールバックメソッド
     *
     * データポートの Connector において発生する各種イベントに対するコー
     * ルバックメソッド
     *
     * @else
     *
     * @brief Virtual Callback method
     *
     * This is a the Callback method to various events generated in Connector. 
     *
     * @endif
     */
    virtual void operator()(const ConnectorInfo& info,
                            const cdrMemoryStream& data) = 0;
  };

  /*!
   * @if jp
   * @class ConnectorDataListenerT クラス
   * @brief ConnectorDataListenerT クラス
   *
   * データポートの Connector において発生する各種イベントに対するコー
   * ルバックを実現するリスナクラスの基底クラス。
   * 
   * このクラスは、operator()() の第2引数に cdrMemoryStream 型ではなく、
   * 実際にデータポートで使用される変数型をテンプレート引数として
   * 渡すことができる。
   *
   * @else
   * @class ConnectorDataListenerT class
   * @brief ConnectorDataListenerT class
   *
   * This class is abstract base class for listener classes that
   * provides callbacks for various events in the data port's
   * connectors.
   *
   * This class template can have practical data types that are used
   * as typed variable for DataPort as an argument of template instead
   * of cdrMemoryStream.
   *
   * @endif
   */
  template <class DataType>
  class ConnectorDataListenerT
    : public ConnectorDataListener
  {
  public:
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~ConnectorDataListenerT(){}

    /*!
     * @if jp
     *
     * @brief コールバックメソッド
     *
     * データをデータポートで使用される変数型に変換して ConnectorDataListenerT
     * のコールバックメソッドを呼び出す。
     *
     * @param info ConnectorInfo 
     * @param cdrdata cdrMemoryStream型のデータ
     *
     * @else
     *
     * @brief Callback method
     *
     * This method invokes the callback method of ConnectorDataListenerT. 
     * Data is converted into the variable type used in DataPort.
     *
     * @param info ConnectorInfo 
     * @param cdrdata Data of cdrMemoryStream type
     *
     * @endif
     */
    virtual void operator()(const ConnectorInfo& info,
                            const cdrMemoryStream& cdrdata)
    {
      DataType data;
      cdrMemoryStream cdr(cdrdata.bufPtr(), cdrdata.bufSize());
      
      // endian type check
      std::string endian_type;
      endian_type = info.properties.getProperty("serializer.cdr.endian",
                                                "little");
      coil::normalize(endian_type);
      std::vector<std::string> endian(coil::split(endian_type, ","));
      if (endian[0] == "little")
        {
          cdr.setByteSwapFlag(true);
        }
      else if (endian[0] == "big")
        {
          cdr.setByteSwapFlag(false);
        }
      data <<= cdr;
      this->operator()(info, data);
    }

    /*!
     * @if jp
     *
     * @brief 仮想コールバックメソッド
     *
     * データポートの Connector において発生する各種イベントに対するコー
     * ルバックメソッド
     *
     * @else
     *
     * @brief Virtual Callback method
     *
     * This method invokes the callback method of ConnectorDataListenerT. 
     * Data is converted into the variable type used in DataPort.
     *
     * @endif
     */
    virtual void operator()(const ConnectorInfo& info,
                            const DataType& data) = 0;
                            
  };


  /*!
   * @if jp
   * @brief ConnectorListener のタイプ
   *  
   * - ON_BUFFER_EMPTY:       バッファが空の場合
   * - ON_BUFFER_READTIMEOUT: バッファが空でタイムアウトした場合
   * - ON_SENDER_EMPTY:       OutPort側バッファが空
   * - ON_SENDER_TIMEOUT:     OutPort側タイムアウト時
   * - ON_SENDER_ERROR:       OutPort側エラー時
   * - ON_CONNECT:            接続確立時
   * - ON_DISCONNECT:         接続切断時
   *
   * @else
   * @brief The types of ConnectorListener
   * 
   * - ON_BUFFER_EMPTY:       At the time of buffer empty
   * - ON_BUFFER_READTIMEOUT: At the time of buffer read timeout
   * - ON_BUFFER_EMPTY:       At the time of empty of OutPort
   * - ON_SENDER_TIMEOUT:     At the time of timeout of OutPort
   * - ON_SENDER_ERROR:       At the time of error of OutPort
   * - ON_CONNECT:            At the time of connection
   * - ON_DISCONNECT:         At the time of disconnection
   *
   * @endif
   */
  enum ConnectorListenerType
    {
      ON_BUFFER_EMPTY = 0,
      ON_BUFFER_READ_TIMEOUT,
      ON_SENDER_EMPTY, 
      ON_SENDER_TIMEOUT, 
      ON_SENDER_ERROR, 
      ON_CONNECT,
      ON_DISCONNECT,
      CONNECTOR_LISTENER_NUM
    };

  /*!
   * @if jp
   * @class ConnectorListener クラス
   * @brief ConnectorListener クラス
   *
   * データポートの Connector において発生する各種イベントに対するコー
   * ルバックを実現するリスナクラスの基底クラス。
   *
   * コアロジックがOutPortに対してデータ書き込み、InPort側でデータが取
   * 得されるまでの間で発生する各種イベントをフックするコールバックを設
   * 定することができる。なお、リスナークラスは2種類存在し、バッファフ
   * ルや送信時のコールバックで、その時点で有効なデータをファンクタの引
   * 数として受け取る ConnectorDataListener であり、もう一方はデータエ
   * ンプティやバッファ読み込み時のタイムアウトなどデータが取得できない
   * 場合などにコールされるファンクタの引数に何もとらならい
   * ConnecotorListener がある。
   *
   * データポートには、接続時にデータの送受信方法についてデータフロー型、
   * サブスクリプション型等を設定することができる。
   * ConnectorDaataListener/ConnectorListener は共にに、様々なイベント
   * に対するコールバックを設定することができるが、これらデータフロー型
   * およびサブスクリプション型の設定に応じて、利用できるもの、できない
   * もの、また呼び出されるタイミングが異なる。以下に、インターフェース
   * がCORBA CDR型の場合のコールバック一覧を示す。
   *
   * OutPort:
   * -  Push型: Subscription Typeによりさらにイベントの種類が分かれる。
   *   - Flush: Flush型にはバッファがないため ON_BUFFER 系のイベントは発生しない
   *     - ON_SEND
   *     - ON_RECEIVED
   *     - ON_RECEIVER_FULL
   *     - ON_RECEIVER_TIMEOUT
   *     - ON_RECEIVER_ERROR
   *     - ON_CONNECT
   *     - ON_DISCONNECT
   *     .
   *   - New型
   *     - ON_BUFFER_WRITE
   *     - ON_BUFFER_FULL
   *     - ON_BUFFER_WRITE_TIMEOUT
   *     - ON_BUFFER_OVERWRITE
   *     - ON_BUFFER_READ
   *     - ON_SEND
   *     - ON_RECEIVED
   *     - ON_RECEIVER_FULL
   *     - ON_RECEIVER_TIMEOUT
   *     - ON_RECEIVER_ERROR
   *     - ON_SENDER_ERROR
   *     - ON_CONNECT
   *     - ON_DISCONNECT
   *     .
   *   - Periodic型
   *     - ON_BUFFER_WRITE
   *     - ON_BUFFER_FULL
   *     - ON_BUFFER_WRITE_TIMEOUT
   *     - ON_BUFFER_READ
   *     - ON_SEND
   *     - ON_RECEIVED
   *     - ON_RECEIVER_FULL
   *     - ON_RECEIVER_TIMEOUT
   *     - ON_RECEIVER_ERROR
   *     - ON_BUFFER_EMPTY
   *     - ON_SENDER_EMPTY
   *     - ON_SENDER_ERROR
   *     - ON_CONNECT
   *     - ON_DISCONNECT
   *     .
   *   .
   * - Pull型
   *   - ON_BUFFER_READ
   *   - ON_SEND
   *   - ON_BUFFER_EMPTY
   *   - ON_BUFFER_READ_TIMEOUT
   *   - ON_SENDER_EMPTY
   *   - ON_SENDER_TIMEOUT
   *   - ON_SENDER_ERROR
   *   - ON_CONNECT
   *   - ON_DISCONNECT
   *
   * InPort:
   * - Push型:
   *     - ON_BUFFER_WRITE
   *     - ON_BUFFER_FULL
   *     - ON_BUFFER_WRITE_TIMEOUT
   *     - ON_BUFFER_WRITE_OVERWRITE
   *     - ON_RECEIVED
   *     - ON_RECEIVER_FULL
   *     - ON_RECEIVER_TIMEOUT
   *     - ON_RECEIVER_ERROR
   *     - ON_CONNECT
   *     - ON_DISCONNECT
   *     .
   * - Pull型
   *     - ON_CONNECT
   *     - ON_DISCONNECT
   *
   * @else
   * @class ConnectorListener class
   * @brief ConnectorListener class
   *
   * This class is abstract base class for listener classes that
   * provides callbacks for various events in the data port's
   * connectors.
   *
   * @endif
   */
  class ConnectorListener
  {
  public:
    /*!
     * @if jp
     *
     * @brief ConnectorListenerType を文字列に変換
     *
     * ConnectorListenerType を文字列に変換する
     *
     * @param type 変換対象 ConnectorListenerType
     *
     * @return 文字列変換結果
     *
     * @else
     *
     * @brief Convert ConnectorListenerType into the string.
     *
     * Convert ConnectorListenerType into the string.
     *
     * @param type The target ConnectorListenerType for transformation
     *
     * @return Trnasformation result of string representation
     *
     * @endif
     */
    static const char* toString(ConnectorListenerType type)
    { 
      static const char* typeStr[] =
        {
          "ON_BUFFER_EMPTY",
          "ON_BUFFER_READ_TIMEOUT",
          "ON_SENDER_EMPTY", 
          "ON_SENDER_TIMEOUT", 
          "ON_SENDER_ERROR", 
          "ON_CONNECT",
          "ON_DISCONNECT",
          "CONNECTOR_LISTENER_NUM"
        };
      if (type < CONNECTOR_LISTENER_NUM) { return typeStr[type]; }
      return "";
    }

    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~ConnectorListener();

    /*!
     * @if jp
     *
     * @brief 仮想コールバックメソッド
     *
     * データポートの Connector において発生する各種イベントに対するコー
     * ルバックメソッド
     *
     * @else
     *
     * @brief Virtual Callback method
     *
     * This method invokes the callback method of ConnectorDataListenerT. 
     * Data is converted into the variable type used in DataPort.
     *
     * @endif
     */
    virtual void operator()(const ConnectorInfo& info) = 0;
  };


  /*!
   * @if jp
   * @class ConnectorDataListenerHolder
   * @brief ConnectorDataListener ホルダクラス
   *
   * 複数の ConnectorDataListener を保持し管理するクラス。
   *
   * @else
   * @class ConnectorDataListenerHolder
   * @brief ConnectorDataListener holder class
   *
   * This class manages one ore more instances of ConnectorDataListener class.
   *
   * @endif
   */
  class ConnectorDataListenerHolder
  {
    typedef std::pair<ConnectorDataListener*, bool> Entry;
    typedef coil::Guard<coil::Mutex> Guard;
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     * @else
     * @brief Constructor
     * @endif
     */
    ConnectorDataListenerHolder();
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~ConnectorDataListenerHolder();
    
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
    void addListener(ConnectorDataListener* listener, bool autoclean);
    
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
    void removeListener(ConnectorDataListener* listener);
    
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
    void notify(const ConnectorInfo& info,
                const cdrMemoryStream& cdrdata);
    
  private:
    std::vector<Entry> m_listeners;
    coil::Mutex m_mutex;
  };


  /*!
   * @if jp
   * @class ConnectorListenerHolder 
   * @brief ConnectorListener ホルダクラス
   *
   * 複数の ConnectorListener を保持し管理するクラス。
   *
   * @else
   * @class ConnectorListenerHolder
   * @brief ConnectorListener holder class
   *
   * This class manages one ore more instances of ConnectorListener class.
   *
   * @endif
   */
  class ConnectorListenerHolder
  {
    typedef std::pair<ConnectorListener*, bool> Entry;
    typedef coil::Guard<coil::Mutex> Guard;
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     * @else
     * @brief Constructor
     * @endif
     */
    ConnectorListenerHolder();
    
    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~ConnectorListenerHolder();
    
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
    void addListener(ConnectorListener* listener, bool autoclean);
    
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
    void removeListener(ConnectorListener* listener);

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
    void notify(const ConnectorInfo& info);
      
  private:
    std::vector<Entry> m_listeners;
    coil::Mutex m_mutex;
  };
  
  /*!
   * @if jp
   * @class ConnectorListeners
   * @brief ConnectorListeners クラス
   *
   *
   * @else
   * @class ConnectorListeners
   * @brief ConnectorListeners class
   *
   *
   * @endif
   */
  class ConnectorListeners
  {
  public:
    /*!
     * @if jp
     * @brief ConnectorDataListenerTypeリスナ配列
     * ConnectorDataListenerTypeリスナを格納
     * @else
     * @brief ConnectorDataListenerType listener array
     * The ConnectorDataListenerType listener is stored.
     * @endif
     */
    ConnectorDataListenerHolder connectorData_[CONNECTOR_DATA_LISTENER_NUM];
    /*!
     * @if jp
     * @brief ConnectorListenerTypeリスナ配列
     * ConnectorListenerTypeリスナを格納
     * @else
     * @brief ConnectorListenerType listener array
     * The ConnectorListenerType listener is stored. 
     * @endif
     */
    ConnectorListenerHolder connector_[CONNECTOR_LISTENER_NUM];
  };
};

#endif // RTC_CONNECTORLISTENER_H
