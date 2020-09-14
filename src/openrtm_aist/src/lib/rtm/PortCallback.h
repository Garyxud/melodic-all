// -*- C++ -*-
/*!
 * @file PortCallback.h
 * @brief PortCallback class
 * @date $Date: 2007-12-31 03:08:05 $
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

#ifndef RTC_PORTCALLBACK_H
#define RTC_PORTCALLBACK_H

class cdrStream;

namespace RTC
{
  //============================================================
  // callback functor base classes
  /*!
   * @if jp
   * @class ConnectionCallback
   * @brief connect/notify_connect() 時のコールバック抽象クラス
   *
   * Portに対してconnect/notify_connect() 等が呼び出される時に呼び出される
   * コールバックファンクタ。引数に RTC::ConnectorProfile を取る。
   *
   * @param profile ConnectorProfile
   *
   * @since 1.0.0
   *
   * @else
   * @class ConnectionCallback
   * @brief Callback functor abstract for connect/notify_connect() funcs
   *
   * This is the interface for callback functor for connect/notify_connect()
   * invocation in Port. Argument is RTC::ConnectorProfile that is given
   * these functions.
   *
   * @param profile ConnectorProfile
   *
   * @since 1.0.0
   *
   * @endif
   */
  class ConnectionCallback
  {
  public:
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
    virtual ~ConnectionCallback(void){}

    /*!
     * @if jp
     *
     * @brief コールバック関数
     *
     * connect/notify_connect() 等が呼び出される時に呼び出される
     * コールバック関数
     *
     * @param profile ConnectorProfile
     *
     * @else
     *
     * @brief Callback method
     *
     * This is the callback method invoked when connect/notify_connect()
     * invocation in Port.
     *
     * @param profile ConnectorProfile
     *
     * @endif
     */
    virtual void operator()(RTC::ConnectorProfile& profile) = 0;
  };


  /*!
   * @if jp
   * @class DisconnectCallback
   * @brief disconnect/notify_disconnect() 時のコールバック抽象クラス
   *
   * Portに対してdisconnect/notify_disconnect() 等が呼び出される時に呼
   * び出されるコールバックファンクタ。引数に接続IDを取る。
   *
   * @param connector_id Connector ID
   *
   * @since 1.0.0
   *
   * @else
   * @class DisconnectCallback
   * @brief Callback functor abstract for disconnect/notify_disconnect() funcs
   *
   * This is the interface for callback functor for
   * disconnect/notify_disconnect() invocation in Port. Argument is
   * connector ID is given these functions.
   *
   * @param connector_id Connector ID
   *
   * @since 1.0.0
   *
   * @endif
   */
  class DisconnectCallback
  {
  public:
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
    virtual ~DisconnectCallback(void){}
    /*!
     * @if jp
     *
     * @brief コールバック関数
     *
     * disconnect/notify_disconnect() 等が呼び出される時に呼び出される
     * コールバック関数
     *
     * @param connector_id Connector ID
     *
     * @else
     *
     * @brief Callback method
     *
     * This is the callback method invoked when disconnect/notify_disconnect()
     * invocation in Port.
     *
     * @param connector_id Connector ID
     *
     * @endif
     */
    virtual void operator()(const char* connector_id) = 0;
  };


  /*!
   * @if jp
   * @class OnWrite
   * @brief write() 時のコールバック抽象クラス
   *
   * OutPortに対してデータがwrite()される直前に呼び出されるコールバック用
   * ファンクタ。
   *
   * @param DataType バッファに書き込むデータ型
   *
   * @since 0.4.0
   *
   * @else
   * @class OnWrite
   * @brief Callback abstract class on write()
   *
   * This is the interface for callback invoked immediately before
   * data is done write() into the DataPort's buffer.
   *
   * @param DataType Data type to write into the buffer
   *
   * @since 0.4.0
   *
   * @endif
   */
  template <class DataType>
  class OnWrite
  {
  public:
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
    virtual ~OnWrite(void){}
    
    /*!
     * @if jp
     *
     * @brief コールバック関数
     *
     * バッファにデータが書き込まれる直前に呼び出されるコールバック関数
     *
     * @param value バッファに書き込まれるデータ
     *
     * @else
     *
     * @brief Callback function
     *
     * This is the callback method invoked immediately before data is written
     * into the buffer.
     *
     * @param value Data that is written into the buffer
     *
     * @endif
     */
    virtual void operator()(const DataType& value) = 0;
  };
  

  /*!
   * @if jp
   * @class OnWriteConvert
   * @brief write() 時のデータ変換コールバック抽象クラス
   *
   * OutPortのバッファにデータが write()される時に呼び出されるコールバッ
   * ク用インターフェース。このコールバックの戻り値がバッファに格納され
   * る。
   *
   * @since 0.4.0
   *
   * @else
   * @class OnWriteConvert
   * @brief Data convert callback abstract class on write()
   *
   * This is the interface for callback invoked when data is done
   * write() into the OutPort's buffer.  The return value of this
   * callback will be stored in the buffer.
   *
   * @since 0.4.0
   *
   * @endif
   */
  template <class DataType>
  struct OnWriteConvert
  {
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
    virtual ~OnWriteConvert(void){}
    
    /*!
     * @if jp
     *
     * @brief コールバック関数
     *
     * バッファにデータが書き込まれる際に呼び出されるコールバック関数。
     *
     * @param value 変換前データ
     * @return 変換後データ
     *
     * @else
     *
     * @brief Callback function
     *
     * This is the callback function invoked when data is written into the
     * buffer.
     *
     * @param value Data to be converted
     * @return Converted data
     *
     * @endif
     */
    virtual DataType operator()(const DataType& value) = 0;
  };
  

  /*!
   * @if jp
   * @class OnRead
   * @brief read() 時のコールバック抽象クラス
   *
   * InPort のバッファからデータが read()される直前に呼び出される
   * コールバック用インターフェース。
   *
   * @since 0.4.0
   *
   * @else
   * @class OnRead
   * @brief Callback abstract class on read()
   *
   * This is the interface for callback invoked immediately before
   * data is done read() from the InPort's buffer.
   *
   * @since 0.4.0
   *
   * @endif
   */
  template <class DataType>
  struct OnRead
  {
    /*!
     * @if jp
     *
     * @brief デストラクタ
     *
     * デストラクタ
     *
     * @else
     *
     * @brief Destructor
     *
     * Destructor
     *
     * @endif
     */
    virtual ~OnRead(void){}
    
    /*!
     * @if jp
     *
     * @brief コールバックメソッド
     *
     * バッファからデータが読み出される直前に呼び出されるコールバック関数。
     *
     * @else
     *
     * @brief Callback function
     *
     * This is the callback method invoked immediately before data is readout
     * from the buffer.
     *
     * @endif
     */
    virtual void operator()() = 0;
  };
  
  /*!
   * @if jp
   * @class OnReadConvert
   * @brief read() 時のデータ変換コールバック抽象クラス
   *
   * InPort のバッファからデータが read()される際に呼び出される
   * コールバック用インターフェース。
   * このコールバックの戻り値がread()の戻り値となる。
   *
   * @since 0.4.0
   *
   * @else
   * @class OnReadConvert
   * @brief Data convert callback abstract class on read()
   *
   * This is the interface for callback invoked when data is done read()
   * from the InPort/OutPort's buffer.
   * The return value of this callback will be the return value of read().
   *
   * @since 0.4.0
   *
   * @endif
   */
  template <class DataType>
  struct OnReadConvert
  {
    /*!
     * @if jp
     *
     * @brief デストラクタ
     *
     * デストラクタ
     *
     * @else
     *
     * @brief Destructor
     *
     * Destructor
     *
     * @endif
     */
    virtual ~OnReadConvert(void){}
    
    /*!
     * @if jp
     *
     * @brief コールバックメソッド
     *
     * バッファからデータが読み出される際に呼び出されるコールバック関数
     * であり、operator()() の戻り値は InPort の read() の戻り値となる、
     * またはデータ変数に格納される。
     *
     * @param value バッファから読みだされたデータ
     * @return 変換後のデータ。データポート変数にはこの値が格納される。
     *
     * @else
     *
     * @brief Callback method
     *
     * This function is the callback function invoked when data is
     * readout from the buffer, and the return value of operator()()
     * is used as return value of InPort's read() or it is stored in
     * the InPort data variable.
     *
     * @param value Data that is readout from buffer
     * @return Converted data. These data are stored in the port's variable.
     *
     * @endif
     */
    virtual DataType operator()(const DataType& value) = 0;
  };
  

};
#endif // RTC_PORTCALLBACK_H
