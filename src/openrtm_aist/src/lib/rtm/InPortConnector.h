// -*- C++ -*-
/*!
 * @file InPortConnector.h
 * @brief InPortConnector base class
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2009-2010
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

#ifndef RTC_INPORTCONNECTOR_H
#define RTC_INPORTCONNECTOR_H

#include <rtm/ConnectorBase.h>

namespace RTC
{
  /*!
   * @if jp
   * @class InPortConnector
   * @brief InPortConnector 基底クラス
   *
   * InPort の Push/Pull 各種 Connector を派生させるための基底クラス。
   *
   * @since 1.0.0
   *
   * @else
   * @class InPortConnector
   * @brief InPortConnector base class
   *
   * The base class to derive subclasses for InPort's Push/Pull Connectors
   *
   * @since 1.0.0
   *
   * @endif
   */
  class InPortConnector
    : public ConnectorBase
  {
  public:
    DATAPORTSTATUS_ENUM
    /*!
     * @if jp
     * @brief コンストラクタ
     *
     * @param info 接続情報を含む ConnectorInfo オブジェクト
     * @param buffer このコネクタのバッファへのポインタ
     *
     * @else
     * @brief Constructor
     *
     * @param info ConnectorInfo object which includes connection information
     * @param buffer A pointer to the buffer of the connector
     *
     * @endif
     */
    InPortConnector(ConnectorInfo& info,
                    CdrBufferBase* buffer);

    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~InPortConnector();

   /*!
     * @if jp
     * @brief ConnectorInfo 取得
     *
     * Connector ConnectorInfo を取得する
     *
     * @return このコネクタが保持する ConnectorInfo オブジェクト
     *
     * @else
     * @brief Getting ConnectorInfo
     *
     * This operation returns ConnectorInfo
     *
     * @return ConnectorInfo object which is owned by this connector
     *
     * @endif
     */
    virtual const ConnectorInfo& profile();

    /*!
     * @if jp
     * @brief Connector ID 取得
     *
     * Connector ID を取得する
     *
     * @return コネクタ ID 文字列へのポインタ
     *
     * @else
     * @brief Getting Connector ID
     *
     * This operation returns Connector ID
     *
     * @return A pointer to the connector id string
     *
     * @endif
     */
    virtual const char* id();

    /*!
     * @if jp
     * @brief Connector 名取得
     *
     * Connector 名を取得する
     *
     * @return コネクタ名文字列へのポインタ
     *
     * @else
     * @brief Getting Connector name
     *
     * This operation returns Connector name
     *
     * @return A pointer to the connector's name string
     *
     * @endif
     */
    virtual const char* name();

    /*!
     * @if jp
     * @brief 接続解除関数
     *
     * Connector が保持している接続を解除する
     *
     * @return ReturnCode
     *
     * @else
     * @brief Disconnect connection
     *
     * This operation disconnect this connection
     *
     * @return ReturnCode
     *
     * @endif
     */
    virtual ReturnCode disconnect() = 0;

    /*!
     * @if jp
     * @brief Buffer を取得する
     *
     * Connector が保持している Buffer を返す
     *
     * @return このコネクタが保持するバッファへのポインタ
     *
     * @else
     * @brief Getting Buffer
     *
     * This operation returns this connector's buffer
     *
     * @return A pointer to the buffer owned by this connector
     *
     * @endif
     */
    virtual CdrBufferBase* getBuffer();

    /*!
     * @if jp
     * @brief read 関数
     *
     * Buffer からデータを InPort へ read する関数
     *
     * @param data このコネクタから読み出されるデータを格納する変数への参照
     * @return ReturnCode
     *
     * @else
     * @brief Destructor
     *
     * The read function to read data from buffer to InPort
     *
     * @param data A reference to a variable to which data from this
     *             connector is stored.
     * @return ReturnCode
     *
     * @endif
     */
    virtual ReturnCode read(cdrMemoryStream& data) = 0;

    /*!
     * @if jp
     * @brief endianタイプ設定
     *
     * endianタイプを設定する
     *
     * @param endian_type true: little, false: big
     *
     * @else
     * @brief Setting an endian type
     *
     * This operation set this connector's endian type
     *
     * @param endian_type true: little, false: big
     *
     * @endif
     */
    virtual void setEndian(const bool endian_type);

    /*!
     * @if jp
     * @brief endian 設定を返す
     *
     * このコネクタに設定されたエンディアンが little endian かどうか。
     *
     * @return true: little endian, false: big endian
     *
     * @else
     * @brief Whether this connector's endian is little
     *
     * This operation returns whether the connector's endian is little or not.
     *
     * @return true: little endian, false: big endian
     *
     * @endif
     */
    virtual bool isLittleEndian();

  protected:
    /*!
     * @if jp
     * @brief ロガーストリーム
     * @else
     * @brief Logger stream
     * @endif
     */
    Logger rtclog;
    /*!
     * @if jp
     * @brief ConnectorInfo
     * @else
     * @brief ConnectorInfo 
     * @endif
     */
    ConnectorInfo m_profile;
    /*!
     * @if jp
     * @brief Connector が保持している Buffer
     * @else
     * @brief Connector's buffer
     * @endif
     */
    CdrBufferBase* m_buffer;
    /*!
     * @if jp
     * @brief 接続エンディアン 
     * @else
     * @brief Connected Endian
     * @endif
     */
    bool m_littleEndian;
  };
}; // namespace RTC

#endif // RTC_INPORTCONNECTOR_H
