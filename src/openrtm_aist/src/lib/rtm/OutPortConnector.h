// -*- C++ -*-
/*!
 * @file OutPortConnector.h
 * @brief OutPortConnector class
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

#ifndef RTC_OUTPORTCONNECTOR_H
#define RTC_OUTPORTCONNECTOR_H

#include <rtm/SystemLogger.h>
#include <rtm/ConnectorBase.h>

namespace RTC
{
  /*!
   * @if jp
   * @class OutPortConnector
   * @brief OutPortConnector 基底クラス
   *
   * OutPort の Push/Pull 各種 Connector を派生させるための
   * 基底クラス。
   *
   * @since 1.0.0
   *
   * @else
   * @class OutPortConnector
   * @brief OutPortConnector base class
   *
   * The base class to derive subclasses for OutPort's Push/Pull Connectors
   *
   * @since 1.0.0
   *
   * @endif
   */
  class OutPortConnector
    : public ConnectorBase
  {
  public:
    DATAPORTSTATUS_ENUM

    /*!
     * @if jp
     * @brief コンストラクタ
     * @else
     * @brief Constructor
     * @endif
     */
    OutPortConnector(ConnectorInfo& info);

    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor
     * @endif
     */
    virtual ~OutPortConnector();
   /*!
     * @if jp
     * @brief Profile 取得
     *
     * Connector Profile を取得する
     *
     * @else
     * @brief Getting Profile
     *
     * This operation returns Connector Profile
     *
     * @endif
     */
    const ConnectorInfo& profile();

    /*!
     * @if jp
     * @brief Connector ID 取得
     *
     * Connector ID を取得する
     *
     * @else
     * @brief Getting Connector ID
     *
     * This operation returns Connector ID
     *
     * @endif
     */
    const char* id();

    /*!
     * @if jp
     * @brief Connector 名取得
     *
     * Connector 名を取得する
     *
     * @else
     * @brief Getting Connector name
     *
     * This operation returns Connector name
     *
     * @endif
     */
    const char* name();

    /*!
     * @if jp
     * @brief 接続解除関数
     *
     * Connector が保持している接続を解除する
     *
     * @else
     * @brief Disconnect connection
     *
     * This operation disconnect this connection
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
     * @else
     * @brief Getting Buffer
     *
     * This operation returns this connector's buffer
     *
     * @endif
     */
    virtual CdrBufferBase* getBuffer() = 0;

    /*!
     * @if jp
     * @brief write 関数
     *
     * OutPort からデータを Buffer へ write する関数
     *
     * @else
     * @brief Destructor
     *
     * The write function to write data from OutPort to Buffer
     *
     * @endif
     */
    virtual ReturnCode write(const cdrMemoryStream& data) = 0;

    /*!
     * @if jp
     * @brief endianタイプ設定
     *
     * endianタイプを設定する
     *
     * @else
     * @brief Setting an endian type
     *
     * This operation set this connector's endian type
     *
     * @endif
     */
    virtual void setEndian(const bool endian_type);

    /*!
     * @if jp
     * @brief endian 設定を返す
     *
     * endian 設定のbool値を返す。
     *
     * @return m_littleEndian がlittleの場合true、bigの場合false を返す。
     *
     * @else
     * @brief
     *
     * return it whether endian setting.
     *
     *@return Return true in the case of "little", false in "big" than it.
     *
     * @endif
     */
    virtual bool isLittleEndian();

    /*!
     * @if jp
     * @brief データ型の変換テンプレート
     *
     * Timed* から CdrMemoryStream に変換する。
     *
     * @else
     * @brief The conversion template of the data type
     *
     * This is convert it from Timed* into CdrStream.
     *
     * @endif
     */
    template <class DataType>
    ReturnCode write(const DataType& data)
    {
      m_cdr.rewindPtrs();
      RTC_TRACE(("connector endian: %s", isLittleEndian() ? "little":"big"));
      m_cdr.setByteSwapFlag(isLittleEndian());
      data >>= m_cdr;
      return write(m_cdr);
    }

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
     * @brief Port の PortProfile
     * @else
     * @brief PortProfile of the Port
     * @endif
     */
    ConnectorInfo m_profile;
    /*!
     * @if jp
     * @brief 接続エンディアン 
     * @else
     * @brief Connected Endian
     * @endif
     */
    bool m_littleEndian;
    /*!
     * @if jp
     * @brief cdrストリーム
     * @else
     * @brief CDR stream
     * @endif
     */
    cdrMemoryStream m_cdr;

  };
}; // namespace RTC

#endif // RTC_CONNECTORBASE_H
