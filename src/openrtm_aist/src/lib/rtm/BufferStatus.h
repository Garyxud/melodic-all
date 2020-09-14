// -*- C++ -*-
/*!
 *
 * @file BufferStatus.h
 * @brief Buffer status enum definition
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
 */

#ifndef RTC_BUFFERSTATUS_H
#define RTC_BUFFERSTATUS_H

namespace RTC
{
  /*!
   * @if jp
   * @class BufferStatus mixin class
   * @brief BufferStatus mixin クラス
   *
   * このクラスは、enum定義されたリターンコードを、バッファ関連サブクラ
   * スで共通利用するための mixin クラスである。このリターンコードを使
   * 用するクラスでは、BufferStatus クラスをpublic 継承し、下に define
   * してある BUFFERSTATUS_ENUM をクラス内に記述することで利用可能とな
   * る。これにより、enum を ReturnCode_t 型として typedef し、以後
   * ReturnCode_t を利用できるようにするとともに、名前空間に enum 定義
   * された各識別子を当該クラス名前空間内に導入する。
   *
   * @else
   * @class BufferStatus mixin class
   * @brief BufferStatus mixin class
   *
   * This is a mixin class to provide enumed return codes that are
   * commonly utilised in buffer realted sub-classes. To use this
   * class, sub-class should inherit this class as a public super
   * class, and declare BUFFERSTATUS_ENUM defined below. Consequently,
   * ReturnCode_t type that is typedefed by this macro can be used in
   * the sub-class, and enumed identifiers are imported to the class's
   * namespace.
   *
   * @endif
   */
  class BufferStatus
  {
  public:
    /*!
     * @if jp
     * @brief BufferStatus リターンコード
     *
     * バッファ関連のクラスで共通のリターンコード
     *
     * - BUFFER_OK:            正常終了
     * - BUFFER_ERROR:         バッファエラー
     * - BUFFER_FULL:          バッファフル
     * - BUFFER_EMPTY:         バッファエンプティ
     * - NOT_SUPPORTED:        未サポート機能
     * - TIMEOUT:              タイムアウト
     * - PRECONDITION_NOT_MET: 事前条件を満たしていない
     *
     * @else
     * @brief DataPortStatus return codes
     *
     * Common return codes for buffer classes.
     *
     * - BUFFER_OK:            Normal return
     * - BUFFER_ERROR:         Buffer error
     * - BUFFER_FULL:          Buffer full
     * - BUFFER_EMPTY:         Buffer empty
     * - NOT_SUPPORTED:        Not supported function
     * - TIMEOUT:              Timeout
     * - PRECONDITION_NOT_MET: Precodition not met
     *
     * @endif
     */
    enum Enum
      {
        BUFFER_OK = 0,
        BUFFER_ERROR,
        BUFFER_FULL,
        BUFFER_EMPTY,
        NOT_SUPPORTED,
        TIMEOUT,
        PRECONDITION_NOT_MET
      };

    /*!
     * @if jp
     *
     * @brief BufferStatus リターンコードを文字列に変換
     *
     * BufferStatus リターンコードを文字列に変換する
     *
     * @param status 変換対象 BufferStatus リターンコード
     *
     * @return 文字列変換結果
     *
     * @else
     *
     * @brief Convert BufferStatus into the string.
     *
     * Convert BufferStatus into the string.
     *
     * @param status The target BufferStatus for transformation
     *
     * @return Trnasformation result of string representation
     *
     * @endif
     */
    static const char* toString(Enum status)
    {
      const char* str[] = {
        "BUFFER_OK",
        "BUFFER_ERROR",
        "BUFFER_FULL",
        "BUFFER_EMPTY",
        "NOT_SUPPORTED",
        "TIMEOUT",
        "PRECONDITION_NOT_MET"
      };
      return str[status];
    }
  };
}; // namespace RTC

/*!
 * @if jp
 *
 * @brief ::RTC::BufferStatus 導入
 * 
 * ::RTC::BufferStatus で宣言されている Enum のすべてのメンバをネーム
 * スペースに導入するためのマクロ。BufferStatus を利用するクラスにお
 * いて、クラス宣言の先頭において DATAPORTSTATUS_ENUM を記載するだけで、
 * BufferStatus で宣言されている enum メンバが名前解決演算子なしにア
 * クセス可能になる。
 *
 * @else
 *
 * @brief Importing ::RTC::BufferStatus macro
 *
 * This macro imports all the member of enum declared in
 * ::RTC::BufferStatus into the current namespace.  Inserting at the
 * head of class declaration, classes which utilize BufferStatus can
 * access Enum members of BufferStatus without using namespace
 * resolve operator.
 *
 * @endif
 */
#define BUFFERSTATUS_ENUM \
  typedef ::RTC::BufferStatus::Enum ReturnCode;       \
  using ::RTC::BufferStatus::BUFFER_OK;               \
  using ::RTC::BufferStatus::BUFFER_ERROR;            \
  using ::RTC::BufferStatus::BUFFER_FULL;             \
  using ::RTC::BufferStatus::BUFFER_EMPTY;            \
  using ::RTC::BufferStatus::NOT_SUPPORTED;           \
  using ::RTC::BufferStatus::TIMEOUT;                 \
  using ::RTC::BufferStatus::PRECONDITION_NOT_MET;

#endif // RTC_BUFFERSTATUS_H
