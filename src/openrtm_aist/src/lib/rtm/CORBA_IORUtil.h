// -*- C++ -*-
/*!
 * @file CORBA_IORUtil.h
 * @brief CORBA IOR manipulation utility functions
 * @date $Date: 2007-12-31 03:06:24 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2010
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef CORBA_IORUTIL_H
#define CORBA_IORUTIL_H

#include <rtm/RTC.h>

/*!
 * @if jp
 * @namespace CORBA_IORUtil
 *
 * @brief CORBA IOR ヘルパー関数
 *
 *
 * @else
 *
 * @namespace CORBA_SeqUtil
 *
 * @brief CORBA IOR helper functions
 *
 * @endif
 */
namespace CORBA_IORUtil
{
  /*!
   * @if jp
   * @brief IOR 文字列をIOR構造体へ変換する
   *
   * @else
   * @brief Convert from IOR string to IOR structure
   *
   * @endif
   */
  bool toIOR(const char* iorstr, IOP::IOR& ior);

  /*!
   * @if jp
   * @brief IOR構造体をIOR文字列へ変換する
   *
   * @else
   * @brief Convert from IOR structure to IOR string 
   *
   * @endif
   */
  bool toString(IOP::IOR& ior, std::string& iorstr);
  
  /*!
   * @if jp
   * @brief IOR内のエンドポイントを置換する
   * @else
   * @brief Replace endpoint address in IOR entry
   * @endif
   */
  bool replaceEndpoint(std::string& iorstr, const std::string& endpoint);

  /*!
   * @if jp
   * @brief IOR文字列から情報を抽出しフォーマット済みの文字列として返す
   * @else
   * @brief Extracts information from IOR string and returns formatted string
   * @endif
   */
  std::string formatIORinfo(const char* iorstr);

}; // namespace CORBA_IORUtil
#endif // CORBA_IORUTIL_H
