// -*- C++ -*-
/*!
 * @file doil.h
 * @brief doil
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */
#ifndef DOIL_DOIL_H
#define DOIL_DOIL_H

#include <string>
#include <vector>

namespace doil
{
  /*!
   * @if jp
   * @brief リターンコード
   *
   * このクラスは通常モジュール境界に位置するため、エラーは
   * リターンコードで返す。
   *
   * - OK: 正常終了
   * - NOT_FOUND: 指定したものが見つからない
   * - ALREADY_EXISTS: 指定したものが既に存在する
   * - UNKNOWN: 不明なエラー
   *
   * @else
   * @brief Return code
   *
   * This class would be used on the module boundary, so errors
   * are returned as return code.
   *
   * - OK: Normal return
   * - NOT_FOUND: Specified something not found
   * - ALREADY_EXISTS: Specified something already existing
   * - UNKNOWN: Unknown error
   *
   * @endif
   */
#define UNUSED_ARG(a) do {} while (&a == 0)

  enum ReturnCode_t
    {
      OK,
      NOT_FOUND,
      ALREADY_EXISTS,
      INVALID_ARGS,
      UNKNOWN,
    };

  static const char* return_codes[] =
    {
      "OK",
      "NOT_FOUND",
      "ALREADY_EXISTS",
      "INVALID_ARGS",
      "UNKNOWN"
    };

  struct NamedReturnCode
  {
    NamedReturnCode(const char* n, ReturnCode_t r)
      : key_(n), ret_(r)
    {}
    std::string  key_;
    ReturnCode_t ret_;
  };

  class ReturnCodes
  {
  public:
    ReturnCodes()
    {
    }
    ReturnCodes(ReturnCode_t ret)
    {
      push_back("", ret);
    }
    void push_back(NamedReturnCode nr)
    {
      retcodes_.push_back(nr);
    }
    void push_back(const char* key, ReturnCode_t ret)
    {
      retcodes_.push_back(NamedReturnCode(key, ret));
    }

    bool isOK()
    {
      if (retcodes_.size() == 1) return (retcodes_[0].ret_ == OK);
      for (int i(0), len(retcodes_.size()); i < len; ++i)
        {
          ReturnCode_t ret(retcodes_[i].ret_); 
          if (ret != OK) return false;
        }
      return true;
    }
    std::vector<NamedReturnCode> retcodes_;
  };
};

#endif // DOIL_DOIL_H
