// -*- C++ -*-
/*!
 * @file ECFactory.cpp
 * @brief ExecutionContext fuctory class
 * @date $Date: 2007-12-31 03:08:03 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2007
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#include <rtm/RTC.h>
#include <rtm/ECFactory.h>

namespace RTC 
{
  
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  ECFactoryCXX::ECFactoryCXX(const char* name,
			     ECNewFunc new_func,
			     ECDeleteFunc delete_func)
    : m_name(name),
      m_New(new_func),
      m_Delete(delete_func)
  {
  }
  
  /*!
   * @if jp
   * @brief 仮想デストラクタ
   * @else
   * @brief Virtual destructor
   * @endif
   */
  ECFactoryCXX::~ECFactoryCXX()
  {
  }
  
  /*!
   * @if jp
   * @brief 生成対象ExecutionContext名称を取得
   * @else
   * @brief Get names of the target ExecutionContext for creation
   * @endif
   */
  const char* ECFactoryCXX::name()
  {
    return m_name.c_str();
  }
  
  /*!
   * @if jp
   * @brief 生成対象ExecutionContextインスタンスを生成
   * @else
   * @brief Create the target ExecutionContext's instances
   * @endif
   */
  ExecutionContextBase* ECFactoryCXX::create()
  {
    return m_New();
  }
  
  /*!
   * @if jp
   * @brief 対象ExecutionContextインスタンスを破棄
   * @else
   * @brief Destroy the target ExecutionContext's instances
   * @endif
   */
  void ECFactoryCXX::destroy(ExecutionContextBase* ec)
  {
    m_Delete(ec);
  }
  
};
