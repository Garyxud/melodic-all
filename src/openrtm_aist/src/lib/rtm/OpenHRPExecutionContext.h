// -*- C++ -*-
/*!
 * @file  OpenHRPExecutionContext.h
 * @brief Execution context for OpenHRP3
 * @date  $Date: 2008-01-14 07:49:59 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006-2008
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

#ifndef RTC_OPENHRPEXECUTIONCONTEXT_H
#define RTC_OPENHRPEXECUTIONCONTEXT_H

#include <rtm/RTC.h>

#include <coil/Task.h>

#include <rtm/Manager.h>
#include <rtm/PeriodicExecutionContext.h>

#ifdef WIN32
#pragma warning( disable : 4290 )
#endif

namespace RTC
{
  /*!
   * @if jp
   * @class OpenHRPExecutionContext
   * @brief OpenHRPExecutionContext クラス
   *
   * OpenHRP3のための実行コンテキスト。外部からの tick() 呼び出しにより、1周期
   * 分だけ処理を進めることができる。
   *
   * @else
   * @class OpenHRPExecutionContext
   * @brief OpenHRPExecutionContext class
   *
   * This is ExecutionContext for OpenHRP3.  This EC can execute only
   * one cycle by tick() call from external framework.
   *
   *
   * @endif
   */
  class OpenHRPExecutionContext
    : public virtual PeriodicExecutionContext
  {
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     * @else
     * @brief Constructor
     * @endif
     */
    OpenHRPExecutionContext();

    /*!
     * @if jp
     * @brief デストラクタ
     * @else
     * @brief Destructor 
     * @endif
     */
    virtual ~OpenHRPExecutionContext(void);

    /*!
     * @if jp
     * @brief ExecutionContextの処理を進める
     *
     * ExecutionContextの処理を１周期分進める。
     *
     * @else
     * @brief Proceed with tick of ExecutionContext
     *
     * Proceed with tick of ExecutionContext for one period.
     *
     * @endif
     */
    virtual void tick(void)
      throw (CORBA::SystemException);

    /*!
     * @if jp
     * @brief ExecutionContext のスレッド実行フラグ
     * @else
     * @brief The thread running flag of ExecutionContext
     * @endif
     */
    virtual int svc(void);

  private:
  };  // class OpenHRPExecutionContext
};  // namespace RTC

#ifdef WIN32
#pragma warning( default : 4290 )
#endif


extern "C"
{
  /*!
   * @if jp
   * @brief ECFactoryへの登録のための初期化関数
   * @else
   * @brief Initialization function to register to ECFactory
   * @endif
   */
  DLL_EXPORT void OpenHRPExecutionContextInit(RTC::Manager* manager);
};

#endif // RTC_OPENHRPEXECUTIONCONTEXT_H

