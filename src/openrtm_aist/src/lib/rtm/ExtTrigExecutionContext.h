// -*- C++ -*-
/*!
 * @file ExtTrigExecutionContext.h
 * @brief ExtTrigExecutionContext class
 * @date $Date: 2008-01-14 07:49:16 $
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

#ifndef RTC_EXTTRIGEXECUTIONCONTEXT_H
#define RTC_EXTTRIGEXECUTIONCONTEXT_H

#include <rtm/RTC.h>

#include <coil/Mutex.h>
#include <coil/Condition.h>
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
   * @class ExtTrigExecutionContext
   * @brief ステップ実行が可能な ExecutionContext クラス
   *
   * 1周期毎の実行が可能なPeriodic Sampled Data Processing(周期実行用)
   * ExecutionContextクラス。
   * 外部からのメソッド呼びだしによって時間が1周期づつ進む。
   *
   * @since 0.4.0
   *
   * @else
   * @brief ExecutionContext class that enables one step execution
   *
   * ExecutionContext class that can execute every one cycle for Periodic
   * Sampled Data Processing.
   * Time(Tick) advances one cycle by invoking method externally.
   *
   * @since 0.4.0
   *
   * @endif
   */
  class ExtTrigExecutionContext
    : public virtual PeriodicExecutionContext
  {
    typedef coil::Mutex Mutex;
    typedef coil::Condition<Mutex> Condition;
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     *
     * コンストラクタ
     *
     * @else
     * @brief Constructor
     *
     * Constructor
     *
     * @endif
     */
    ExtTrigExecutionContext();
    
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
    virtual ~ExtTrigExecutionContext(void);
    
    /*!
     * @if jp
     * @brief 処理を1ステップ進める
     *
     * ExecutionContextの処理を1周期分進める。
     *
     * @else
     * @brief Move forward one step of ExecutionContext
     *
     * Move forward one step of the ExecutionContext processing.
     *
     * @endif
     */
    virtual void tick()
      throw (CORBA::SystemException);
    
    /*!
     * @if jp
     * @brief 各 Component の処理を呼び出す。
     *
     * ExecutionContext に attach されている各 Component の処理を呼び出す。
     * 全 Component の処理を呼び出した後、次の呼出が発生するまで休止する。
     *
     * @return 処理結果
     *
     * @else
     * @brief Invoke each component's operation
     *
     * Invoke each component's operation which is attached this ExecutionContext.
     * Stop until the next operation is invoked after all component operations
     * are invoked.
     *
     * @return Operation result
     *
     * @endif
     */
    virtual int svc(void);
    
  private:
    struct Worker
    {
      Worker() : _cond(_mutex), _called(false) {};
      Mutex _mutex;
      Condition _cond;
      bool _called;
    };
    // A condition variable for external triggered worker
    Worker m_worker;
  };  // class ExtTrigExecutionContext
};  // namespace RTC

#ifdef WIN32
#pragma warning( default : 4290 )
#endif


extern "C"
{
  /*!
   * @if jp
   * @brief 当該 ExecutionContext 用Factoryクラスの登録。
   *
   * このExecutionContextを生成するFactoryクラスを
   * ExecutionContext管理用ObjectManagerに登録する。
   *
   * @else
   * @brief Register Factory class for this ExecutionContext
   *
   * Register the Factory class to create this ExecutionContext
   * to the ObjectManager for management ExecutionContext
   *
   * @endif
   */
  void ExtTrigExecutionContextInit(RTC::Manager* manager);
};

#endif // RTC_EXTTRIGEXECUTIONCONTEXT_H
