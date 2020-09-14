// -*- C++ -*-
/*!
 * @file ExtTrigExecutionContext.cpp
 * @brief ExtTrigExecutionContext class
 * @date $Date: 2008-01-14 07:49:14 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2007-2008
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

#include <coil/TimeValue.h>
#include <rtm/ExtTrigExecutionContext.h>
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
  ExtTrigExecutionContext::ExtTrigExecutionContext()
    : PeriodicExecutionContext()
  {
    rtclog.setName("exttrig_ec");
  }
  
  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  ExtTrigExecutionContext::~ExtTrigExecutionContext()
  {
  }
  
  /*!
   * @if jp
   * @brief 処理を1ステップ進める
   * @else
   * @brief Move forward one step of ExecutionContext
   * @endif
   */
  void ExtTrigExecutionContext::tick()
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("tick()"));
    m_worker._mutex.lock();
    m_worker._called = true;
    m_worker._cond.signal();
    m_worker._mutex.unlock();
    return;
  }
  
  /*!
   * @if jp
   * @brief 各 Component の処理を呼び出す。
   * @else
   * @brief Invoke each component's operation
   * @endif
   */
  int ExtTrigExecutionContext::svc(void)
  {
    RTC_TRACE(("svc()"));
    do
      {
	m_worker._mutex.lock();
	while (!m_worker._called && m_running)
	  {
	    m_worker._cond.wait();
	  }
	if (m_worker._called)
	  {
	    m_worker._called = false;
	    std::for_each(m_comps.begin(), m_comps.end(), invoke_worker());
	  }
	m_worker._mutex.unlock();
      } while (m_running);
    
    return 0;
  }
};


extern "C"
{
  /*!
   * @if jp
   * @brief 当該 ExecutionContext 用Factoryクラスの登録。
   * @else
   * @brief Register Factory class for this ExecutionContext
   * @endif
   */
  void ExtTrigExecutionContextInit(RTC::Manager* manager)
  {
    manager->registerECFactory("ExtTrigExecutionContext",
			       RTC::ECCreate<RTC::ExtTrigExecutionContext>,
			       RTC::ECDelete<RTC::ExtTrigExecutionContext>);
  }
};
