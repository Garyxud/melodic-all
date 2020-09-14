// -*- C++ -*-

#include "OpenHRPExecutionContext.h"
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
  OpenHRPExecutionContext::OpenHRPExecutionContext()
    : PeriodicExecutionContext()
  {
  }

  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor 
   * @endif
   */
  OpenHRPExecutionContext::~OpenHRPExecutionContext()
  {
  }

  /*!
   * @if jp
   * @brief ExecutionContextの処理を進める
   * @else
   * @brief Proceed with tick of ExecutionContext
   * @endif
   */
  void OpenHRPExecutionContext::tick()
    throw (CORBA::SystemException)
  {
    std::for_each(m_comps.begin(), m_comps.end(), invoke_worker());
    return;
  }

  /*!
   * @if jp
   * @brief ExecutionContext のスレッド実行フラグ
   * @else
   * @brief The thread running flag of ExecutionContext
   * @endif
   */
  int OpenHRPExecutionContext::svc(void)
  {
    return 0;
  }
};


extern "C"
{
  /*!
   * @if jp
   * @brief ECFactoryへの登録のための初期化関数
   * @else
   * @brief Initialization function to register to ECFactory
   * @endif
   */
  void OpenHRPExecutionContextInit(RTC::Manager* manager)
  {
    manager->registerECFactory("SynchExtTriggerEC",
			       RTC::ECCreate<RTC::OpenHRPExecutionContext>,
			       RTC::ECDelete<RTC::OpenHRPExecutionContext>);
    
  }
};
