// -*- C++ -*-
/*!
 * @file DataFlowComponentBase.h
 * @brief DataFlowParticipant RT-Component base class
 * @date $Date: 2007-12-31 03:08:02 $
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

#ifndef RTC_DATAFLOWCOMPONENTBASE_H
#define RTC_DATAFLOWCOMPONENTBASE_H

#include <rtm/idl/RTCSkel.h>
#include <rtm/RTObject.h>
#include <rtm/PeriodicExecutionContext.h>

/*!
 * @if jp
 * @namespace RTC
 *
 * @brief RTコンポーネント
 *
 * @else
 *
 * @namespace RTC
 *
 * @brief RT-Component
 *
 * @endif
 */

namespace RTC
{

  class Manager;
  /*!
   * @if jp
   * @class DataFlowComponentBase
   * @brief DataFlowComponentBase クラス
   *
   * データフロー型RTComponentの基底クラス。
   * 各種データフロー型RTComponentを実装する場合は、本クラスを継承する形で実装
   * する。
   *
   * @since 0.4.0
   *
   * @else
   * @class DataFlowComponentBase
   * @brief DataFlowComponentBase class
   *
   * This is a base class of the data flow type RT-Component.
   * Inherit this class when implementing various data flow type RT-Components.
   *
   * @since 0.4.0
   *
   * @endif
   */
  
  class DataFlowComponentBase
    : public RTObject_impl
  {
  public:
    /*!
     * @if jp
     * @brief コンストラクタ
     *
     * コンストラクタ
     *
     * @param manager マネージャオブジェクト
     *
     * @else
     * @brief Constructor
     *
     * Constructor
     *
     * @param manager Manager object
     *
     * @endif
     */
    DataFlowComponentBase(Manager* manager);
    
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
    virtual ~DataFlowComponentBase(void);
    
    /*!
     * @if jp
     * @brief 初期化
     *
     * データフロー型 RTComponent の初期化を実行する。
     * 実際の初期化処理は、各具象クラス内に記述する。
     *
     * @else
     * @brief Initialization
     *
     * Initialization the data flow type RT-Component.
     * Write the actual initialization code in each concrete class.
     *
     * @endif
     */
    void init();
    
  private:
    //    OpenRTM::DataFlowComponent_var m_ref;
    //    PeriodicExecutionContext* m_pec;
    //    ExecutionContextService_var m_ecref;
  };
}; // namespace RTC
#endif // RTC_DATAFLOWCOMPONENTBASE_H
