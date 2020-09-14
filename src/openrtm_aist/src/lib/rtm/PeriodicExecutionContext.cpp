// -*- C++ -*-
/*!
 * @file PeriodicExecutionContext.cpp
 * @brief PeriodicExecutionContext class
 * @date $Date: 2008-01-14 07:53:01 $
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

#include <coil/Time.h>
#include <coil/TimeValue.h>
#include <rtm/PeriodicExecutionContext.h>
#include <rtm/RTObject.h>
#include <algorithm>
#include <iostream>

#define DEEFAULT_PERIOD 0.000001
namespace RTC
{
  /*!
   * @if jp
   * @brief デフォルトコンストラクタ
   * @else
   * @brief Default constructor
   * @endif
   */
  PeriodicExecutionContext::
  PeriodicExecutionContext()
    : rtclog("periodic_ec"), m_running(false), m_svc(true), m_nowait(false)
  {
    RTC_TRACE(("PeriodicExecutionContext()"));

    m_period = (double)DEEFAULT_PERIOD;
    RTC_DEBUG(("Actual rate: %d [sec], %d [usec]",
               m_period.sec(), m_period.usec()));

    // getting my reference
    m_ref = this->_this();

    // profile initialization
    m_profile.kind = PERIODIC;
    m_profile.rate = 1.0 / m_period;
    m_profile.owner = RTC::RTObject::_nil();
    m_profile.participants.length(0);
    m_profile.properties.length(0);
  }
  
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Construnctor
   * @endif
   */
  PeriodicExecutionContext::
  PeriodicExecutionContext(OpenRTM::DataFlowComponent_ptr owner,
			   double rate)
    : rtclog("periodic_ec"), m_running(false), m_svc(true), m_nowait(true)
  {
    RTC_TRACE(("PeriodicExecutionContext(owner, rate = %f)", rate));

    if (rate == 0) { rate = 1.0 / DEEFAULT_PERIOD; }
    m_period = coil::TimeValue(1.0 / rate);
    if (m_period < 0.000001) { m_nowait = true; }
    RTC_DEBUG(("Actual rate: %d [sec], %d [usec]",
               m_period.sec(), m_period.usec()));

    // getting my reference
    m_ref = this->_this();

    // profile initialization
    m_profile.kind = PERIODIC;
    m_profile.rate = 1.0 / m_period;
    m_profile.owner = RTC::RTObject::_nil();
    m_profile.participants.length(0);
    m_profile.properties.length(0);
  }
  
  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  PeriodicExecutionContext::~PeriodicExecutionContext()
  {
    RTC_TRACE(("~PeriodicExecutionContext()"));
    m_worker.mutex_.lock();
    m_worker.running_ = true;
    m_worker.cond_.signal();
    m_worker.mutex_.unlock();
    m_svc = false;
    wait();

    // cleanup EC's profile
    m_profile.owner = RTC::RTObject::_nil();
    m_profile.participants.length(0);
    m_profile.properties.length(0);
  }
  
  /*------------------------------------------------------------
   * Start activity
   * ACE_Task class method over ride.
   *------------------------------------------------------------*/
  /*!
   * @if jp
   * @brief ExecutionContext用アクティビティスレッドを生成する
   * @else
   * @brief Generate internal activity thread for ExecutionContext
   * @endif
   */
  int PeriodicExecutionContext::open(void *args)
  {
    RTC_TRACE(("open()"));
    activate();
    return 0;
  }
  
  /*------------------------------------------------------------
   * Run by a daemon thread to handle deferred processing
   * ACE_Task class method over ride.
   *------------------------------------------------------------*/
  /*!
   * @if jp
   * @brief ExecutionContext 用のスレッド実行関数
   * @else
   * @brief Thread execution function for ExecutionContext
   * @endif
   */
  int PeriodicExecutionContext::svc(void)
  {
    RTC_TRACE(("svc()"));
    int count(0);
    do 
      {
        m_worker.mutex_.lock();
	while (!m_worker.running_)
	  {
	    m_worker.cond_.wait();
	  }
        coil::TimeValue t0(coil::gettimeofday());
	if (m_worker.running_)
	  {
	    std::for_each(m_comps.begin(), m_comps.end(), invoke_worker());
	  }
	m_worker.mutex_.unlock();
        coil::TimeValue t1(coil::gettimeofday());
        if (count > 1000)
          {
            RTC_PARANOID(("Period:    %f [s]", (double)m_period));
            RTC_PARANOID(("Execution: %f [s]", (double)(t1 - t0)));
            RTC_PARANOID(("Sleep:     %f [s]", (double)(m_period - (t1 - t0))));
          }
        coil::TimeValue t2(coil::gettimeofday());
        if (!m_nowait && m_period > (t1 - t0))
          {
            if (count > 1000) { RTC_PARANOID(("sleeping...")); }
            coil::sleep((coil::TimeValue)(m_period - (t1 - t0)));
          }
        if (count > 1000)
          {
            coil::TimeValue t3(coil::gettimeofday());
            RTC_PARANOID(("Slept:     %f [s]", (double)(t3 - t2)));
            count = 0;
          }
        ++count;
      } while (m_svc);

    return 0;
  }
  
  /*!
   * @if jp
   * @brief ExecutionContext 用のスレッド実行関数
   * @else
   * @brief Thread execution function for ExecutionContext
   * @endif
   */
  int PeriodicExecutionContext::close(unsigned long flags)
  {
    RTC_TRACE(("close()"));
    
    // At this point, this component have to be finished.
    // Current state and Next state should be RTC_EXITING.
    //    delete this;
    return 0;
  }
  
  
  //============================================================
  // ExecutionContext
  //============================================================
  /*!
   * @if jp
   * @brief ExecutionContext 実行状態確認関数
   * @else
   * @brief Check for ExecutionContext running state
   * @endif
   */
  CORBA::Boolean PeriodicExecutionContext::is_running()
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("is_running()"));
    return m_running;
  }
  
  /*!
   * @if jp
   * @brief ExecutionContext の実行を開始
   * @else
   * @brief Start the ExecutionContext
   * @endif
   */
  ReturnCode_t PeriodicExecutionContext::start()
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("start()"));
    if (m_running) return RTC::PRECONDITION_NOT_MET;
    
    // invoke ComponentAction::on_startup for each comps.
    std::for_each(m_comps.begin(), m_comps.end(), invoke_on_startup());
    
    // change EC thread state
    m_running = true;
    {
      m_worker.mutex_.lock();
      m_worker.running_ = true;
      m_worker.cond_.signal();
      m_worker.mutex_.unlock();
    }
    
    this->open(0);
    
    return RTC::RTC_OK;
  }
  
  /*!
   * @if jp
   * @brief ExecutionContext の実行を停止
   * @else
   * @brief Stop the ExecutionContext
   * @endif
   */
  ReturnCode_t PeriodicExecutionContext::stop()
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("stop()"));
    if (!m_running) return RTC::PRECONDITION_NOT_MET;
    
    // stop thread
    m_running = false;
    {
      m_worker.mutex_.lock();
      m_worker.running_ = false;
      m_worker.mutex_.unlock();
    }
    // invoke on_shutdown for each comps.
    std::for_each(m_comps.begin(), m_comps.end(), invoke_on_shutdown());
   
    return RTC::RTC_OK;
  }
  
  /*!
   * @if jp
   * @brief ExecutionContext の実行周期(Hz)を取得する
   * @else
   * @brief Get execution rate(Hz) of ExecutionContext
   * @endif
   */
  CORBA::Double PeriodicExecutionContext::get_rate()
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("get_rate()"));
    Guard guard(m_profileMutex);
    return m_profile.rate;
  }
  
  /*!
   * @if jp
   * @brief ExecutionContext の実行周期(Hz)を設定する
   * @else
   * @brief Set execution rate(Hz) of ExecutionContext
   * @endif
   */
  ReturnCode_t PeriodicExecutionContext::set_rate(CORBA::Double rate)
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("set_rate(%f)", rate));
    if (rate > 0.0)
      {
        {
          Guard guard(m_profileMutex);
          m_profile.rate = rate;
        }
	m_period = coil::TimeValue(1.0/rate);
	if (m_period == 0.0) { m_nowait = true; }
	std::for_each(m_comps.begin(), m_comps.end(), invoke_on_rate_changed());
        RTC_DEBUG(("Actual rate: %d [sec], %d [usec]",
                   m_period.sec(), m_period.usec()));
	return RTC::RTC_OK;
      }
    return RTC::BAD_PARAMETER;
  }
  
  /*!
   * @if jp
   * @brief RTコンポーネントをアクティブ化する
   * @else
   * @brief Activate an RT-Component
   * @endif
   */ 
  ReturnCode_t
  PeriodicExecutionContext::activate_component(LightweightRTObject_ptr comp)
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("activate_component()"));
// Why RtORB does not allow STL's find_if and iterator?
#ifndef ORB_IS_RTORB
    CompItr it;
    it = std::find_if(m_comps.begin(), m_comps.end(),
		      find_comp(comp));

    if (it == m_comps.end())
      return RTC::BAD_PARAMETER;

    if (!(it->_sm.m_sm.isIn(INACTIVE_STATE)))
        return RTC::PRECONDITION_NOT_MET;

    it->_sm.m_sm.goTo(ACTIVE_STATE);
    return RTC::RTC_OK;
#else // ORB_IS_RTORB
    for (int i(0); i < (int)m_comps.size() ; ++i)
      {
        if(m_comps.at(i)._ref->_is_equivalent(comp))
          {

            if (!(m_comps.at(i)._sm.m_sm.isIn(INACTIVE_STATE)))
              {
                return RTC::PRECONDITION_NOT_MET;
              }
            m_comps.at(i)._sm.m_sm.goTo(ACTIVE_STATE);
            return RTC::RTC_OK;
          }
      }
    return RTC::BAD_PARAMETER;
#endif // ORB_IS_RTORB
  }
  
  /*!
   * @if jp
   * @brief RTコンポーネントを非アクティブ化する
   * @else
   * @brief Deactivate an RT-Component
   * @endif
   */  
  ReturnCode_t
  PeriodicExecutionContext::deactivate_component(LightweightRTObject_ptr comp)
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("deactivate_component()"));
// Why RtORB does not allow STL's find_if and iterator?
#ifndef ORB_IS_RTORB
    CompItr it;
    it = std::find_if(m_comps.begin(), m_comps.end(),
		      find_comp(comp));
    if (it == m_comps.end()) { return RTC::BAD_PARAMETER; }
    if (!(it->_sm.m_sm.isIn(ACTIVE_STATE)))
      {
        return RTC::PRECONDITION_NOT_MET;
      }
    
    it->_sm.m_sm.goTo(INACTIVE_STATE);
    int count(0);
    const double usec_per_sec(1.0e6);
    double sleeptime(10.0 * usec_per_sec / get_rate());
    RTC_PARANOID(("Sleep time is %f [us]", sleeptime));
    while (it->_sm.m_sm.isIn(ACTIVE_STATE))
      {
        RTC_TRACE(("Waiting to be the INACTIVE state %d %f", count, (double)coil::gettimeofday()));
        coil::usleep(sleeptime);
        if (count > 1000)
          {
            RTC_ERROR(("The component is not responding."));
            break;
          }
        ++count;
      }
    if (it->_sm.m_sm.isIn(INACTIVE_STATE))
      {
        RTC_TRACE(("The component has been properly deactivated."));
        return RTC::RTC_OK;
      }
    RTC_ERROR(("The component could not be deactivated."));
    return RTC::RTC_ERROR;
#else // ORB_IS_RTORB
    for (int i(0); i < (int)m_comps.size(); ++i)
      {
        if(m_comps.at(i)._ref->_is_equivalent(comp))
          {
            if (!(m_comps.at(i)._sm.m_sm.isIn(ACTIVE_STATE)))
              {
                return RTC::PRECONDITION_NOT_MET;
              }
            m_comps.at(i)._sm.m_sm.goTo(INACTIVE_STATE);
            int count(0);
            const double usec_per_sec(1.0e6);
            double sleeptime(usec_per_sec / get_rate());
            RTC_PARANOID(("Sleep time is %f [us]", sleeptime));
            while (m_comps.at(i)._sm.m_sm.isIn(ACTIVE_STATE))
              {
                RTC_TRACE(("Waiting to be the INACTIVE state"));
                coil::usleep(sleeptime);
                
                if (count > 1000)
                  {
                    RTC_ERROR(("The component is not responding."));
                    break;
                  }
                ++count;
              }
            if (m_comps.at(i)._sm.m_sm.isIn(INACTIVE_STATE))
              {
                RTC_TRACE(("The component has been properly deactivated."));
                return RTC::RTC_OK;
              }
            RTC_ERROR(("The component could not be deactivated."));
            return RTC::RTC_ERROR;
          }
      }
    return RTC::BAD_PARAMETER;
#endif // ORB_IS_RTORB
  }
  
  /*!
   * @if jp
   * @brief RTコンポーネントをリセットする
   * @else
   * @brief Reset the RT-Component
   * @endif
   */  
  ReturnCode_t
  PeriodicExecutionContext::reset_component(LightweightRTObject_ptr comp)
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("reset_component()"));
    CompItr it;
    it = std::find_if(m_comps.begin(), m_comps.end(),
		      find_comp(comp));
    if (it == m_comps.end())
      return RTC::BAD_PARAMETER;
    
    if (!(it->_sm.m_sm.isIn(ERROR_STATE)))
      return RTC::PRECONDITION_NOT_MET;
    
    it->_sm.m_sm.goTo(INACTIVE_STATE);
    return RTC::RTC_OK;
  }
  
  /*!
   * @if jp
   * @brief RTコンポーネントの状態を取得する
   * @else
   * @brief Get RT-Component's state
   * @endif
   */
  LifeCycleState
  PeriodicExecutionContext::get_component_state(LightweightRTObject_ptr comp)
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("get_component_state()"));
#ifndef ORB_IS_RTORB
    CompItr it;
    it = std::find_if(m_comps.begin(), m_comps.end(), find_comp(comp));

    if (it == m_comps.end())
      {
	return RTC::CREATED_STATE;
      }
    
    return it->_sm.m_sm.getState();
#else // ORB_IS_RTORB
    for (int i(0); i < (int)m_comps.size(); ++i)
      {
        if(m_comps.at(i)._ref->_is_equivalent(comp))
          {
            return m_comps.at(i)._sm.m_sm.getState();
          }
      }
    return RTC::CREATED_STATE; 
#endif // ORB_IS_RTORB
  }
  
  /*!
   * @if jp
   * @brief ExecutionKind を取得する
   * @else
   * @brief Get the ExecutionKind
   * @endif
   */
  ExecutionKind PeriodicExecutionContext::get_kind()
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("get_kind()"));
    return m_profile.kind;
  }
  
  /*!
   * @if jp
   * @brief RTコンポーネントを追加する
   * @else
   * @brief Add an RT-Component
   * @endif
   */
  ReturnCode_t
  PeriodicExecutionContext::add_component(LightweightRTObject_ptr comp)
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("add_component()"));
    if (CORBA::is_nil(comp)) return RTC::BAD_PARAMETER;
    
    try
      {
	OpenRTM::DataFlowComponent_var dfp;
	dfp = OpenRTM::DataFlowComponent::_narrow(comp);
        RTC::RTObject_var rtc;
        rtc = RTC::RTObject::_narrow(comp);
        //Check the pointer.
        if(CORBA::is_nil(dfp) || CORBA::is_nil(rtc))
          {
	    return RTC::BAD_PARAMETER;
          }
	ExecutionContextHandle_t id;
	id = dfp->attach_context(m_ref);
	m_comps.push_back(Comp(comp, dfp, id));
        CORBA_SeqUtil::push_back(m_profile.participants, rtc._retn());
	return RTC::RTC_OK;
      }
    catch (CORBA::Exception& e)
      {
	(void)(e);
	return RTC::BAD_PARAMETER;
      }
    return RTC::RTC_OK;
  }

  RTC::ReturnCode_t PeriodicExecutionContext::bindComponent(RTObject_impl* rtc)
  {
    RTC_TRACE(("bindComponent()"));
    if (rtc == NULL) return RTC::BAD_PARAMETER;

    LightweightRTObject_var comp = RTC::RTObject::_duplicate(rtc->getObjRef());
    OpenRTM::DataFlowComponent_var dfp;
    dfp = OpenRTM::DataFlowComponent::_narrow(comp);

    ExecutionContextHandle_t id = rtc->bindContext(m_ref);
    if (id < 0 || id > ECOTHER_OFFSET) 
      {
        // id should be owned context id < ECOTHER_OFFSET
        RTC_ERROR(("bindContext returns invalid id: %d", id));
        return RTC::RTC_ERROR;
      }
    RTC_DEBUG(("bindContext returns id = %d", id));

    // rtc is owner of this EC
    m_comps.push_back(Comp(comp,dfp,id));
    m_profile.owner = RTC::RTObject::_duplicate(dfp);

    return RTC::RTC_OK;
  }
  
  /*!
   * @if jp
   * @brief コンポーネントをコンポーネントリストから削除する
   * @else
   * @brief Remove the RT-Component from participant list
   * @endif
   */	
  ReturnCode_t
  PeriodicExecutionContext::remove_component(LightweightRTObject_ptr comp)
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("remove_component()"));
    CompItr it;
    it = std::find_if(m_comps.begin(), m_comps.end(),
		      find_comp(comp));
    if (it == m_comps.end())
      {
        RTC_TRACE(("remove_component(): no RTC found in this context."));
        return RTC::BAD_PARAMETER;
      }

    Comp& c(*it);
    c._ref->detach_context(c._sm.ec_id);
    c._ref = RTC::LightweightRTObject::_nil();
    m_comps.erase(it);
    RTC_TRACE(("remove_component(): an RTC removed from this context."));

    //RTObject_var rtcomp = RTObject::_narrow(LightweightRTObject::_duplicate(comp));
    RTObject_var rtcomp = RTObject::_narrow(comp);
    if (CORBA::is_nil(rtcomp))
      {
        RTC_ERROR(("Invalid object reference."));
        return RTC::RTC_ERROR;
      }
    {
      Guard guard(m_profileMutex);
      CORBA_SeqUtil::erase_if(m_profile.participants,
                              find_participant(rtcomp));
    }
    return RTC::RTC_OK;
  }
  
  //============================================================
  // ExecutionContextService interfaces
  //============================================================
  /*!
   * @if jp
   * @brief ExecutionContextProfile を取得する
   * @else
   * @brief Get the ExecutionContextProfile
   * @endif
   */
  ExecutionContextProfile* PeriodicExecutionContext::get_profile()
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("get_profile()"));
    ExecutionContextProfile_var p;
    {
      Guard guard(m_profileMutex);
      p = new ExecutionContextProfile(m_profile);
    }
    return p._retn();
  }
}; // namespace RTC  

extern "C"
{
  /*!
   * @if jp
   * @brief ECFactoryへの登録のための初期化関数
   * @else
   * @brief Initialization function to register to ECFactory
   * @endif
   */
  void PeriodicExecutionContextInit(RTC::Manager* manager)
  {
    manager->registerECFactory("PeriodicExecutionContext",
			       RTC::ECCreate<RTC::PeriodicExecutionContext>,
			       RTC::ECDelete<RTC::PeriodicExecutionContext>);
  }
};

