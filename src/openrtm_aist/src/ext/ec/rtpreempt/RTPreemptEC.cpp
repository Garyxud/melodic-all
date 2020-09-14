// -*- C++ -*-
/*!
 * @file RTPreemptEC.cpp
 * @brief RTPreemptEC class
 * @date $Date$
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

#include "RTPreemptEC.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>
#include <rtm/ECFactory.h>
#include <rtm/Manager.h>
#include <coil/stringutil.h>

#define MAX_SAFE_STACK (8*1024)
#define NSEC_PER_SEC 1000000000

namespace OpenRTM
{
  /*!
   * @if jp
   * @brief デフォルトコンストラクタ
   * @else
   * @brief Default Constructor
   * @endif
   */
  RTPreemptEC::RTPreemptEC()
    : ::RTC::PeriodicExecutionContext(),
      m_priority(49), m_policy(SCHED_FIFO), m_waitoffset(0)
  {
    rtclog.setName("RTPreemptEC");
    coil::Properties& prop(::RTC::Manager::instance().getConfig());

    // Priority
    getProperty(prop, "exec_cxt.periodic.priority", m_priority);
    getProperty(prop, "exec_cxt.periodic.rtpreempt.priority", m_priority);
    RTC_DEBUG(("Priority: %d", m_priority));

    // Policy
    {
      std::string policy;
      getProperty(prop, "exec_cxt.periodic.rtpreempt.sched_policy", policy);
      if (!policy.empty())
        {
          coil::normalize(policy);
          if (policy == "rr")   { m_policy = SCHED_RR; }
          if (policy == "fifo") { m_policy = SCHED_FIFO; }
          RTC_DEBUG(("Scheduling policy: %s", policy.c_str()));
        }
      else
        {
          RTC_DEBUG(("Scheduling policy: fifo"));
        }
    }

    // Wait offset
    getProperty(prop, "exec_cxt.periodic.rtpreempt.wait_offset", m_waitoffset);
    RTC_DEBUG(("Wait offset: %d [ns]", m_waitoffset));
    
  }

  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  RTPreemptEC::~RTPreemptEC()
  {
  }

  /*!
   * @if jp
   * @brief ExecutionContext 用のスレッド実行関数
   * @else
   * @brief Thread execution function for ExecutionContext
   * @endif
   */ 
  int RTPreemptEC::svc(void)
  {
    // schedulaer setting
    struct sched_param param;
    param.sched_priority = m_priority;
    if(sched_setscheduler(0, m_policy, &param) == -1)
      {
        std::cerr << "sched_setscheduler failed" << std::endl;
        return -1;
      }

    // memory locking
    if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
      {
        std::cerr << "mlockall failed" << std::endl;
        return -1;
      }

    // stack preallocation
    unsigned char dummy[MAX_SAFE_STACK];
    memset(&dummy, 0, MAX_SAFE_STACK);
    struct timespec t0, t1, t;
    do
      {
        clock_gettime(CLOCK_MONOTONIC ,&t0);
        m_worker.mutex_.lock();
        while (!m_worker.running_)
          {
            m_worker.cond_.wait();
          }
        if (m_worker.running_)
          {
            std::for_each(m_comps.begin(), m_comps.end(), invoke_worker());
          }
        m_worker.mutex_.unlock();
        clock_gettime(CLOCK_MONOTONIC ,&t1);

        if (!m_nowait)
          {
            if (t0.tv_nsec > t1.tv_nsec)
              {
                t.tv_nsec = m_period.usec() * 1000
                  - (NSEC_PER_SEC - t0.tv_nsec + t1.tv_nsec) + m_waitoffset;
                t.tv_sec  = m_period.sec()
                  - (t1.tv_sec - 1 - t0.tv_sec);
              }
            else
              {
                t.tv_nsec = m_period.usec() * 1000
                  - (t1.tv_nsec - t0.tv_nsec) + m_waitoffset;
                t.tv_sec  = m_period.sec()
                  - (t1.tv_sec - t0.tv_sec);
              }

            if (t.tv_nsec < 0 || t.tv_sec < 0)
              {
                std::cerr << "faital error: deadline passed. " << std::endl;
                std::cerr << "Wait time: ";
                std::cerr << t.tv_sec << "[s], ";
                std::cerr << t.tv_nsec << "[ns]" << std::endl;
                std::cerr << "Next wait time force to: 0.0 [s]" << std::endl;
                continue;
              }
            clock_nanosleep(CLOCK_MONOTONIC, !TIMER_ABSTIME, &t, NULL);
          }
      } while (m_svc);
 
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
  void RTPreemptECInit(RTC::Manager* manager)
  {
    RTC::Manager::instance().
      registerECFactory("RTPreemptEC",
                        RTC::ECCreate<OpenRTM::RTPreemptEC>,
                        RTC::ECDelete<OpenRTM::RTPreemptEC>);
    
  }
};
