// -*- C++ -*-
/*!
 * @file ArtExecutionContext.cpp
 * @brief ArtExecutionContext class
 * @date $Date$
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

/*
 * $Log$
 */

#include "ArtExecutionContext.h"
#include <rtm/ECFactory.h>
#include <linux/art_task.h>

#include <iostream>

namespace RTC
{
  ArtExecutionContext::ArtExecutionContext()
    : PeriodicExecutionContext(),
      m_priority(ART_PRIO_MAX-1)
  {
    rtclog.setName("ArtEC");
    coil::Properties& prop(Manager::instance().getConfig());

    // Priority
    getProperty(prop, "exec_cxt.periodic.priority", m_priority);
    getProperty(prop, "exec_cxt.periodic.art.priority", m_priority);
    RTC_DEBUG(("Priority: %d", m_priority));
  }

  ArtExecutionContext::~ArtExecutionContext()
  {
  }


  int ArtExecutionContext::svc(void)
  {
    int usec(m_period.sec() * 1000000 + m_period.usec());
    if (art_enter(m_priority, ART_TASK_PERIODIC, usec) == -1)
      {
        std::cerr << "fatal error: art_enter" << std::endl;
      }
    do
      {
        std::for_each(m_comps.begin(), m_comps.end(), invoke_worker());
        if (art_wait() == -1)
          {
            std::cerr << "fatal error: art_wait " << std::endl;
          }
      } while (m_running);

    if (art_exit() == -1)
      {
        std::cerr << "fatal error: art_exit" << std::endl;
      }
    return 0;
  }
};


extern "C"
{
  void ArtExecutionContextInit(RTC::Manager* manager)
  {
    RTC::Manager::instance().registerECFactory("ArtExecutionContext",
                                   RTC::ECCreate<RTC::ArtExecutionContext>,
                                   RTC::ECDelete<RTC::ArtExecutionContext>);
  }
};
