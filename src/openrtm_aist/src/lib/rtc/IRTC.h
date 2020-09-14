// -*- C++ -*-
/*!
 * @file  IRTC.h
 * @brief RTC interfaces
 * @date  $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
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

#ifndef RTC_IRTC_H
#define RTC_IRTC_H

#include <vector>
#include <utility>

namespace RTC
{
namespace Local
{
  enum ReturnCode_t
    {
      RTC_OK,
      RTC_ERROR,
      BAD_PARAMETER,
      UNSUPPORTED,
      OUT_OF_RESOURCES,
      PRECONDITION_NOT_MET
    };

  enum LifeCycleState
    {
      CREATED_STATE,
      INACTIVE_STATE,
      ACTIVE_STATE,
      ERROR_STATE
    };
  
  enum ExecutionKind
    {
      PERIODIC,
      EVENT_DRIVEN,
      OTHER
    };

    enum PortInterfacePolarity
    {
      PROVIDED,
      REQUIRED
    };

#define EXECUTION_HANDLE_TYPE_NATIVE long
  
  typedef EXECUTION_HANDLE_TYPE_NATIVE ExecutionContextHandle_t;
  typedef EXECUTION_HANDLE_TYPE_NATIVE ECHandle_t;
  typedef char * UniqueIdentifier;
  
  typedef std::vector<std::pair<std::string, std::string> > NVList;
  
  class ExecutionContext;
  typedef std::vector<ExecutionContext*> ExecutionContextList;
  
  struct LightweightRTObject;
};     // namespace Local
};     // namespace RTC
#endif // RTC_IRTC_H

