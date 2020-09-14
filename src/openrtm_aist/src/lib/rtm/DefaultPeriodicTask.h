// -*- C++ -*-
/*!
 * @file DefaultPeriodicTask.h
 * @brief PeiodicTaskFactory class
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2009
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

#ifndef RTC_DEFAULTPERIODICTASK_H
#define RTC_DEFAULTPERIODICTASK_H

namespace coil
{
  class PeriodicTask;
};
namespace RTC
{
  typedef coil::PeriodicTask DefaultPeriodicTask;
};
extern "C"
{
  void DefaultPeriodicTaskInit();
};

#endif // RTC_DEFAULTPERIODICTASK_H
