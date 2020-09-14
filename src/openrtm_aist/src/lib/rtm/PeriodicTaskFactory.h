// -*- C++ -*-
/*!
 * @file PeriodicTaskFactory.h
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

#ifndef RTC_PERIODICTASKFACTORY_H
#define RTC_PERIODICTASKFACTORY_H

#include <coil/Factory.h>
#include <coil/PeriodicTaskBase.h>

namespace RTC
{

  typedef ::coil::GlobalFactory<coil::PeriodicTaskBase> PeriodicTaskFactory;

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  EXTERN template class DLL_PLUGIN
                     ::coil::GlobalFactory<coil::PeriodicTaskBase>;
#endif
};

#endif // RTC_PERIODICTASKFACTORY_H
