// -*- C++ -*-
/*!
 * @file CORBA.h
 * @brief doil CORBA header
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef DOIL_CORBA_CORBA_H
#define DOIL_CIRBA_CORBA_H

#include <rtm/config_rtc.h>

#ifdef ORB_IS_MICO
#include <CORBA.h>
#include <coss/CosNaming.h>
#endif

#ifdef ORB_IS_TAO
#include <tao/corba.h>
#include <orbsvcs/CosNamingC.h>
#endif

#ifdef ORB_IS_OMNIORB
#include <omniORB4/CORBA.h>
#include <omnithread.h>
#endif

#ifdef RTC_CORBA_CXXMAPPING11
#define RefCountServantBase ServantBase
#endif

#endif // RTM_CORBA_H
