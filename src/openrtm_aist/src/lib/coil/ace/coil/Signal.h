// -*- C++ -*-
/*!
 * @file  SignalPosix.h
 * @brief RT-Middleware Service interface
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

#ifndef COIL_SIGNAL_H
#define COIL_SIGNAL_H
#if defined (WIN32)
#pragma warning( disable : 4244 ) 
#pragma warning( disable : 4312 ) 
#endif
#include <ace/Signal.h>
#if defined (WIN32)
#pragma warning( default : 4244 ) 
#pragma warning( default : 4312 ) 
#endif

namespace coil
{
  typedef ACE_Sig_Action SignalAction;
  typedef void (*SignalHandler)(int);
};
#endif // COIL_SIGNAL_H
