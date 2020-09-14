// -*- C++ -*-
/*!
 * @file  MutexPosix.h
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

#ifndef COIL_UUID_H
#define COIL_UUID_H

#include <coil/config_coil.h>
#if defined (WIN32)
#pragma warning( disable : 4244 ) 
#pragma warning( disable : 4312 ) 
#endif
#include <ace/UUID.h>
#if defined (WIN32)
#pragma warning( default : 4244 ) 
#pragma warning( default : 4312 ) 
#endif

namespace coil
{
  typedef ACE_Utils::UUID UUID;

  class UUID_Generator
    : public ACE_Utils::UUID_Generator
  {
  public:
    UUID_Generator() : ACE_Utils::UUID_Generator() {};
#ifndef ACE_5_6_1_OR_EARLIER  
    coil::UUID*
    generateUUID(ACE_UINT16 version=0x0001, u_char variant=0x80)
    {
      return ACE_Utils::UUID_Generator::generate_UUID(version, variant);
    }
#endif
  };
};
#endif // COIL_UUID_H
