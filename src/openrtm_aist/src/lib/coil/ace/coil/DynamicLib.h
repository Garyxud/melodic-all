// -*- C++ -*-
/*!
 * @file DynamicLib_ace.h
 * @brief DynamicLib class using ACE
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008 Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef COIL_DYNAMICLIB_H
#define COIL_DYNAMICLIB_H

#include <coil/config_coil.h>
#include <ace/DLL.h>

#define COIL_DEFAULT_DYNLIB_MODE ACE_DEFAULT_SHLIB_MODE

namespace coil
{
  typedef ACE_DLL DynamicLib;
};

#endif // DynamicLib_h
