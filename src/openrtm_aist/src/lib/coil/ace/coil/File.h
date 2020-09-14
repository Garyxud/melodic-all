// -*- C++ -*-
/*!
 * @file Task.cpp
 * @brief Task class
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

#ifndef COIL_FILE_H
#define COIL_FILE_H

#include <coil/config_coil.h>
#include <ace/ACE.h>

namespace coil
{

  inline const char* dirname(const char* path)
  {
    return ACE::dirname(path);
  }

  inline const char* basename(const char* path)
  {
    return ACE::basename(path);
  }
};

#endif // COIL_FILE_H
