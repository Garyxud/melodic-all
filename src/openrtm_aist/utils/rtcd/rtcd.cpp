// -*- C++ -*-
/*!
 * @file rtcd.cpp
 * @brief RT component server daemon
 * @date $Date: 2005-05-12 09:06:19 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2003-2008
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#include <iostream>
#include <rtm/Manager.h>


int main (int argc, char** argv)
{
  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  manager->activateManager();

  manager->runManager();

  return 0;
}
