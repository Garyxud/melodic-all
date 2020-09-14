// -*- C++ -*-
/*!
 * @file Composite.cpp
 * @brief RT component server daemon
 * @date $Date: 2009-01-15 09:06:19 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2003-2008
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
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
