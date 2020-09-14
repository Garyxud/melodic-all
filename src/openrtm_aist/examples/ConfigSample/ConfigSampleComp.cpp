// -*- C++ -*-
/*!
 * @file ConfigSampleComp.cpp
 * @brief Standalone component
 * @date $Date: 2008-01-14 07:45:52 $
 *
 * $Id$
 */

#include <rtm/Manager.h>
#include <iostream>
#include <string>
#include "ConfigSample.h"


void MyModuleInit(RTC::Manager* manager)
{
  ConfigSampleInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("ConfigSample");


  // Example
  // The following procedure is examples how handle RT-Components.
  // These should not be in this function.

  // Get the component's object reference
  RTC::RTObject_var rtobj;
  rtobj = RTC::RTObject::_narrow(comp->_default_POA()->servant_to_reference(comp));

  RTC::ExecutionContextList_var ecs;
  ecs = rtobj->get_owned_contexts();
  ecs[(CORBA::ULong)0]->activate_component(rtobj);

  return;
}

int main (int argc, char** argv)
{
  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  // Set module initialization proceduer
  // This procedure will be invoked in activateManager() function.
  manager->setModuleInitProc(MyModuleInit);

  // Activate manager and register to naming service
  manager->activateManager();

  // run the manager in blocking mode
  // runManager(false) is the default.
  manager->runManager();

  // If you want to run the manager in non-blocking mode, do like this
  // manager->runManager(true);

  return 0;
}
