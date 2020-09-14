// -*- C++ -*-
/*!
 * @file HelloRTWorldComp.h
 * @brief Hello RT world component
 * @date $Date: 2005-05-12 09:06:20 $
 *
 * $Id$
 */



#include <rtm/RtcManager.h>
#include <string>
#include "HelloRTWorld.h"


void MyModuleInit(RtcManager* manager)
{
  HelloRTWorldInit(manager);

  std::string name;
  RtcBase* comp;
  comp = manager->createComponent("HelloRTWorld", "example", name);
  coil::usleep(10000);
  comp->rtc_start();

}


int main (int argc, char** argv)
{
  RTM::RtcManager manager(argc, argv);
  // Initialize manager
  manager.initManager();
  // Activate manager and register to naming service
  manager.activateManager();
  // Initialize my module on this maneger
  manager.initModuleProc(MyModuleInit);
  // Main loop
  manager.runManager();
  return 0;
}

