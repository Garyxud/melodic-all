// -*- C++ -*-
/*!
 * @file HelloRTWorld.cpp
 * @brief Hello RT world component
 * @date $Date: 2005-05-12 09:06:20 $
 *
 * $Id$
 */



#include "HelloRTWorld.h"
#include <iostream>

using namespace std;

HelloRTWorld::HelloRTWorld(RtcManager* manager)
  : RtcBase(manager) 
	
{
  appendAlias("/example/HelloRTWorld|rtc");
}

RtmRes HelloRTWorld::rtc_active_do()
{
  std::cout << "Hello RT World!" << std::endl;
  return RTM_OK;
}

extern "C" {
  
  RtcBase* HelloRTWorldNew(RtcManager* manager)
  {
	return new HelloRTWorld(manager);
  }
  
  
  void HelloRTWorldDelete(RtcBase* p)
  {
	delete (HelloRTWorld *)p;
	return;
  }
  
  
  void HelloRTWorldInit(RtcManager* manager)
  {
	RtcModuleProfile profile(hellortworld_spec);
	manager->registerComponent(profile, HelloRTWorldNew, HelloRTWorldDelete);
  }
};

