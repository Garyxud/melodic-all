// -*- C++ -*-
/*!
 * @file HelloRTWorld.h
 * @brief Hello RT world component
 * @date $Date: 2005-05-12 09:06:20 $
 *
 * $Id$
 */

#ifndef __HELLORTWORLD_h__
#define __HELLORTWORLD_h__


#include <rtm/RtcBase.h>
#include <rtm/RtcManager.h>
#include <rtm/RtcInPort.h>
#include <rtm/RtcOutPort.h>

using namespace RTM;

static RtcModuleProfSpec hellortworld_spec[] =
  {
  
  {RTC_MODULE_NAME, "HelloRTWorld"},
  {RTC_MODULE_DESC, "Hello RT world component"},
  {RTC_MODULE_VERSION, "0.1"},
  {RTC_MODULE_AUTHOR, "DrSample"},
  {RTC_MODULE_CATEGORY, "example"},
  {RTC_MODULE_COMP_TYPE, "COMMUTATIVE"},
  {RTC_MODULE_ACT_TYPE, "SPORADIC"},
  {RTC_MODULE_MAX_INST, "10"},
  {RTC_MODULE_LANG, "C++"},
  {RTC_MODULE_LANG_TYPE, "COMPILE"},
  {RTC_MODULE_SPEC_END, NULL}
  };


	
class HelloRTWorld
  : public RTM::RtcBase
{
 public:
  HelloRTWorld(RtcManager* manager);
  virtual RtmRes rtc_active_do();
};


extern "C" {
  RtcBase* HelloRTWorldNew(RtcManager* manager);
  void HelloRTWorldDelete(RtcBase* p);
  void HelloRTWorldInit(RtcManager* manager);
};
#endif // __HELLORTWORLD_h__

