// -*- C++ -*-
/*!
 * @file StringOut.h
 * @brief Sample string out component
 * @date $Date: 2005-05-12 09:06:20 $
 *
 * $Id$
 */

#ifndef __STRINGOUT_h__
#define __STRINGOUT_h__


#include <rtm/RtcBase.h>
#include <rtm/RtcManager.h>
#include <rtm/RtcInPort.h>
#include <rtm/RtcOutPort.h>

using namespace RTM;

static RtcModuleProfSpec stringout_spec[] =
  {
  
  {RTC_MODULE_NAME, "StringOut"},
  {RTC_MODULE_DESC, "Sample string out component"},
  {RTC_MODULE_VERSION, "0.1"},
  {RTC_MODULE_AUTHOR, "DrSample"},
  {RTC_MODULE_CATEGORY, "Generic"},
  {RTC_MODULE_COMP_TYPE, "COMMUTATIVE"},
  {RTC_MODULE_ACT_TYPE, "SPORADIC"},
  {RTC_MODULE_MAX_INST, "10"},
  {RTC_MODULE_LANG, "C++"},
  {RTC_MODULE_LANG_TYPE, "COMPILE"},
  {RTC_MODULE_SPEC_END, NULL}
  };


	
class StringOut
  : public RTM::RtcBase
{
 public:
  StringOut(RtcManager* manager);

  // [Initializing state]
  //  virtual RtmRes rtc_init_entry();

  // [Ready state]
  //  virtual RtmRes rtc_ready_entry();
  //  virtual RtmRes rtc_ready_do();
  //  virtual RtmRes rtc_ready_exit();

  // [Starting state]
  //  virtual RtmRes rtc_starting_entry();
  
  // [Active state]
  //  virtual RtmRes rtc_active_entry();
  virtual RtmRes rtc_active_do();
  //  virtual RtmRes rtc_active_exit();

  // [Stopping state]
  //  virtual RtmRes rtc_stopping_entry();

  // [Aborting state]
  //  virtual RtmRes rtc_aborting_entry();
  
  // [Error state]
  //  virtual RtmRes rtc_error_entry();
  //  virtual RtmRes rtc_error_do();
  //  virtual RtmRes rtc_error_exit();

  // [Fatal Error state]
  //  virtual RtmRes rtc_fatal_entry();
  //  virtual RtmRes rtc_fatal_do();
  //  virtual RtmRes rtc_fatal_exit();
  
  // [Exiting state]
  //  virtual RtmRes rtc_exiting_entry();

  TimedString m_string_out;
  OutPortAny<TimedString> m_string_outOut;

};


extern "C" {
  RtcBase* StringOutNew(RtcManager* manager);
  void StringOutDelete(RtcBase* p);
  void StringOutInit(RtcManager* manager);
};
#endif // __STRINGOUT_h__

