// -*- C++ -*-
/*!
 * @file StringOut.cpp
 * @brief Sample string out component
 * @date $Date: 2005-05-12 09:06:20 $
 *
 * $Id$
 */



#include "StringOut.h"
#include <iostream>
#include <string>
#include <limits.h>

using namespace std;

StringOut::StringOut(RtcManager* manager)
  : RtcBase(manager) ,
     m_string_outOut("string_out", m_string_out)
{
  registerOutPort(m_string_outOut);
  appendAlias("/example/StringOut|rtc");
}

/*
RtmRes StringOut::rtc_init_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_ready_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_ready_do()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_ready_exit()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_starting_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_active_entry()
{
  return RTM_OK;
}
*/




RtmRes StringOut::rtc_active_do()
{
  std::string s;
  std::cout << "Please input string: ";
  std::getline(cin, s);
  m_string_out.data = CORBA::string_dup(s.c_str());
  m_string_outOut.write();
  return RTM_OK;
}




/*
RtmRes StringOut::rtc_active_exit()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_stopping_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_aborting_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_error_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_error_do()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_error_exit()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_fatal_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_fatal_do()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_fatal_exit()
{
  return RTM_OK;
}
*/



/*
RtmRes StringOut::rtc_exiting_entry()
{
  return RTM_OK;
}
*/



extern "C" {
  
  RtcBase* StringOutNew(RtcManager* manager)
  {
	return new StringOut(manager);
  }
  
  
  void StringOutDelete(RtcBase* p)
  {
	delete (StringOut *)p;
	return;
  }
  
  
  void StringOutInit(RtcManager* manager)
  {
	RtcModuleProfile profile(stringout_spec);
	manager->registerComponent(profile, StringOutNew, StringOutDelete);
  }
};

