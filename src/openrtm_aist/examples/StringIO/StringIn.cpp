// -*- C++ -*-
/*!
 * @file StringIn.cpp
 * @brief Sample string in component
 * @date $Date: 2005-05-12 09:06:20 $
 *
 * $Id$
 */



#include "StringIn.h"
#include <unistd.h>
#include <iostream>
#include <string>

using namespace std;

StringIn::StringIn(RtcManager* manager)
  : RtcBase(manager) ,
    m_string_inIn("string_in", m_string_in)

{
  registerInPort(m_string_inIn);
  appendAlias("/example/StringIn|rtc");
}

/*
RtmRes StringIn::rtc_init_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_ready_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_ready_do()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_ready_exit()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_starting_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_active_entry()
{
  return RTM_OK;
}
*/




RtmRes StringIn::rtc_active_do()
{
  while (!m_string_inIn.isNew()) coil::usleep(100000);
  m_string_inIn.read();
  std::cout << m_string_in.data << std::endl;
  return RTM_OK;
}




/*
RtmRes StringIn::rtc_active_exit()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_stopping_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_aborting_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_error_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_error_do()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_error_exit()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_fatal_entry()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_fatal_do()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_fatal_exit()
{
  return RTM_OK;
}
*/



/*
RtmRes StringIn::rtc_exiting_entry()
{
  return RTM_OK;
}
*/



extern "C" {
  
  RtcBase* StringInNew(RtcManager* manager)
  {
	return new StringIn(manager);
  }
  
  
  void StringInDelete(RtcBase* p)
  {
	delete (StringIn *)p;
	return;
  }
  
  
  void StringInInit(RtcManager* manager)
  {
	RtcModuleProfile profile(stringin_spec);
	manager->registerComponent(profile, StringInNew, StringInDelete);
  }
};

