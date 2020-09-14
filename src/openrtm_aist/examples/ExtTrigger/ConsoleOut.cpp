// -*- C++ -*-
/*!
 * @file  ConsoleOut.cpp
 * @brief Console output component
 * $Date: 2007-04-13 14:59:51 $
 *
 * $Id$
 */

#include "ConsoleOut.h"

// Module specification
// <rtc-template block="module_spec">
static const char* consoleout_spec[] =
  {
    "implementation_id", "ConsoleOut",
    "type_name",         "ConsoleOut",
    "description",       "Console output component",
    "version",           "1.0",
    "vendor",            "Noriaki Ando, AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

ConsoleOut::ConsoleOut(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_inIn("in", m_in)
    
    // </rtc-template>
{
}

ConsoleOut::~ConsoleOut()
{
}


RTC::ReturnCode_t ConsoleOut::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("in", m_inIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  return RTC::RTC_OK;
}

RTC::ReturnCode_t ConsoleOut::onExecute(RTC::UniqueId ec_id)
{
  if (m_inIn.isNew())
    {
      m_inIn.read();
      std::cout << "Received: " << m_in.data << std::endl;
      std::cout << "TimeStamp: " << m_in.tm.sec << "[s] ";
      std::cout << m_in.tm.nsec << "[ns]" << std::endl;
    }
  coil::usleep(1000);

  return RTC::RTC_OK;
}


extern "C"
{
 
  void ConsoleOutInit(RTC::Manager* manager)
  {
    coil::Properties profile(consoleout_spec);
    manager->registerFactory(profile,
                             RTC::Create<ConsoleOut>,
                             RTC::Delete<ConsoleOut>);
  }
  
};


