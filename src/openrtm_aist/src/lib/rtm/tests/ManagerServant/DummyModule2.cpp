// -*- C++ -*-
/*!
 * @file  DummyModule2.cpp
 * @brief Console output component
 * $Date: 2007-10-09 07:33:08 $
 *
 * $Id: DummyModule2.cpp 960 2008-10-23 08:12:20Z kojima $
 */

#include "DummyModule2.h"

// Module specification
// <rtc-template block="module_spec">
static const char* DummyModule2_spec[] =
  {
    "implementation_id", "DummyModule2",
    "type_name",         "DummyModule2",
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

DummyModule2::DummyModule2(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
//    m_inIn("in", m_in),
    
    // </rtc-template>
	dummy(0)
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

}

DummyModule2::~DummyModule2()
{
}

/*
RTC::ReturnCode_t DummyModule2::onInitialize()
{
  registerInPort("in", m_inIn);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t DummyModule2::onExecute(RTC::UniqueId ec_id)
{
  if (m_inIn.isNew())
    {
      m_inIn.read();
      std::cout << "Received: " << m_in.data << std::endl;
      std::cout << "TimeStamp: " << m_in.tm.sec << "[s] ";
      std::cout << m_in.tm.nsec << "[ns]" << std::endl;
    }
  usleep(1000);

  return RTC::RTC_OK;
}
*/

extern "C"
{
 
  void DummyModule2Init(RTC::Manager* manager)
  {
//    RTC::Properties profile(consoleout_spec);
    coil::Properties profile(DummyModule2_spec);
    manager->registerFactory(profile,
                             RTC::Create<DummyModule2>,
                             RTC::Delete<DummyModule2>);
  }
  
};


