// -*- C++ -*-
/*!
 * @file  DummyModule1.cpp
 * @brief 
 * $Date: 2007-10-09 07:33:04 $
 *
 * $Id: DummyModule1.cpp 960 2008-10-23 08:12:20Z kojima $
 */

#include "DummyModule1.h"
#include <iostream>

// Module specification
// <rtc-template block="module_spec">
static const char* DummyModule1_spec[] =
  {
    "implementation_id", "DummyModule1",
    "type_name",         "DummyModule1",
    "description",       "Console input component",
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

DummyModule1::DummyModule1(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
//    m_outOut("out", m_out),
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

DummyModule1::~DummyModule1()
{
}

/*
RTC::ReturnCode_t DummyModule1::onInitialize()
{
  registerOutPort("out", m_outOut);
  return RTC::RTC_OK;
}
RTC::ReturnCode_t DummyModule1::onExecute(RTC::UniqueId ec_id)
{
  std::cout << "Please input number: ";
  std::cin >> m_out.data;
  std::cout << "Sending to subscriber: " << m_out.data << std::endl;
  m_outOut.write();

  return RTC::RTC_OK;
}
*/

extern "C"
{
 
  void DummyModule1Init(RTC::Manager* manager)
  {
    coil::Properties profile(DummyModule1_spec);
    manager->registerFactory(profile,
                             RTC::Create<DummyModule1>,
                             RTC::Delete<DummyModule1>);

  }
  
};


