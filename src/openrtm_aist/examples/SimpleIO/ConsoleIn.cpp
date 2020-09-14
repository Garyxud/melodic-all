// -*- C++ -*-
/*!
 * @file  ConsoleIn.cpp
 * @brief Console input component
 * $Date: 2007-10-09 07:33:04 $
 *
 * $Id$
 */

#include "ConsoleIn.h"
#include <iostream>

// Module specification
// <rtc-template block="module_spec">
static const char* consolein_spec[] =
  {
    "implementation_id", "ConsoleIn",
    "type_name",         "ConsoleIn",
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

ConsoleIn::ConsoleIn(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_outOut("out", m_out)
    // </rtc-template>
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

ConsoleIn::~ConsoleIn()
{
}


RTC::ReturnCode_t ConsoleIn::onInitialize()
{
  addOutPort("out", m_outOut);

  m_outOut.addConnectorDataListener(ON_BUFFER_WRITE,
                                    new DataListener("ON_BUFFER_WRITE"));
  m_outOut.addConnectorDataListener(ON_BUFFER_FULL, 
                                    new DataListener("ON_BUFFER_FULL"));
  m_outOut.addConnectorDataListener(ON_BUFFER_WRITE_TIMEOUT, 
                                    new DataListener("ON_BUFFER_WRITE_TIMEOUT"));
  m_outOut.addConnectorDataListener(ON_BUFFER_OVERWRITE, 
                                    new DataListener("ON_BUFFER_OVERWRITE"));
  m_outOut.addConnectorDataListener(ON_BUFFER_READ, 
                                    new DataListener("ON_BUFFER_READ"));
  m_outOut.addConnectorDataListener(ON_SEND, 
                                    new DataListener("ON_SEND"));
  m_outOut.addConnectorDataListener(ON_RECEIVED,
                                    new DataListener("ON_RECEIVED"));
  m_outOut.addConnectorDataListener(ON_RECEIVER_FULL, 
                                    new DataListener("ON_RECEIVER_FULL"));
  m_outOut.addConnectorDataListener(ON_RECEIVER_TIMEOUT, 
                                    new DataListener("ON_RECEIVER_TIMEOUT"));

  m_outOut.addConnectorListener(ON_BUFFER_EMPTY,
                                    new ConnListener("ON_BUFFER_EMPTY"));
  m_outOut.addConnectorListener(ON_BUFFER_READ_TIMEOUT,
                                    new ConnListener("ON_BUFFER_READ_TIMEOUT"));
  m_outOut.addConnectorListener(ON_SENDER_EMPTY,
                                    new ConnListener("ON_SENDER_EMPTY"));
  m_outOut.addConnectorListener(ON_SENDER_TIMEOUT,
                                    new ConnListener("ON_SENDER_TIMEOUT"));
  m_outOut.addConnectorListener(ON_SENDER_ERROR,
                                    new ConnListener("ON_SENDER_ERROR"));
  m_outOut.addConnectorListener(ON_CONNECT,
                                    new ConnListener("ON_CONNECT"));
  m_outOut.addConnectorListener(ON_DISCONNECT,
                                    new ConnListener("ON_DISCONNECT"));


  return RTC::RTC_OK;
}
RTC::ReturnCode_t ConsoleIn::onExecute(RTC::UniqueId ec_id)
{
  std::cout << "Please input number: ";
  std::cin >> m_out.data;
  if (m_out.data == 666) return RTC::RTC_ERROR;
  m_outOut.write();

  return RTC::RTC_OK;
}


extern "C"
{
 
  void ConsoleInInit(RTC::Manager* manager)
  {
    RTC::Properties profile(consolein_spec);
    manager->registerFactory(profile,
                             RTC::Create<ConsoleIn>,
                             RTC::Delete<ConsoleIn>);
  }
  
};


