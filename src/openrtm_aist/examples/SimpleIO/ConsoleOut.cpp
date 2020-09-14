// -*- C++ -*-
/*!
 * @file  ConsoleOut.cpp
 * @brief Console output component
 * $Date: 2007-10-09 07:33:08 $
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
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

}

ConsoleOut::~ConsoleOut()
{
}


RTC::ReturnCode_t ConsoleOut::onInitialize()
{
  addInPort("in", m_inIn);
 
  m_inIn.addConnectorDataListener(ON_BUFFER_WRITE,
                                    new DataListener("ON_BUFFER_WRITE"));
  m_inIn.addConnectorDataListener(ON_BUFFER_FULL, 
                                    new DataListener("ON_BUFFER_FULL"));
  m_inIn.addConnectorDataListener(ON_BUFFER_WRITE_TIMEOUT, 
                                    new DataListener("ON_BUFFER_WRITE_TIMEOUT"));
  m_inIn.addConnectorDataListener(ON_BUFFER_OVERWRITE, 
                                    new DataListener("ON_BUFFER_OVERWRITE"));
  m_inIn.addConnectorDataListener(ON_BUFFER_READ, 
                                    new DataListener("ON_BUFFER_READ"));
  m_inIn.addConnectorDataListener(ON_SEND, 
                                    new DataListener("ON_SEND"));
  m_inIn.addConnectorDataListener(ON_RECEIVED,
                                    new DataListener("ON_RECEIVED"));
  m_inIn.addConnectorDataListener(ON_RECEIVER_FULL, 
                                    new DataListener("ON_RECEIVER_FULL"));
  m_inIn.addConnectorDataListener(ON_RECEIVER_TIMEOUT, 
                                    new DataListener("ON_RECEIVER_TIMEOUT"));

  m_inIn.addConnectorListener(ON_BUFFER_EMPTY,
                                    new ConnListener("ON_BUFFER_EMPTY"));
  m_inIn.addConnectorListener(ON_BUFFER_READ_TIMEOUT,
                                    new ConnListener("ON_BUFFER_READ_TIMEOUT"));
  m_inIn.addConnectorListener(ON_SENDER_EMPTY,
                                    new ConnListener("ON_SENDER_EMPTY"));
  m_inIn.addConnectorListener(ON_SENDER_TIMEOUT,
                                    new ConnListener("ON_SENDER_TIMEOUT"));
  m_inIn.addConnectorListener(ON_SENDER_ERROR,
                                    new ConnListener("ON_SENDER_ERROR"));
  m_inIn.addConnectorListener(ON_CONNECT,
                                    new ConnListener("ON_CONNECT"));
  m_inIn.addConnectorListener(ON_DISCONNECT,
                                    new ConnListener("ON_DISCONNECT"));


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
//    RTC::Properties profile(consoleout_spec);
    coil::Properties profile(consoleout_spec);
    manager->registerFactory(profile,
                             RTC::Create<ConsoleOut>,
                             RTC::Delete<ConsoleOut>);
  }
  
};


