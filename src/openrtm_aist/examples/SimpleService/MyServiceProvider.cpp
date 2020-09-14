// -*- C++ -*-
/*!
 * @file  MyServiceProvider.cpp
 * @brief MyService Provider Sample component
 * $Date: 2007-04-13 15:03:20 $
 *
 * $Id$
 */

#include "MyServiceProvider.h"

// Module specification
// <rtc-template block="module_spec">
static const char* myserviceprovider_spec[] =
  {
    "implementation_id", "MyServiceProvider",
    "type_name",         "MyServiceProvider",
    "description",       "MyService Provider Sample component",
    "version",           "0.1",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

MyServiceProvider::MyServiceProvider(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_MyServicePort("MyService")
    // </rtc-template>
{
}

MyServiceProvider::~MyServiceProvider()
{
}


RTC::ReturnCode_t MyServiceProvider::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_MyServicePort.registerProvider("myservice0", "MyService", m_myservice0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_MyServicePort);
  
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MyServiceProvider::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceProvider::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceProvider::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceProvider::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceProvider::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceProvider::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceProvider::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceProvider::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceProvider::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceProvider::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceProvider::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void MyServiceProviderInit(RTC::Manager* manager)
  {
    coil::Properties profile(myserviceprovider_spec);
    manager->registerFactory(profile,
                             RTC::Create<MyServiceProvider>,
                             RTC::Delete<MyServiceProvider>);
  }
  
};


