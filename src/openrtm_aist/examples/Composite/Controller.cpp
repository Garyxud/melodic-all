// -*- C++ -*-
/*!
 * @file  Controller.cpp * @brief Controller component * $Date$ 
 *
 * $Id$ 
 */
#include "Controller.h"

// Module specification
// <rtc-template block="module_spec">
static const char* controller_spec[] =
  {
    "implementation_id", "Controller",
    "type_name",         "Controller",
    "description",       "Controller component",
    "version",           "1.0",
    "vendor",            "Noriaki Ando, AIST",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

Controller::Controller(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in),
    m_outOut("out", m_out)
    // </rtc-template>
{
}

Controller::~Controller()
{
}


RTC::ReturnCode_t Controller::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("in", m_inIn);

  // Set OutPort buffer
  addOutPort("out", m_outOut);

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t Controller::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Controller::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Controller::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Controller::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Controller::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Controller::onExecute(RTC::UniqueId ec_id)
{
  if (m_inIn.isNew()) {
    m_inIn.read();
    std::cout << "Controller Received data: " << m_in.data << std::endl;
    m_out.data = m_in.data * 2;
    m_outOut.write();
  }
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t Controller::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Controller::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Controller::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Controller::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Controller::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void ControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(controller_spec);
    manager->registerFactory(profile,
                             RTC::Create<Controller>,
                             RTC::Delete<Controller>);
  }
  
};



