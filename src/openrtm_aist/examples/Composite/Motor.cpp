// -*- C++ -*-
/*!
 * @file  Motor.cpp * @brief Motor component * $Date$ 
 *
 * $Id$ 
 */
#include <iostream>
#include "Motor.h"

// Module specification
// <rtc-template block="module_spec">
static const char* motor_spec[] =
  {
    "implementation_id", "Motor",
    "type_name",         "Motor",
    "description",       "Motor component",
    "version",           "1.0",
    "vendor",            "Noriaki Ando, AIST",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.motor_id", "0",
    ""
  };
// </rtc-template>

Motor::Motor(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in),
    m_outOut("out", m_out)

    // </rtc-template>
{
}

Motor::~Motor()
{
}


RTC::ReturnCode_t Motor::onInitialize()
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
  bindParameter("motor_id", m_motor_id, "0");

  m_configsets.update("default");

  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t Motor::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Motor::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Motor::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Motor::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Motor::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Motor::onExecute(RTC::UniqueId ec_id)
{
  if (m_inIn.isNew()) {
    m_inIn.read();
    std::cout << "Motor Received data: " << m_in.data << std::endl << std::endl;
    m_out.data = m_in.data * 2;
    m_outOut.write();
  }
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t Motor::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Motor::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Motor::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Motor::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Motor::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void MotorInit(RTC::Manager* manager)
  {
    // RTC::Properties profile(motor_spec);
    coil::Properties profile(motor_spec);
    manager->registerFactory(profile,
                             RTC::Create<Motor>,
                             RTC::Delete<Motor>);
  }
  
};



