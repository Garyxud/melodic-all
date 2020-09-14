// -*- C++ -*-
/*!
 * @file  Sensor.cpp * @brief Sensor component * $Date$ 
 *
 * $Id$ 
 */
#include "Sensor.h"

// Module specification
// <rtc-template block="module_spec">
static const char* sensor_spec[] =
  {
    "implementation_id", "Sensor",
    "type_name",         "Sensor",
    "description",       "Sensor component",
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

Sensor::Sensor(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in),
    m_outOut("out", m_out)

    // </rtc-template>
{
}

Sensor::~Sensor()
{
}


RTC::ReturnCode_t Sensor::onInitialize()
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
RTC::ReturnCode_t Sensor::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Sensor::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Sensor::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Sensor::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Sensor::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Sensor::onExecute(RTC::UniqueId ec_id)
{
  if (m_inIn.isNew()) {
    m_inIn.read();
    std::cout << "Sensor Received data: " << m_in.data << std::endl;
    m_out.data = m_in.data * 2;
    m_outOut.write();
  }
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t Sensor::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Sensor::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Sensor::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Sensor::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t Sensor::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void SensorInit(RTC::Manager* manager)
  {
    coil::Properties profile(sensor_spec);
    manager->registerFactory(profile,
                             RTC::Create<Sensor>,
                             RTC::Delete<Sensor>);
  }
  
};



