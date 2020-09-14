// -*- C++ -*-
/*!
 * @file  RTSample.cpp * @brief Realtime periodic execution example component * $Date$ 
 *
 * $Id$ 
 */
#include "RTSample.h"

// Module specification
// <rtc-template block="module_spec">
static const char* rtsample_spec[] =
  {
    "implementation_id", "RTSample",
    "type_name",         "RTSample",
    "description",       "Realtime periodic execution example component",
    "version",           "1.0",
    "vendor",            "Noriaki Ando, AIST",
    "category",          "example",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

RTSample::RTSample(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager)
    // </rtc-template>
{
}

RTSample::~RTSample()
{
}


RTC::ReturnCode_t RTSample::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer

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
RTC::ReturnCode_t RTSample::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RTSample::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RTSample::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RTSample::onActivated(RTC::UniqueId ec_id)
{
  m_tm.tick();
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RTSample::onDeactivated(RTC::UniqueId ec_id)
{
  double tm_max, tm_min, tm_mean, tm_stddev;
  m_tm.getStatistics(tm_max, tm_min, tm_mean, tm_stddev);
  std::cout << "max:    " << tm_max * 1000 << " [ms]" << std::endl;
  std::cout << "min:    " << tm_min * 1000 << " [ms]" << std::endl;
  std::cout << "mean:   " << tm_mean * 1000 << " [ms]" << std::endl;
  std::cout << "stddev: " << tm_stddev * 1000 << " [ms]" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RTSample::onExecute(RTC::UniqueId ec_id)
{
  static int count;
  m_tm.tack();
  m_tm.tick();

  if (count > 1000)
    {
      count = 0;
      double tm_max, tm_min, tm_mean, tm_stddev;
      m_tm.getStatistics(tm_max, tm_min, tm_mean, tm_stddev);
      std::cout << "max:    " << tm_max * 1000 << " [ms]" << std::endl;
      std::cout << "min:    " << tm_min * 1000 << " [ms]" << std::endl;
      std::cout << "mean:   " << tm_mean * 1000 << " [ms]" << std::endl;
      std::cout << "stddev: " << tm_stddev * 1000 << " [ms]" << std::endl;
    }

  ++count;
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RTSample::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RTSample::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RTSample::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RTSample::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RTSample::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void RTSampleInit(RTC::Manager* manager)
  {
    coil::Properties profile(rtsample_spec);
    manager->registerFactory(profile,
                             RTC::Create<RTSample>,
                             RTC::Delete<RTSample>);
  }
  
};



