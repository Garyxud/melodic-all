// -*- C++ -*-
/*!
 * @file  AutoTestIn.cpp
 * @brief Sample component for auto-test
 * @date $Date$
 *
 * $Id$
 */
#include <iomanip>
#include "AutoTestIn.h"

// Module specification
// <rtc-template block="module_spec">
static const char* autotestin_spec[] =
  {
    "implementation_id", "AutoTestIn",
    "type_name",         "AutoTestIn",
    "description",       "Sample component for auto-test",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    "exec_cxt.periodic.rate", "1.0",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
AutoTestIn::AutoTestIn(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in),
    m_seqinIn("seqin", m_seqin),
    m_MyServicePort("MyService")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
AutoTestIn::~AutoTestIn()
{
}



RTC::ReturnCode_t AutoTestIn::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("in", m_inIn);
  addInPort("seqin", m_seqinIn);
  
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
RTC::ReturnCode_t AutoTestIn::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AutoTestIn::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AutoTestIn::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t AutoTestIn::onActivated(RTC::UniqueId ec_id)
{
 //書き出すファイルを開く
  fout.open("received-data");
  if (!fout){
    std::cout << "Can not open received-data..." << std::endl;
  }  
  m_msg = "";
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AutoTestIn::onDeactivated(RTC::UniqueId ec_id)
{
  fout.close();
  m_myservice0.reset_message();
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AutoTestIn::onExecute(RTC::UniqueId ec_id)
{
  if (m_msg == "") {
    m_msg = m_myservice0.get_echo_message();
  }

  if ( m_inIn.isNew() && m_seqinIn.isNew() && m_msg != ""){
      m_inIn.read(); 
      m_seqinIn.read();

      fout << std::fixed;
      fout << std::showpoint;
      fout << std::setprecision(6);
      /*
      std::cout << std::fixed;
      std::cout << std::showpoint;
      std::cout << std::setprecision(1);
      std::cout << m_in.data << std::endl;
      std::cout << "flen : " << m_in.data << std::endl;
      */
      fout << m_in.data << std::endl;

      /*
      std::cout << "FloatSeqIn//////////////////////////////////////////" << std::endl;
      std::cout << "Received: " << m_seqin.data[0] << " " << m_seqin.data[1]  << " " << m_seqin.data[2]  << " " << m_seqin.data[3]  << " " << m_seqin.data[4] <<std::endl;

      */
      fout <<  m_seqin.data[0] << " " << m_seqin.data[1]<< " "<<m_seqin.data[2] << " " << m_seqin.data[3] << " " << m_seqin.data[4] <<  std::endl;

 
      //      std::cout << "echo//////////////////////////////////////////" << std::endl;
      //std::cout << coil::eraseBothEndsBlank(m_myservice0.get_echo()) << std::endl;
      //std::cout << strlen(coil::eraseBothEndsBlank(m_myservice0.get_echo())) << std::endl;
      fout << m_msg <<  std::endl;
      m_msg = "";
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AutoTestIn::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AutoTestIn::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AutoTestIn::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AutoTestIn::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AutoTestIn::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void AutoTestInInit(RTC::Manager* manager)
  {
    coil::Properties profile(autotestin_spec);
    manager->registerFactory(profile,
                             RTC::Create<AutoTestIn>,
                             RTC::Delete<AutoTestIn>);
  }
  
};


