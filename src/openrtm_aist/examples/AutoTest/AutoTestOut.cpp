// -*- C++ -*-
/*!
 * @file  AutoTestOut.cpp
 * @brief Sample component for auto-test
 * @date $Date$
 *
 * $Id$
 */

#include <coil/Time.h>
#include <iostream>
#include <iomanip>

#include "AutoTestOut.h"
// Module specification
// <rtc-template block="module_spec">
static const char* autotestout_spec[] =
  {
    "implementation_id", "AutoTestOut",
    "type_name",         "AutoTestOut",
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
AutoTestOut::AutoTestOut(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_outOut("out", m_out),
    m_seqoutOut("seqout", m_seqout),
    m_MyServicePort("MyService")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
AutoTestOut::~AutoTestOut()
{
}



RTC::ReturnCode_t AutoTestOut::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  addOutPort("out", m_outOut);
  addOutPort("seqout", m_seqoutOut);
  m_seqout.data.length(5);
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_MyServicePort.registerConsumer("myservice0", "MyService", m_myservice0);
  
  // Set CORBA Service Ports
  addPort(m_MyServicePort);
  
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AutoTestOut::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AutoTestOut::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AutoTestOut::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t AutoTestOut::onActivated(RTC::UniqueId ec_id)
{
  //ここでファイルを開く
  fin.open("original-data");
  if (!fin){
    std::cout << "Can't open original-data..." << std::endl;
    return RTC::RTC_ERROR;
  }
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AutoTestOut::onDeactivated(RTC::UniqueId ec_id)
{
  fin.close();
  fin.clear(); // for win32.
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AutoTestOut::onExecute(RTC::UniqueId ec_id)
{

  coil::usleep(100000);

  //onexecuteでファイルから3行ずつ読み込み送る
  std::vector<std::string> vstr;
  std::string ss;
  
  if (getline(fin, ss)){ //Float

    //std::cout << "1 " << ss << std::endl;
    std::cout << std::fixed;
    std::cout << std::showpoint;
    std::cout << std::setprecision(6);
    std::stringstream iss;
    std::string str;
    iss.str(ss);
    iss >> m_out.data;
    std::cout << m_out.data << std::endl;
    //fout << m_out.data << std::endl;


    if (getline(fin, ss)){ //SeqFloat
      std::cout << ss << std::endl;
      //std::cout << strlen(ss) << std::endl;
      vstr = coil::split(ss, " ");
      std::stringstream isss;

      for (int len=0; len<5; ++len){
	//std::cout << vstr[len] << std::endl;
	isss.str(vstr[len]);
	isss >>m_seqout.data[len];
	std::cout << "m_seqout.data[len] = " << m_seqout.data[len] << std::endl;
	isss.clear();
      }
      //fout << m_seqout.data[0] << " " << m_seqout.data[1] << " " << m_seqout.data[2] << " " << m_seqout.data[3] << " " <<m_seqout.data[4] << std::endl;

      if (getline(fin,ss)){ //echo用
	std::cout << ss << std::endl;
	//fout << ss << std::endl;
	char* retmsg;

	if (!CORBA::is_nil(m_myservice0._ptr())){
	  retmsg = m_myservice0->echo(ss.c_str());
	}

	m_outOut.write();
	m_seqoutOut.write();

      }
    }
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AutoTestOut::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AutoTestOut::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AutoTestOut::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AutoTestOut::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AutoTestOut::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void AutoTestOutInit(RTC::Manager* manager)
  {
    coil::Properties profile(autotestout_spec);
    manager->registerFactory(profile,
                             RTC::Create<AutoTestOut>,
                             RTC::Delete<AutoTestOut>);
  }
  
};


