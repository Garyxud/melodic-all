// -*- C++ -*-
/*!
 * @file  USBCameraMonitor.cpp
 * @brief 'hellow
 * $Date: 2007-07-20 20:33:29 $
 *
 * $Id$
 */

#include "USBCameraMonitor.h"
#include <iostream>
using namespace std;

// Module specification
// <rtc-template block="module_spec">
static const char* usbcameramonitor_spec[] =
  {
    "implementation_id", "USBCameraMonitor",
    "type_name",         "USBCameraMonitor",
    "description",       "USB Camera Acquire component",
    "version",           "1.0",
    "vendor",            "Noriaki Ando, AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.image_height", "240",
    "conf.default.image_width", "320",
    ""
  };
// </rtc-template>

USBCameraMonitor::USBCameraMonitor(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_inIn("in", m_in),
    
    // </rtc-template>
    dummy(0)
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  registerInPort("in", m_inIn);
  
  /* RTM-Win-113 add 20070404 SEC)T.Shimoji */
  m_in.data = 0;
  //  m_inIn.write(m_in);
  
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>
}

USBCameraMonitor::~USBCameraMonitor()
{
}


RTC::ReturnCode_t USBCameraMonitor::onInitialize()
{
  bindParameter("image_height", m_img_height, "240");
  bindParameter("image_width", m_img_width, "320");
  return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t USBCameraMonitor::onFinalize()
  {
  //return RTC::OK;
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t USBCameraMonitor::onStartup(RTC::UniqueId ec_id)
  {
  //return RTC::OK;
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t USBCameraMonitor::onShutdown(RTC::UniqueId ec_id)
  {
  //return RTC::OK;
  return RTC::RTC_OK;
  }
*/


RTC::ReturnCode_t USBCameraMonitor::onActivated(RTC::UniqueId ec_id)
{
  m_img=cvCreateImage(cvSize(m_img_width,m_img_height),IPL_DEPTH_8U,3);
  
  //画像表示用ウィンドウの作成
  cvNamedWindow("CaptureImage", CV_WINDOW_AUTOSIZE);
  
  std::cout << "m_img->nChannels :" << m_img->nChannels << std::endl;
  std::cout << "m_img->width :" << m_img->width << std::endl;
  std::cout << "m_img->height :" << m_img->height << std::endl;
  
  return RTC::RTC_OK;
}



RTC::ReturnCode_t USBCameraMonitor::onDeactivated(RTC::UniqueId ec_id)
{
  cvReleaseImage(&m_img);
  //表示ウィンドウの消去
  cvDestroyWindow("CaptureImage");
  return RTC::RTC_OK;
}



RTC::ReturnCode_t USBCameraMonitor::onExecute(RTC::UniqueId ec_id)
{
  static coil::TimeValue tm_pre;
  static int count = 0;
  
  if (!m_inIn.isNew())
    {
      return RTC::RTC_OK;
    }
  
  m_inIn.read();
  if (!(m_in.data.length() > 0))
    {
      return RTC::RTC_OK;
    }
  
  memcpy(m_img->imageData,(void *)&(m_in.data[0]),m_in.data.length());
  
  //画像表示
  cvShowImage("CaptureImage", m_img);
  
  cvWaitKey(1);
  if (count > 100)
    {
      count = 0;
      coil::TimeValue tm;
      tm = coil::gettimeofday();
      double sec(tm - tm_pre);
      if (sec > 1.0 && sec < 1000.0)
        {
          std::cout << 100.0/sec << " [FPS]" << std::endl;
        }
      tm_pre = tm;
    }
  ++count;
  
  return RTC::RTC_OK;
}


/*
  RTC::ReturnCode_t USBCameraMonitor::onAborting(RTC::UniqueId ec_id)
  {
  //return RTC::OK;
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t USBCameraMonitor::onError(RTC::UniqueId ec_id)
  {
  //return RTC::OK;
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t USBCameraMonitor::onReset(RTC::UniqueId ec_id)
  {
  //return RTC::OK;
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t USBCameraMonitor::onStateUpdate(RTC::UniqueId ec_id)
  {
  //return RTC::OK;
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t USBCameraMonitor::onRateChanged(RTC::UniqueId ec_id)
  {
  //return RTC::OK;
  return RTC::RTC_OK;
  }
*/



extern "C"
{
  
  void USBCameraMonitorInit(RTC::Manager* manager)
  {
    coil::Properties profile(usbcameramonitor_spec);
    manager->registerFactory(profile,
                             RTC::Create<USBCameraMonitor>,
                             RTC::Delete<USBCameraMonitor>);
  }
  
};


