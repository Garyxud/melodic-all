// -*- C++ -*-
/*!
 * @file  MyServiceConsumer.cpp
 * @brief MyService Consumer Sample component
 * $Date: 2008-01-14 07:46:42 $
 *
 * $Id$
 */

#include "MyServiceConsumer.h"
#include <rtm/CORBA_SeqUtil.h>
#include <vector>
#include <stdlib.h>
#include <coil/Async.h>
#include <functional>

// Module specification
// <rtc-template block="module_spec">
static const char* myserviceconsumer_spec[] =
  {
    "implementation_id", "MyServiceConsumer",
    "type_name",         "MyServiceConsumer",
    "description",       "MyService Consumer Sample component",
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

MyServiceConsumer::MyServiceConsumer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_MyServicePort("MyService"),
    // </rtc-template>
    async_set_value(0), async_echo(0)
{
}

MyServiceConsumer::~MyServiceConsumer()
{
}


RTC::ReturnCode_t MyServiceConsumer::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_MyServicePort.registerConsumer("myservice0", "MyService", m_myservice0);
  
  // Set CORBA Service Ports
  addPort(m_MyServicePort);
  
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MyServiceConsumer::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceConsumer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceConsumer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceConsumer::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceConsumer::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t MyServiceConsumer::onExecute(RTC::UniqueId ec_id)
{
  try
    {
      std::cout << std::endl;
      std::cout << "Command list: " << std::endl;
      std::cout << " echo [msg]       : echo message." << std::endl;
      std::cout << " set_value [value]: set value." << std::endl;
      std::cout << " get_value        : get current value." << std::endl;
      std::cout << " get_echo_history : get input messsage history." << std::endl;
      std::cout << " get_value_history: get input value history." << std::endl;
      std::cout << "> ";
      
      std::string args;
      std::string::size_type pos;
      std::vector<std::string> argv;
      std::getline(std::cin, args);
      
      pos = args.find_first_of(" ");
      if (pos != std::string::npos)
	{
	  argv.push_back(args.substr(0, pos));
	  argv.push_back(args.substr(++pos));
	}
      else
	{
	  argv.push_back(args);
	}
      
      if (async_echo != 0 && async_echo->finished())
        {
          std::cout << "echo() finished: " <<  m_result << std::endl;
          delete async_echo;
          async_echo = 0;
        }
      
      if (argv[0] == "echo" && argv.size() > 1)
	{
          if (async_echo == 0)
            {
              // char* retmsg;
              // retmsg = m_myservice0->echo(argv[1].c_str());
              async_echo = 
                coil::AsyncInvoker(&m_myservice0,
                                   echo_functor(argv[1], m_result));
              async_echo->invoke();
              // std::cout << "echo return: " << retmsg << std::endl;
            }
          else
            {
              std::cout << "set_value() still invoking" << std::endl;
            }
	  return RTC::RTC_OK;
	}
      
      if (argv[0] == "set_value" && argv.size() > 1)
	{
          CORBA::Float val(atof(argv[1].c_str()));
          coil::AsyncInvoker(&m_myservice0, set_value_functor(val),
                             true)->invoke();
          std::cout << "Set remote value: " << val << std::endl;

          return RTC::RTC_OK;
	}
      
      if (argv[0] == "get_value")
	{
	  std::cout << "Current remote value: "
		    << m_myservice0->get_value() << std::endl;
	  return RTC::RTC_OK;
	}
      
      if (argv[0] == "get_echo_history")
	{
	  CORBA_SeqUtil::for_each(*(m_myservice0->get_echo_history()),
				  seq_print<const char*>());
	  return RTC::RTC_OK;
	}
      
      if (argv[0] == "get_value_history")
	{
	  CORBA_SeqUtil::for_each(*(m_myservice0->get_value_history()),
				  seq_print<CORBA::Float>());
	  return RTC::RTC_OK;
	}
      
      std::cout << "Invalid command or argument(s)." << std::endl;
    }
  catch (...)
    {
      std::cout << "No service connected." << std::endl;
    }
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t MyServiceConsumer::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceConsumer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceConsumer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceConsumer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MyServiceConsumer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void MyServiceConsumerInit(RTC::Manager* manager)
  {
    coil::Properties profile(myserviceconsumer_spec);
    manager->registerFactory(profile,
                             RTC::Create<MyServiceConsumer>,
                             RTC::Delete<MyServiceConsumer>);
  }
  
};


