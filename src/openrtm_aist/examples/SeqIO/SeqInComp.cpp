// -*- C++ -*-
/*!
 * @file SeqInComp.cpp
 * @brief Standalone component
 * @date $Date: 2008-01-14 07:46:00 $
 *
 * $Id$
 */

#include <rtm/Manager.h>
#include <iostream>
#include <string>
#include "SeqIn.h"


// Connector Listener Dump Flag : default OFF
extern bool g_Listener_dump_enabled;

void MyModuleInit(RTC::Manager* manager)
{
  SeqInInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("SeqIn");


  // Example
  // The following procedure is examples how handle RT-Components.
  // These should not be in this function.

  // Get the component's object reference
//  RTC::RTObject_var rtobj;
//  rtobj = RTC::RTObject::_narrow(manager->getPOA()->servant_to_reference(comp));

  // Get the port list of the component
//  PortList* portlist;
//  portlist = rtobj->get_ports();

  // getting port profiles
//  std::cout << "Number of Ports: ";
//  std::cout << portlist->length() << std::endl << std::endl; 
//  for (CORBA::ULong i(0), n(portlist->length()); i < n; ++i)
//  {
//    Port_ptr port;
//    port = (*portlist)[i];
//    std::cout << "Port" << i << " (name): ";
//    std::cout << port->get_port_profile()->name << std::endl;
//    
//    RTC::PortInterfaceProfileList iflist;
//    iflist = port->get_port_profile()->interfaces;
//    std::cout << "---interfaces---" << std::endl;
//    for (CORBA::ULong i(0), n(iflist.length()); i < n; ++i)
//    {
//      std::cout << "I/F name: ";
//      std::cout << iflist[i].instance_name << std::endl;
//      std::cout << "I/F type: ";
//      std::cout << iflist[i].type_name << std::endl;
//      const char* pol;
//      pol = iflist[i].polarity == 0 ? "PROVIDED" : "REQUIRED";
//      std::cout << "Polarity: " << pol << std::endl;
//    }
//    std::cout << "---properties---" << std::endl;
//    NVUtil::dump(port->get_port_profile()->properties);
//    std::cout << "----------------" << std::endl << std::endl;
//  }

  return;
}

int main (int argc, char** argv)
{
  // check parameter
  if(argc > 1)
    {
      for (int i = 1; i < argc; ++i)
        {
          std::string arg(argv[i]);
          coil::normalize(arg);
          if (arg == "-listener")
            {
              // Connector Listener Dump Flag ON
              g_Listener_dump_enabled = true;
              std::cout << "Connector Listener Callback Dump start (Port: TimedLong)" << std::endl;
            }
        }
    }

  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  // Set module initialization proceduer
  // This procedure will be invoked in activateManager() function.
  manager->setModuleInitProc(MyModuleInit);

  // Activate manager and register to naming service
  manager->activateManager();

  // run the manager in blocking mode
  // runManager(false) is the default.
  manager->runManager();

  // If you want to run the manager in non-blocking mode, do like this
  // manager->runManager(true);

  return 0;
}
