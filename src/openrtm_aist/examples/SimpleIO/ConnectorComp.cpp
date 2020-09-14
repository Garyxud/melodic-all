// -*- C++ -*-
/*!
 * @file ConnectorComp.cpp
 * @brief connector application
 * @date $Date: 2008-01-13 07:24:08 $
 *
 * Copyright (c) 2003-2007 Noriaki Ando <n-ando@aist.go.jp>
 *          Task-intelligence Research Group,
 *          Intelligent System Research Institute,
 *          National Institute of Industrial Science (AIST), Japan
 *          All rights reserved.
 *
 * $Id$
 */

#include <iostream>
#include <vector>
#include <string>
#include <rtm/CorbaNaming.h>
#include <rtm/RTObject.h>
#include <rtm/NVUtil.h>
#include <rtm/CORBA_SeqUtil.h>
#include <rtm/CorbaConsumer.h>
#include <assert.h>
#include <coil/stringutil.h>


using namespace RTC;

#ifdef __QNX__
using std::exit;
#endif

void usage()
{
  std::cout << std::endl;
  std::cout << "usage: ConnectorComp [options].." << std::endl;
  std::cout << std::endl;
  std::cout << "  --port *        ";
  std::cout << ": Set port no 2809 (default:9876)" << std::endl;
  std::cout << "  --origin *      ";
  std::cout << ": Set origin of connection OUT (default:IN)" << std::endl;
  std::cout << "                  : Set origin OUT ,     ConsoleOut -> ConsoleIn" << std::endl;
  std::cout << "                  :  default connection, ConsoleIn  -> ConsoleOut" << std::endl;
  std::cout << "  --flush         ";
  std::cout << ": Set subscription type flush (default:flush)" << std::endl;
  std::cout << "  --new           ";
  std::cout << ": Set subscription type new" << std::endl;
  std::cout << "  --periodic [Hz] ";
  std::cout << ": Set subscription type periodic" << std::endl;
  std::cout << "  --policy [any]  ";
  std::cout << ": Set push policy ALL or FIFO or SKIP or NEW" << std::endl;
  std::cout << "  --skip [n]      ";
  std::cout << ": Set skip count 0..n" << std::endl;
  std::cout << "  --endian *      ";
  std::cout << ": Set endian type Little or Big" << std::endl;
  std::cout << "  --buffer : Set buffer configuration" << std::endl;
  std::cout << "  --buffer [inport|outport].buffer.[length|[read|write.[policy|timeout]] value" << std::endl;
  std::cout << "    policy:  [write.full_policy,read.empty_policy]" << std::endl;
  std::cout << "    timeout: [write.timeout,read.timeout]" << std::endl;
  std::cout << "    value:  " << std::endl;
  std::cout << "       full_policy:  [overwrite, do_nothing, block]" << std::endl;
  std::cout << "       empty_policy: [readback,  do_nothing, block]" << std::endl;
  std::cout << "       timeout: sec (e.g. 1.0)" << std::endl;
  std::cout << "       length:  The length of the ring buffer. (e.g. 10)" << std::endl;
  std::cout << std::endl;
  std::cout << "exsample:" << std::endl;
  std::cout << "  ConnectorComp --port 2809 --origin OUT" << std::endl;
  std::cout << "  ConnectorComp --flush" << std::endl;
  std::cout << "  ConnectorComp --new" << std::endl;
  std::cout << "  ConnectorComp --new --policy ALL" << std::endl;
  std::cout << "  ConnectorComp --new --policy SKIP --skip 100" << std::endl;
  std::cout << "  ConnectorComp --periodic 10" << std::endl;
  std::cout << "  ConnectorComp --periodic 10 --policy FIFO" << std::endl;
  std::cout << "  ConnectorComp --periodic 10 --policy NEW" << std::endl;
  std::cout << "  ConnectorComp --flush --endian Little" << std::endl;
  std::cout << "  ConnectorComp --buffer inport.buffer.read.empty_policy block \\" << std::endl;
  std::cout << "                --buffer inport.buffer.read.timeout 0.1 \\ " << std::endl;
  std::cout << "                --buffer inport.buffer.length 10 " << std::endl;
  std::cout << std::endl;
}

int main (int argc, char** argv)
{
  int _argc(0);
  char** _argv(0);

  std::string subs_type("flush");
  std::string period;
  std::string push_policy;
  std::string skip_count;
  std::string endian;
  std::string port_no("9876");
  std::string connect_origin("in");
  std::map<std::string, std::string> buffer_prop;

  if (argc < 2)
    {
      usage();
    }

  for (int i = 1; i < argc; ++i)
    {
      std::string arg(argv[i]);
      coil::normalize(arg);
      if (arg == "--flush")         subs_type = "flush";
      else if (arg == "--new")      subs_type = "new";
      else if (arg == "--periodic")
	{
	  subs_type = "periodic";
	  if (++i < argc) period = argv[i];
	  else            period = "1.0";
	}
      else if (arg == "--help")
	{
	  usage();
	  exit(1);
	}
      else if (arg == "--policy")
	{
	  if (++i < argc)
	    {
	      std::string arg2(argv[i]);
	      coil::normalize(arg2);
	      push_policy = arg2;
	    }
	  else            push_policy = "new";
	}
      else if (arg == "--skip")
	{
	  if (++i < argc) skip_count = argv[i];
	  else            skip_count = "0";
	}

      if (arg == "--endian")
	{
	  if (++i < argc)
	    {
	      std::string arg2(argv[i]);
	      coil::normalize(arg2);
	      endian = arg2;
	    }
	  else            endian = "";
	}
      if (arg == "--port")
	{
	  if (++i < argc)
	    {
	      port_no = argv[i];
	    }
	}
      if (arg == "--origin")
	{
	  if (++i < argc)
	    {
	      std::string arg2(argv[i]);
	      coil::normalize(arg2);
	      connect_origin = arg2;
	    }
	}
      if (arg == "--buffer")
	{
	  if (++i < argc)
	    {
	      std::string key(argv[i]);
	      key = coil::normalize(key);
	      std::string val(argv[++i]);
	      val = coil::normalize(val);
	      buffer_prop.insert(std::pair< std::string, std::string >(key,val));
	    }
	}
    }
  
  CORBA::ORB_var orb = CORBA::ORB_init(_argc, _argv);
  std::string name_server("localhost:");
  name_server.append(port_no);
  CorbaNaming naming(orb, name_server.c_str());

  CorbaConsumer<RTObject> conin, conout;
  PortServiceList_var pin;
  PortServiceList_var pout;

  // option dump
  std::cout << "      Name Server: " << name_server << std::endl;
  std::cout << "Subscription Type: " << subs_type << std::endl;
  if (period != "")
    std::cout << "           Period: " << period << " [Hz]" << std::endl;

  std::cout << "      push policy: " << push_policy << std::endl;
  std::cout << "       skip count: " << skip_count << std::endl;

  if (endian != "")
    std::cout << "           endian: " << endian << std::endl;

  if (connect_origin == "in")
   std::cout << "          connect: ConsoleIn -> ConsoleOut" << std::endl;
  else
    std::cout << "          connect: ConsoleOut -> ConsoleIn" << std::endl;
  std::cout << std::endl;

  // find ConsoleIn0 component
  conin.setObject(naming.resolve("ConsoleIn0.rtc"));

  // get ports
  pin = conin->get_ports();
  pin[(CORBA::ULong)0]->disconnect_all();
  assert(pin->length() > 0);
  // activate ConsoleIn0
  ExecutionContextList_var eclisti;
  eclisti = conin->get_owned_contexts();
  eclisti[(CORBA::ULong)0]->activate_component(RTObject::_duplicate(conin._ptr()));

  // find ConsoleOut0 component
  conout.setObject(naming.resolve("ConsoleOut0.rtc"));
  // get ports
  pout = conout->get_ports();
  pout[(CORBA::ULong)0]->disconnect_all();
  assert(pout->length() > 0);
  // activate ConsoleOut0
  ExecutionContextList_var eclisto;
  eclisto = conout->get_owned_contexts();
  eclisto[(CORBA::ULong)0]->activate_component(RTObject::_duplicate(conout._ptr()));

  // connect ports
  ConnectorProfile prof;
  prof.connector_id = (char*)"";
  prof.name = CORBA::string_dup("connector0");
  prof.ports.length(2);
  prof.ports[0] = pin[(CORBA::ULong)0];
  prof.ports[1] = pout[(CORBA::ULong)0];
  CORBA_SeqUtil::push_back(prof.properties,
			   NVUtil::newNV("dataport.interface_type",
					 "corba_cdr"));
  CORBA_SeqUtil::push_back(prof.properties,
			   NVUtil::newNV("dataport.dataflow_type",
					 "push"));
  if (subs_type != "")
    CORBA_SeqUtil::push_back(prof.properties,
			   NVUtil::newNV("dataport.subscription_type",
					 subs_type.c_str()));
  else
    CORBA_SeqUtil::push_back(prof.properties,
			   NVUtil::newNV("dataport.subscription_type",
					 "flush"));
  if (subs_type == "periodic" && period != "")
    CORBA_SeqUtil::push_back(prof.properties,
			   NVUtil::newNV("dataport.publisher.push_rate",
					 period.c_str()));
  if (push_policy != "")
    CORBA_SeqUtil::push_back(prof.properties,
			   NVUtil::newNV("dataport.publisher.push_policy",
					 push_policy.c_str()));
  if (push_policy == "skip" && skip_count != "")
    CORBA_SeqUtil::push_back(prof.properties,
			   NVUtil::newNV("dataport.publisher.skip_count",
					 skip_count.c_str()));
  if (endian != "")
  CORBA_SeqUtil::push_back(prof.properties,
			   NVUtil::newNV("dataport.serializer.cdr.endian",
					 endian.c_str()));

  std::map<std::string, std::string>::iterator it=buffer_prop.begin();
  while (it != buffer_prop.end()) {
    std::string key("dataport.");
    key += it->first;
    CORBA_SeqUtil::push_back(prof.properties,
			     NVUtil::newNV(key.c_str(),
					   it->second.c_str()));
    ++it;
  }

  ReturnCode_t ret;
  if (connect_origin == "in")
    {
      ret = pin[(CORBA::ULong)0]->connect(prof);
    }
  else
    {
      ret = pout[(CORBA::ULong)0]->connect(prof);
    }

  assert(ret == RTC::RTC_OK);

  std::cout << "Connector ID: " << prof.connector_id << std::endl;
  NVUtil::dump(prof.properties);

  orb->destroy();
  exit(1);
}
