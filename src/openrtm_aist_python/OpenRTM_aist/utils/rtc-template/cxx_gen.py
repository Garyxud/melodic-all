#!/usr/bin/env python
# -*- python -*-
#
#  @file cxx_gen.py
#  @brief rtc-template C++ source code generator class
#  @date $Date: 2007/07/20 17:28:56 $
#  @author Noriaki Ando <n-ando@aist.go.jp>
# 
#  Copyright (C) 2004-2007
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 
#  $Id: cxx_gen.py,v 1.8.2.1 2007/07/20 17:28:56 n-ando Exp $
# 

#
#  $Log: cxx_gen.py,v $
#  Revision 1.8.2.1  2007/07/20 17:28:56  n-ando
#  A fix for win32 porting.
#
#  Revision 1.8  2007/05/29 01:44:29  n-ando
#  make clean target was modified to delete *Compl.o
#
#  Revision 1.7  2007/04/23 01:41:16  n-ando
#  New option "--config" and configuration template code were added.
#
#  Revision 1.6  2007/04/13 15:05:50  n-ando
#  Now RTC::OK becomes RTC::RTC_OK in RTC.idl.
#
#  Revision 1.5  2007/01/14 23:38:30  n-ando
#  Updated to use setModuleInitProc() to initialize component locally.
#
#  Revision 1.4  2007/01/11 07:46:38  n-ando
#  Modified for OMG RTC specificatin and OpenRTM-aist-0.4.0.
#  Almost all template codes were modified according to the new framework.
#
#  Revision 1.3  2005/09/08 09:23:55  n-ando
#  - A bug fix for merge function.
#
#  Revision 1.2  2005/09/06 14:37:03  n-ando
#  rtc-template's command options and data structure for ezt (Easy Template)
#  are changed for RTComponent's service features.
#  Now rtc-template can generate services' skeletons, stubs and
#  implementation files.
#  The implementation code generation uses omniidl's IDL parser.
#
#  Revision 1.1  2005/08/26 12:02:14  n-ando
#  This code generator module uses ezt (Easy Template).
#
#  Revision 1.1.1.1  2005/05/12 09:06:18  n-ando
#  Public release.
#
#

import re
import os
import sys
import StringIO
import ezt
import gen_base


def description():
	return "C++ component code generator"

def usage_short():
	"""
	C++ generator specific usage (short version)
	"""
	return """
Options for C++ backend:

    [--svc-impl-suffix[=suffix]]          Suffix of implementation class
    [--svc-skel-suffix[=suffix]]          Suffix of server skeleton files
    [--svc-stub-suffix[=suffix]]          Suffix of client stub files
"""

def usage():
	"""
	C++ generator specific usage
	"""
	return """
-------------------------------
  Help for C++ code geenrator
-------------------------------
    --svc-impl-suffix=[Suffix]:
        Specify the suffix for implementation class name. This suffix is also
        used for implementation class header file and code file.
	
    --svc-skel-suffix=[Suffix]:
        Specify the suffix for server skeleton files.
	
    --svc-stub-suffix=[Suffix]:
        Specify the suffix for client stub files.

C++ code generator generates the following files.
    [Component name].h.............Component class header
    [Component name].cpp...........Component class soruce code
    [Component name]Comp.cpp.......Component startup code
    [IDL basename]Skel.h...........Server skeleton header
    [IDL basename]Skel.cpp.........Server skeleton source code
    [IDL basename]Stub.h...........Client stub header
    [IDL basename]Stub.cpp.........Client stub source code
    [IDL basename]SVC_impl.h.......Server implementation header
    [IDL basename]SVC_impl.cpp.....Server implementation source code
    Makefile.[Component name]......Makefile to compile this codes
    README.[Component name]........Specification template of the component

 Suffixes (Skel, Stub, SVC_impl) can be specified --svc-*-suffix option

 Other CORBA implementation specific skeleton/stub code would be generated.
 --omniORB--
    [IDL basename].hh..............Client stub header
    [IDL basename]SK.cc............Server skeleton source code
    [IDL basename]DynSK.cc.........Dynamic server skeleton source code
 --TAO--
    [IDL basename]S.h..............Server skeleton header
    [IDL basename]S.h..............Server skeleton source code
    [IDL basename]C.h..............Client stub header
    [IDL basename]C.h..............Client stub source code
 --MICO--
    [IDL basename].h...............Server skeleton header
    [IDL basename]_skel.cc.........Server skeleton source code
    [IDL basename].h...............Client stub header
    [IDL basename].cc..............Client stub source code

"""

def get_opt_fmt():
	opt_args_fmt = ["svc-impl-suffix=",
			"svc-skel-suffix=",
			"svc-stub-suffix="]
	return opt_args_fmt

	
#------------------------------------------------------------
# Component header template code
#------------------------------------------------------------
comp_header = """// -*- C++ -*-
/*!
 * @file  [fname_h]
 * @brief [module.desc]
 * @date  [rcs_date]
 *
 * [rcs_id]
 */

#ifndef [u_name]_H
#define [u_name]_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
// </rtc-template>

using namespace RTC;

class [module.name]
  : public RTC::DataFlowComponentBase
{
 public:
  [module.name](RTC::Manager* manager);
  ~[module.name]();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry() 
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  // virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  // virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  // </rtc-template>

 private:
  int dummy;

};


extern "C"
{
  void [module.name]Init(RTC::Manager* manager);
};

#endif // [u_name]_H
"""


#------------------------------------------------------------
# Component source template code
#------------------------------------------------------------
comp_soruce = """// -*- C++ -*-
/*!
 * @file  [fname_cpp]
 * @brief [module.desc]
 * [rcs_date]
 *
 * [rcs_id]
 */

#include "[fname_h]"

// Module specification
// <rtc-template block="module_spec">
// </rtc-template>

[module.name]::[module.name](RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    // </rtc-template>
	dummy(0)
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // </rtc-template>

}

[module.name]::~[module.name]()
{
}


RTC::ReturnCode_t [module.name]::onInitialize()
{
  // <rtc-template block="bind_config">
  // </rtc-template>
  return RTC::RTC_OK;
}


[for activity]
/*
RTC::ReturnCode_t [module.name]::[activity.name]([activity.args])
{
  return RTC::RTC_OK;
}
*/
[end]


extern "C"
{
 
  void [module.name]Init(RTC::Manager* manager)
  {
    RTC::Properties profile([l_name]_spec);
    manager->registerFactory(profile,
                             RTC::Create<[module.name]>,
                             RTC::Delete<[module.name]>);
  }
  
};


"""


#------------------------------------------------------------
# Component soruce template
#------------------------------------------------------------
comp_compsrc = """// -*- C++ -*-
/*!
 * @file [module.name]Comp.cpp
 * @brief Standalone component
 * @date [rcs_date]
 *
 * [rcs_id]
 */

#include <rtm/Manager.h>
#include <iostream>
#include <string>
#include "[fname_h]"


void MyModuleInit(RTC::Manager* manager)
{
  [module.name]Init(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("[module.name]");


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
//    port = (*portlist)[begin_brace]i[end_brace];
//    std::cout << "Port" << i << " (name): ";
//    std::cout << port->get_port_profile()->name << std::endl;
//    
//    RTC::PortInterfaceProfileList iflist;
//    iflist = port->get_port_profile()->interfaces;
//    std::cout << "---interfaces---" << std::endl;
//    for (CORBA::ULong i(0), n(iflist.length()); i < n; ++i)
//    {
//      std::cout << "I/F name: ";
//      std::cout << iflist[begin_brace]i[end_brace].instance_name << std::endl;
//      std::cout << "I/F type: ";
//      std::cout << iflist[begin_brace]i[end_brace].type_name << std::endl;
//      const char* pol;
//      pol = iflist[begin_brace]i[end_brace].polarity == 0 ? "PROVIDED" : "REQUIRED";
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
  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  // Initialize manager
  manager->init(argc, argv);

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
"""


#------------------------------------------------------------
# Makefile template
#------------------------------------------------------------
makefile = """# -*- Makefile -*-
#
# @file  Makefile.[module.name]
# @brief RTComponent makefile for "[module.name] component"
# @date  [rcs_date]
#
# This file is generated by rtc-template with the following argments.
#
[for fmtd_args]#  [fmtd_args]
[end]#
#
# [rcs_id]
#
CXXFLAGS = `rtm-config --cflags`
LDFLAGS  = `rtm-config --libs`
SHFLAGS  = -shared

IDLC     = `rtm-config --idlc`
IDLFLAGS = `rtm-config --idlflags` -I`rtm-config --prefix`/include/rtm/idl
WRAPPER  = rtm-skelwrapper
WRAPPER_FLAGS = --include-dir="" --skel-suffix=Skel --stub-suffix=Stub

SKEL_OBJ = [for service_idl][service_idl.skel_basename].o [end] \
	[for consumer_idl][consumer_idl.stub_basename].o [end]
STUB_OBJ = [for service_idl][service_idl.stub_basename].o [end]
IMPL_OBJ = [for service_idl][service_idl.impl_basename].o [end]
OBJS     = [module.name].o $(SKEL_OBJ) $(IMPL_OBJ)

.SUFFIXES : .so

all: [module.name].so [module.name]Comp


.cpp.o:
	rm -f $@
	$(CXX) $(CXXFLAGS) -c -o $@ $<

.o.so:
	rm -f $@
	$(CXX) $(SHFLAGS) -o $@ $(OBJS) $(LDFLAGS)

[module.name]Comp: [module.name]Comp.o $(OBJS)
	$(CXX) -o $@ $(OBJS) [module.name]Comp.o $(LDFLAGS) 


clean: clean_objs clean_skelstub
	rm -f *~

clean_objs:
	rm -f $(OBJS) [module.name]Comp.o [module.name].so [module.name]Comp

clean_skelstub:
	rm -f *[skel_suffix].h *[skel_suffix].cpp
	rm -f *[stub_suffix].h *[stub_suffix].cpp



[for service_idl]
[service_idl.skel_basename].cpp : [service_idl.idl_fname]
	$(IDLC) $(IDLFLAGS) [service_idl.idl_fname]
	$(WRAPPER) $(WRAPPER_FLAGS) --idl-file=[service_idl.idl_fname]
[service_idl.skel_basename].h : [service_idl.idl_fname]
	$(IDLC) $(IDLFLAGS) [service_idl.idl_fname]
	$(WRAPPER) $(WRAPPER_FLAGS) --idl-file=[service_idl.idl_fname]
[service_idl.stub_basename].cpp : [service_idl.idl_fname]
	$(IDLC) $(IDLFLAGS) [service_idl.idl_fname]
	$(WRAPPER) $(WRAPPER_FLAGS) --idl-file=[service_idl.idl_fname]
[service_idl.stub_basename].h : [service_idl.idl_fname]
	$(IDLC) $(IDLFLAGS) [service_idl.idl_fname]
	$(WRAPPER) $(WRAPPER_FLAGS) --idl-file=[service_idl.idl_fname]
[end]

[for consumer_idl]
[consumer_idl.stub_basename].cpp : [consumer_idl.idl_fname]
	$(IDLC) $(IDLFLAGS) [consumer_idl.idl_fname]
	$(WRAPPER) $(WRAPPER_FLAGS) --idl-file=[consumer_idl.idl_fname]
[consumer_idl.stub_basename].h : [consumer_idl.idl_fname]
	$(IDLC) $(IDLFLAGS) [consumer_idl.idl_fname]
	$(WRAPPER) $(WRAPPER_FLAGS) --idl-file=[consumer_idl.idl_fname]
[end]


[module.name].so: $(OBJS)
[module.name].o: [module.name].h \
	[for service_idl][service_idl.skel_basename].h [service_idl.impl_basename].h [end] \
	[for consumer_idl][consumer_idl.stub_basename].h [end]
[module.name]Comp.o: [module.name]Comp.cpp [module.name].cpp [module.name].h [for service_idl][service_idl.skel_basename].h [service_idl.impl_basename].h [end]

[for service_idl]
[service_idl.impl_basename].o: [service_idl.impl_basename].cpp [service_idl.impl_basename].h [service_idl.skel_basename].h [service_idl.stub_basename].h
[service_idl.skel_basename].o: [service_idl.skel_basename].cpp [service_idl.skel_basename].h [service_idl.stub_basename].h
[service_idl.stub_basename].o: [service_idl.stub_basename].cpp [service_idl.stub_basename].h
[end]

[for consumer_idl]
[consumer_idl.stub_basename].o: [consumer_idl.stub_basename].cpp [consumer_idl.stub_basename].h
[end]

# end of Makefile
"""


#============================================================
# Replaced strings definition for <rtc-template> tags
#============================================================
service_impl_h = """[for service_idl]#include "[service_idl.impl_h]"
[end]"""
consumer_stub_h = """[for consumer_idl]#include "[consumer_idl.stub_h]"
[end]"""

module_spec = """static const char* [l_name]_spec[] =
  {
    "implementation_id", "[module.name]",
    "type_name",         "[module.name]",
    "description",       "[module.desc]",
    "version",           "[module.version]",
    "vendor",            "[module.vendor]",
    "category",          "[module.category]",
    "activity_type",     "[module.comp_type]",
    "max_instance",      "[module.max_inst]",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
[for config]    "conf.default.[config.name]", "[config.default]",
[end]
    ""
  };"""

config_declare = \
"""  [for config][config.type] m_[config.name];
  [end]"""
  

inport_declare = \
"""  [for inport][inport.type] m_[inport.name];
  InPort<[inport.type]> m_[inport.name]In;
  [end]"""

outport_declare = \
"""  [for outport][outport.type] m_[outport.name];
  OutPort<[outport.type]> m_[outport.name]Out;
  [end]"""

corbaport_declare = \
"""  [for corbaport]RTC::CorbaPort m_[corbaport.name]Port;
  [end]"""
 
service_declare = \
"""  [for service][service.type]SVC_impl m_[service.name];
  [end]"""

consumer_declare = \
"""  [for consumer]RTC::CorbaConsumer<[consumer.type]> m_[consumer.name];
  [end]"""

initializer = \
"""    [for inport]m_[inport.name]In("[inport.name]", m_[inport.name]),
    [end][for outport]m_[outport.name]Out("[outport.name]", m_[outport.name]),
    [end][for corbaport]m_[corbaport.name]Port("[corbaport.name]"),[end]"""


registration = \
"""  // Set InPort buffers
  [for inport]registerInPort("[inport.name]", m_[inport.name]In);
  [end]
  // Set OutPort buffer
  [for outport]registerOutPort("[outport.name]", m_[outport.name]Out);
  [end]
  // Set service provider to Ports
  [for service]m_[service.port]Port.registerProvider("[service.name]", "[service.type]", m_[service.name]);
  [end]
  // Set service consumers to Ports
  [for consumer]m_[consumer.port]Port.registerConsumer("[consumer.name]", "[consumer.type]", m_[consumer.name]);
  [end]
  // Set CORBA Service Ports
  [for corbaport]registerPort(m_[corbaport.name]Port);
  [end]"""

bind_config = \
"""  // Bind variables and configuration variable
  [for config]bindParameter("[config.name]", m_[config.name], "[config.default]");
  [end]"""

#------------------------------------------------------------


#============================================================
# Classes and Functions
#============================================================

class Struct:
	def __init__(self):
		return


def MakeSuffix(opts, dict):
	impl_suffix = "SVC_impl"
	skel_suffix = "Skel"
	stub_suffix = "Stub"
	for opt, arg in opts:
		if opt.find("--svc-impl-suffix") == 0:
			impl_suffix = arg
		if opt.find("--svc-skel-suffix") == 0:
			skel_suffix = arg
		if opt.find("--svc-stub-suffix") == 0:
			stub_suffix = arg
	dict["impl_suffix"] = impl_suffix
	dict["skel_suffix"] = skel_suffix
	dict["stub_suffix"] = stub_suffix


def MakeServiceIDL(dict):
	for d in dict["service_idl"]:
		d.impl_basename = d.idl_basename + dict["impl_suffix"]
		d.impl_h        = d.impl_basename + ".h"
		d.impl_cpp      = d.impl_basename + ".cpp"
		d.skel_basename = d.idl_basename + dict["skel_suffix"]
		d.skel_h        = d.skel_basename + ".h"
		d.skel_cpp      = d.skel_basename + ".cpp"
		d.stub_suffix   = dict["stub_suffix"]
		d.stub_basename = d.idl_basename + dict["stub_suffix"]
		d.stub_h        = d.stub_basename + ".h"
		d.stub_cpp      = d.stub_basename + ".cpp"


def MakeConsumerIDL(dict):
	conslist = []
	for cons in dict["consumer_idl"]:
		dup = False
		for svc in dict["service_idl"]:
			if cons.idl_fname == svc.idl_fname:
				dup = True
		if not dup:
			tmp = cons
			tmp.stub_suffix   = dict["stub_suffix"]
			tmp.stub_basename = tmp.idl_basename \
			    + dict["stub_suffix"]
			tmp.stub_h        = tmp.stub_basename + ".h"
			tmp.stub_cpp      = tmp.stub_basename + ".cpp"
			conslist.append(tmp)
	dict["consumer_idl"] = conslist


def MakeActivityFuncs(dict):
	acts = (("onFinalize",    ""), \
		("onStartup",     "RTC::UniqueId ec_id"), \
		("onShutdown",    "RTC::UniqueId ec_id"), \
		("onActivated",   "RTC::UniqueId ec_id"), \
		("onDeactivated", "RTC::UniqueId ec_id"), \
		("onExecute",     "RTC::UniqueId ec_id"), \
		("onAborting",    "RTC::UniqueId ec_id"), \
		("onError",       "RTC::UniqueId ec_id"), \
		("onReset",       "RTC::UniqueId ec_id"), \
		("onStateUpdate", "RTC::UniqueId ec_id"), \
		("onRateChanged", "RTC::UniqueId ec_id"))
	actlist = []
	for name, args in acts:
		a = Struct()
		a.name = name
		a.args = args
		actlist.append(a)

	dict["activity"] = actlist


class cxx_gen(gen_base.gen_base):
	"""
	C++ component source code generator
	"""
	_fname_space = 16
	def __init__(self, data, opts):
		self.data = data.copy()
		
		MakeSuffix(opts, self.data)
		MakeServiceIDL(self.data)
		MakeConsumerIDL(self.data)
		MakeActivityFuncs(self.data)
		self.data["begin_brace"] = "["
		self.data["end_brace"] = "]"
		self.data["rcs_date"] = "$" + "Date" + "$"
		self.data["rcs_id"] = "$" + "Id" + "$"
		self.data["fname_h"] = self.data["fname"] + ".h"
		self.data["fname_cpp"] = self.data["fname"] + ".cpp"
		self.data["fname_comp"] = self.data["fname"] + "Comp.cpp"
		self.data["makefile"] = "Makefile." + self.data["fname"]
		self.data["u_name"] = self.data["module"].name.upper()
		self.data["l_name"] = self.data["module"].name.lower()


		self.tags = {}
		self.tags["service_impl_h"]    = service_impl_h
		self.tags["consumer_stub_h"]   = consumer_stub_h
		self.tags["module_spec"]       = module_spec
		self.tags["config_declare"]    = config_declare
		self.tags["inport_declare"]    = inport_declare
		self.tags["outport_declare"]   = outport_declare
		self.tags["corbaport_declare"] = corbaport_declare
		self.tags["service_declare"]   = service_declare
		self.tags["consumer_declare"]  = consumer_declare
		self.tags["initializer"]       = initializer
		self.tags["registration"]      = registration
		self.tags["bind_config"]       = bind_config

		self.gen_tags(self.tags)
		return


	def print_header(self):
		"""
		Generate component class header
		"""
		self.gen(self.data["fname_h"],
			 comp_header, self.data, self.tags)
		return


	def print_source(self):
		"""
		Generate component class source code
		"""
		self.gen(self.data["fname_cpp"],
			 comp_soruce, self.data, self.tags)
		return


	def print_compsrc(self):
		"""
		Generate component source code
		"""
		self.gen(self.data["fname_comp"],
			 comp_compsrc, self.data, self.tags)
		return


	def print_makefile(self):
		"""
		Generate Makefile
		"""
		self.gen(self.data["makefile"],
			 makefile, self.data, self.tags)


	def print_impl(self):
		for svc_idl in self.data["service_idl"]:
			fd_h = None
			fd_cpp = None
			lines_h = None
			lines_cpp = None
			if not os.access(svc_idl.idl_fname, os.F_OK):
				sys.stderr.write("Error: IDL file \"" \
						 + svc_idl.idl_fname \
						 + "\" not found.\n")
				sys.exit(1)
			fd_h, lines_h = \
			    self.check_overwrite(svc_idl.impl_h)
			fd_cpp, lines_cpp = \
			    self.check_overwrite(svc_idl.impl_cpp)
			if not fd_h:
				sys.stderr.write("Cannot open file:" + 
						 svc_idl.impl_h + "\n")
				sys.exit(1)
			if not fd_cpp:
				sys.stderr.write("Cannot open file:" + 
						 svc_idl.impl_cpp + "\n")
				sys.exit(1)
			if lines_h or lines_cpp:
				sys.stderr.write("Merge of service impl." +
						 "code is not supported.\n")
				return

			try:
				idl_include = self.data["idl_include"]
				impl_suffix = self.data["impl_suffix"]
				skel_suffix = self.data["skel_suffix"]
				import cxx_svc_impl
				ifs = cxx_svc_impl.generate(svc_idl.idl_fname,
							    idl_include,
							    impl_suffix,
							    skel_suffix,
							    fd_h, fd_cpp)
				print "  File \"" \
				    + svc_idl.impl_h \
				    + "\" was generated."
				print "  File \"" \
				    + svc_idl.impl_cpp \
				    + "\" was generated."
			except:
				sys.stderr.write("Generate error: " \
						 + svc_idl.impl_h
						 + ", "
						 + svc_idl.impl_cpp + "\n")
				
	def print_all(self):
		self.print_impl()
		self.print_header()
		self.print_source()
		self.print_compsrc()
		self.print_makefile()
		return


class idl2char:
	def __init__(self, filename = None):
		self.lines = ""
		self.inComment = False
		if filename:
			f = open(filename)
			l = f.readlines()
			self.erase_comments(l)
			f.close()

	def open_file(self, filename):
		f = open(filename)
		l = f.readlines()
		self.erase_comments(l)
		f.close()

	def cleanup():
		self.lines = ""

	def erase_comments(self, lines):
		for l in lines:
			m = re.search("/\*.*\*/", l)
			if m:
				l = l.replace(m.group(), "")
				
			m = re.search(".*\*/", l)
			if m:
				l = l.replace(m.group(), "")
				if self.inComment == True:
					self.inComment = False

			m = re.search("/\*.*$", l)
			if m:
				l = l.replace(m.group(), "")
				if self.inComment == False:
					self.inComment = True

			m = re.search("//.*$", l)
			if m:
				l = l.replace(m.group(), "")

			m = re.search("\s+$", l)
			if m:
				l = l.replace(m.group(), "")
				l = l + "\n"

			m = re.search("\S", l)
			if m:
				if not self.inComment:
					self.lines += l

					
	def get_lines(self):
		return self.lines

	def make_chardata(self, var_name):
		self.lines = self.lines.replace("\n","\\n\\\n")
		self.lines = self.lines.replace("\"","\\\"")
		self.lines = "static char* " + var_name  \
		    + " = \"\\\n" + self.lines
		self.lines += "\";\n"
		return self.lines

	def add_slash(self):
		self.lines = self.lines.replace("\n","\\\n")
		return self.lines

	def write_file(self, filename):
		f = open(filename, "w")
		f.write(self.lines)
		f.close()


