#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  @file Py_src.py
#  @brief rtc-template Python soruce code generator class
#  @date $Date: 2005/08/26 12:02:37 $
#  @author Noriaki Ando <n-ando@aist.go.jp>
# 
#  Copyright (C) 2004-2005
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 
#  $Id: python_gen.py,v 1.1 2005/08/26 12:02:37 n-ando Exp $
# 

#
#  $Log: python_gen.py,v $
#  Revision 1.1  2005/08/26 12:02:37  n-ando
#  This code generator module uses ezt (Easy Template).
#
#  Revision 1.1.1.1  2005/05/12 09:06:18  n-ando
#  Public release.
#
#

import re
import os
import ezt
import gen_base

import string
import sys

def description():
  return "Python component code generator"

def usage():
  """
  Python generator specific usage
  """
  return """
----------------------------------
  Help for Python code generator
----------------------------------
Python code generator generates the following files.
    [Component name].py............Component class and executable
    README.[Component name]........Specification template of the component

No additional options are available for Python code generator.

"""

def get_opt_fmt():
  return []


service_impl = """\
[for service_idl]from [service_idl.idl_basename]_idl_example import *
[end]"""

consumer_import = """\
import _GlobalIDL, _GlobalIDL__POA
"""

initialize_configuration_param = """\
    [for config]self._[config.name] = [config.default_data]
    [end]"""

module_spec = """\
[l_name]_spec = ["implementation_id", "[module.name]", 
     "type_name",         "[module.name]", 
     "description",       "[module.desc]", 
     "version",           "[module.version]", 
     "vendor",            "[module.vendor]", 
     "category",          "[module.category]", 
     "activity_type",     "[module.comp_type]", 
     "max_instance",      "[module.max_inst]", 
     "language",          "Python", 
     "lang_type",         "SCRIPT",
[for config]     "conf.default.[config.name]", "[config.default]",
[end]    ""]"""

#------------------------------------------------------------
# Python component source code template
#------------------------------------------------------------
py_source = """#!/usr/bin/env python
# -*- Python -*-

import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

# Import Service implementation class
# <rtc-template block="service_impl">
# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
# </rtc-template>

class [module.name](OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

    [for inport]self._d_[inport.name] = RTC.[inport.type]([inport.data_type_args])
    self._[inport.name]In = OpenRTM_aist.InPort("[inport.name]", self._d_[inport.name])
    [end][for outport]self._d_[outport.name] = RTC.[outport.type]([outport.data_type_args])
    self._[outport.name]Out = OpenRTM_aist.OutPort("[outport.name]", self._d_[outport.name])
    [end]
    [for corbaport]self._[corbaport.name]Port = OpenRTM_aist.CorbaPort("[corbaport.name]")
    [end]
    [for service]self._[service.name] = [service.type]_i()
    [end]
    [for consumer]self._[consumer.name] = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.[consumer.type])
    [end]
    # initialize of configuration-data.
    # <rtc-template block="init_conf_param">
    # </rtc-template>


     
  def onInitialize(self):
    # Bind variables and configuration variable
    [for config]self.bindParameter("[config.name]", self._[config.name], "[config.default]")
    [end]

    # Set InPort buffers
    [for inport]self.addInPort("[inport.name]",self._[inport.name]In)
    [end]
    # Set OutPort buffers
    [for outport]self.addOutPort("[outport.name]",self._[outport.name]Out)
    [end]

    # Set service provider to Ports
    [for service]self._[service.port]Port.registerProvider("[service.name]", "[service.type]", self._[service.name])
    [end]
    # Set service consumers to Ports
    [for consumer]self._[consumer.port]Port.registerConsumer("[consumer.name]", "[consumer.type]", self._[consumer.name])
    [end]
    # Set CORBA Service Ports
    [for corbaport]self.addPort(self._[corbaport.name]Port)
    [end]

    return RTC.RTC_OK


  [for activity]
  #def [activity.name](self, ec_id):
  #
  # return RTC.RTC_OK
  [end]


def [module.name]Init(manager):
  profile = OpenRTM_aist.Properties(defaults_str=[l_name]_spec)
  manager.registerFactory(profile,
                          [module.name],
                          OpenRTM_aist.Delete)

def MyModuleInit(manager):
  [module.name]Init(manager)

  # Create a component
  comp = manager.createComponent("[module.name]")



def main():
  mgr = OpenRTM_aist.Manager.init(sys.argv)
  mgr.setModuleInitProc(MyModuleInit)
  mgr.activateManager()
  mgr.runManager()

if __name__ == "__main__":
  main()

"""



class Struct:
  def __init__(self):
    return


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
  

class python_gen(gen_base.gen_base):
  """
  Python component source code generation class
  """
  def __init__(self, data, opts):
    self.data = data.copy()
    MakeActivityFuncs(self.data)
    MakeSuffix(opts, self.data)
    self.data['fname_py'] = self.data['fname'] + ".py"
    self.data["begin_brace"] = "["
    self.data["end_brace"] = "]"
    self.data["u_name"] = self.data["module"].name.upper()
    self.data["l_name"] = self.data["module"].name.lower()


    self.tags = {}
    if self.data["service_idl"]:
      for svc in self.data["service_idl"]:
        svc.impl_py = svc.idl_basename + "_idl_example.py"
      self.tags["service_impl"]    = service_impl

    if self.data["consumer_idl"]:
      for cons in self.data["consumer_idl"]:
        try:
          cons.modulename = "_GlobalIDL"
          f = open(cons.idl_fname,'a+')
          while 1:
            _str = f.readline()
            if not _str:
              break
            mod_idx = _str.find("module",0)
            if mod_idx > -1:
              _str = _str[mod_idx+6:]
              idx = _str.find("{",0)
              if idx > -1:
                _str = _str[:idx]
              cons.modulename = string.strip(_str)
              break
          f.close()
        except IOError:
          print "Can't find file:", file

      self.tags["consumer_import"] = consumer_import
        
    if self.data["config"]:
      for i in range(len(self.data["config"])):
        split_data = self.data["config"][i].default.split(",")
        if len(split_data) > 1:
          _data = []
          _type = self.get_type(self.data["config"][i].type)
          for d in split_data:
            _data.append(_type(d))
          self.data["config"][i].default_data = [_data]
        else:
          _type = self.get_type(self.data["config"][i].type)
          self.data["config"][i].default_data = [_type(self.data["config"][i].default)]
        
    self.tags["init_conf_param"] = initialize_configuration_param

    if self.data["inport"]:
      for inp in self.data["inport"]:
        if self.check_data_type(inp.type) == "sequence":
          inp.data_type_args = "RTC.Time(0,0),[]"
        else:
          inp.data_type_args = "RTC.Time(0,0),0"

    if self.data["outport"]:
      for outp in self.data["outport"]:
        if self.check_data_type(outp.type) == "sequence":
          outp.data_type_args = "RTC.Time(0,0),[]"
        else:
          outp.data_type_args = "RTC.Time(0,0),0"


      
    self.tags["module_spec"]       = module_spec
    self.gen_tags(self.tags)
    return


  def check_data_type(self, _type):
    if str(_type) in ["TimedShortSeq", "TimedLongSeq", "TimedUShortSeq",
          "TimedULongSeq", "TimedFloatSeq", "TimedDoubleSeq",
          "TimedCharSeq","TimedBooleanSeq", "TimedOctetSeq",
          "TimedStringSeq"]:
      return "sequence"
    else:
      return None
       

  def get_type(self, _type):
    if str(_type) == "int":
      return int
    elif str(_type) == "long":
      return long
    elif str(_type) == "float":
      return float
    elif str(_type) == "double":
      return float
    elif str(_type) == "string":
      return str
    else:
      return str
    

  def print_impl(self):
    for svc_idl in self.data["service_idl"]:
      if not os.access(svc_idl.idl_fname, os.F_OK):
        sys.stderr.write("Error: IDL file \"" \
             + svc_idl.idl_fname \
             + "\" not found.\n")
        sys.exit(1)

      try:
        cmd = "omniidl -bpython -Wbexample "+svc_idl.idl_fname
        os.system(cmd)
      except:
        sys.stderr.write("Generate error: " \
             + svc_idl.impl_py + "\n")


      print "  File \"" \
            + svc_idl.impl_py \
            + "\" was generated."

    for cons in self.data["consumer_idl"]:
      dup = False
      for svc in self.data["service_idl"]:
        if cons.idl_fname == svc.idl_fname:
          dup = True

      if not dup:
        if not os.access(cons.idl_fname, os.F_OK):
          sys.stderr.write("Error: IDL file \"" \
               + cons.idl_fname \
               + "\" not found.\n")
          sys.exit(1)

        try:
          cmd = "omniidl -bpython "+cons.idl_fname
          os.system(cmd)
        except:
          sys.stderr.write("Generate error: omniidl -bpython "+cons.idl_fname)


  def print_pysrc(self):
    """
    Generate component class script
    """
    self.gen(self.data["fname_py"],py_source,self.data, self.tags)
    return


  def print_all(self):
    self.print_impl()
    self.print_pysrc()
    return
