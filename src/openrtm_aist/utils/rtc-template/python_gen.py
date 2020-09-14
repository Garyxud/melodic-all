#!/usr/bin/env python
# -*- Python -*-
#
#  @file Py_src.py
#  @brief rtc-template Python soruce code generator class
#  @date $Date: 2005/08/26 12:02:37 $
#  @author Noriaki Ando <n-ando@aist.go.jp>
# 
#  Copyright (C) 2004-2008
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 
#  $Id$
# 

import re
import os
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
[for sidl in service_idl]
from [sidl.idl_basename]_idl_example import *
[endfor]"""

global_idl = """\
import _GlobalIDL, _GlobalIDL__POA
"""
no_global_idl = ""

module_spec = """\
[l_name]_spec = ["implementation_id", "[basicInfo.name]", 
		 "type_name",         "[basicInfo.name]", 
		 "description",       "[basicInfo.description]", 
		 "version",           "[basicInfo.version]", 
		 "vendor",            "[basicInfo.vendor]", 
		 "category",          "[basicInfo.category]", 
		 "activity_type",     "[basicInfo.componentType]", 
		 "max_instance",      "[basicInfo.maxInstances]", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
[if-any configurationSet.configuration]
[for config in configurationSet.configuration]
		 "conf.default.[config.name]", "[config.defaultValue]",
[endfor][endif]
		 ""]
"""

data_ports = """\
[for dport in dataPorts]
    self._d_[dport.name] = RTC.[dport.type]([dport.data_type_args])
[if dport.portType is DataInPort]
    self._[dport.name]In = OpenRTM_aist.InPort("[dport.name]", self._d_[dport.name])
    self.addInPort("[dport.name]",self._[dport.name]In)

[elif dport.portType is DataOutPort]
    self._[dport.name]Out = OpenRTM_aist.OutPort("[dport.name]", self._d_[dport.name])
    self.addOutPort("[dport.name]",self._[dport.name]Out)

[endif]
[endfor]
"""

service_ports = """\
[for sport in servicePorts]
    self._[sport.name]Port = OpenRTM_aist.CorbaPort("[sport.name]")
[for sif in sport.serviceInterface]
[if sif.direction is Provided]
    self._[sif.name] = [sif.type]_i()
    self._[sport.name]Port.registerProvider("[sif.name]", "[sif.type]", self._[sif.name])
[elif sif.direction is Required]
    self._[sif.name] = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.[sif.type])
    self._[sport.name]Port.registerConsumer("[sif.name]", "[sif.type]", self._[sif.name])
[endif]
    self.addPort(self._[sport.name]Port)

[endfor]
[endfor]
"""

configurations = """\
[for config in configurationSet.configuration]
    self._[config.name] = [config.defaultData]

[endfor]
"""

bind_config = """
[for conf in configurationSet.configuration]
    self.bindParameter("[conf.name]", self._[conf.name], "[conf.defaultValue]")
[endfor]"""


#------------------------------------------------------------
# Python component source code template
#------------------------------------------------------------
py_source = """#!/usr/bin/env python
# -*- Python -*-

import sys
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

# Import Service implementation class
# <rtc-template block="service_impl">
# </rtc-template>

# Import Service stub modules
# <rtc-template block="global_idl">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
# </rtc-template>

class [basicInfo.name](OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

    # initialize of configuration-data.
    # <rtc-template block="configurations">
    # </rtc-template>


  def onInitialize(self):
    # DataPorts initialization
    # <rtc-template block="data_ports">
    # </rtc-template>

    # ServicePorts initialization
    # <rtc-template block="service_ports">
    # </rtc-template>

    # Bind variables and configuration variable
    # <rtc-template block="bind_config">
    # </rtc-template>
    return RTC.RTC_OK


[for act in activity]
  #def [act.name](self[if-any act.args], [act.args][else][endif]):
  #  return RTC.RTC_OK

[endfor]


def [basicInfo.name]Init(manager):
  profile = OpenRTM_aist.Properties(defaults_str=[l_name]_spec)
  manager.registerFactory(profile,
                          [basicInfo.name],
                          OpenRTM_aist.Delete)


def MyModuleInit(manager):
  [basicInfo.name]Init(manager)

  # Create a component
  comp = manager.createComponent("[basicInfo.name]")



def main():
  mgr = OpenRTM_aist.Manager.init(sys.argv)
  mgr.setModuleInitProc(MyModuleInit)
  mgr.activateManager()
  mgr.runManager()

if __name__ == "__main__":
  main()

"""




class python_gen(gen_base.gen_base):
  """
  Python component source code generation class
  """
  def __init__(self, data, opts):
    self.data = data
    self.data['fname'] = self.data['basicInfo']['name']
    self.data['fname_py'] = self.data['fname'] + ".py"
    self.data["u_name"] = self.data["fname"].upper()
    self.data["l_name"] = self.data["fname"].lower()

    self.CreateActivityFuncs(self.data)
    self.CreateDataPorts(self.data)
    self.CreateService(self.data)
    self.CreateConfiguration(self.data)
    self.tags = {}
    self.tags["service_impl"]    = service_impl
    if len(self.data["service_idl"]) > 0 or \
          len(self.data["consumer_idl"]) > 0:
      self.tags["global_idl"] = global_idl
    else:
      self.tags["global_idl"] = no_global_idl
    self.tags["module_spec"]     = module_spec
    self.tags["data_ports"]      = data_ports
    self.tags["service_ports"]   = service_ports
    self.tags["configurations"]  = configurations
    self.tags["bind_config"]     = bind_config
    self.gen_tags(self.tags)
    return


  def CreateActivityFuncs(self, dict):
    acts = (("onFinalize",    None), \
              ("onStartup",     "ec_id"), \
              ("onShutdown",    "ec_id"), \
              ("onActivated",   "ec_id"), \
              ("onDeactivated", "ec_id"), \
              ("onExecute",     "ec_id"), \
              ("onAborting",    "ec_id"), \
              ("onError",       "ec_id"), \
              ("onReset",       "ec_id"), \
              ("onStateUpdate", "ec_id"), \
              ("onRateChanged", "ec_id"))
    actlist = []
    for name, args in acts:
      a = {}
      a["name"] = name
      a["args"] = args
      actlist.append(a)
	
    dict["activity"] = actlist
    return
	
  def CreateService(self, dict):
    if dict["service_idl"]:
      for svc in dict["service_idl"]:
        svc["impl_py"] = svc["idl_basename"] + \
            "_idl_example.py"
			
    if dict["consumer_idl"]:
      for cons in dict["consumer_idl"]:
        try:
          cons["modulename"] = "_GlobalIDL"
          f = open(cons["idl_fname"], 'a+')
          while 1:
            _str = f.readline()
            if not _str:
              break
            mod_idx = _str.find("module", 0)
            if mod_idx < 0:
              break;
            _str = _str[mod_idx + 6:]
            idx = _str.find("{", 0)
            if idx < 0:
              break
            _str = _str[:idx]
            cons["modulename"] = \
                string.strip(_str)
            break
          f.close()
        except IOError:
          print "Can't find file:", file
    return

  def CreateDataPorts(self, dict):
    if dict["dataPorts"] == None:
      return
    for dport in dict["dataPorts"]:
      if self.check_data_type(dport["type"]) == "sequence":
        dport["data_type_args"] = "RTC.Time(0,0),[]"
      else:
        dport["data_type_args"] = "RTC.Time(0,0),0"
    return
	
  def CreateConfiguration(self, dict):
    config = dict["configurationSet"]["configuration"]
    if config:
      for i, conf in enumerate(config):
        split_data = conf["defaultValue"].split(",")
        if len(split_data) > 1:
          _data = []
          _type = self.get_type(conf["type"])
          for d in split_data:
            _data.append(_type(d))
          conf["defaultData"] = str([_data])
        else:
          _type = self.get_type(conf["type"])
          conf["defaultData"] = \
              str([_type(conf["defaultValue"])])
    return

  def check_data_type(self, _type):
    if str(_type) in ["TimedShortSeq", "TimedLongSeq",
                      "TimedUShortSeq", "TimedULongSeq",
                      "TimedFloatSeq", "TimedDoubleSeq",
                      "TimedCharSeq","TimedBooleanSeq",
                      "TimedOctetSeq", "TimedStringSeq"]:
      return "sequence"
    else:
      return None
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

    return str
		
  def print_impl(self):
    for sidl in self.data["service_idl"]:
      if not os.access(sidl["idl_fname"], os.F_OK):
        sys.stderr.write("Error: IDL file \"" \
                           + sidl["idl_fname"] \
                           + "\" not found.\n")
        sys.exit(1)
          
      try:
        cmd = "omniidl -bpython -Wbexample " + \
            sidl["idl_fname"]
        os.system(cmd)
      except:
        sys.stderr.write("Generate error: " \
                           + sidl["impl_py"] + "\n")
        
      print "  File \"" \
          + sidl["impl_py"] \
          + "\" was generated."

    for cons in self.data["consumer_idl"]:
      dup = False
      for svc in self.data["service_idl"]:
        if cons["idl_fname"] == svc["idl_fname"]:
          dup = True

      if not dup:
        if not os.access(cons["idl_fname"], os.F_OK):
          sys.stderr.write("Error: IDL file \"" \
                             + cons["idl_fname"] \
                             + "\" not found.\n")
          sys.exit(1)

        try:
          cmd = "omniidl -bpython " + \
              cons["idl_fname"]
          os.system(cmd)
        except:
          sys.stderr.write("Generate error: omniidl -bpython " + cons["idl_fname"])
    return

  def print_pysrc(self):
    """
    Generate component class script
    """
    self.gen(self.data["fname_py"], py_source, self.data, self.tags)
    return


  def print_all(self):
    self.print_impl()
    self.print_pysrc()
    return
