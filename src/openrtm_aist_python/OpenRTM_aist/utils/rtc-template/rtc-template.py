#!/usr/bin/env python
# -*- python -*-
#
#  @file rtc-template
#  @brief rtc-template RTComponent source code generator tool
#  @date $Date: 2007/07/23 08:06:27 $
#  @author Noriaki Ando <n-ando@aist.go.jp>
# 
#  Copyright (C) 2004-2007
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 
#  $Id: rtc-template,v 1.8.2.2 2007/07/23 08:06:27 n-ando Exp $
#

#
#  $Log: rtc-template,v $
#  Revision 1.8.2.2  2007/07/23 08:06:27  n-ando
#  Modified for win32 porting about rtm-config.
#
#  Revision 1.8.2.1  2007/07/20 17:29:03  n-ando
#  A fix for win32 porting.
#
#  Revision 1.8  2007/04/27 00:56:35  n-ando
#  Example shown in help message was modified for new version.
#
#  Revision 1.7  2007/04/23 07:31:28  n-ando
#  Now "--conf" option can accept scope resolution operator of C++.
#
#  Revision 1.6  2007/04/23 01:41:21  n-ando
#  New option "--config" and configuration template code were added.
#
#  Revision 1.5  2007/01/11 07:42:25  n-ando
#  Modified for OMG RTC specificatin and OpenRTM-aist-0.4.0
#  - Some command option was chaged and removed.
#  - Now empty Struct class is used instead of *Profile classes for ezt dict.
#  - Some bugs were fixed.
#
#  Revision 1.4  2005/09/08 09:24:18  n-ando
#  - A bug fix for merge function.
#
#  Revision 1.3  2005/09/06 14:37:40  n-ando
#  rtc-template's command options and data structure for ezt (Easy Template)
#  are changed for RTComponent's service features.
#  Now rtc-template can generate services' skeletons, stubs and
#  implementation files.
#  The implementation code generation uses omniidl's IDL parser.
#
#  Revision 1.2  2005/08/26 11:32:26  n-ando
#  "rtc-template" was completely rewritten to use ezt (Easy Template) module.
#  "ezt" module is originally included in "Subversion".
#
#  Now template code generator modules, which are named xxx_gen.py, are
#  automatically imported from rtc-template, and command options and help
#  menu are automatically generated.
#
#  New template code generator has to inherit base_gen class in
#  "base_gen.py" module to utilize this framework.
#
#  Revision 1.1.1.1  2005/05/12 09:06:18  n-ando
#  Public release.
#
#

import getopt, sys
import re
import os

platform = sys.platform

class Struct:
  def __init__(self):
    return

conf_path = ['']

if platform == "win32":
  python_path = os.environ['PYTHONPATH'].split(";")
  pyhelper_path = python_path[0] + "\\OpenRTM_aist\\utils\\rtc-template"
else:
  conf_path = os.popen("which rtm-config", "r").read().split("\n")
  if conf_path[0] != '':
    libdir_path = os.popen("rtm-config --libdir", "r").read().split("\n")
    pyhelper_path = libdir_path[0] + "/py_helper"
  else:
    python_path = os.environ['PYTHONPATH'].split(":")
    pyhelper_path = python_path[0] + "/OpenRTM_aist/utils/rtc-template"
sys.path.append(pyhelper_path)

# Option format
opt_args_fmt = ["help",
    "module-name=",
    "module-type=",
    "module-desc=",
    "module-version=",
    "module-vendor=",
    "module-category=",
    "module-comp-type=",
    "module-act-type=",
    "module-max-inst=",
    "module-lang=",
                "config=",
    "inport=",
    "outport=",
    "service=",
    "service-idl=",
    "consumer=",
    "consumer-idl=",
    "idl-include=",
    "backend="]


def usage_short():
  """
  Help message
  """
  print """
Usage: rtc-template [OPTIONS]

Options:

    [-h]                                  Print short help.
    [--help]                              Print details help.
    [--backend[=backend] or -b]           Specify template code generator.
    [--module-name[=name]]                Your module name.
    [--module-desc[=description]]         Module description.
    [--module-version[=version]]          Module version.
    [--module-vendor[=vendor]]            Module vendor.
    [--module-category[=category]]        Module category.
    [--module-comp-type[=component_type]] Component type.
    [--module-act-type[=activity_type]]   Component's activity type.
    [--module-max-inst[=max_instance]]    Number of maximum instance.
    [--module-lang[=language]]            Language.
    [--config[=ParamName:Type:Default]]   Configuration variable.
    [--inport[=PortName:Type]]            InPort's name and type.
    [--outport[=PortName:Type]]           OutPort's name and type
    [--service[=PortName:Name:Type]]      Service Provider Port
    [--service-idl[=IDL_file]]            IDL file name for service
    [--consumer[=PortName:Name:Type]]     Service Consumer Port
    [--consumer-idl[=IDL_file]]           IDL file name for consumer
    [--idl-include=[path]]                Search path for IDL compile

"""
def usage_long():
  """
  Help message
  """
  print """
    --output[=output_file]:
        Specify base name of output file. If 'XXX' is specified,
        C++ source codes XXX.cpp, XXX.h, XXXComp.cpp Makefile.XXX is generated.

    --module-name[=name]:
        Your component's base name. This string is used as module's
        name and component's base name. A generated new component
        class name is also names as this RTC_MODULE_NAME.
        Only alphabetical and numerical characters are acceptable.

    --module-desc[=description]:
        Short description. If space characters are included, string should be
        quoted.

    --module-version[=version]:
        Your module version. ex. 1.0.0

    --module-vendor[=vendor]:
        Vendor's name of this component.

    --module-category[=category]:
        This component module's category. ex. Manipulator MobileRobot, etc...

    --module-comp-type[=component_type]:
        Specify component type.
      'STATIC', 'UNIQUE', 'COMMUTATIVE' are acceptable.

    --module-act-type[=activity_type]:
        Specify component activity's type.
        'PERIODIC', 'SPORADIC', 'EVENT_DRIVEN' ace acceptable.

    --module-max-inst[=max_instance]:
        Specify maximum number of component instance.

    --config=[ParamName:Type:Default]:
        Specify configuration value. The 'ParamName' is used as the
        configuration value identifier. This character string is also used as
        variable name in the source code. The 'Type' is type of configuration
        value. The type that can be converted to character string is allowed.
        In C++ language, the type should have operators '<<' and '>>' that
        are defined as
        'istream& operator<<(Type)'
        and
        'ostream& operator>>(Type)'.

    --inport=[PortName:Type]:
        Specify InPort's name and type. 'PortName' is used as this InPort's
        name. This string is also used as variable name in soruce code.
        'Type' is InPort's variable type. The acceptable types are,
        Timed[ Short | Long | UShort | ULong | Float | Double | Char | Boolean
        | Octet | String ] and its sequence types.

    --outport=[PortName:Type]:
        Specify OutPort's name and type. 'PortName' is used as this OutPort's
        name. This string is also used as variable name in soruce code.
        'Type' is OutPort's variable type. The acceptable types are,
        Timed[ Short | Long | UShort | ULong | Float | Double | Char | Boolean
        | Octet | String ] and its sequence types.
    
    --service=[PortName:Name:Type]:
        Specify service name, type and port name.
        PortName: The name of Port to which the interface belongs.
              This name is used as CorbaPort's name.
        Name: The name of the service interface. This name is used as 
              the name of the interface, instance name and variable name.
        Type: The type of the serivce interface.
              This name is used as type name of the service.

    --service-idl=[IDL filename]:
        Specify IDL file of service interface.
        For simplicity, please define one interface in one IDL file, although
        this IDL file can include two or more interface definition,
    
    --consumer=[PortName:Name:Type]:
        Specify consumer name, type and port name.
        PortName: The name of Port to which the consumer belongs.
              This name is used as CorbaPort's name.
        Name: The name of the consumer. This name is used as 
              the name of the consumer, instance name and variable name.
        Type: The serivce interface type that is required by the consumer.
              This name is used as type name of the consumer.

    --consumer-idl=[IDL filename]:
        Specify IDL file of service consumer.
        For simplicity, please define one interface in one IDL file, although
        this IDL file can include two or more interface definition,
  

Example:
    rtc-template -bcxx \\
    --module-name=Sample --module-desc='Sample component' \\
    --module-version=0.1 --module-vendor=AIST --module-category=Generic \\
    --module-comp-type=DataFlowComponent --module-act-type=SPORADIC \\
    --module-max-inst=10  \\
    --config=int_param0:int:0 --config=int_param1:int:1 \\
    --config=double_param0:double:3.14 --config=double_param1:double:9.99 \\
    --config="str_param0:std::string:hoge" \\
    --config="str_param1:std::string:foo" \\
    --inport=Ref:TimedFloat --inport=Sens:TimedFloat \\
    --outport=Ctrl:TimedDouble --outport=Monitor:TimedShort \\
    --service=MySvcPort:myservice0:MyService \\
    --consumer=YourSvcPort:yourservice0:YourService \\
    --service-idl=MyService.idl --consumer-idl=YourService.idl

"""
  return

def usage():
  usage_short()
  usage_long()
  return

class ModuleProfile:
  """
  ModuleProfile class

  This class create RTM module profile for ezt.
  """
  
  def __init__(self, name="", desc="", type="", version="", vendor="",
         category="", comp_type="", act_type="",
         max_inst="", lang=""):

    self.name = name
    self.desc = desc
    self.type = type
    self.version = version
    self.vendor = vendor
    self.category = category
    self.comp_type = comp_type
    self.act_type = act_type
    self.max_inst = max_inst
    self.lang = lang
    return
  

  def setValue(self, member, value):
    member = member.replace("-", "_")
    if hasattr(self, member):
      setattr(self, member, value)
    else:
      print "Invalid option: --module-" + member + " " + value
    return
  
  def setName(self, name):
    self.name = name
    return
  
  def setDesc(self, desc):
    self.desc = desc
    return
  
  def setVersion(self, version):
    self.version = version
    return
  
  def setVendor(self, vendor):
    self.vendor = vendor
    return
  
  def setCategory(self, vategory):
    self.category = category
    return

  def setCompType(self, comp_type):
    self.comp_type = comp_type
    return

  def setActType(self, act):
    self.act_type = act_type
    return

  def setMaxInst(self, max_inst):
    self.max_inst = max_inst
    return

  def printProfile(self):
    print "----- Module Profile -----"
    print "Name           ", self.name
    print "Description    ", self.desc
    print "Version        ", self.version
    print "Vendor         ", self.vendor
    print "Category       ", self.category
    print "Component Type ", self.comp_type
    print "Activity Type  ", self.act_type
    print "Max Instancese ", self.max_inst
    print "Language       ", self.lang
    return
    
    

def MakeModuleProfile(opts):
  """
  MakeModuleProfile

  Create ModuleProfile list from command options
  """
  prof = ModuleProfile()
  for opt, arg in opts:
    if opt.find("--module-") == 0:
      var = opt.replace("--module-","")
      prof.setValue(var, arg)
  return prof


def MakeConfig(opts):
  """
  MakeConfigurationParameters

  Create Configuration list from command options
  """
  prof_list = []
  cnt = 0
  for opt, arg in opts:
    if opt == ("--config"):
      try:
        # For C++ scope resolution operator 
        arg = re.sub("::", "@@", arg)
        name, type, default = arg.split(":")
        name    = re.sub("@@", "::", name)
        type    = re.sub("@@", "::", type)
        default = re.sub("@@", "::", default)
      except:
        sys.stderr("Invalid option: " \
             + opt \
             + "=" \
             + arg)
      prof = Struct()
      prof.name = name
      prof.l_name = name.lower()
      prof.u_name = name.upper()
      prof.type = type
      prof.default  = default
      prof_list.append(prof)
      cnt += 1
  return prof_list


def MakeDataPort(opts, port_type):
  """
  MakePortProfile

  Create PortProfile list from command options
  """
  prof_list = []
  cnt = 0
  for opt, arg in opts:
    if opt == ("--" + port_type):
      try:
        name, type = arg.split(":")
      except:
        sys.stderr("Invalid option: " \
             + opt \
             + "=" \
             + arg)
      prof = Struct()
      prof.name = name
      prof.type = type
      prof.num  = cnt
      prof_list.append(prof)
      cnt += 1
  return prof_list


def MakePortInterface(opts, port_type):
  """
  MakePortInterface

  Create Port interface profile list from command options
  """
  prof_list = []
  cnt = 0
  for opt, arg in opts:
    if opt == "--" + port_type:
      try:
        port, name, type = arg.split(":")
      except:
        sys.stderr.write("Invalid option: " \
             + opt \
             + "=" \
             + arg)
      prof = Struct()
      prof.port = port
      prof.name = name
      prof.type = type
      prof.num  = cnt
      prof_list.append(prof)
      cnt += 1
  return prof_list

def MakeCorbaPort(opts):
  """
  MakeCorbaPort

  Create Corba Port profile list from command options
  """
  prof_list = []
  cnt = 0
  for opt, arg in opts:
    if opt == ("--" + "service") or opt == ("--" + "consumer"):
      try:
        port, name, type = arg.split(":")
      except:
        sys.stderr.write("Invalid option: " \
             + opt \
             + "=" \
             + arg)
      dup = False
      for p in prof_list:
        if p.name == port:
          dup = True
      if dup == False:
        prof = Struct()
        prof.name = port
        prof.num  = cnt
        prof_list.append(prof)
        cnt += 1
  return prof_list



def MakeServiceIDL(opts):
  """
  MakeServiceIDL

  Create ServiceIDL list from command options
  """
  idl_list = []

  for opt, arg in opts:
    if opt.find("--service-idl") == 0:
      svc_idl = Struct()
      svc_idl.idl_fname = arg
      svc_idl.idl_basename, dummy = arg.split(".")
      idl_list.append(svc_idl)
  return idl_list


def MakeConsumerIDL(opts):
  idl_list = []
  for opt, arg in opts:
    if opt == "--consumer-idl":
      svc_idl = Struct()
      svc_idl.idl_fname = arg
      svc_idl.idl_basename, dummy = arg.split(".")
      idl_list.append(svc_idl)
  return idl_list



def find_opt(opts, value, default):
  for opt, arg in opts:
    if opt.find(value) == 0:
      return arg

  return default


def find_opt_list(opts, value, default):
  list = []
  if len(default) > 0:
    list += default
  for opt, arg in opts:
    if opt == ("--" + value):
      list.append(arg)
  return list


class Backend:
  def __init__(self, mod_name, mod):
    self.mod = mod
    self.obj = getattr(mod, mod_name)
    self.mod_name = mod_name


class BackendLoader:
  def __init__(self):
    self.backends = {}
    self.opts = []
    self.available()
    return
    

  def available(self):
    path_list = [pyhelper_path, "."]
    for path in path_list:
      for f in os.listdir(path):
        if re.compile("_gen.py$").search(f):
          mod_name = f.replace(".py", "")
          opt_name = f.replace("_gen.py", "")
          mod = __import__(mod_name, globals(), locals(), [])
          try:
            mod.usage()
            be = Backend(mod_name, mod)
            self.backends[opt_name] = be
          except:
            print "Invalid backend: ", f
            pass

    return self.backends


  def check_args(self, args):
    for opt in args:
      if opt.find('-b') == 0:
        backend_name = opt.replace("-b", "")
        if self.backends.has_key(backend_name):
          self.opts.append(backend_name)
        else:
          print "No such backend: ", backend_name
          sys.exit(-1)
      elif opt.find('--backend=') == 0:
        backend_name = opt.replace("--backend=", "")
        if self.backends.has_key(backend_name):
          self.opts.append(backend_name)
        else:
          print "No such backend: ", backend_name
          sys.exit(-1)
    return self.opts


  def get_opt_fmts(self):
    fmts = []
    for be in self.opts:
      fmts += self.backends[be].mod.get_opt_fmt()
    return fmts


  def usage_available(self):
    print "The following backends are available."
    space = 10
    for key in self.backends:
      desc = self.backends[key].mod.description()     
      print "    -b" + key + ("." * (space - len(key))) + desc
    print """
Backend [xxx] specific help can be available by the following options.
    -bxxx --help|-h or --backend=xxx --help|-h
  """
    return


  def usage(self):
    for be in self.opts:
      print self.backends[be].mod.usage()     
      print ""
    return

  def usage_short(self):
    for be in self.opts:
      print self.backends[be].mod.usage_short()
      print ""
    return


  def generate_code(self, data, opts):
    for be in self.opts:
      self.backends[be].obj(data, opts).print_all()
    return
    

def fmtd_args(width, args):
  arg_fmt = [""]
  w = 0
  line = 0
  for a in args:
    w += len(a) + 1
    if w > width:
      w = len(a) + 1
      line += 1
      arg_fmt.append("")
    arg_fmt[line] += a + " "
  return arg_fmt



def main():
  global opt_args_fmt
  global conf_path

  backends = BackendLoader()
  backends.check_args(sys.argv[1:])
  opt_args_fmt += backends.get_opt_fmts()

  try:
    opts, args = getopt.getopt(sys.argv[1:], "b:ho:v", opt_args_fmt)
  except getopt.GetoptError:
    print "Error: Invalid option.", getopt.GetoptError
    usage_short()
    backends.usage_available()
    sys.exit(-1)

  if not opts:
    usage_short()
    backends.usage_available()
    sys.exit(-1)

  output = None
  verbose = False
  output_cxx = False
  output_python = False

  for o, a in opts:
    if o == "-v":
      verbose = True
    if o in ("-h"):
      usage_short()
      backends.usage_available()
      backends.usage_short()
      sys.exit(0)
    if o in ("--help"):
      usage()
      backends.usage_available()
      backends.usage()
      sys.exit(0)
    if o in ("-o", "--output"):
      output = a
      # ...

  prefix = [' ']
  if conf_path[0] != '':
    prefix = os.popen("rtm-config --prefix", "r").read().split("\n")
  idl_inc = []
  if prefix[0] != '':
    idl_inc.append(prefix[0] + "/include/rtm/idl")
    idl_inc.append(prefix[0] + "/include/rtm")
  idl_inc.append(".")

  # Create dictionary for ezt
  data = {
    'module':       MakeModuleProfile(opts),
                'config':       MakeConfig(opts),
    'inport':       MakeDataPort(opts, "inport"),
    'outport':      MakeDataPort(opts, "outport"),
    'service':      MakePortInterface(opts, "service"),
    'consumer':     MakePortInterface(opts, "consumer"),
                'corbaport':    MakeCorbaPort(opts),
    'service_idl':  MakeServiceIDL(opts),
    'consumer_idl': MakeConsumerIDL(opts),
    'idl_include':  find_opt_list(opts, "--idl-include", idl_inc),
    'fname':        output,
    'args':         sys.argv,
    'fmtd_args':    fmtd_args(70, sys.argv)
    }

  if data['fname'] == None:
    data['fname'] = data['module'].name

  backends.generate_code(data, opts)

  import README_src
  readme_src = README_src.README_src(data)
  readme_src.print_all()
  return
    

if __name__ == "__main__":
  main()
