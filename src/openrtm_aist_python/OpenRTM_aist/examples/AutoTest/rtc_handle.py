#/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
from omniORB import CORBA, URI
from omniORB import any

import OpenRTM_aist
import RTC


from CorbaNaming import *
import SDOPackage

# class RtmEnv :
#       rtm environment manager
#       orb, naming service, rtc proxy list
#
class RtmEnv : 

    def __init__(self, orb_args, nserver_names=["localhost"], 
                                     orb=None, naming=None):
        if not orb :
            orb = CORBA.ORB_init(orb_args)
        self.orb = orb
        self.name_space = {}
        if naming : # naming can specify only one naming service
            self.name_space['default']=NameSpace(orb, naming=naming)
        else :
            for ns in nserver_names :
              self.name_space[ns]=NameSpace(orb, server_name=ns)

    def __del__(self):
        self.orb.shutdown(wait_for_completion=CORBA.FALSE)
        self.orb.destroy()
#
# class NameSpace :
#       rtc_handles and object list in naming service
#
class NameSpace :
    def __init__(self, orb, server_name=None, naming=None):
        self.orb = orb
        self.name = server_name
        if naming :
          self.naming = naming
        else :
          self.naming = CorbaNaming(self.orb, server_name) 

        self.b_len = 10 # iteration cut off no.
        self.rtc_handles = {}
        self.obj_list = {}

    def get_object_by_name(self, name, cl=RTC.RTObject):
        ref = self.naming.resolveStr(name)
        if ref is None: return None     # return CORBA.nil ?
        if cl :
            return ref._narrow(cl)
        else :
            return ref

    def list_obj(self) :
        self.rtc_handes = {}
        self.obj_list = {}
        return self.list_obj1(self.naming._rootContext, "")

    def list_obj1(self, name_context, parent) :
        if not name_context :
            name_context = self.naming._rootContext 
        rslt = []
        b_list = name_context.list(self.b_len)
        for bd in b_list[0] :
            rslt = rslt + self.proc_bd(bd, name_context, parent)
        if b_list[1] :  # iterator : there exists remaining.
            t_list = b_list[1].next_n(self.b_len)
            while t_list[0] :
                for bd in t_list[1] :
                    rslt = rslt + self.proc_bd(bd, name_context, parent)
                t_list = b_list[1].next_n(self.b_len)
        return rslt

    def proc_bd(self, bd, name_context, parent) :
#        print '-------------------------------------------------------------------'
#        print 'bd= ', bd
#        print 'name_context= ', name_context
#        print 'parent= ', parent
        rslt = []
        pre = ""
        if parent :
            pre = parent + "/"
        nam = pre + URI.nameToString(bd.binding_name)
        if bd.binding_type == CosNaming.nobject :
            tmp = name_context.resolve(bd.binding_name)
            self.obj_list[nam]=tmp
            print 'objcet '+nam+' was listed.'
            try :
                tmp = tmp._narrow(RTC.RTObject)
            except :
                print nam+' is not RTC.'
                tmp = None
            try :
                if tmp :
                   rslt = [[nam, tmp]]
                   self.rtc_handles[nam]=RtcHandle(nam,self,tmp)
                   print 'handle for '+nam+' was created.'
                else :
                   pass
            except :
                print nam+' is not alive.'
                pass
        else :
            tmp = name_context.resolve(bd.binding_name)
            tmp = tmp._narrow(CosNaming.NamingContext)
            rslt = self.list_obj1(tmp, nam)
        return rslt

#
# data conversion
#
def nvlist2dict(nvlist) :
    rslt = {}
    for tmp in nvlist :
        rslt[tmp.name]=tmp.value.value()   # nv.value and any.value()
    return rslt
def dict2nvlist(dict) :
    rslt = []
    for tmp in dict.keys() :
        rslt.append(SDOPackage.NameValue(tmp, any.to_any(dict[tmp])))
    return rslt
#
#  connector, port, inport, outport, service
#                

class Connector :
    def __init__(self, plist, name = None, id="", prop_dict={}) :
       self.plist = plist
       self.port_reflist = [tmp.port_profile.port_ref for tmp in plist]
       if name :
           self.name = name
       else :
           self.name = string.join([tmp.name for tmp in plist],'_')
       self.prop_dict = prop_dict
       self.prop_nvlist = dict2nvlist(self.prop_dict)
       self.profile = RTC.ConnectorProfile(self.name, id, self.port_reflist, self.prop_nvlist)
       self.nego_prop()

    def nego_prop(self) :
       self.possible = True
       for kk in self.def_prop :
           if kk in self.prop_dict :
               if not self.prop_dict[kk] :
                   self.prop_dict[kk]=self.def_prop[kk]
           else :
                self.prop_dict[kk]=self.def_prop[kk]
           for pp in self.plist :  
               if not ((self.prop_dict[kk] in pp.prop[kk]) or 
                                 ('Any' in    pp.prop[kk])) :
                   self.prop_dict[kk] = ""
                   self.possible = False
       self.prop_nvlist = dict2nvlist(self.prop_dict)
       self.profile.properties = self.prop_nvlist
       return self.possible

    def connect(self) :
#
#   out and inout parameters are retuned as a tuple   
#
       ret, self.profile = self.port_reflist[0].connect(self.profile)
       self.prop_nvlist = self.profile.properties
       self.prop_dict = nvlist2dict(self.prop_nvlist)
       return ret

    def disconnect(self) :
       ret = self.port_reflist[0].disconnect(self.profile.connector_id)
       return ret

class IOConnector(Connector) :
    def __init__(self, plist, name = None, id="", prop_dict={}) :
#      self.def_prop = {'dataport.dataflow_type':'Push' ,
#                       'dataport.interface_type':'CORBA_Any' ,
#                       'dataport.subscription_type':'Flush'}
       self.def_prop = {'dataport.dataflow_type':'push' ,
                        'dataport.interface_type':'corba_cdr' ,
                        'dataport.subscription_type':'flush'}
       Connector.__init__(self, plist, name, id, prop_dict)

class ServiceConnector(Connector) :
    def __init__(self, plist, name = None, id="", prop_dict={}) :
       self.def_prop = {'port.port_type':'CorbaPort' }
       Connector.__init__(self, plist, name, id, prop_dict)


class Port :
    def __init__(self, profile,nv_dict=None) :
        self.name=profile.name    
        self.port_profile = profile
        if not nv_dict :
            nv_dict = nvlist2dict(profile.properties)
        self.prop = nv_dict
        self.con = None           # this must be set in each subclasses
    def get_info(self) :
        self.con.connect()
        tmp1 = self.get_connections()
        tmp2 = [pp.connector_id for pp in tmp1]
        if self.con.profile.connector_id in tmp2 :
            self.con.disconnect()

    def get_connections(self) :
        return self.port_profile.port_ref.get_connector_profiles()

class CorbaServer :
    def __init__(self, profile, port) :
        self.profile = profile
        self.port = port
        self.name = profile.instance_name
        self.type = profile.type_name
        self.ref = None
        ref_key = 'port.' + self.type + '.' + self.name
        self.ref=self.port.con.prop_dict[ref_key]
#
# if we import stubs before we create instances,
# we rarely need to narrow the object references.
# we need to specify global symbol table to evaluate class symbols.
#
    def narrow_ref(self, gls) :
        if self.type.find('::') == -1 :
            self.narrow_sym = eval('_GlobalIDL.' + self.type, gls)
        else :
            self.narrow_sym = eval(self.type.replace('::','.'), gls)
        self.ref = self.ref._narrow(self.narrow_sym)

class CorbaClient :
    def __init__(self, profile) :
        self.profile = profile
        self.name = profile.instance_name
        self.type = profile.type_name
#
# to connect to an outside corba client,
# we need an implementation of the corresponding corba server.
# but ....
#

class RtcService(Port) :
    def __init__(self, profile,nv_dict=None) :
        Port.__init__(self, profile, nv_dict)
        self.con = ServiceConnector([self])
        self.get_info()
        self.provided={}
        self.required={}
        tmp = self.port_profile.interfaces
        for itf in tmp :
            if itf.polarity == RTC.PROVIDED :
                self.provided[itf.instance_name] = CorbaServer(itf,self)
            elif itf.polarity == RTC.REQUIRED :
                self.required[itf.instance_name] = CorbaClient(itf)

class RtcInport(Port) :
    def __init__(self, profile, nv_dict=None) :
        Port.__init__(self, profile, nv_dict)
        self.con = IOConnector([self])
        self.get_info() 
#       self.ref = self.con.prop_dict['dataport.corba_any.inport_ref']
        self.ref = self.con.prop_dict['dataport.corba_cdr.inport_ref']
        self.data_class = eval('RTC.' + self.prop['dataport.data_type'])
        self.data_tc = eval('RTC._tc_' + self.prop['dataport.data_type'])
    def write(self,data) :
        self.ref.put(CORBA.Any(self.data_tc,
                         self.data_class(RTC.Time(0,0),data)))

class RtcOutport(Port) :
    def __init__(self, profile,nv_dict=None) :
        Port.__init__(self, profile, nv_dict)
        self.con = IOConnector([self])
        self.get_info()
        if 'dataport.corba_any.outport_ref' in self.con.prop_dict :
           self.ref = self.con.prop_dict['dataport.corba_cdr.outport_ref']
#          self.ref = self.con.prop_dict['dataport.corba_any.outport_ref']
        else :
           self.ref=None
        self.data_class = eval('RTC.' + self.prop['dataport.data_type'])
        self.data_tc = eval('RTC._tc_' + self.prop['dataport.data_type'])

    def read(self) :
        if self.ref :
           return self.ref.get().value()
        else :
           print "not supported"
           return None
#
# RtcHandle
#
class RtcHandle :
  def __init__(self, name, env, ref=None) :
    self.name = name
    self.env = env
    if ref :
        self.rtc_ref = ref
    else :
        self.rtc_ref = env.naming.resolve(name)._narrow(RTC.RTObject)
    self.conf_ref = None
    self.retrieve_info()

  def retrieve_info(self) :
    self.conf_set={}
    self.conf_set_data={}
    self.port_refs = []
    self.execution_contexts =[]
    if self.rtc_ref :
        self.conf_ref = self.rtc_ref.get_configuration()
        conf_set = self.conf_ref.get_configuration_sets()
        for cc in conf_set :
            self.conf_set[cc.id]=cc
            self.conf_set_data[cc.id]=nvlist2dict(cc.configuration_data)
        self.profile = self.rtc_ref.get_component_profile()
        self.prop = nvlist2dict(self.profile.properties)
        #self.execution_contexts = self.rtc_ref.get_contexts()
        self.execution_contexts = self.rtc_ref.get_owned_contexts()
        self.port_refs = self.rtc_ref.get_ports()  
    # this includes inports, outports and service ports
    self.ports = {}
    self.services = {}
    self.inports = {}
    self.outports = {}
    for pp in self.port_refs :
        tmp = pp.get_port_profile()
        tmp_prop = nvlist2dict(tmp.properties)
#       self.ports[tmp.name]=Port(tmp, tmp_prop)
        if tmp_prop['port.port_type']=='DataInPort' :
            self.inports[tmp.name]=RtcInport(tmp,tmp_prop)
#            self.inports[tmp.name]=Port(tmp, tmp_prop)
        elif tmp_prop['port.port_type']=='DataOutPort' :
            self.outports[tmp.name]=RtcOutport(tmp, tmp_prop)
#            self.outports[tmp.name]=Port(tmp, tmp_prop)
        elif tmp_prop['port.port_type']=='CorbaPort' :
            self.services[tmp.name]=RtcService(tmp, tmp_prop)
#            self.services[tmp.name]=Port(tmp, tmp_prop)

  def set_conf(self,conf_set_name,param_name,value) :
      conf_set=self.conf_set[conf_set_name]
      conf_set_data=self.conf_set_data[conf_set_name]
      conf_set_data[param_name]=value
      conf_set.configuration_data=dict2nvlist(conf_set_data)
      self.conf_ref.set_configuration_set_values(conf_set_name,conf_set)
  def set_conf_activate(self,conf_set_name,param_name,value) :
      self.set_conf(conf_set_name,param_name,value)
      self.conf_ref.activate_configuration_set(conf_set_name)
  def activate(self):
      return self.execution_contexts[0].activate_component(self.rtc_ref)
  def deactivate(self):
      return self.execution_contexts[0].deactivate_component(self.rtc_ref)
#
# pipe
#  a pipe is an port (interface & implementation)
#  whhich corresponds to an outside port interface.
#  you can subscribe and communicate to the outside port with the pipe.
#  you need an rtc implementation to use pipes.
#
class Pipe :
    pass

class PipeOut(Pipe):
    def __init__(self, name, data_buf, size=8) :
        self.name =  name
        self.data =  data_buf
        self.OpenRTM_aist.InPort(name,data_buf,OpenRTM_aist.RingBuffer(size))
