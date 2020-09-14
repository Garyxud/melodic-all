#!/usr/bin/env python
# -*- coding: euc-jp -*-
#
## DataPortTest.py
##
## メモリーリークチェック
## DataPort.idlで定義されているオペレーション
## データポートに関するオペレーション
#
# $Id$
#

from rtc_handle import *
from BasicDataType_idl import *
import time
import commands
import SDOPackage
from omniORB import *
from omniORB import any
from omniORB import CORBA
import OpenRTM

env = RtmEnv(sys.argv, ["localhost:9898"])
list0 = env.name_space["localhost:9898"].list_obj()
env.name_space['localhost:9898'].rtc_handles.keys()

time.sleep(2)

ns = env.name_space['localhost:9898']

compo0 = ns.rtc_handles["ConsoleIn0.rtc"]
compo1 = ns.rtc_handles["ConsoleOut0.rtc"]

#compo0 = ns.rtc_handles["SequenceInComponent0.rtc"]
#compo1 = ns.rtc_handles["SequenceOutComponent0.rtc"]

def mem_rss():
    (stat, output) = commands.getstatusoutput("ps alxww | grep \"[r]\"tcd")
    return output.split()[7]

## file and console out
def print_file_and_cons(out_data, out_flag=0):
    ## out_flag:1 is file out only
    if out_flag == 1:
      fout.write(out_data + '\n')
      fout.flush()
    ## out_flag:2 is console out only
    elif out_flag == 2:
      print out_data
    ## out_flag:0 is console and file out (default)
    else:
      print out_data
      fout.write(out_data + '\n')
      fout.flush()
    return

## memory leak check
def leak_check(rss_start, rss_end):
    if rss_start != rss_end:
        fodat = "  result: memory leak was found !!!"
    else:
        fodat = "  result: memory leak was not found."
    print_file_and_cons(fodat)
    return

## file out setting
test_case = "DataPortTest"
fout = open(test_case + ".log", 'w')

fodat = "=== " + test_case + " start ==="
print_file_and_cons(fodat)

consin_ports = compo0.rtc_ref.get_ports()
#print "List consin_ports=%d" % len(consin_ports)
consout_ports = compo1.rtc_ref.get_ports()
#print "List consout_ports=%d" % len(consout_ports)

loop_cnt = 1000
##--------------------------------------------------------------------
# dataflow_type: push

# Connector Porfile: corba_cdr, push, flush <<<  In -> Out
conprof = RTC.ConnectorProfile("connector0", "123", [consin_ports[0],consout_ports[0]], [SDOPackage.NameValue("dataport.interface_type",any.to_any("corba_cdr")),SDOPackage.NameValue("dataport.dataflow_type",any.to_any("push")),SDOPackage.NameValue("dataport.subscription_type",any.to_any("flush"))])

# Connector Porfile: corba_cdr, push, flush <<<  Out -> In
#conprof = RTC.ConnectorProfile("connector0", "123", [consout_ports[0],consin_ports[0]], [SDOPackage.NameValue("dataport.interface_type",any.to_any("corba_cdr")),SDOPackage.NameValue("dataport.dataflow_type",any.to_any("push")),SDOPackage.NameValue("dataport.subscription_type",any.to_any("flush"))])

##--------------------------------------------------------------------
# Connector Porfile: corba_cdr, push, new <<<  In -> Out
#conprof = RTC.ConnectorProfile("connector0", "123", [consin_ports[0],consout_ports[0]], [SDOPackage.NameValue("dataport.interface_type",any.to_any("corba_cdr")),SDOPackage.NameValue("dataport.dataflow_type",any.to_any("push")),SDOPackage.NameValue("dataport.subscription_type",any.to_any("new"))])

# Connector Porfile: corba_cdr, push, new <<<  Out -> In
#conprof = RTC.ConnectorProfile("connector0", "123", [consout_ports[0],consin_ports[0]], [SDOPackage.NameValue("dataport.interface_type",any.to_any("corba_cdr")),SDOPackage.NameValue("dataport.dataflow_type",any.to_any("push")),SDOPackage.NameValue("dataport.subscription_type",any.to_any("new"))])

##--------------------------------------------------------------------
# Connector Porfile: corba_cdr, push, periodic <<<  In -> Out
#conprof = RTC.ConnectorProfile("connector0", "123", [consin_ports[0],consout_ports[0]], [SDOPackage.NameValue("dataport.interface_type",any.to_any("corba_cdr")),SDOPackage.NameValue("dataport.dataflow_type",any.to_any("push")),SDOPackage.NameValue("dataport.subscription_type",any.to_any("periodic"))])

# Connector Porfile: corba_cdr, push, periodic <<<  Out -> In
#conprof = RTC.ConnectorProfile("connector0", "123", [consout_ports[0],consin_ports[0]], [SDOPackage.NameValue("dataport.interface_type",any.to_any("corba_cdr")),SDOPackage.NameValue("dataport.dataflow_type",any.to_any("push")),SDOPackage.NameValue("dataport.subscription_type",any.to_any("periodic"))])

##--------------------------------------------------------------------
#print "ConnectorProfile=\n",conprof

##--------------------------------------------------------------------
# dataflow_type: pull
# Connector Porfile: corba_cdr, pull, flush <<<  Out -> In
conprof2 = RTC.ConnectorProfile("connector0", "123", [consout_ports[0], consin_ports[0]], [SDOPackage.NameValue("dataport.interface_type",any.to_any("corba_cdr")),SDOPackage.NameValue("dataport.dataflow_type",any.to_any("pull")),SDOPackage.NameValue("dataport.subscription_type",any.to_any("flush"))])

#print "ConnectorProfile2=\n",conprof2

#ec1 = compo0.rtc_ref.get_owned_contexts()
#ec2 = compo1.rtc_ref.get_owned_contexts()
#ec1[0].activate_component(compo0.rtc_ref)
#ec2[0].activate_component(compo1.rtc_ref)

## -----------------------------------------------------------------------------
fodat = "put()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    # DataflowType:push接続
    # InportCdr::put(in CdrData data)
    ret0,conprof0 = consin_ports[0].connect(conprof)
    # print "   connect() ret=",ret0
    if ret0 != RTC.RTC_OK:
        fodat = "     connect() error ret=" + str(ret0)
        print_file_and_cons(fodat)
    # print "   conprof0.properties=",conprof0.properties
    # print "   prop[0]=",conprof0.properties[0].value		#corba_cdr
    # print "   prop[1]=",conprof0.properties[1].value		#push
    # print "   prop[2]=",conprof0.properties[2].value		#flush,new,periodic
    # print "   prop[3]=",conprof0.properties[3].value  	#little,big
    # print "   prop[4]=",conprof0.properties[4].value  	#IOR:
    # print "   prop[5]=",conprof0.properties[5].value  	#InportCdr
    ior = any.from_any(conprof0.properties[4].value, keep_structs=True)
    # print "   ior=",ior
    inportobj = env.orb.string_to_object(ior)
    inportcdr = inportobj._narrow(OpenRTM.InPortCdr)
    data = RTC.TimedLong(RTC.Time(0,0),12345)
    data0 = cdrMarshal(any.to_any(data).typecode(), data, 1)
    ret1 = inportcdr.put(data0)
    # print "   put() ret=" + str(ret1)
    #if ret1 != OpenRTM.PORT_OK:
    #  if ret1 == OpenRTM.BUFFER_FULL:		#9件目からBUFFER_FULLになる
    #    fodat = "     put() ret=BUFFER_FULL  count=%d" % (i+1)
    #    print_file_and_cons(fodat)
    #  else:
    #    fodat = "     put() ret=" + str(ret1) + "  count=%d" % (i+1)
    #    print_file_and_cons(fodat)
    time.sleep(0.1)
    ret2 = consin_ports[0].disconnect(conprof.connector_id)    # set used
    #if ret2 != RTC.RTC_OK:
    #    fodat = "     disconnect() error ret=" + str(ret2)
    #    print_file_and_cons(fodat)

    if i == 0:
        rss0 = mem_rss() ; j0 = 0 ; rssStart = rss0
        fodat = "   %05d: %s KB start" % (1, rss0)
        print_file_and_cons(fodat,1)
    rss1 = mem_rss() ; j1 = i
    if rss0 != rss1:
        fodat = "   %05d: %s KB -> %d KB. count diff -> %d" % (i+1, rss1,int(rss1)-int(rss0),int(j1)-int(j0) )
        print_file_and_cons(fodat,1)
        rss0 = rss1 ; j0 = j1

rssEnd = mem_rss()
fodat = "   %05d: %s KB end" % (i+1, rssEnd)
print_file_and_cons(fodat,1)
leak_check(rssStart, rssEnd)
## -----------------------------------------------------------------------------
fodat = "get()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    # DataflowType:pull接続
    # OutportCdr::get(out CdrData data)  
    ret10,conprof10 = consout_ports[0].connect(conprof2)
    # print "   connect() ret=",ret10
    if ret10 != RTC.RTC_OK:
        fodat = "     connect() error ret=" + str(ret10)
        print_file_and_cons(fodat)
    # print "   conprof10.properties=",conprof10.properties
    # print "   prop[0]=",conprof10.properties[0].value		#corba_cdr
    # print "   prop[1]=",conprof10.properties[1].value		#pull
    # print "   prop[2]=",conprof10.properties[2].value		#flush,new,periodic
    # print "   prop[3]=",conprof10.properties[3].value  		#little,big
    # print "   prop[4]=",conprof10.properties[4].value  		#IOR:
    # print "   prop[5]=",conprof10.properties[5].value  		#OutportCdr
    ior10 = any.from_any(conprof10.properties[4].value, keep_structs=True)
    # print "   ior10=",ior10

    outportobj = env.orb.string_to_object(ior10)
    outportcdr = outportobj._narrow(OpenRTM.OutPortCdr)
    # data set

    ret11,data1 = outportcdr.get()
    # print "   get() ret=" + str(ret11)		# BUFFER_EMPTY
    #if ret11 != OpenRTM.PORT_OK:
    #    fodat = "     get() ret=" + str(ret11) + "  count=%d" % (i+1)
    #    print_file_and_cons(fodat)
    #else:
    #    value12 = cdrUnmarshal(any.to_any(123).typecode(),data1[0],1)
    #    #print "   data1=",value12
    time.sleep(0.1)
    ret12 = consin_ports[0].disconnect(conprof2.connector_id)    # set used
    #if ret12 != RTC.RTC_OK:
    #    fodat = "     disconnect() error ret=" + str(ret12)
    #    print_file_and_cons(fodat)

    if i == 0:
        rss0 = mem_rss() ; j0 = 0 ; rssStart = rss0
        fodat = "   %05d: %s KB start" % (1, rss0)
        print_file_and_cons(fodat,1)
    rss1 = mem_rss() ; j1 = i
    if rss0 != rss1:
        fodat = "   %05d: %s KB -> %d KB. count diff -> %d" % (i+1, rss1,int(rss1)-int(rss0),int(j1)-int(j0) )
        print_file_and_cons(fodat,1)
        rss0 = rss1 ; j0 = j1

rssEnd = mem_rss()
fodat = "   %05d: %s KB end" % (i+1, rssEnd)
print_file_and_cons(fodat,1)
leak_check(rssStart, rssEnd)
## -----------------------------------------------------------------------------
#ec2[0].deactivate_component(compo1.rtc_ref)
#ec1[0].deactivate_component(compo0.rtc_ref)

fodat = "=== " + test_case + " end ==="
print_file_and_cons(fodat)
fout.close()
