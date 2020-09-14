#!/usr/bin/env python
# -*- coding: euc-jp -*-
#
## ConnectRTCTest.py
##
## メモリーリークチェック
## RTC.idlで定義されているオペレーション
## ポートに関するオペレーション
#
# $Id$
#

from rtc_handle import *
from BasicDataType_idl import *
import time
import commands
import SDOPackage

env = RtmEnv(sys.argv, ["localhost:9898"])
list0 = env.name_space["localhost:9898"].list_obj()
env.name_space['localhost:9898'].rtc_handles.keys()

time.sleep(2)

ns = env.name_space['localhost:9898']

compo0 = ns.rtc_handles["ConsoleIn0.rtc"]
compo1 = ns.rtc_handles["ConsoleOut0.rtc"]

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
test_case = "ConnectRTCTest"
fout = open(test_case + ".log", 'w')

fodat = "=== " + test_case + " start ==="
print_file_and_cons(fodat)

loop_cnt = 1000
## -----------------------------------------------------------------------------
consin_ports = compo0.rtc_ref.get_ports()
#print "List consin_ports=%d" % len(consin_ports)
consout_ports = compo1.rtc_ref.get_ports()
#print "List consout_ports=%d" % len(consout_ports)

##--------------------------------------------------------------------
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
#conprof = RTC.ConnectorProfile("connector0", "123", [consin_ports[0],consout_ports[0]], [SDOPackage.NameValue("dataport.interface_type",any.to_any("corba_cdr")),SDOPackage.NameValue("dataport.dataflow_type",any.to_any("push")),SDOPackage.NameValue("dataport.subscription_type",any.to_any("periodic")),SDOPackage.NameValue("dataport.publisher.push_rate",any.to_any(2000))])

# Connector Porfile: corba_cdr, push, periodic <<<  Out -> In
#conprof = RTC.ConnectorProfile("connector0", "123", [consout_ports[0],consin_ports[0]], [SDOPackage.NameValue("dataport.interface_type",any.to_any("corba_cdr")),SDOPackage.NameValue("dataport.dataflow_type",any.to_any("push")),SDOPackage.NameValue("dataport.subscription_type",any.to_any("periodic")),SDOPackage.NameValue("dataport.publisher.push_rate",any.to_any(2000))])

##--------------------------------------------------------------------
#print "ConnectorProfile=\n",conprof

## -----------------------------------------------------------------------------
fodat = "get_ports()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    # RTObject::get_ports()
    #consin_ports[0].connect(conprof)
    compo1.rtc_ref.get_ports()
    #consin_ports[0].disconnect(conprof.connector_id)   # set used

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
fodat = "get_port_profile()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    # PortService::get_port_profile()
    #consin_ports[0].connect(conprof)
    consin_ports[0].get_port_profile()
    #consin_ports[0].disconnect(conprof.connector_id)   # set used

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
fodat = "notify_connect() and notify_disconnect()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    # PortService::notify_connect(inout ConnectorProfile connector_profile)
    ret0 = consin_ports[0].notify_connect(conprof)
    #print "ret0=",ret0
    # PortService::notify_disconnect(in UniqueIdentifier connector_id)
    ret1 = consin_ports[0].notify_disconnect(conprof.connector_id)    # set used
    #if ret1 != RTC.RTC_OK:
    #    print "ret1=",ret1

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
fodat = "get_connector_profiles()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    # PortService::get_connector_profiles()
    consin_ports[0].connect(conprof)
    consin_ports[0].get_connector_profiles()
    consin_ports[0].disconnect(conprof.connector_id)   # set used

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
fodat = "get_connector_profile()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    # PortService::get_connector_profile(in UniqueIdentifier connector_id)
    consin_ports[0].connect(conprof)
    consin_ports[0].get_connector_profile(conprof.connector_id)
    consin_ports[0].disconnect(conprof.connector_id)               # set used

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
fodat = "connect() and disconnect()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    # PortService::connect(inout ConnectorProfile connector_profile)
    ret0 = consin_ports[0].connect(conprof)
    #print "ret0=",ret0
    ret1 = consin_ports[0].disconnect(conprof.connector_id)    # set used
    #if ret1 != RTC.RTC_OK:
    #    print "ret1=",ret1

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
fodat = "connect() and disconnect_all()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    # PortService::connect(inout ConnectorProfile connector_profile)
    ret0 = consin_ports[0].connect(conprof)
    #print "ret0=",ret0
    # PortService::disconnect_all()
    consin_ports[0].disconnect_all()

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

fodat = "=== " + test_case + " end ==="
print_file_and_cons(fodat)
fout.close()
