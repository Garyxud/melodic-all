#!/usr/bin/env python
# -*- coding: euc-jp -*-
#
## CreateDeleteRTCTest.py
##
## メモリーリークチェック
## Manager.idlで定義されているオペレーション
## ステートフルなオペレーション
#
# $Id$
#

from rtc_handle import *
from BasicDataType_idl import *
from CorbaNaming import *
import time
import commands
import socket
import RTM

env = RtmEnv(sys.argv, ["localhost:9898"])

mgr_name=socket.gethostname()+".host_cxt/manager.mgr"
naming = CorbaNaming(env.orb,"localhost:9898")
manager = naming.resolve(mgr_name)._narrow(RTM.Manager)

time.sleep(2)

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
test_case = "CreateDeleteRTCTest"
fout = open(test_case + ".log", 'w')

fodat = "=== " + test_case + " start ==="
print_file_and_cons(fodat)

loop_cnt = 1000
## -----------------------------------------------------------------------------
fodat = "load_module() and unload_module()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::unload_module(in string pathname);
    try:
        #retcode = manager.unload_module(".//ConsoleIn.so")
        retcode = manager.unload_module(".//ConsoleOut.so")
    except:
        pass
    #print "unload_module() ret=",retcode

    ## Manager::load_module(in string pathname, in string initfunc); 
    #retcode = manager.load_module("ConsoleIn.so", "ConsoleInInit")
    retcode = manager.load_module("ConsoleOut.so", "ConsoleOutInit")
    #print "load_module()   ret=",retcode

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
fodat = "create_component() and delete_component()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::delete_component(in string instance_name);
    ret0 = manager.delete_component("ConsoleOut0")
    #ret0 = manager.delete_component("ConsoleIn0")
    #print "  delete_comp() ret=",ret0

    ## Manager::create_component(in string module_name); 
    obj0 = manager.create_component("ConsoleOut")
    #obj0 = manager.create_component("ConsoleIn")
    #print "  create_comp() ret_Ref=",obj0

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

manager.delete_component("ConsoleOut0")
#manager.delete_component("ConsoleIn0")
#retcode = manager.unload_module(".//ConsoleOut.so")
