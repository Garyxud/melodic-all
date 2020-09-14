#!/usr/bin/env python
# -*- coding: euc-jp -*-
#
## ManagerTest.py
##
## メモリーリークチェック
## Manager.idlで定義されているオペレーション
## ステートレスなオペレーション
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
test_case = "ManagerTest"
fout = open(test_case + ".log", 'w')

fodat = "=== " + test_case + " start ==="
print_file_and_cons(fodat)

loop_cnt = 1000
## -----------------------------------------------------------------------------
fodat = "get_loadable_modules()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::get_loadable_modules(); 
    manager.get_loadable_modules()

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
fodat = "get_loaded_modules()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::get_loaded_modules(); 
    manager.get_loaded_modules()

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
fodat = "get_factory_profiles()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::get_factory_profiles(); 
    manager.get_factory_profiles()

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
fodat = "get_components()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::get_components(); 
    manager.get_components()

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
fodat = "get_component_profiles()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::get_component_profiles(); 
    manager.get_component_profiles()

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
fodat = "get_profile()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::get_profile(); 
    manager.get_profile()

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
fodat = "get_configuration()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::get_configuration(); 
    manager.get_configuration()

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
fodat = "set_configuration()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::set_configuration(in string name, in string value); 
    manager.set_configuration("module.load_path", "./")

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
##fodat = "get_owner()"
##print_file_and_cons(fodat)
##
##for i in range(loop_cnt):
##    ## Manager::get_owner(); 
##    manager.get_owner()
##
##    if i == 0:
##        rss0 = mem_rss() ; j0 = 0 ; rssStart = rss0
##        fodat = "   %05d: %s KB start" % (1, rss0)
##        print_file_and_cons(fodat,1)
##    rss1 = mem_rss() ; j1 = i
##    if rss0 != rss1:
##        fodat = "   %05d: %s KB -> %d KB. count diff -> %d" % (i+1, rss1,int(rss1)-int(rss0),int(j1)-int(j0) )
##        print_file_and_cons(fodat,1)
##        rss0 = rss1 ; j0 = j1
##
##rssEnd = mem_rss()
##fodat = "   %05d: %s KB end" % (i+1, rssEnd)
##print_file_and_cons(fodat,1)
##leak_check(rssStart, rssEnd)
## -----------------------------------------------------------------------------
##fodat = "set_owner()"
##print_file_and_cons(fodat)
##
##for i in range(loop_cnt):
##    ## Manager::set_owner(in Manager mgr); 
##    manager.set_owner(manager)
##
##    if i == 0:
##        rss0 = mem_rss() ; j0 = 0 ; rssStart = rss0
##        fodat = "   %05d: %s KB start" % (1, rss0)
##        print_file_and_cons(fodat,1)
##    rss1 = mem_rss() ; j1 = i
##    if rss0 != rss1:
##        fodat = "   %05d: %s KB -> %d KB. count diff -> %d" % (i+1, rss1,int(rss1)-int(rss0),int(j1)-int(j0) )
##        print_file_and_cons(fodat,1)
##        rss0 = rss1 ; j0 = j1
##
##rssEnd = mem_rss()
##fodat = "   %05d: %s KB end" % (i+1, rssEnd)
##print_file_and_cons(fodat,1)
##leak_check(rssStart, rssEnd)
## -----------------------------------------------------------------------------
##fodat = "get_child()"
##print_file_and_cons(fodat)
##
##for i in range(loop_cnt):
##    ## Manager::get_child(); 
##    manager.get_child()
##
##    if i == 0:
##        rss0 = mem_rss() ; j0 = 0 ; rssStart = rss0
##        fodat = "   %05d: %s KB start" % (1, rss0)
##        print_file_and_cons(fodat,1)
##    rss1 = mem_rss() ; j1 = i
##    if rss0 != rss1:
##        fodat = "   %05d: %s KB -> %d KB. count diff -> %d" % (i+1, rss1,int(rss1)-int(rss0),int(j1)-int(j0) )
##        print_file_and_cons(fodat,1)
##        rss0 = rss1 ; j0 = j1
##
##rssEnd = mem_rss()
##fodat = "   %05d: %s KB end" % (i+1, rssEnd)
##print_file_and_cons(fodat,1)
##leak_check(rssStart, rssEnd)
## -----------------------------------------------------------------------------
##fodat = "set_child()"
##print_file_and_cons(fodat)
##
##for i in range(loop_cnt):
##    ## Manager::set_child(in Manager mgr); 
##    manager.set_child(manager)
##
##    if i == 0:
##        rss0 = mem_rss() ; j0 = 0 ; rssStart = rss0
##        fodat = "   %05d: %s KB start" % (1, rss0)
##        print_file_and_cons(fodat,1)
##    rss1 = mem_rss() ; j1 = i
##    if rss0 != rss1:
##        fodat = "   %05d: %s KB -> %d KB. count diff -> %d" % (i+1, rss1,int(rss1)-int(rss0),int(j1)-int(j0) )
##        print_file_and_cons(fodat,1)
##        rss0 = rss1 ; j0 = j1
##
##rssEnd = mem_rss()
##fodat = "   %05d: %s KB end" % (i+1, rssEnd)
##print_file_and_cons(fodat,1)
##leak_check(rssStart, rssEnd)
## -----------------------------------------------------------------------------
fodat = "fork()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::fork(); 
    manager.fork()

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
##fodat = "shutdown()"
##print_file_and_cons(fodat)
##
##for i in range(loop_cnt):
##    ## Manager::shutdown(); 
##    ###manager.shutdown()             # can't test, because manager is destroyed.
##
##    if i == 0:
##        rss0 = mem_rss() ; j0 = 0 ; rssStart = rss0
##        fodat = "   %05d: %s KB start" % (1, rss0)
##        print_file_and_cons(fodat,1)
##    rss1 = mem_rss() ; j1 = i
##    if rss0 != rss1:
##        fodat = "   %05d: %s KB -> %d KB. count diff -> %d" % (i+1, rss1,int(rss1)-int(rss0),int(j1)-int(j0) )
##        print_file_and_cons(fodat,1)
##        rss0 = rss1 ; j0 = j1
##
##rssEnd = mem_rss()
##fodat = "   %05d: %s KB end" % (i+1, rssEnd)
##print_file_and_cons(fodat,1)
##leak_check(rssStart, rssEnd)
## -----------------------------------------------------------------------------
fodat = "restart()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::restart(); 
    manager.restart()

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
fodat = "get_service()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Manager::get_service(in string name);
    manager.get_service("test")

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
