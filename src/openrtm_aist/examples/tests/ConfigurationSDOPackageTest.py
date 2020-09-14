#!/usr/bin/env python
# -*- coding: euc-jp -*-
#
## ConfigurationSDOPackageTest.py
##
## メモリーリークチェック
## SDOPackage.idlで定義されているオペレーション
## Configurationに関するオペレーション
#
# $Id$
#

from rtc_handle import *
from BasicDataType_idl import *
from omniORB import any
import time
import commands
import SDOPackage
import socket
import RTM

env = RtmEnv(sys.argv, ["localhost:9898"])

## Get Manager object reference
mgr_name = socket.gethostname()+".host_cxt/manager.mgr"
naming = CorbaNaming(env.orb, "localhost:9898")
manager = naming.resolve(mgr_name)._narrow(RTM.Manager)
## Create Composite RTC.
manager.create_component("PeriodicECSharedComposite?instance_name=a&exported_ports=ConsoleIn0.out,ConsoleOut0.in")

time.sleep(2)

listo = env.name_space["localhost:9898"].list_obj()
env.name_space['localhost:9898'].rtc_handles.keys()

ns = env.name_space['localhost:9898']

consin0 = ns.rtc_handles["ConsoleIn0.rtc"]
consout0 = ns.rtc_handles["ConsoleOut0.rtc"]

a0 = ns.rtc_handles["a.rtc"]
org = a0.rtc_ref.get_owned_organizations()[0]
config = a0.rtc_ref.get_configuration()
configset = config.get_configuration_sets()
id = configset[0].id

config.add_configuration_set(configset[0]) 

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
test_case = "ConfigurationSDOPackageTest"
fout = open(test_case + ".log", 'w')

fodat = "=== " + test_case + " start ==="
print_file_and_cons(fodat)

loop_cnt = 1000
## -----------------------------------------------------------------------------
fodat = "add_configuration_set() and remove_configuration_set()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Configuration::add_configuration_set (in ConfigurationSet configuration_set); 
    config.add_configuration_set(configset[0]) 

    ## Configuration::remove_configuration_set (in UniqueIdentifier config_id); 
    config.remove_configuration_set(id) 

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
fodat = "activate_configuration_set() and get_active_configuration_set()"

print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Configuration::activate_configuration_set (in UniqueIdentifier config_id); 
    config.activate_configuration_set(id) 

    ## Configuration::get_active_configuration_set ();
    config.get_active_configuration_set()

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
fodat = "get_configuration_set()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Configuration::get_configuration_set (in UniqueIdentifier config_id); 
    config.get_configuration_set(id) 

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
fodat = "set_configuration_set_values()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    ## Configuration::set_configuration_set_values (in ConfigurationSet configuration_set); 
    config.set_configuration_set_values (configset[0]) 

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

config.remove_configuration_set(id) 
