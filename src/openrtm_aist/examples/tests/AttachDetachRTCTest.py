#!/usr/bin/env python
# -*- coding: euc-jp -*-
#
## AttachDetachRTCTest.py
##
## メモリーリークチェック
## RTC.idlで定義されているオペレーション
## ECのアタッチ・デタッチに関するオペレーション
#
# $Id$
#

from rtc_handle import *
from BasicDataType_idl import *
import time
import commands

env = RtmEnv(sys.argv, ["localhost:9898"])
list0 = env.name_space["localhost:9898"].list_obj()
env.name_space['localhost:9898'].rtc_handles.keys()

ns = env.name_space['localhost:9898']

time.sleep(2)

compo1 = ns.rtc_handles["ConsoleIn0.rtc"]
compo0 = ns.rtc_handles["ConsoleOut0.rtc"]
seqin0 = ns.rtc_handles["SequenceInComponent0.rtc"]

ec = compo0.rtc_ref.get_owned_contexts()

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
test_case = "AttachDetachRTCTest"
fout = open(test_case + ".log", 'w')

fodat = "=== " + test_case + " start ==="
print_file_and_cons(fodat)

loop_cnt = 1000
## -----------------------------------------------------------------------------
fodat = "attach_context() and detach_context()"
print_file_and_cons(fodat)

for i in range(loop_cnt):
    # LightweightRTObject::attach_context(in ExecutionContext exec_context)
    ec_id = compo0.rtc_ref.attach_context(ec[0])        # set used OK.  single use NG.
    #print "attach_context() ret=",ec_id

    # LightweightRTObject::detach_context(in ExecutionContextHandle_t exec_handle)
    retcode = compo0.rtc_ref.detach_context(ec_id)
    #print "detach_context() ret=",retcode

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
