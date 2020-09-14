#!/usr/bin/env python
# -*- python -*-
#
#  @file util.py
#  @brief doil backend utility module
#  @date $Date$
#  @author Noriaki Ando <n-ando@aist.go.jp>
# 
# This module is almost same as omniidl_be/cxx/util.py
#
#  $Id$
# 

"""General utility functions used by the doil backend"""

from omniidl import idlutil, idltype
from omniidl_be.doil import config
import sys, re, string

try:
    import traceback
    have_traceback = 1
except:
    have_traceback = 0


## Fatal error handling function ##################################
##
def fatalError(explanation):
    if config.state['Debug']:
        # don't exit the program in debug mode...
        print "omniidl: fatalError occurred, in debug mode."
        for line in string.split(explanation, "\n"):
            print ">> " + line
        print "Configuration state:"
        print "-------------------------"
        config.state.dump()

        if have_traceback:
            print "Stack:"
            print "-------------------------"
            traceback.print_stack()
            print "Exception:"
            print "-------------------------"
            traceback.print_exc()
        sys.exit(1)
    
    lines = string.split(explanation, "\n")
    lines = [ "Fatal error in doil backend", "" ] + lines

    for line in lines:
        sys.stderr.write("omniidl: " + line + "\n")

    sys.stderr.write("""\

For more information (mailing list archives, bug reports etc.) please visit
the webpage:

  http://www.openrtm.org/

""")
    sys.exit(1)

# Called whenever an unsupported IDL construct is found in the input
# (necessary because the front end supports all the new CORBA 2.3
# constructs whereas the ORB and correspondingly this backend does not)
def unsupportedIDL():
    e = """\
Unsupported IDL construct encountered in input.

omniORB does not currently support:
  IDL type valuetype
"""
    fatalError(e)
    

## Set manipulation functions ######################################
##
def union(a, b):
    result = a[:]
    for x in b:
        if x not in result:
            result.append(x)
    return result

def minus(a, b):
    result = []
    for x in a:
        if x not in b:
            result.append(x)
    return result

def intersect(a, b):
    result = []
    for x in a:
        if x in b:
            result.append(x)
    return result

def setify(set):
    new_set = []
    for x in set:
        if x not in new_set:
            new_set.append(x)

    return new_set

## List manipulation functions #####################################
##
def zip(a, b):
    if a == [] or b == []: return []
    return [(a[0], b[0])] + zip(a[1:], b[1:])

def fold(list, base, fn):
    if len(list) == 1:
        return fn(list[0], base)
    first = fn(list[0], list[1])
    rest = [first] + list[2:]
    return fold(rest, base, fn)

## Assorted other functions ########################################
##
class Stack:
    def __init__(self):
        self.__list = []
    def push(self, thing):
        self.__list.append(thing)
    def pop(self):
        if self.__list == []: raise "Stack Empty"
        thing = self.__list[-1]
        self.__list = self.__list[0:-1]
        return thing
