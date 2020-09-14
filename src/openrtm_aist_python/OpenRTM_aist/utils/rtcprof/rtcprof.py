#!/usr/bin/env python
# -*- Python -*-

##
# @file rtcprof.py
# @brief RT-Component profile dump command
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2010
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$

import sys
import os

import OpenRTM_aist

def main():

  if len(sys.argv) != 2:
    print "usage: "
    print sys.argv[0], " *.py "
    return

  # file name with full path
  fullname  = sys.argv[1]
  # directory name
  dirname   = os.path.dirname(sys.argv[1])
  tmp_path = sys.path
  sys.path.append(dirname)

  # basename
  basename  = os.path.basename(sys.argv[1])
  # classname
  classname  = basename.split(".")[0].lower()

  opts =[]
  opts.append("dummy")
  opts.append("-o")
  opts.append("manager.modules.load_path: " + dirname)
  opts.append("-o")
  opts.append("logger.enable:NO")
  opts.append("-o")
  opts.append("manager.corba_servant:NO")

  # Manager initialization
  OpenRTM_aist.Manager.init(opts)
  mgr = OpenRTM_aist.Manager.instance()

  # loaded profile = old profiles - new profiles
  # for old
  oldp = mgr.getFactoryProfiles()

  # for new
  comp_spec_name = classname+"_spec"

  try:
    imp_file = __import__(basename.split(".")[0])
  except:
    sys.path = tmp_path
    return

  comp_spec = getattr(imp_file,comp_spec_name,None)
  if not comp_spec:
    sys.path = tmp_path
    return

  newp = OpenRTM_aist.Properties(defaults_str=comp_spec)

  profs = []
    
  exists = False
  for i in range(len(oldp)):
    if    oldp[i].getProperty("implementation_id") == newp.getProperty("implementation_id") and \
          oldp[i].getProperty("type_name") == newp.getProperty("type_name") and \
          oldp[i].getProperty("description") == newp.getProperty("description") and \
          oldp[i].getProperty("version") == newp.getProperty("version"):
      exists = True
  if not exists:
    profs.append(newp)

        
  # loaded component profile have to be one
  if len(profs) == 0:
    print "Load failed. file name: ", fname
    sys.path = tmp_path
    return OpenRTM_aist.Properties()

  if len(profs) > 1:
    print "One or more modules loaded."
    sys.path = tmp_path
    return OpenRTM_aist.Properties()

  keys = profs[0].propertyNames()
  for key in keys:
    print "%s:%s"%(key,profs[0].getProperty(key))

  sys.path = tmp_path
  return

if __name__ == "__main__":
  main()
