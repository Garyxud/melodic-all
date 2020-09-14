#!/usr/bin/env python
# -*- python -*-
#
# @file omniidl_be/doil/config.py
# @brief configuration module for doil backend
# @date $Date$
# @author Norkai Ando <n-ando@aist.go.jp>
#
#  Copyright (C) 2008
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
#
# $Id$
#

import string
import config
from omniidl_be.cxx import util

class ConfigurationState:
    def __init__(self):
        self._config = {
            # Name of this program
            'Program Name':          'omniidl (doil adapter backend)',
            # Useful data from CVS
            'CVS ID':                '$Id$',
            # Relevant omniORB C++ library version
            'Library Version':       'omniORB_4_0',
            # Debug mode
            'Debug':                 True,
            #
            # generated classes for doil
            # - doil Interface
            # - CORBA Servant
            # - CORBA Adapter
            # - Ice slice
            # - Ice servant
            # - Ice adapter
            #
            'Interface':             False,
            'CORBAServant':          False,
            'CORBAAdapter':          False,
            'CORBAProxy':            False,
            'IceSlice':              False,
            'IceServant':            False,
            'IceAdapter':            False,    
            'IceProxy':              False,    
            #
            # Type mapping
            #
            'MappingFile':           '',
            'TypeMapping':           None,
            #
            # Additional includes
            #
            'IncludeHeaders':        [],
            #
            #
            #
            'ImplicitInclude':       False,
            #
            # Servant stuff
            #
            # Suffix of generated servant class
            'ServantSuffix':         'Servant',
            # Prefix of generated servant class
            'ServantPrefix':         '',
            # Namespace of servant class
            'ServantNs':             [],
            # servant class header directory
            'ServantDir':            '',
            #
            # Adapter stuff
            #
            # Suffix of generated servant class
            'AdapterSuffix':         'Adapter',
            # Prefix of generated servant class
            'AdapterPrefix':          '',
            # Namespace of servant class
            'AdapterNs':             [],
            # servant class header directory
            'AdapterDir':            '',
            #
            # Proxy stuff
            #
            # Suffix of generated proxy class
            'ProxySuffix':           'Proxy',
            # Prefix of generated proxy class
            'ProxyPrefix':           '',
            # Namespace of proxy class
            'ProxyNs':               [],
            # proxy class header directory
            'ProxyDir':              '',
            #
            # Interface stuff
            #
            # Suffix of generated servant class
            'IfaceSuffix':           '',
            # Prefix of generated servant class
            'IfacePrefix':           'I',
            # Namespace of delegated interface class
            'IfaceNs':               [],
            # interface class header directory
            'IfaceDir':              '',
            # Base name of file being processed
            'Basename':              None,
            # Directory name of file being processed
            'Directory':             None
            }

    def __getitem__(self, key):
        if self._config.has_key(key):
            return self._config[key]
        util.fatalError("Configuration key not found (" + key + ")")

    def __setitem__(self, key, value):
        if self._config.has_key(key):
            self._config[key] = value
            return
        util.fatalError("Configuration key not found (" + key + ")")

    def dump(self):
        # find the longest key string
        max = 0
        for key in self._config.keys():
            if len(key) > max: max = len(key)
        # display the table
        for key in self._config.keys():
            print string.ljust(key, max), ":  ", repr(self._config[key])

# Create state-holding singleton object
if not hasattr(config, "state"):
    config.state = ConfigurationState()
