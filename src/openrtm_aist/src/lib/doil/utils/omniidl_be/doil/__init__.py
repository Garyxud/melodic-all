#!/usr/bin/env python
# -*- python -*-
#
#  @file omniidl_be/doil/__init__.py
#  @brief doil backend for omniidl
#  @date $Date$
#  @author Noriaki Ando <n-ando@aist.go.jp>
# 
#  Copyright (C) 2008
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 
#  $Id$
# 

#   Entrypoint to the doil backend

# modules from omniidl
from omniidl import idlast, idlvisitor, idlutil

# module from omniidl_be.cxx
from omniidl_be.cxx import support, ast, id

# module from omniidl_be.doil
from omniidl_be.doil import util, config

import re, sys, os.path

cpp_args = ["-D__OMNIIDL_CXX__"]
usage_string = """\
  -Wbm=<mapfile>  Specify type mapping rule file (mandatory)
  -Wbi=<include>  Specify include files
  -Wbimplicit     Process included IDLs implicitly
  -Wbinterface    Generate doil C++ interface header
  -Wbcorbaservant Generate doil servant class for CORBA
  -Wbcorbaadapter Generate doil adapter class for CORBA
  -Wbcorbaproxy   Generate doil proxy class for CORBA
  -Wbiceslice     Generate Ice slice file from CORBA IDL
  -Wbiceservant   Generate doil servant class for Ice
  -Wbiceadapter   Generate doil adapter class for Ice
  -Wbiceproxy     Generate doil proxy class for Ice

[ options for servant class ]
  -Wbss=<suffix>  Specify suffix for generated servant files [default Servant]
  -Wbsp=<prefix>  Specify prefix for generated servant files [default None]
  -Wbsns=<ns>     Specify namespsace of generated servant class [default None]
  -Wbsdir=<ns>    Specify directory of generated servant header [default None]

[ options for adapter class ]
  -Wbas=<suffix>  Specify suffix for generated adapter class [default Adapter]
  -Wbap=<prefix>  Specify prefix for generated adapter class [default None]
  -Wbans=<ns>     Specify namespsace of generated adapter class [default None]
  -Wbadir=<ns>    Specify directory of generated adapter header [default None]

[ options for proxy class ]
  -Wbps=<suffix>  Specify suffix for generated proxy class [default Proxy]
  -Wbpp=<prefix>  Specify prefix for generated proxy class [default None]
  -Wbpns=<ns>     Specify namespsace of generated proxy class [default None]
  -Wbpdir=<ns>    Specify directory of generated proxy header [default None]

[ options for interface class ]
  -Wbis=<suffix>  Specify suffix for local interface class [default None]
  -Wbip=<prefix>  Specify prefix for local interface class [default I]
  -Wbins=<ns>     Specify namespsace of local interface class [default None]
  -Wbidir=<ns>    Specify directory of local interface class [default None]

"""

# Encountering an unknown AST node will cause an AttributeError exception
# to be thrown in one of the visitors. Store a list of those not-supported
# so we can produce a friendly error message later.
AST_unsupported_nodes = [ "Native", "StateMember", "Factory", "ValueForward",
                          "ValueBox", "ValueAbs", "Value" ]

def process_args(args):
    for arg in args:
        if arg == "d":
            config.state['Debug']                  = True
        elif arg[:2] == "m=":
            config.state['MappingFile']            = arg[2:]
        elif arg[:2] == "i=":
            config.state['IncludeHeaders'].append(arg[2:])
        # generator options
        elif arg == "implicit":
            config.state['ImplicitInclude']        = True
        elif arg == "interface":
            config.state['Interface']              = True
        elif arg == "corbaservant":
            config.state['CORBAServant']           = True
        elif arg == "corbaadapter":
            config.state['CORBAAdapter']           = True
        elif arg == "corbaproxy":
            config.state['CORBAProxy']             = True
        elif arg == "iceslice":
            config.state['IceSlice']               = True
        elif arg == "iceservant":
            config.state['IceServant']             = True
        elif arg == "iceadapter":
            config.state['IceAdapter']             = True
        elif arg == "iceproxy":
            config.state['IceProxy']               = True
        # for servant
        elif arg[:3] == "ss=":
            config.state['ServantSuffix']          = arg[3:]
        elif arg[:3] == "sp=":
            config.state['ServantPrefix']          = arg[3:]
        elif arg[:4] == "sns=":
            config.state['ServantNs']              = arg[4:].split('::')
        elif arg[:5] == "sdir=":
            config.state['ServantDir']             = arg[5:]
        # for adapter
        elif arg[:3] == "as=":
            config.state['AdapterSuffix']          = arg[3:]
        elif arg[:3] == "ap=":
            config.state['AdapterPrefix']          = arg[3:]
        elif arg[:4] == "ans=":
            config.state['AdapterNs']              = arg[4:].split('::')
        elif arg[:5] == "adir=":
            config.state['AdapterDir']             = arg[5:]
        # for proxy 
        elif arg[:3] == "ps=":
            config.state['ProxySuffix']            = arg[3:]
        elif arg[:3] == "pp=":
            config.state['ProxyPrefix']            = arg[3:]
        elif arg[:4] == "pns=":
            config.state['ProxyNs']                = arg[4:].split('::')
        elif arg[:5] == "pdir=":
            config.state['ProxyDir']               = arg[5:]
        # for interface
        elif arg[:3] == "is=":
            config.state['IfaceSuffix']            = arg[3:]
        elif arg[:3] == "ip=":
            config.state['IfacePrefix']            = arg[3:]
        elif arg[:4] == "ins=":
            config.state['IfaceNs']                = arg[4:].split('::')
        elif arg[:5] == "idir=":
            config.state['IfaceDir']               = arg[5:]
        else:
            util.fatalError("Argument \"" + str(arg) + "\" is unknown")

run_before = 0


def run(tree, args):
    """Entrypoint to the doil backend"""

    global run_before

    if run_before:
        util.fatalError("Sorry, the doil backend cannot process more "
                        "than one IDL file at a time.")
    run_before = 1

    dirname, filename = os.path.split(tree.file())
    basename,ext      = os.path.splitext(filename)
    config.state['Basename']  = basename
    config.state['Directory'] = dirname
    
    process_args(args)

    if config.state['MappingFile'] == '':
        config.state['TypeMapping'] = {}
    else:
        import yaml
        mapping_file = config.state['MappingFile']
        f = open(mapping_file, "r")
        mapping = yaml.load(f.read())
        f.close()
        config.state['TypeMapping'] = mapping


    try:
        # Check the input tree only contains stuff we understand
        support.checkIDL(tree)

        # initialise the handy ast module
        ast.__init__(tree)

        tree.accept(id.WalkTree())
        # Initialise the descriptor generating code
        # currently doil does not use descriptor
        #        descriptor.__init__(tree)
        from omniidl_be.doil import dictbuilder
        dict = dictbuilder.run(tree, config.state)

        if config.state['Interface']:
            from omniidl_be.doil import interface
            interface.generate_interface(dict)
            interface.generate_types(dict)
        if config.state['CORBAServant']:
            from omniidl_be.doil import corba
            corba.generate_servant(dict)
            corba.generate_types(dict)
        if config.state['CORBAAdapter']:
            from omniidl_be.doil import corba
            corba.generate_adapter(dict)
            corba.generate_types(dict)
        if config.state['CORBAProxy']:
            from omniidl_be.doil import corba
            corba.generate_proxy(dict)
            corba.generate_types(dict)
        if config.state['IceSlice']:
            from omniidl_be.doil import ice
            ice.generate_slice(dict)

        if config.state['IceAdapter']:
            from omniidl_be.doil import ice
            ice.generate_adapter(dict)

        if config.state['IceProxy']:
            from omniidl_be.doil import ice
            ice.generate_proxy(dict)

        if config.state['IceServant']:
            from omniidl_be.doil import ice
            ice.generate_servant(dict)

    except AttributeError, e:
        name = e[0]
        unsupported_visitors = map(lambda x:"visit" + x,
                                   AST_unsupported_nodes[:])
        if name in unsupported_visitors:
            util.unsupportedIDL()
            
        util.fatalError("An AttributeError exception was caught")
    except SystemExit, e:
        raise e
    except:
        util.fatalError("An internal exception was caught")
