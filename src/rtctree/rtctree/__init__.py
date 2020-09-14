# -*- Python -*-
# -*- coding: utf-8 -*-

'''rtctree

Copyright (C) 2009-2014
    Geoffrey Biggs
    RT-Synthesis Research Group
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.
Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt

Library for easy access to name servers running components, the components
themselves, and managers.

'''


# Add the IDL path to the Python path
import sys
import os
_openrtm_idl_path = os.path.join(os.path.dirname(__file__), 'rtmidl')
if _openrtm_idl_path not in sys.path:
    sys.path.insert(1, _openrtm_idl_path)
del _openrtm_idl_path
del os
del sys


RTCTREE_VERSION = '1.0.0'
NAMESERVERS_ENV_VAR = 'RTCTREE_NAMESERVERS'
ORB_ARGS_ENV_VAR = 'RTCTREE_ORB_ARGS'


# vim: tw=79

