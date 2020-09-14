#!/usr/bin/env python
# -*- coding: shift_jis -*-
# -*- python -*-
# @file omniidl_be/doil/servant/__init__.py
# @brief servant code generator for doil backend
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

import os

from omniidl_be.doil.interface import template
import omniidl_be.doil.yat as yat
import string

decl_map = {
    'interface':         template.ifacefwd_decl,
    'interface_forward': template.ifacefwd_decl,
    'struct':            template.struct_decl,
    'struct_forward':    template.structfwd_decl,
    'union':             template.union_decl,
    'union_forward':     template.unionfwd_decl,
    'enum':              template.enum_decl,
    'exception':         template.exception_decl,
    'typedef':           template.typedef_decl
}

def generate_interface(dict):
    ifdict = {}
    for key in ['idl_fname', 'types_h', 'types_h_path', 'idl_includes']:
        ifdict[key] = dict[key]

    for d in dict['tree']:
        if d['corba']['decl_type'] == 'interface':
            ifdict.update(d)

            t = yat.Template(template.interface_h)
            text = t.generate(ifdict)

            fname = d['local']['iface_h']
            f = open(fname, "w")
            f.write(text)
            f.close()

def generate_types(dict):
    decl = []
    pre_ns = []
    pre_ns_flat = ""
    for d in dict['tree']:
        curr_ns = d['local']['iface_ns']
        curr_ns_flat = string.join(curr_ns, '::')
        if pre_ns != curr_ns:
            nsdict = {'end_ns': pre_ns, 'begin_ns': curr_ns}
            t = yat.Template(template.ns_decl)
            ns_text = t.generate(nsdict)
            decl.append(ns_text)
        pre_ns = curr_ns
        pre_ns_flat = curr_ns_flat

        decl_type = d['decl_type']
        t = yat.Template(decl_map[decl_type])
        decl_text = t.generate(d)
        decl.append(decl_text)

    nsdict = {'end_ns': curr_ns, 'begin_ns': []}
    t = yat.Template(template.ns_decl)
    ns_text = t.generate(nsdict)
    decl.append(ns_text)

    dict['declarations'] = decl
    t = yat.Template(template.types_h)
    text = t.generate(dict)

    f = open(dict['types_h'], "w")
    f.write(text)
    f.close()

