#!/usr/bin/env python
# -*- coding: shift_jis -*-
# -*- python -*-
#
# @file omniidl_be/doil/corba/__init__.py
# @brief corba servant/adapter code generator for doil backend
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

from omniidl_be.doil.corba import template
import omniidl_be.doil.yat as yat

keys = ['idl_fname', 'idl_includes',
        'types_h', 'types_h_path', 
        'typeconv_h', 'typeconv_h_path',
        'include_h']

decl_map = {
    'interface': {
        'decl': template.object_conv_h,    'impl': template.object_conv_cpp},
    'struct':    {
        'decl': template.struct_conv_h,    'impl': template.struct_conv_cpp},
    'union':     {
        'decl': template.union_conv_h,     'impl': template.union_conv_cpp},
    'enum':      {
        'decl': template.enum_conv_h,      'impl': template.enum_conv_cpp},
    'exception': {
        'decl': template.exception_conv_h, 'impl': template.exception_conv_cpp},
    'typedef':   {
        'decl': template.typedef_decl_h,   'impl': template.typedef_dec_cpp}
}

def generate_servant(dict):
    for d in dict['tree']:
        if d['corba']['decl_type'] == 'interface':
            ifdict = {}
            for key in keys:
                ifdict[key] = dict[key]

            ifdict.update(d)

            t = yat.Template(template.servant_h)
            text = t.generate(ifdict)
            fname = d['local']['servant_h']
            f = open(fname, "w")
            f.write(text)
            f.close()

            t = yat.Template(template.servant_cpp)
            text = t.generate(ifdict)
            fname = d['local']['servant_cpp']
            f = open(fname, "w")
            f.write(text)
            f.close()

def generate_adapter(dict):
    for d in dict['tree']:
        if d['corba']['decl_type'] == 'interface':
            ifdict = {}
            for key in keys:
                ifdict[key] = dict[key]

            ifdict.update(d)

            t = yat.Template(template.adapter_h)
            text = t.generate(ifdict)
            fname = d['local']['adapter_h']
            f = open(fname, "w")
            f.write(text)
            f.close()

            t = yat.Template(template.adapter_cpp)
            text = t.generate(ifdict)
            fname = d['local']['adapter_cpp']
            f = open(fname, "w")
            f.write(text)
            f.close()

def generate_proxy(dict):
    for d in dict['tree']:
        if d['corba']['decl_type'] == 'interface':
            ifdict = {}
            for key in keys:
                ifdict[key] = dict[key]

            ifdict.update(d)

            t = yat.Template(template.proxy_h)
            text = t.generate(ifdict)
            fname = d['local']['proxy_h']
            f = open(fname, "w")
            f.write(text)
            f.close()

            t = yat.Template(template.proxy_cpp)
            text = t.generate(ifdict)
            fname = d['local']['proxy_cpp']
            f = open(fname, "w")
            f.write(text)
            f.close()

def generate_types(dict):
    decl = []
    impl = []
    for d in dict['tree']:
        decl_type = d['decl_type']

        if not decl_map.has_key(decl_type): continue

        decl_temp = yat.Template(decl_map[decl_type]['decl'])
        decl_text = decl_temp.generate(d)
        decl.append(decl_text)

        impl_temp = yat.Template(decl_map[decl_type]['impl'])
        impl_text = impl_temp.generate(d)
        impl.append(impl_text)


    dict['declarations'] = decl
    dict['implementations'] = impl

    typeconv_h   = dict['typeconv_h']
    typeconv_cpp = dict['typeconv_cpp']

    t = yat.Template(template.typeconv_h)
    f = open(typeconv_h, "w")
    text = t.generate(dict)
    f.write(text)
    f.close()

    t = yat.Template(template.typeconv_cpp)
    f = open(typeconv_cpp, "w")
    text = t.generate(dict)
    f.write(text)
    f.close()
