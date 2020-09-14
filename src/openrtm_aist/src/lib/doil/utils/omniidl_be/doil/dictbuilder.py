#!/usr/bin/env python
# -*- coding: sjis -*-
# -*- python -*-
#
#  @file omniidl_be/doil/dictbuilder
#  @brief Dictionary Builder class for doil backend
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

"""Produce Dictionary from IDL AST"""

import string

# module from omniidl
from omniidl import idlast, idlvisitor, idltype

# module from omniidl_be/cxx
from omniidl_be.cxx import ast, id, types, output
import omniidl_be.cxx.id as corba_cxx_id

# module from omniidl_be/doil
from omniidl_be.doil import util

import omniidl_be.doil.yat as yat

#import dictbuilder

#self = dictbuilder

def impl_fullname(name):
    bits = name.suffix("Servant").fullName()
    return string.join(bits, "_")

# Main code entrypoint
def run(tree, config):
    # first thing is to build the interface implementations
    bsi = BuildDictionaryFromAST(tree, config)
    tree.accept(bsi)
    dict = bsi.get_dict()
#    import yaml
#    print yaml.dump(dict['tree'], default_flow_style=False)
    return dict




# Build the interface implementations
#
class BuildDictionaryFromAST(idlvisitor.AstVisitor):
    def get_dict(self):
        return self.dict

    def __init__(self, tree, config):
        # configuration parameters from command options
        self.config = config

        # module's namespace stack
        self.module = []

        # main dictionary
        self.dict = {}
        self.dict['tree'] = []

        # idl file name
        idl_fname = ast.mainFile()
        self.dict['idl_fname'] = idl_fname

        # included idl files
        incs = []
        idl_incs = ast.includes()
        for inc in idl_incs:
            d = self.createHeaderInfo(inc)
            if inc == idl_fname:
                self.dict.update(d)
            elif self.config['ImplicitInclude']:
                # -Wbimplicit option makes process included IDLs
                d = self.createHeaderInfo(inc)
                incs.append(d)
        self.dict['idl_includes'] = incs

        # other includes
        self.dict['include_h'] = self.config["IncludeHeaders"]

        # type mapping
        self.typemap = self.config['TypeMapping']
        # now configurations can be accessed by self.config["key"]

        # keep track of all interfaces for later use
        self.__allInterfaces = []
        self.tk_map = {
            idltype.tk_null               : "tk_null",
            idltype.tk_void               : "tk_void",
            idltype.tk_short              : "tk_short",
            idltype.tk_long               : "tk_long",
            idltype.tk_ushort             : "tk_ushort",
            idltype.tk_ulong              : "tk_ulong",
            idltype.tk_float              : "tk_float",
            idltype.tk_double             : "tk_double",
            idltype.tk_boolean            : "tk_boolean",
            idltype.tk_char               : "tk_char",
            idltype.tk_octet              : "tk_octet",
            idltype.tk_any                : "tk_any",
            idltype.tk_TypeCode           : "tk_TypeCode",
            idltype.tk_Principal          : "tk_Principal",
            idltype.tk_objref             : "tk_objref",
            idltype.tk_struct             : "tk_struct",
            idltype.tk_union              : "tk_union",
            idltype.tk_enum               : "tk_enum",
            idltype.tk_string             : "tk_string",
            idltype.tk_sequence           : "tk_sequence",
            idltype.tk_array              : "tk_array",
            idltype.tk_alias              : "tk_alias",
            idltype.tk_except             : "tk_except",
            idltype.tk_longlong           : "tk_longlong",
            idltype.tk_ulonglong          : "tk_ulonglong",
            idltype.tk_longdouble         : "tk_longdouble",
            idltype.tk_wchar              : "tk_wchar",
            idltype.tk_wstring            : "tk_wstring",
            idltype.tk_fixed              : "tk_fixed",
            idltype.tk_value              : "tk_value",
            idltype.tk_value_box          : "tk_value_box",
            idltype.tk_native             : "tk_native",
            idltype.tk_abstract_interface : "tk_abstract_interface",
            idltype.tk_local_interface    : "tk_local_interface"
            }

        self.corba_primitive = {
            "CORBA::Short"                : "short int",
            "CORBA::UShort"               : "unsigned short int",
            "CORBA::Long"                 : "int",
            "CORBA::ULong"                : "unsigned int",
            "CORBA::Float"                : "float",
            "CORBA::Double"               : "double",
            "CORBA::Char"                 : "char",
            "CORBA::Boolean"              : "bool",
            "char*"                       : "::std::string",
            "CORBA::Any"                  : "::std::string",
            "CORBA::TypeCode_ptr"         : "::std::string"
            }

    def createHeaderInfo(self, idl_path):
        dict = {}
        idl_path_list = idl_path.split('/')
        idl_fname     = idl_path_list[-1]

        dict['idl_fname']     = idl_fname
        dict['idl_fname_path'] = idl_path

        # types.h
        base_name                   = idl_fname.split('.')[0]
        inc_guard                   = base_name.upper() + 'TYPES_H'
        dict['types_h']             = types_h = base_name + 'Types.h'
        dict['typeconv_h']          = base_name + 'TypeConversion.h'
        dict['typeconv_cpp']        = base_name + 'TypeConversion.cpp'
        dict['types_include_guard'] = inc_guard
        if self.config['IfaceDir'] != "":
            inc_types_h_path = self.config['IfaceDir'] \
                + '/' + types_h
        else:
            inc_types_h_path = types_h
        dict['types_h_path'] = inc_types_h_path

        # typeconv.h
        dict['typeconv_h'] = typeconv_h = base_name + 'TypeConversion.h'
        dict['typeconv_cpp']            = base_name + 'TypeConversion.cpp'
        tc_inc_guard                    = base_name.upper() + 'TYPECONVERSION_H'
        dict['typeconv_include_guard']  = tc_inc_guard
        if self.config['ServantDir'] != "":
            inc_typeconv_h_path = self.config['ServantDir'] \
                + '/' + typeconv_h
        else:
            inc_typeconv_h_path = typeconv_h
        dict['typeconv_h_path'] = inc_typeconv_h_path
        return dict



    def createDecl(self, decl_type):
        """
        宣言情報の基本ディクショナリの生成

        decl_type:     宣言のタイプ, struct, interface, union など
        corba:
          decl_type:     宣言のタイプ, struct, interface, union など
          corba_ns: []   ネームスペースのリスト
        local:
          decl_type:     宣言のタイプ, struct, interface, union など
          local_ns: []   ローカルインターフェースのネームスペース
          adapter_ns: [] アダプタのネームスペース
          servant_ns: [] サーバントのネームスペース
        """
        cdict = {'decl_type': decl_type}
        ldict = {'decl_type': decl_type}
        cdict['corba_ns'] = self.module
        ldict['iface_ns'] = self.module + self.config['IfaceNs']
        ldict['adapter_ns'] = self.module + self.config['AdapterNs']
        ldict['proxy_ns'] = self.module + self.config['ProxyNs']
        ldict['servant_ns'] = self.module + self.config['ServantNs']
        return {'decl_type': decl_type, 'corba': cdict, 'local': ldict}


    def getType(self, typeobj):
        """
        CORBA と Local の型名を取得する
        """
        corba_type = typeobj.base()
        # for omniidl4 4.1.1-2
        if corba_type[:2] == "::":
            corba_type = corba_type[2:]
        is_primitive = None
        # if CORBA to Local mapping is specified explicitly
        if self.typemap.has_key(corba_type):
            local_type = self.typemap[corba_type]

        # if CORBA type is primitive, string or Any
        elif self.corba_primitive.has_key(corba_type):
            local_type = self.corba_primitive[corba_type]
            if corba_type[:5] == 'CORBA':
                corba_type = '::' + corba_type
            tk = self.tk_map[typeobj.kind()]
            primitive = ["tk_short", "tk_long", "tk_ushort", 
                         "tk_ulong", "tk_float", "tk_double",
                         "tk_boolean", "tk_char", "tk_octet"]
            if primitive.count(tk) > 0:
                is_primitive = 'YES'


        # other case
        else:
            corba_scoped_type = corba_type.split('::')
            corba_ns = corba_scoped_type[:-1]
            corba_base = corba_scoped_type[-1]
            local_ns = corba_ns + self.config['IfaceNs']
            local_scope = string.join(local_ns, '::')
            if typeobj.objref():
                corba_base = corba_base[:corba_base.rfind('_ptr')]
                local_type = local_scope + '::' + \
                    self.config['IfacePrefix'] + \
                    corba_base + \
                    self.config['IfaceSuffix']
            elif typeobj.sequence():
                seqType = types.Type(typeobj.type().seqType())
                # get type of element of sequence
                (corba_etype, local_etype, eis_primitive) = self.getType(seqType)
                if seqType.objref():
                    local_etype = local_etype + '*'
                local_type = "std::vector< " + local_etype + " >"

            else:
                local_type = local_scope + '::' + corba_base
            corba_type = '::' + corba_type
            local_type = '::' + local_type
        return (corba_type, local_type, is_primitive)       


    def createIdent(self, dict, node):
        """
        宣言の識別子に関するディクショナリの生成
        主に、識別子のIDL、C++、Local名を生成する
        createDeclで生成したディクショナリとnodeを引数に取る

        corba:
          idl_name:       宣言のidl上の識別子
          name:           C++にマッピングされた識別子
          name_fq:        C++識別子の完全修飾名
          scoped_name: [] リスト形式の完全修飾名
        local:
          idl_name:       宣言のidl上の識別子
          name:           C++にマッピングされた識別子
          name_fq:        C++識別子の完全修飾名
          scoped_name: [] リスト形式の完全修飾名

        """
        cdict = dict['corba']
        ldict = dict['local']

        cdict['idl_name'] = idl_name = node.identifier()
        cdict['name'] = cxx_name = id.mapID(idl_name)
        ns = node.scopedName()[:-1]
        cdict['corba_ns'] = ns
        ldict['iface_ns'] = ns + self.config['IfaceNs']
        ldict['adapter_ns'] = ns + self.config['AdapterNs']
        ldict['proxy_ns'] = ns + self.config['ProxyNs']
        ldict['servant_ns'] = ns + self.config['ServantNs']
        cxx_fq_name = id.Name(node.scopedName()).fullyQualify()
        cdict['name_fq'] = '::' + cxx_fq_name
        cdict['scoped_name'] = node.scopedName()

        iface_ns = '::' + string.join(ldict['iface_ns'], '::')

        if self.typemap.has_key(cxx_fq_name):
            local_fq_name = self.typemap[cxx_fq_name]
            local_name = local_fq_name.split('::')[-1]
        elif self.corba_primitive.has_key(cxx_fq_name):
            local_fq_name = self.corba_primitive[cxx_fq_name]
            local_name    = local_fq_name
        else:
            local_name = cxx_name
            local_fq_name = iface_ns + '::' + local_name

        ldict['name'] = local_name
        ldict['name_fq'] = local_fq_name
        ldict['scoped_name'] = ldict['iface_ns'] + [local_name]
        return dict


    def createInterfaceIdent(self, dict, node):
        """
        インターフェース宣言の識別子に関するディクショナリの生成
        interface/servant/adapter 名を作成しディクショナリに追加する

        corba:
          name_poa:            CORBA POAクラス名
        local:
          iface_name:          Interfaceの識別子
          iface_name_fq:       Interfaceの完全修飾名
          iface_scoped_name:   Interfaceのリスト形式完全修飾名
          servant_name:        Servantの識別子
          servant_name_fq:     Servantの完全修飾名
          servant_scoped_name: Servantのリスト形式完全修飾名
          adapter_name:        Adapterの識別子
          adapter_name_fq:     Adapterの完全修飾名
          adapter_scoped_name: Adapterのリスト形式完全修飾名

        """
        self.createIdent(dict, node)
        cdict = dict['corba']
        ldict = dict['local']
        cdict['name_poa'] = '::POA_' + cdict['name_fq'].strip(':')

        # set iface_name, servant_name adapter_name
        name = ldict['name']
        p = "Prefix"
        s = "Suffix"
        n = "Ns"
        for t in ['Iface', 'Servant', 'Adapter', 'Proxy']:
        #for t in ['Iface', 'Servant', 'Adapter']:
            key = t.lower() + '_name'
            local_name = self.config[t + p] + name + self.config[t + s]
            scoped_name = cdict['corba_ns'] + self.config[t + n] + [local_name]
            local_fq_name = '::' + string.join(scoped_name, '::')
            ldict[key] = local_name
            ldict[key + '_fq'] = local_fq_name
            ldict[key + '_scoped_name'] = scoped_name
        return dict


    def createInterfaceFileInfo(self, dict, node):
        """
        インターフェース関連ファイル名のディクショナリの生成

        local:
          iface_h:               Interfaceヘッダファイル名
          iface_cpp:             Interface実装ファイル名
          iface_h_path:          Interfaceヘッダのインクルードパス
          iface_include_guard:   Interfaceヘッダののインクルードガード
          servant_h:             Servantヘッダファイル名
          servant_cpp:           Servant実装ファイル名
          servant_h_path:        Servantヘッダのインクルードパス
          servant_include_guard: Servantヘッダののインクルードガード
          adapter_h:             Adapterヘッダファイル名
          adapter_cpp:           Adapter実装ファイル名
          adapter_h_path:        Adapterヘッダのインクルードパス
          adapter_include_guard: Adapterヘッダののインクルードガード

        """
        cdict = dict['corba']
        ldict = dict['local']
        ldict['include_h'] = self.config["IncludeHeaders"]

        # set [iface|servant|adapter]_[h|cpp|h_path|include_guard]
        
        for t in ['Iface', 'Servant', 'Adapter', 'Proxy']:
        #for t in ['Iface', 'Servant', 'Adapter']:
            k = t.lower()
            ldict[k + '_h']   = ldict[k + '_name'] + ".h"
            ldict[k + '_cpp'] = ldict[k + '_name'] + ".cpp"
            if self.config[t + 'Dir'] == '':
                ldict[k + '_h_path'] = ldict[k + '_h']
            else:
                ldict[k + '_h_path'] = \
                    self.config[t + 'Dir'] + '/' + ldict[k + '_h']
            ns = string.join(map(lambda x: x + '_',
                                 ldict[k + '_ns']), '')
            ns = ns.upper()
            name = ldict[k + '_name'].upper()
            ldict[k + '_include_guard'] = ns + name + "_H"
        return dict


    def createUnionIdent(self, dict, node):
        """
        共用体宣言のの識別子に関するディクショナリの生成
        
        corba:
          idl_name:          宣言のidl上の識別子
          name:              C++にマッピングされた識別子
          name_fq:           C++識別子の完全修飾名
          scoped_name: []    リスト形式の完全修飾名
          switch_type:       switchの型
          switch_fq_type:    switchの完全修飾型
          deref_switch_type: switchの非参照型
        local:
          idl_name:          宣言のidl上の識別子
          name:              C++にマッピングされた識別子
          name_fq:           C++識別子の完全修飾名
          scoped_name: []    リスト形式の完全修飾名

        """
        self.createIdent(dict, node)
        cdict = dict['corba']
        ldict = dict['local']

        switchType = types.Type(node.switchType())
        ast.markDefaultCase(node)
        hasDefault = ast.defaultCase(node) != None

        (ctype, ltype, is_primitive) = self.getType(switchType)
        cdict['switch_fq_type'] = ctype
        cdict['switch_type']    = ctype.split('::')[-1]
        ldict['switch_fq_type'] = ltype
        ldict['switch_type']    = ltype.split('::')[-1]
        return 


    def createStructIdent(self, dict, node):
        return self.createIdent(dict, node)


    def createEnumIdent(self, dict, node):
        return self.createIdent(dict, node)


    def createExceptionIdent(self, dict, node):
        return self.createIdent(dict, node)


    def createMembers(self, dict, node):
        corba_name = dict['corba']['name']
        outer_environment = id.lookup(node)
        environment = outer_environment.enter(id.mapID(corba_name))
        scope = environment.scope()

        members = []
        for member in node.members():
            #------------------------------------------------------------
            # member
            # - type_d: type of a member
            # - decl_d: decralation list
            memberType = types.Type(member.memberType())

#            self.createType(memberType, environment)
            memtype = memberType.member(environment)
            for decl in member.declarators():
                m = self.createMember(decl, member, environment)
                if m != None:
                    members.append(m)
        dict['members'] = members
        return dict


    def createMember(self, decl, member, env):
        dict = self.createDecl('member')
        cdict = dict['corba']
        ldict = dict['local']

        memberType = types.Type(member.memberType())
        memtype = memberType.member(env)

        (ctype, ltype, is_primitive) = self.getType(memberType)
        cdict['base_type'] = ctype
        cdict['tk'] = self.tk_map[memberType.kind()]
        ldict['base_type'] = ltype
        cdict['member_name'] = id.mapID(decl.identifier())
        cdict['member_dims'] = decl.sizes()
        ldict['member_name'] = cdict['member_name']
        ldict['member_dims'] = cdict['member_dims']

        if memberType.objref():
            corba_mtype = ctype
            local_mtype = ltype + '*'
        else:
            corba_mtype = ctype
            local_mtype = ltype

        cdict['member_type'] = corba_mtype
        ldict['member_type'] = local_mtype
        return dict


    def createUnionCases(self, dict, node):
        cases = []
        switchType = types.Type(node.switchType())
        ast.markDefaultCase(node) 
        outer_environment = id.lookup(node)
        environment = outer_environment.enter(node.identifier())

        for case in node.cases():
            c = self.createUnionCase(case, node, switchType, environment)
            if c != None:
                cases.append(c)
        dict['cases'] = cases
        return dict
            

    def createUnionCase(self, case, node, switchtype, environment):
        dict = self.createDecl('union_case')
        cdict = dict['corba']
        ldict = dict['local']

        caseType = types.Type(case.caseType())
        d_caseType = caseType.deref()
        (corba_ctype, local_ctype, is_primitive) = self.getType(caseType)
        cdict['case_type'] = corba_ctype
        ldict['case_type'] = local_ctype

        decl = case.declarator()
        case_member = id.mapID(decl.identifier())
        cdict['case_member'] = case_member
        ldict['case_member'] = case_member

        decl_dims = decl.sizes()
        full_dims = decl_dims + caseType.dims()
        is_array = full_dims != []
        if is_array: raise "array union case type is not supported."

        # ------------------------------------------------------------
        # generate default discriminator
        def choose(switchType = switchtype,
                   values = ast.allCaseLabelValues(node),
                   environment = environment):
            switchType = switchType.deref()
            def min_unused(start, used = values):
                x = start
                while x in used:
                    x = x + 1
                return x
            kind = switchType.type().kind()
            if switchType.integer():
                (low, high) = ast.integer_type_ranges[kind]
                s = switchType.literal(min_unused(low+1))
                return s
            elif kind == idltype.tk_char:
                all = map(chr, range(0, 255))
            elif kind == idltype.tk_boolean:
                all = [0, 1]
            elif kind == idltype.tk_enum:
                all = switchType.type().decl().enumerators()
            else:
                util.fatalError("Failed to generate a default union " +\
                                    "discriminator value")
            possibles = util.minus(all, values)
            return switchType.literal(possibles[0], environment)
        # ------------------------------------------------------------

        labels = case.labels()
        if labels != []:
            non_default_labels = filter(lambda x:not x.default(), labels)
            if non_default_labels == []:
                # only one label and it's the default
                label = labels[0]
                discrimvalue = choose()
            elif len(non_default_labels) > 1:
                # oooh, we have a choice. Let's pick the second one.
                # no-one will be expecting that
                label = non_default_labels[1]
            else:
                # just the one interesting label
                label = non_default_labels[0]

            if label.default():
                discrimvalue = choose()
            else:
                discrimvalue = switchtype.literal(label.value(),
                                                      environment)
        cdict['discriminator'] = discrimvalue
        ldict['discriminator'] = discrimvalue

        if switchtype.enum():
            corba_ns = '::' + string.join(cdict['corba_ns'], '::')
            local_ns = '::' + string.join(ldict['iface_ns'], '::')
            cdict['discriminator_fq'] = corba_ns + '::' + discrimvalue
            ldict['discriminator_fq'] = local_ns + '::' + discrimvalue
        else:
            cdict['discriminator_fq'] = discrimvalue
            ldict['discriminator_fq'] = discrimvalue
        non_default_labels = filter(lambda x:not x.default(), labels)
        return dict


    def createTypedef(self, aliasType, decl, env):
        """
        typedef宣言に関するディクショナリの生成

        corba:
          derived_type:    導出型名
          derived_fq_type: 完全修飾導出型名
          deref_type:      非参照型名
          deref_fq_type:   完全修飾非参型名
          tk: TypeCode
        local:
          derived_type:    導出型名
          derived_fq_type: 完全修飾導出型名
          deref_type:      非参照型名
          deref_fq_type:   完全修飾非参型名
        """

        dict = self.createDecl('typedef')
        cdict = dict['corba']
        ldict = dict['local']

        (cdict['base_type'], ldict['base_type'], is_primitive) = self.getType(aliasType)
        derivedName = id.mapID(decl.identifier())
        alias_dims = aliasType.dims()
        cdict['derived_type'] = derivedName
        ldict['derived_type'] = derivedName

        corba_ns = '::' + string.join(cdict['corba_ns'], '::')
        local_ns = '::' + string.join(ldict['iface_ns'], '::')
        cdict['derived_type_fq'] = corba_ns + '::' + derivedName
        ldict['derived_type_fq'] = local_ns + '::' + derivedName
        cdict['tk'] = tk = self.tk_map[aliasType.kind()]
        primitive = ["tk_short", "tk_long", "tk_ushort", 
                     "tk_ulong", "tk_float", "tk_double",
                     "tk_boolean", "tk_char", "tk_octet"]
        if primitive.count(tk) > 0:
            cdict['is_primitive'] = 'YES'

        if aliasType.sequence():
            seqType = types.Type(aliasType.type().seqType())
            # get type of element of sequence
            (corba_etype, local_etype, is_primitive) = self.getType(seqType)
            cdict['element_tk'] = self.tk_map[seqType.kind()]
            if seqType.objref():
                cdict['element_type_fq'] = corba_etype
                ldict['element_type_fq'] = local_etype + '*'
            else:
                cdict['element_type_fq'] = corba_etype
                ldict['element_type_fq'] = local_etype
        return dict


    # ------------------------------------------------------------
    # オペレーションに関する辞書を作成する
    #
    # name: オペレーションの名称
    #
    def createOperation(self, operation):
        dict = {}
        dict['name'] = id.mapID(operation.identifier())
        dict['raises'] = []
        for r in operation.raises():
            edict = self.createDecl('exception')
            self.createExceptionIdent(edict, r)
            dict['raises'].append(edict)
        return dict

    def createAttribute(self, ident):
        dict = {}
        dict['name'] = id.mapID(ident)
        return dict

    # ------------------------------------------------------------
    # オペレーションの戻り値に関する辞書を作成する
    #
    # type_r: remote戻り値の型名
    # type_l: local戻り値の型名
    # var_r: remote戻り値の変数名
    # var_l: local戻り値の変数名
    # to_doil: to_remote変換関数
    # to_local: to_local変換関数
    #
    def createReturn(self, operation):
        """
        corba:
          base_type:
          ret_type:
          decl_type:
          tk:
        local:
          base_type:
          ret_type:
          decl_type:
          tk:
        """
        dict = self.createDecl('return')
        cdict = dict['corba']
        ldict = dict['local']

        retType = types.Type(operation.returnType())
        (corba_type, local_type, is_primitive) = self.getType(retType)
        cdict['base_type'] = corba_type
        ldict['base_type'] = local_type 
        if is_primitive != None:
            cdict['is_primitive'] = is_primitive
        retn_type = types.Type(operation.returnType()).op(types.RET)
        retn_type = retn_type.replace('CORBA', '::CORBA')
        retn_type = retn_type.replace('RTC', '::RTC')
        retn_type = retn_type.replace('SDOPackage', '::SDOPackage')
        retn_type = retn_type.replace('::::', '::')
        cdict['retn_type'] = retn_type

        if retType.objref(): local_rtype = local_type + '*'
        else:                local_rtype = local_type

        ldict['retn_type'] = local_rtype
        cdict['tk'] = ldict['tk'] = self.tk_map[retType.kind()]

        if retType.deref().sequence():
            retType = retType.deref()

        if retType.sequence():
            seqType = types.Type(retType.type().seqType())
            # get type of element of sequence
            (corba_etype, local_etype, is_primitive) = self.getType(seqType)
            cdict['deref_tk'] = self.tk_map[seqType.kind()]
        else:
            derefType = retType.deref()
            (corba_dtype, local_dtype, is_primitive) = self.getType(derefType)
            cdict['deref_tk'] = self.tk_map[derefType.kind()]
        return dict


    # ------------------------------------------------------------
    # オペレーションの引数に関する辞書を作成する
    #
    # type_r: remoteの引数の型
    # type_l: localの引数の型
    # var_r: remoteの変数名
    # var_l: localの変数名
    # to_doil: to_remote変換関数
    # to_local: to_local変換関数
    #
    def createArgs(self, operation, env):
        """

        corba:
          base_type:
          arg_type:
          arg_name:
          var_name:
          decl_type:
          direction:
          tk:
        local:
          base_type:
          arg_type:
          arg_name:
          var_name:
          decl_type:
          direction:
          tk:


        """
        args = []
        direction = ['in', 'out', 'inout','return']
        for arg in operation.parameters():
            # corba args information
            dict = self.createDecl('arg')
            cdict = dict['corba']
            ldict = dict['local']

            paramType = types.Type(arg.paramType())
            (corba_type, local_type, is_primitive) = self.getType(paramType)

            cdict['base_type'] = corba_type
            ldict['base_type'] = local_type
            if is_primitive != None:
                cdict['is_primitive'] = is_primitive

            arg_name = id.mapID(arg.identifier())
            cdict['arg_name'] = arg_name
            ldict['arg_name'] = arg_name
            cdict['var_name'] = '_' + arg_name
            ldict['var_name'] = '_' + arg_name

            direction_val = direction[arg.direction()]
            cdict['direction'] = direction_val
            ldict['direction'] = direction_val

            cdict['tk'] = ldict['tk'] = self.tk_map[paramType.kind()]
            arg_type = paramType.op(types.direction(arg), use_out = 0)
            arg_type = arg_type.replace('CORBA', '::CORBA')
            arg_type = arg_type.replace('RTC', '::RTC')
            arg_type = arg_type.replace('SDOPackage', '::SDOPackage')
            arg_type = arg_type.replace('::::', '::')
            cdict['arg_type']  = arg_type
            out = arg.is_out()
            self.createArg(dict, paramType, out)
            args.append(dict)
        return args

    def createArg(self, dict, typeobj, out):
        (corba_type, local_type, is_primitive) = self.getType(typeobj)
        cdict = dict['corba']
        ldict = dict['local']
        if is_primitive != None:
            cdict['is_primitive'] = is_primitive

        paramType = typeobj
        if paramType.typedef():
            paramType = paramType.deref()
        # primitive type
        if paramType.is_basic_data_types():
            if out: ldict['arg_type'] = local_type + '&'
            else:   ldict['arg_type'] = local_type
            ldict['var_type'] = local_type
            cdict['var_type'] = corba_type
        # Enum type
        elif paramType.enum():
            if out: ldict['arg_type'] = local_type + '&'
            else:   ldict['arg_type'] = local_type
            ldict['var_type'] = local_type
            cdict['var_type'] = corba_type
        # Struct type
        # Sequence type
        # Union type           
        elif paramType.struct() or paramType.sequence() or \
                paramType.union():
            if out: ldict['arg_type'] = local_type + '&'
            else:   ldict['arg_type'] = 'const '+local_type+'&'
            ldict['var_type'] = local_type
            cdict['var_type'] = corba_type + '*'
        # Object type
        elif paramType.objref():
            if out: ldict['arg_type'] = local_type + '*'
            else:   ldict['arg_type'] = 'const '+local_type+'*'
            ldict['var_type'] = local_type + '*'
            cdict['var_type'] = corba_type
        # String type
        # Any type
        elif paramType.any() or paramType.string():
            if out: ldict['arg_type'] = local_type + '&'
            else:   ldict['arg_type'] = 'const ' + local_type + '&'
            ldict['var_type'] = local_type
            cdict['var_type'] = corba_type
        else:
            raise "UNKNOWN TYPE", (
                self.tk_map[paramType.kind()],
                paramType.typedef())


    #------------------------------------------------------------
    # AST visitor functions
    #
    # Tree walking code
    def visitAST(self, node):
        for n in node.declarations():
            if ast.shouldGenerateCodeForDecl(n):
                n.accept(self)

    # modules can contain interfaces
    def visitModule(self, node):
        module = id.mapID(node.identifier())
        self.module.append(module)
        for n in node.definitions():
            # enter tree inside this module
            n.accept(self)
        self.module.pop()


    def visitDeclarator(self, node):
        print "Declarator", id.mapID(node.identifier())


    def visitStructForward(self, node):
        dict = self.createDecl('struct_forward')
        # set corba/local struct identifier
        self.createStructIdent(dict, node)
        # append dicts to tree
        self.dict['tree'].append(dict)
        return


    def visitUnionForward(self, node):
        dict = self.createDecl('union_forward')
        # set corba/local union identifier
        self.getUnionIdent(dict, node)
        # append dicts to tree
        self.dict['tree'].append(dict)
        return


    def visitForward(self, node):
        dict = self.createDecl('interface_forward')
        # set corba/local interface identifier
        self.createInterfaceIdent(dict, node)
        # append dicts to tree
        self.dict['tree'].append(dict)
        return

    def visitException(self, node):
        dict = self.createDecl('exception')
        # set corba/local exception identifier
        self.createExceptionIdent(dict, node)
        # create members
        self.createMembers(dict, node)
        # add to dict
        self.dict['tree'].append(dict)
        return


    def visitUnion(self, node):
        dict = self.createDecl('union')
        # set corba/local union identifier
        self.createUnionIdent(dict, node)
        # create union cases
        self.createUnionCases(dict, node)
        # add to dict
        self.dict['tree'].append(dict)
        return


    def visitEnum(self, node):
        dict = self.createDecl('enum')
        cdict = dict['corba']
        ldict = dict['local']
        self.createEnumIdent(dict, node)

        enumerators = node.enumerators()
        memberlist = map(lambda x: id.Name(x.scopedName()).simple(),
                         enumerators)
        cdict['members'] = memberlist
        ldict['members'] = memberlist
        self.dict['tree'].append(dict)
        return


    def visitTypedef(self, node):
        environment = id.lookup(node)
        scope = environment.scope()
        aliasType = types.Type(node.aliasType())
        aliasTypeID = aliasType.member(environment)

        if node.constrType():
            node.aliasType().decl().accept(self)


        for decl in node.declarators():
            dict = self.createTypedef(aliasType, decl, environment)
            self.dict['tree'].append(dict)
        return


    def visitStruct(self, node):
        dict = self.createDecl('struct')
        self.createStructIdent(dict, node)
        self.createMembers(dict, node)
        self.dict['tree'].append(dict)
        return       


    # interfaces cannot be further nested
    def visitInterface(self, node):
        self.__allInterfaces.append(node)
        # listed scope and interface name
        dict = self.createDecl('interface')
        self.createInterfaceIdent(dict, node)
        self.createInterfaceFileInfo(dict, node)

        dict['inherits'] = []
        for ihnode in ast.allInherits(node):
            idict = self.createDecl('inherit')
            self.createInterfaceIdent(idict, ihnode)
            self.createInterfaceFileInfo(idict, ihnode)
            dict['inherits'].append(idict)

        env = id.lookup(node)

        allInterfaces = [node]# + ast.allInherits(node)
        allCallables = util.fold( map(lambda x:x.callables(), allInterfaces),
                                  [], lambda x, y: x + y )

        dict['operations'] = []
        dict['attributes'] = []
        for c in allCallables:
            if isinstance(c, idlast.Attribute):
                attrType = types.Type(c.attrType())
                d_attrType = attrType.deref()
                (corba_atype, local_atype, is_primitive) = self.getType(attrType)

                for i in c.identifiers():
                    ident = id.mapID(i)
                    returnType = attrType.op(types.RET)
                    inType = attrType.op(types.IN)

                    adict = createDecl('attribute')
                    cdict = adict['corba']
                    ldict = adict['local']
                    cdict['base_type'] = corba_atype;
                    ldict['base_type'] = local_atype
                    cdict['name'] = ident
                    adict['return'] = self.createReturn(c)
                    adict['arg'] = {}
                    self.createArg(adict['arg'], attrType, False)

                    dict['attributes'].append(adict)
                    if c.readonly():
                        dict['readonly'] = 'yes'
                    dict['attributes'].append(gdict)

            elif isinstance(c, idlast.Operation):
                # operations
                op_dict           = self.createOperation(c)
                op_dict['return'] = self.createReturn(c)
                op_dict['args']   = self.createArgs(c, env)
                dict['operations'].append(op_dict)
            else:
                util.fatalError("Internal error generating interface member")
                raise "No code for interface member: " + repr(c)

#        self.dict['interfaces'].append(dict)
        self.dict['tree'].append(dict)
        return


