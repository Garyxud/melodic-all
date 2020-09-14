#!/usr/bin/env python
# -*- python -*-
#
#  @file template.py
#  @brief doil servant class template
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


# doil type conversion header
typeconv_h = """\
// -*- C++ -*-
/*!
 * @file TypeConversion.h
 * @brief doil-CORBA type conversion
 * @date $Date$
 * @author This file was automatically generated from [idl_fname] 
 *         by omniidl/doil backend
 *
 * $Id$
 */

#ifndef [typeconv_include_guard] 
#define [typeconv_include_guard] 

//
[for inc in include_h]
#include <[inc]>
[endfor]

#include <[types_h_path]>

[for decl in declarations]
[decl]
[endfor]

#endif // [typeconv_include_guard]

"""
typeconv_cpp = """\
// -*- C++ -*-
/*!
 * @file TypeConversion.h
 * @brief doil-CORBA type conversion
 * @date $Date$
 * @author This file was automatically generated from [idl_fname] 
 *         by omniidl/doil backend
 *
 * $Id$
 */

#include <doil/corba/BasicTypeConversion.h>
#include <[typeconv_h_path]>
#include <doil/corba/SeqTypeConversion.h>
#include <doil/corba/CORBAManager.h>

[for node in tree]
[if node.corba.decl_type is "interface"]
#include <[node.local.iface_h_path]>
[endif]
[endfor]

[for impl in implementations]
[impl]
[endfor]

// end of TypeConversion

"""


#------------------------------------------------------------
# object conversion declaration
object_conv_h = """\
// corba object -> local object
bool
corba_to_local([corba.name_fq]_ptr _from,
               [local.iface_name_fq]* _to);

// local object -> corba object
bool
local_to_corba([local.iface_name_fq]* _from,
               [corba.name_fq]_ptr _to);

"""

#------------------------------------------------------------
# object conversion implementation
object_conv_cpp = """\
// corba object -> local object
bool
corba_to_local([corba.name_fq]_ptr _from,
               [local.iface_name_fq]* _to)
{
  doil::LocalBase* lobj;
  lobj = doil::CORBA::CORBAManager::instance().toLocal((::CORBA::Object_ptr&)_from);
  if (lobj == NULL) return false;
  _to = dynamic_cast< [local.iface_name_fq]* >(lobj);
  if (_to == NULL) return false;
  return true;
}

// local object -> corba object
bool
local_to_corba([local.iface_name_fq]* _from,
               [corba.name_fq]_ptr _to)
{
  CORBA::Object_ptr obj;
  obj = doil::CORBA::CORBAManager::instance().toReference((doil::LocalBase*)_from);
  _to = [corba.name_fq]::_narrow(obj);
  if (CORBA::is_nil(_to)) return false;
  return true;
}

"""

#------------------------------------------------------------
# object declaration
struct_conv_h = """\
// struct corba -> local
bool
corba_to_local(const [corba.name_fq]& _from,
               [local.name_fq]& _to);

// struct local -> corba
bool
local_to_corba(const [local.name_fq]& _from,
               [corba.name_fq]& _to);

"""
#------------------------------------------------------------
# struct conversion declaration
struct_conv_cpp = """\
// struct corba -> local
bool
corba_to_local(const [corba.name_fq]& _from,
               [local.name_fq]& _to)
{
[for mem in members]
[if mem.corba.tk is "tk_objref"]
  if (!corba_to_local(([mem.corba.base_type]&)_from.[mem.corba.member_name],
                      _to.[mem.local.member_name]))
[else]
  if (!corba_to_local(_from.[mem.corba.member_name],
                      _to.[mem.local.member_name]))
[endif]
     return false;
[endfor]
  return true;
}

// struct local -> corba
bool
local_to_corba(const [local.name_fq]& _from,
               [corba.name_fq]& _to)
{
[for mem in members]
[if mem.corba.tk is "tk_objref"]
  if (!local_to_corba(_from.[mem.local.member_name],
                      ([mem.corba.base_type]&)_to.[mem.corba.member_name]))
[else]
  if (!local_to_corba(_from.[mem.local.member_name],
                      _to.[mem.corba.member_name]))
[endif]
     return false;
[endfor]
  return true;
}

"""

#------------------------------------------------------------
# union conversion declaration
union_conv_h = """\
// union corba -> local
bool
corba_to_local(const [local.name_fq]& _from,
               [corba.name_fq]& _to);

// union local -> corba
bool
corba_to_local(const [corba.name_fq]& _from,
               [local.name_fq]& _to);

"""

#------------------------------------------------------------
# union conversion implementation
union_conv_cpp = """\
// union corba -> local
bool
corba_to_local(const [corba.name_fq]& _from,
               [local.name_fq]& _to)
{
  switch (_from._d())
    {
[for cas in cases]
     case [cas.corba.discriminator_fq] :
       {
         [cas.local.case_type] l[cas.local.case_member];
         [cas.corba.case_type] c[cas.corba.case_member];
         c[cas.corba.case_member] = _from.[cas.corba.case_member]();
         corba_to_local(c[cas.corba.case_member], l[cas.local.case_member]);
         _to.set_[cas.local.case_member](l[cas.local.case_member]);
         break;
       }
[endfor]
      default:
        return false;
    }
  return true;
}

// union local -> corba
bool
local_to_corba(const [local.name_fq]& _from,
               [corba.name_fq]& _to)
{
  switch (_from.get_type())
    {
[for cas in cases]
    case [cas.local.discriminator_fq] :
      {
        [cas.corba.case_type] c[cas.corba.case_member];
        [cas.local.case_type] l[cas.local.case_member];
        l[cas.local.case_member] = _from.get_[cas.corba.case_member]();
        local_to_corba(l[cas.local.case_member], c[cas.corba.case_member]);
        _to.[cas.local.case_member](c[cas.corba.case_member]);
        break;
      }
[endfor]
      default:
        return false;
    }
  return true;
}

"""

#------------------------------------------------------------
# enum conversion declaration
enum_conv_h = """\
// enum corba -> local
inline bool
corba_to_local(const [corba.name_fq]& _from,
               [local.name_fq]& _to)
{
  _to = [local.name_fq]((long)_from);
  return true;
}

// enum local -> corba
inline bool
local_to_corba(const [local.name_fq]& _from,
               [corba.name_fq]& _to)
{
  _to = [corba.name_fq]((long)_from);
  return true;
}

"""

#------------------------------------------------------------
# enum conversion implementation
enum_conv_cpp = """\
"""

exception_conv_h = """\
// exxception corba -> local
bool
corba_to_local(const [local.name_fq]& _from,
               [corba.name_fq]& _to);

// exception local -> corba
bool
corba_to_local(const [corba.name_fq]& _from,
               [local.name_fq]& _to);

"""
exception_conv_cpp = """\
// exception corba -> local
bool
corba_to_local(const [corba.name_fq]& _from,
               [local.name_fq]& _to)
{
[for mem in members]
  if (!corba_to_local(_from.[mem.corba.member_name],
                      _to.[mem.local.member_name]))
     return false;
[endfor]
  return true;
}

// exception local -> corba
bool
local_to_corba(const [local.name_fq]& _from,
               [corba.name_fq]& _to)
{
[for mem in members]
  if (!local_to_corba(_from.[mem.local.member_name],
                      _to.[mem.corba.member_name]))
     return false;
[endfor]
  return true;
}

"""

#------------------------------------------------------------
# typedef conversion declaration
typedef_decl_h = """\
[if corba.tk is "tk_alias"][elif corba.tk is "tk_string"][elif corba.tk is "tk_long"][else]
// typedef corba -> local : [corba.tk] 
bool
corba_to_local(const [corba.derived_type_fq]& _from,
               [local.derived_type_fq]& _to);

// typedef local -> corba : [corba.tk] 
bool
local_to_corba(const [local.derived_type_fq]& _from,
               [corba.derived_type_fq]& _to);

[endif]
"""
#------------------------------------------------------------
# typedef conversion implementation
typedef_dec_cpp = """\
[if corba.tk is "tk_alias"][elif corba.tk is "tk_string"][elif corba.tk is "tk_long"][else]
// typedef corba -> local : [corba.tk] 
bool
corba_to_local(const [corba.derived_type_fq]& _from,
               [local.derived_type_fq]& _to)
{
[if corba.tk is "tk_sequence"]
  return corba_to_local_seq< [corba.derived_type_fq],
                             [corba.element_type_fq],
                             [local.derived_type_fq],
                             [local.element_type_fq] >(_from, _to);
[elif corba.tk is "tk_enum"]
  _to = [corba.derived_type_fq]((long)_from);
  return true;
[elif corba.tk is "tk_string"]
  return corba_to_local((const [corba.base_type])_from,
                        ([local.base_type]&)_to);
[else]
[if-any corba.is_primitive]
  _to = _from;
[else]
  return corba_to_local((const [corba.base_type])_from,
                        ([local.base_type]&)_to);
[endif]
[endif]
}

// typedef local -> corba : [corba.tk] 
bool
local_to_corba(const [local.derived_type_fq]& _from,
               [corba.derived_type_fq]& _to)
{
[if corba.tk is "tk_sequence"]
  return local_to_corba_seq< [local.derived_type_fq],
                             [local.element_type_fq],
                             [corba.derived_type_fq],
                             [corba.element_type_fq] >(_from, _to);
[elif corba.tk is "tk_enum"]
  _to = [corba.derived_type]((CORBA::Long)rhs);
  return true;
[elif corba.tk is "tk_string"]
  return local_to_corba((const [local.base_type]&)_from,
                        ([corba.base_type])_to);
[else]
[if-any corba.is_primitive]
  _to = _from;
[else]
  return local_to_corba(([local.base_type])_from,
                        ([corba.base_type]&)_to);
[endif]
[endif]
}
[endif]
"""



#------------------------------------------------------------
# doil servant header file template
#
# Given keys.
# - servant_name   : doil servant class name
# - iface_name : interface class name to be delegated by this servant
# - include_guard  : include guard definition name
# - fq_POA_name    : fully qualified POA name
# - operations     : operation definitions
#
servant_h = """\
// -*- C++ -*-
/*!
 * @file [local.servant_h] 
 * @brief [local.servant_name] CORBA servant for doil
 * @date $Date$
 * @author This file was automatically generated from [idl_fname]
 *         by omniidl/doil backend
 *
 * $Id$
 */
#ifndef [local.servant_include_guard] 
#define [local.servant_include_guard] 

#include <coil/Properties.h>
#include <doil/corba/CORBAServantBase.h>
#include <doil/corba/BasicTypeConversion.h>
[for inc in include_h]
#include <[inc]>
[endfor]
[for inc in inherits]
#include <[inc.local.servant_h]>
[endfor]

#include <[types_h_path]>

namespace doil
{
  class ImplBase;
};

// interface class forward declaration
[for ns in local.iface_ns]
namespace [ns] 
{
[endfor]
  class [local.iface_name];
[for-inv ns in local.iface_ns]
}; // namespace [ns]

[endfor]


[for ns in local.servant_ns]
namespace [ns] 
{
[endfor]

  class [local.servant_name] 
   : public virtual [corba.name_poa],
[for inc in inherits]
     public virtual [inc.local.servant_name_fq],
[endfor]
     public virtual ::doil::CORBA::CORBAServantBase
  {
  public:
    [local.servant_name](doil::ImplBase* impl);
    virtual ~[local.servant_name]();

[for op in operations]
    virtual [op.return.corba.retn_type] [op.name]
([for a in op.args]
[if-index a is last][a.corba.arg_type] [a.corba.arg_name]
[else][a.corba.arg_type] [a.corba.arg_name], [endif]
[endfor]);
[endfor]

  private:
    [local.iface_name_fq]* m_impl;
  };

[for-inv ns in local.servant_ns]
}; // namespace [ns] 
[endfor]

extern "C"
{
  void [local.servant_name]CORBAInit(coil::Properties& prop);
};

#endif // [local.servant_include_guard]

"""

#
# doil servant code file template
#
# Given keys.
# - servant_name   : doil servant class name
# - iface_name : interface class name to be delegated by this servant
# - include_guard  : include guard definition name
# - fq_POA_name    : fully qualified POA name
# - operations     : operation definitions
#
servant_cpp = """\
// -*- C++ -*-
/*!
 * @file [local.servant_cpp] 
 * @brief [local.iface_name] CORBA servant for doil
 * @date $Date$
 * @author This file was automatically generated from [idl_fname]
 *         by omniidl/doil backend
 *
 * $Id$
 */

#include <doil/ImplBase.h>
#include <doil/corba/CORBAManager.h>
#include <[local.iface_h_path]>
#include <[local.servant_h_path]>
#include <[typeconv_h_path]>

[for ns in local.servant_ns]
namespace [ns] 
{
[endfor]
  /*!
   * @brief ctor
   */ 
  [local.servant_name]::[local.servant_name](doil::ImplBase* impl)
   : ::doil::CORBA::CORBAServantBase(impl), m_impl(NULL)[for inc in inherits],
     [inc.local.servant_name_fq](impl)[endfor] 
  {
    m_impl = dynamic_cast< [local.iface_name_fq]* >(impl);
    if (m_impl == NULL) throw std::bad_alloc();
    m_impl->incRef();
  }

  /*!
   * @brief dtor
   */ 
  [local.servant_name]::~[local.servant_name]()
  {
    m_impl->decRef();
  }

  [for op in operations]

  /*!
   * @brief [op.name] 
   */ 
  [op.return.corba.retn_type] [local.servant_name]::[op.name]
([for a in op.args]
[if-index a is last][a.corba.arg_type] [a.corba.arg_name]
[else][a.corba.arg_type] [a.corba.arg_name], [endif]
[endfor])
  {
[for a in op.args]
    [a.local.var_type] [a.local.var_name];
[endfor]

[for a in op.args][if a.corba.direction is "out"][else]
[if-any a.corba.is_primitive]
    [a.local.var_name] = [a.corba.arg_name];
[else]
    corba_to_local([a.corba.arg_name], [a.local.var_name]);
[endif]
[endif][endfor]

[if op.return.corba.tk is "tk_void"]
[elif op.return.corba.tk is "tk_any"]
    [op.return.local.retn_type] local_ret;
    [op.return.corba.retn_type] corba_ret = 
        new [op.return.corba.base_type] ();
    local_ret =
[elif op.return.corba.tk is "tk_struct"]
    [op.return.local.retn_type] local_ret;
    [op.return.corba.retn_type] corba_ret = 
        new [op.return.corba.base_type] ();
    local_ret =
[elif op.return.corba.tk is "tk_alias"]
[if op.return.corba.deref_tk is "tk_long"]
    [op.return.local.retn_type] local_ret;
    [op.return.corba.retn_type] corba_ret;
    local_ret =
[elif op.return.corba.deref_tk is "tk_string"]
    [op.return.local.retn_type] local_ret;
    [op.return.corba.retn_type] corba_ret;
    local_ret =
[else]
    [op.return.local.retn_type] local_ret;
    [op.return.corba.retn_type] corba_ret = 
        new [op.return.corba.base_type] ();
    local_ret =
[endif]
[else]
    [op.return.local.retn_type] local_ret;
    [op.return.corba.retn_type] corba_ret;
    local_ret = 
[endif]
        m_impl->[op.name]
([for a in op.args][if-index a is last][a.local.var_name][else][a.local.var_name], [endif][endfor]);

[for a in op.args][if a.corba.direction is "in"][else]
    local_to_corba([a.local.var_name], [a.corba.arg_name]);
[endif][endfor]
[if op.return.corba.tk is "tk_void"][else]
[if op.return.corba.tk is "tk_any"]
    local_to_corba(local_ret, *corba_ret);
[elif op.return.corba.tk is "tk_struct"]
    local_to_corba(local_ret, *corba_ret);
[elif op.return.corba.tk is "tk_alias"]
[if op.return.corba.deref_tk is "tk_long"]
    local_to_corba(local_ret, corba_ret);
[elif op.return.corba.deref_tk is "tk_string"]
    local_to_corba(local_ret, corba_ret);
[else]
    local_to_corba(local_ret, *corba_ret);
[endif]
[else]
[if-any op.return.corba.is_primitive]
    corba_ret = local_ret;
[else]
    local_to_corba(local_ret, corba_ret);
[endif]
[endif]
    return corba_ret;
[endif]
  }
[endfor]

[for-inv ns in local.servant_ns]
}; // namespace [ns] 
[endfor]

extern "C"
{
  void [local.servant_name]CORBAInit(coil::Properties& prop)
  {
    doil::CORBA::CORBAManager& mgr(doil::CORBA::CORBAManager::instance());
    mgr.registerFactory("[local.servant_name]",
                        doil::New< [local.servant_name_fq] >,
                        doil::Delete< [local.servant_name_fq] >);
  }
};
"""





adapter_h = """\
// -*- C++ -*-
/*!
 * @file [local.adapter_h] 
 * @brief [local.adapter_name] CORBA adapter for doil
 * @date $Date$
 * @author This file was automatically generated from [idl_fname] 
 *         by omniidl/doil backend
 *
 * $Id$
 */
#ifndef [local.adapter_include_guard] 
#define [local.adapter_include_guard] 

#include <coil/Properties.h>
#include <coil/Mutex.h>
#include <coil/Guard.h>
#include <doil/corba/CORBAManager.h>
#include <doil/ImplBase.h>
#include <[local.iface_h]>
[for inc in inherits]
#include <[inc.local.adapter_h]>
[endfor]
[for inc in include_h]
#include <[inc]>
[endfor]


[for ns in local.adapter_ns]
namespace [ns] 
{
[endfor]

  class [local.adapter_name] 
  : public virtual ::doil::LocalBase,
[for inc in inherits]
    public virtual [inc.local.adapter_name_fq],
[endfor]
    public virtual [local.iface_name_fq]

  {
    typedef coil::Mutex Mutex;
    typedef coil::Guard<Mutex> Guard;
  public:
    [local.adapter_name](::CORBA::Object_ptr obj);
    virtual ~[local.adapter_name]();

[for op in operations]
    virtual [op.return.local.retn_type] [op.name]
([for a in op.args]
[if-index a is last][a.local.arg_type] [a.local.arg_name]
[else][a.local.arg_type] [a.local.arg_name], [endif]
[endfor])
      throw ([for raise in op.raises]
[if-index raise is first]
[raise.local.name_fq]
[else]
,
             [raise.local.name_fq]
[endif]
[endfor]
);


[endfor]

    const char* id() {return "[corba.idl_name]";}
    const char* name() {return m_name.c_str();}
    void incRef()
    {
      Guard guard(m_refcountMutex);
      ++m_refcount;
    }
    void decRef()
    {
      Guard guard(m_refcountMutex);
      --m_refcount;
      if (m_refcount == 0)
        delete this;
    }

  private:
    [corba.name_fq]_ptr m_obj;
    std::string m_name;
    Mutex m_refcountMutex;
    int m_refcount;
  };

[for ns in local.adapter_ns]
}; // namespace [ns] 
[endfor]

#ifndef [local.servant_include_guard] 


#endif // [local.servant_include_guard]


extern "C"
{
  void [local.adapter_name]CORBAInit(coil::Properties& prop);
};

#endif // [local.adapter_include_guard]

"""


adapter_cpp = """\
// -*- C++ -*-
/*!
 * @file [local.adapter_cpp] 
 * @brief [local.iface_name] CORBA adapter for doil
 * @date $Date$
 * @author This file was automatically generated form [idl_fname] 
 *         by omniidl/doil backend
 *
 * $Id$
 */

#include <doil/ImplBase.h>
#include <doil/corba/CORBAManager.h>
#include <[local.iface_h_path]>
#include <[local.adapter_h_path]>
#include <[typeconv_h_path]>
#include <doil/corba/BasicTypeConversion.h>

[for ns in local.adapter_ns]
namespace [ns] 
{
[endfor]
  /*!
   * @brief ctor
   */ 
  [local.adapter_name]::[local.adapter_name](::CORBA::Object_ptr obj)
   : m_obj([corba.name_fq]::_nil()),
     m_refcount(1)[for inc in inherits],
     [inc.local.adapter_name_fq](obj)[endfor]

  {
    m_obj = [corba.name_fq]::_narrow(obj);
    if (::CORBA::is_nil(m_obj)) throw std::bad_alloc();
    m_obj = [corba.name_fq]::_duplicate(m_obj);
  }

  /*!
   * @brief dtor
   */ 
  [local.adapter_name]::~[local.adapter_name]()
  {
    ::CORBA::release(m_obj);
  }

  [for op in operations]

  /*!
   * @brief [op.name] 
   */ 
  [op.return.local.retn_type] [local.adapter_name]::[op.name]
([for a in op.args]
[if-index a is last][a.local.arg_type] [a.local.arg_name]
[else][a.local.arg_type] [a.local.arg_name], [endif]
[endfor])
      throw ([for raise in op.raises]
[if-index raise is first]
[raise.local.name_fq]
[else]
,
             [raise.local.name_fq]
[endif]
[endfor]
)
  {
    // Convert Local to CORBA.
    // (The direction of the argument is 'in' or 'inout'.)
[for a in op.args]
    [a.corba.base_type] [a.corba.var_name];
[endfor]
[for a in op.args][if a.local.direction is "out"][else]
[if-any a.corba.is_primitive]
    [a.corba.var_name] = [a.local.arg_name];
[else]
[if a.local.tk is "tk_objref"]
    local_to_corba(const_cast< [a.local.var_type] >([a.local.arg_name]), [a.corba.var_name]);
[else]
    local_to_corba([a.local.arg_name], [a.corba.var_name]);
[endif]
[endif]
[endif][endfor]

    // Execute the method. 
[if op.return.local.tk is "tk_void"][else]
    [op.return.corba.retn_type] corba_ret;
    [op.return.local.retn_type] local_ret;
    corba_ret = [endif]
m_obj->[op.name]
([for a in op.args][if-index a is last][a.corba.var_name][else][a.corba.var_name], [endif][endfor]);

    // Convert CORBA to Local.
    // (The direction of the argument is 'out' or 'inout'.)
[for a in op.args][if a.local.direction is "in"][else]
[if-any a.corba.is_primitive]
    [a.local.arg_name] = [a.corba.var_name];
[else]
    corba_to_local([a.corba.var_name], [a.local.arg_name]);
[endif]
[endif][endfor]

    // Generate the return value.
[if op.return.local.tk is "tk_void"][else]
[if op.return.local.tk is "tk_objref"]
    corba_to_local(corba_ret, local_ret);
[elif op.return.local.tk is "tk_enum"]
    corba_to_local(corba_ret, local_ret);
[else]
[if-any op.return.corba.is_primitive]
    local_ret = corba_ret;
[else]
[if op.return.corba.deref_tk is "tk_long"]
    local_ret = corba_ret;
[elif op.return.corba.deref_tk is "tk_string"]
    corba_to_local(corba_ret, local_ret);
[else]
    corba_to_local(*corba_ret, local_ret);
[endif]
[endif]
[endif]
[if op.return.corba.tk is "tk_any"]
    delete corba_ret; 
[elif op.return.corba.tk is "tk_struct"]
    delete corba_ret; 
[elif op.return.corba.tk is "tk_string"]
    ::CORBA::string_free(corba_ret); 
[elif op.return.local.tk is "tk_objref"]
    ::CORBA::release(corba_ret);
[elif op.return.corba.tk is "tk_alias"]
[if op.return.corba.deref_tk is "tk_long"]
[elif op.return.corba.deref_tk is "tk_string"]
    ::CORBA::string_free(corba_ret); 
[else]
    delete corba_ret; 
[endif]
[endif]
    return local_ret;
[endif]
  }
[endfor]

[for ns in local.adapter_ns]
}; // namespace [ns] 
[endfor]

extern "C"
{
  void [local.adapter_name]CORBAInit(coil::Properties& prop)
  {
    doil::CORBA::CORBAManager& mgr(doil::CORBA::CORBAManager::instance());
    mgr.registerAdapterFactory("[local.adapter_name]",
                        doil::New< [local.adapter_name_fq] >,
                        doil::Delete< [local.adapter_name_fq] >);
  }
};

"""





proxy_h = """\
// -*- C++ -*-
/*!
 * @file [local.proxy_h] 
 * @brief [local.proxy_name] CORBA proxy for doil
 * @date $Date$
 * @author This file was automatically generated from [idl_fname] 
 *         by omniidl/doil backend
 *
 * $Id$
 */
#ifndef [local.proxy_include_guard] 
#define [local.proxy_include_guard] 

#include <coil/Properties.h>
#include <coil/Mutex.h>
#include <coil/Guard.h>
#include <doil/corba/CORBAManager.h>
#include <doil/corba/CORBAProxyBase.h>
#include <[local.iface_h]>
[for inc in inherits]
#include <[inc.local.proxy_h]>
[endfor]
[for inc in include_h]
#include <[inc]>
[endfor]


[for ns in local.proxy_ns]
namespace [ns] 
{
[endfor]

  class [local.proxy_name] 
  : public virtual ::doil::CORBA::CORBAProxyBase,
[for inc in inherits]
    public virtual [inc.local.proxy_name_fq],
[endfor]
    public virtual [local.iface_name_fq]

  {
    typedef coil::Mutex Mutex;
    typedef coil::Guard<Mutex> Guard;
  public:
    [local.proxy_name](::CORBA::Object_ptr obj);
    virtual ~[local.proxy_name]();

[for op in operations]
    virtual [op.return.local.retn_type] [op.name]
([for a in op.args]
[if-index a is last][a.local.arg_type] [a.local.arg_name]
[else][a.local.arg_type] [a.local.arg_name], [endif]
[endfor])
      throw ([for raise in op.raises]
[if-index raise is first]
[raise.local.name_fq]
[else]
,
             [raise.local.name_fq]
[endif]
[endfor]
);


[endfor]

    const char* id() {return "[corba.idl_name]";}
    const char* name() {return m_name.c_str();}
    void incRef()
    {
      Guard guard(m_refcountMutex);
      ++m_refcount;
    }
    void decRef()
    {
      Guard guard(m_refcountMutex);
      --m_refcount;
      if (m_refcount == 0)
        delete this;
    }

  private:
    [corba.name_fq]_ptr m_obj;
  private:
    std::string m_name;
    Mutex m_refcountMutex;
    int m_refcount;
  };

[for ns in local.proxy_ns]
}; // namespace [ns] 
[endfor]

#ifndef [local.servant_include_guard] 


#endif // [local.servant_include_guard]


extern "C"
{
  void [local.proxy_name]CORBAInit(coil::Properties& prop);
};

#endif // [local.proxy_include_guard]

"""


proxy_cpp = """\
// -*- C++ -*-
/*!
 * @file [local.proxy_cpp] 
 * @brief [local.iface_name] CORBA proxy for doil
 * @date $Date$
 * @author This file was automatically generated form [idl_fname] 
 *         by omniidl/doil backend
 *
 * $Id$
 */

#include <doil/corba/CORBAManager.h>
#include <[local.iface_h_path]>
#include <[local.proxy_h_path]>
#include <[typeconv_h_path]>
#include <doil/corba/BasicTypeConversion.h>

[for ns in local.proxy_ns]
namespace [ns] 
{
[endfor]
  /*!
   * @brief ctor
   */ 
  [local.proxy_name]::[local.proxy_name](::CORBA::Object_ptr obj)
   : m_obj([corba.name_fq]::_nil()),
     m_refcount(1)[for inc in inherits],
     [inc.local.proxy_name_fq](obj)[endfor]
//   : m_obj([corba.name_fq]::_nil())[for inc in inherits],
//     [inc.local.proxy_name_fq](obj)[endfor]

  {
    m_obj = [corba.name_fq]::_narrow(obj);
    if (::CORBA::is_nil(m_obj)) throw std::bad_alloc();
    m_obj = [corba.name_fq]::_duplicate(m_obj);
  }

  /*!
   * @brief dtor
   */ 
  [local.proxy_name]::~[local.proxy_name]()
  {
    ::CORBA::release(m_obj);
  }

  [for op in operations]

  /*!
   * @brief [op.name] 
   */ 
  [op.return.local.retn_type] [local.proxy_name]::[op.name]
([for a in op.args]
[if-index a is last][a.local.arg_type] [a.local.arg_name]
[else][a.local.arg_type] [a.local.arg_name], [endif]
[endfor])
      throw ([for raise in op.raises]
[if-index raise is first]
[raise.local.name_fq]
[else]
,
             [raise.local.name_fq]
[endif]
[endfor]
)
  {
    // Convert Local to CORBA.
    // (The direction of the argument is 'in' or 'inout'.)
[for a in op.args]
    [a.corba.base_type] [a.corba.var_name];
[endfor]
[for a in op.args][if a.local.direction is "out"][else]
[if-any a.corba.is_primitive]
//    [a.corba.var_name] = [a.local.arg_name];
    local_to_corba([a.local.arg_name], [a.corba.var_name]);
[else]
[if a.local.tk is "tk_objref"]
    local_to_corba(const_cast< [a.local.var_type] >([a.local.arg_name]), [a.corba.var_name]);
[else]
    local_to_corba([a.local.arg_name], [a.corba.var_name]);
[endif]
[endif]
[endif][endfor]

    // Execute the method. 
[if op.return.local.tk is "tk_void"][else]
    [op.return.corba.retn_type] corba_ret;
    [op.return.local.retn_type] local_ret;
    corba_ret = [endif]
m_obj->[op.name]
([for a in op.args][if-index a is last][a.corba.var_name][else][a.corba.var_name], [endif][endfor]);

    // Convert CORBA to Local.
    // (The direction of the argument is 'out' or 'inout'.)
[for a in op.args][if a.local.direction is "in"][else]
[if-any a.corba.is_primitive]
//    [a.local.arg_name] = [a.corba.var_name];
    corba_to_local([a.corba.var_name], [a.local.arg_name]);
[else]
    corba_to_local([a.corba.var_name], [a.local.arg_name]);
[endif]
[endif][endfor]

    // Generate the return value.
[if op.return.local.tk is "tk_void"][else]
[if op.return.local.tk is "tk_objref"]
    corba_to_local(corba_ret, local_ret);
[elif op.return.local.tk is "tk_enum"]
    corba_to_local(corba_ret, local_ret);
[else]
[if-any op.return.corba.is_primitive]
//    local_ret = corba_ret;
    corba_to_local(corba_ret, local_ret);
[else]
[if op.return.corba.deref_tk is "tk_long"]
//    local_ret = corba_ret;
    corba_to_local(corba_ret, local_ret);
[elif op.return.corba.deref_tk is "tk_string"]
    corba_to_local(corba_ret, local_ret);
[else]
    corba_to_local(*corba_ret, local_ret);
[endif]
[endif]
[endif]
[if op.return.corba.tk is "tk_any"]
    delete corba_ret; 
[elif op.return.corba.tk is "tk_struct"]
    delete corba_ret; 
[elif op.return.corba.tk is "tk_string"]
    ::CORBA::string_free(corba_ret); 
[elif op.return.local.tk is "tk_objref"]
    ::CORBA::release(corba_ret);
[elif op.return.corba.tk is "tk_alias"]
[if op.return.corba.deref_tk is "tk_long"]
[elif op.return.corba.deref_tk is "tk_string"]
    ::CORBA::string_free(corba_ret); 
[else]
    delete corba_ret; 
[endif]
[endif]
    return local_ret;
[endif]
  }
[endfor]

[for ns in local.proxy_ns]
}; // namespace [ns] 
[endfor]

extern "C"
{
  void [local.proxy_name]CORBAInit(coil::Properties& prop)
  {
    doil::CORBA::CORBAManager& mgr(doil::CORBA::CORBAManager::instance());
    mgr.registerProxyFactory("[local.proxy_name]",
                        doil::New< [local.proxy_name_fq] >,
                        doil::Delete< [local.proxy_name_fq] >);
  }
};

"""

