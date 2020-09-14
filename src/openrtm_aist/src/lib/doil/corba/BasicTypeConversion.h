// -*- C++ -*-
/*!
 * @file  BasicTypeConversion.h
 * @brief doil CORBA basic type conversion functions
 * @date  $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef DOIL_CORBA_BASICTYPECONVERSION_H
#define DOIL_CORBA_BASICTYPECONVERSION_H

#include <coil/stringutil.h>
#include <doil/corba/CORBA.h>

//-------------------------------------------------------------
// local -> corba
inline bool local_to_corba(const short int _from, ::CORBA::Short& _to)
{
  _to = _from;
  return true;
}

inline bool local_to_corba(const unsigned short _from, ::CORBA::UShort& _to)
{
  _to = _from;
  return true;
}

inline bool local_to_corba(const long int _from, ::CORBA::Long& _to)
{
  _to = _from;
  return true;
}

inline bool local_to_corba(const unsigned long int _from, ::CORBA::ULong& _to)
{
  _to = _from;
  return true;
}

inline bool local_to_corba(const float _from, ::CORBA::Float& _to)
{
  _to = _from;
  return true;
}

inline bool local_to_corba(const double _from, ::CORBA::Double& _to)
{
  _to = _from;
  return true;
}

inline bool local_to_corba(const char _from, ::CORBA::Char& _to)
{
  _to = _from;
  return true;
}

inline bool local_to_corba(const bool _from, ::CORBA::Boolean& _to)
{
  _to = _from;
  return true;
}

inline bool local_to_corba(const unsigned char _from, ::CORBA::Octet& _to)
{
  _to = _from;
  return true;
}

#if   defined ORB_IS_TAO
#elif defined ORB_IS_OMNIORB
inline bool local_to_corba(const std::string& _from, ::CORBA::String_member& _to)
{
  _to.out() = ::CORBA::string_dup(_from.c_str());
  return true;
}


inline bool local_to_corba(const std::string& _from, _CORBA_String_element::_CORBA_String_element _to)
{
  _to.out() = ::CORBA::string_dup(_from.c_str());
  return true;
}
#elif defined ORB_IS_MICO
#elif defined ORB_IS_ORBIT2
#endif
inline bool local_to_corba(const ::std::string& _from, char*& _to)
{
  _to = ::CORBA::string_dup(_from.c_str());
  return true;
}

inline bool local_to_corba(const std::string& _from, ::CORBA::Any& _to)
{
  _to <<= ::CORBA::Any::from_string(_from.c_str(), _from.size());
  return true;
}

inline bool local_to_corba(const std::string& _from, CORBA::TypeCode* _to)
{
  //    if (_from == "tk_null")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_null);
  //    else if (_from == "tk_void")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_void);
  //    else if (_from == "tk_short")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_short);
  //    else if (_from == "tk_long")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_long);
  //    else if (_from == "tk_ushort")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_ushort);
  //    else if (_from == "tk_ulong")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_ulong);
  //    else if (_from == "tk_float")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_float);
  //    else if (_from == "tk_double")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_double);
  //    else if (_from == "tk_boolean")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_boolean);
  //    else if (_from == "tk_char")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_char);
  //    else if (_from == "tk_octet")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_octet);
  //    else if (_from == "tk_any")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_any);
  //    else if (_from == "tk_TypeCode")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_TypeCode);
  //    else if (_from == "tk_Principal")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_Principal);
  //    else if (_from == "tk_longlong")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_longlong);
  //    else if (_from == "tk_ulonglong")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_ulonglong);
  //    else if (_from == "tk_longdouble")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_longdouble);
  //    else if (_from == "tk_wchar")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_wchar);
  //    else if (_from == "tk_fixed")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_fixed);
  //    else if (_from == "tk_value")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_value);
  //    else if (_from == "tk_value_box")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_value_box);
  //    else if (_from == "native")
  //      _to = new ::CORBA::TypeCode(CORBA::native);
  //    else if (_from == "tk_abstract_interface")
  //      _to = new ::CORBA::TypeCode(CORBA::tk_abstract_interface);
  //    else
  //      return false;
  return true;
}

//-------------------------------------------------------------
// corba -> local
inline bool corba_to_local(const ::CORBA::Short _from, short& _to)
{
  _to = _from;
  return true;
}

inline bool corba_to_local(const ::CORBA::UShort _from, unsigned short& _to)
{
  _to = _from;
  return true;
}

inline bool corba_to_local(const ::CORBA::Long _from, int& _to)
{
  _to = _from;
  return true;
}

inline bool corba_to_local(const ::CORBA::ULong _from, unsigned int& _to)
{
  _to = _from;
  return true;
}

inline bool corba_to_local(const ::CORBA::Float _from, float& _to)
{
  _to = _from;
  return true;
}

inline bool corba_to_local(const ::CORBA::Double _from, double& _to)
{
  _to = _from;
  return true;
}

inline bool corba_to_local(const ::CORBA::Char _from, char& _to)
{
  _to = _from;
  return true;
}

inline bool corba_to_local(const ::CORBA::Boolean _from, unsigned char& _to)
{
  _to = _from;
  return true;
}

inline bool corba_to_local(const ::CORBA::Octet _from, unsigned char& _to)
{
  _to = _from;
  return true;
}

inline bool corba_to_local(const ::CORBA::Boolean _from, bool& _to)
{
  _to = _from;
  return true;
}

inline bool corba_to_local(const char* _from, std::string& _to)
{
  _to = _from;
  return true;
}

//  inline bool corba_to_local(char* _from, std::string& _to)
//  {
//    _to = _from;
//  }

inline bool corba_to_local(const ::CORBA::Any& _from, std::string& _to)
{
  const char* str;
  if (_from >>= str)
    {
      _to = str;
      return true;
    }
  ::CORBA::Short short_var;
  if (_from >>= short_var)
    {
      _to = ::coil::otos(short_var);
      return true;
    }
  ::CORBA::UShort ushort_var;
  if (_from >>= ushort_var)
    {
      _to = ::coil::otos(ushort_var);
      return true;
    }
  ::CORBA::Long long_var;
  if (_from >>= long_var)
    {
      _to = ::coil::otos(long_var);
      return true;
    }
  ::CORBA::ULong ulong_var;
  if (_from >>= ulong_var)
    {
      _to = ::coil::otos(ulong_var);
      return true;
    }
  ::CORBA::Float float_var;
  if (_from >>= float_var)
    {
      _to = ::coil::otos(float_var);
      return true;
    }
  ::CORBA::Double double_var;
  if (_from >>= double_var)
    {
      _to = ::coil::otos(double_var);
      return true;
    }
}

inline bool
corba_to_local(const CORBA::TypeCode_member& _from, std::string& _to)
{
  try
    {
      _to = _from->name();
      return true;
    }
  catch ( ::CORBA::TypeCode::BadKind& e )
    {
      return false;
    }
  return true;
}


#endif // DOIL_CORBA_UTIL_H
