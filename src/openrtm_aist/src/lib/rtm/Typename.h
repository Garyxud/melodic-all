// -*- C++ -*-
/*!
 * @file Typename.h
 * @brief Typename function
 * @date $Date: 2007-12-31 03:08:03 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2003-2009
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id: InPort.h 1225 2009-02-28 02:30:25Z n-ando $
 *
 */

#ifndef RTC_TYPENAME_H
#define RTC_TYPENAME_H

#include <iostream>
#include <rtm/RTC.h>

namespace CORBA_Util
{
  /*!
   * @brief has nil helper
   *
   * typename T::_ptr_type (*)(void) matches _nil() static member function
   *
   */
  template <class T, typename T::_ptr_type (*)(void)>
  struct has_nil_helper
  {
    typedef void type;
  };
  
  /*!
   * @brief has nil impl: void case
   */
  template <class T, class U = void>
  struct has_nil_impl
  {
    static const bool value = false;
  };
  
  /*!
   * @brief has nil impl: valid case
   *
   * This class is instantiated in case of T has _nil() member function
   *
   */
  template <class T>
  struct has_nil_impl<T, typename has_nil_helper<T, &T::_nil>::type>
  {
    static const bool value = true;
  };
  
  /*!
   * @brief has nil traits class template
   *
   * T has _nil() static function -> value = true
   * T has no _nil() static function -> value = false
   * 
   */
  template <class T>
  struct has_nil : has_nil_impl<T>
  {
  };
  
  /*!
   * @brief is corba object traits class
   *
   * T is CORBA object     -> value = true
   * T is not CORBA object -> value = false
   * 
   */
  template <typename T>
  struct is_corba_object
  {
    static const bool value = has_nil<T>::value;
  };
  
  /*!
   * @brief typecode class template
   *
   * typecode class provides typecode object from type not instance
   * By using this class, you can get repository id and name in inline code.
   * 
   */
  template <bool cond, class T>
  class typecode;
  
  template <class T>
  class typecode<true, T>
  {
  public:
    static const char* id()
    {
      CORBA::Any any_var;
      typename T::_ptr_type tmp_var;
      tmp_var = T::_nil();
      any_var <<= tmp_var;
      return any_var.type()->id();
    }
    static const char* name()
    {
      CORBA::Any any_var;
      typename T::_ptr_type tmp_var;
      tmp_var = T::_nil();
      any_var <<= tmp_var;
      return any_var.type()->name();
    }
  };
  
  template <class T>
  class typecode<false, T>
  {
  public:
    static const char* id()
    {
      CORBA::Any any_var;
      T tmp_var;
      any_var <<= tmp_var;
      return any_var.type()->id();
    }
    static const char* name()
    {
      CORBA::Any any_var;
      T tmp_var;
      any_var <<= tmp_var;
      return any_var.type()->name();
    }
  };
  
  
  /*!
   * @if jp
   * @brief CORBA型のタイプ名を文字列で取得する
   *
   * CORBA IDLによって定義されたクラスまたは構造体などの型名を取得する。
   * テンプレート引数には、タイプコードが生成される型を与えることができる。
   *
   * <pre> 
   * std::cout << toTypename<RTC::TimedFloat>() << std::endl;
   * std::cout << toTypename<RTC::RTObject>() << std::endl;
   * </pre>
   * を実行すると結果は
   * <pre>
   * TimedFloat
   * RTObject
   * </pre>
   * となる。
   * 
   * @else
   * @brief Getting CORBA defined type as characters
   *
   * This function returns the type name if class or struct type that is
   * defined CORBA IDL. The template parameter can be a type which has
   * type code.
   *
   * <pre> 
   * std::cout << toTypename<RTC::TimedFloat>() << std::endl;
   * std::cout << toTypename<RTC::RTObject>() << std::endl;
   * </pre>
   * gives the following results.
   * <pre>
   * TimedFloat
   * RTObject
   * </pre>
   *
   * @endif
   */
  template <class T>
  const char* toTypename()
  {
    return typecode<is_corba_object<T>::value, T>::name();
  }
  
  template <class T>
  const char* toTypenameOfStruct()
  {
    return typecode<false, T>::name();
  }
  
  template <class T>
  const char* toTypenameOfObject()
  {
    return typecode<true, T>::name();
  }
  
  /*!
   * @if jp
   * @brief CORBA型のリポジトリIDを文字列で取得する
   *
   * CORBA IDLによって定義されたクラスまたは構造体などのリポジトリIDを取得する。
   * テンプレート引数には、タイプコードが生成される型を与えることができる。
   *
   * <pre> 
   * std::cout << toRepositoryId<RTC::TimedFloat>() << std::endl;
   * std::cout << toRepositoryId<RTC::RTObject>() << std::endl;
   * </pre>
   * を実行すると結果は
   * <pre>
   * IDL:RTC/TimedFloat:1.0
   * IDL:omg.org/RTC/RTObject:1.0
   * </pre>
   * となる。
   * 
   * @else
   * @brief Getting CORBA defined type as characters
   *
   * This function returns the type name if class or struct type that is
   * defined CORBA IDL. The template parameter can be a type which has
   * type code.
   *
   * <pre> 
   * std::cout << toRepositoryId<RTC::TimedFloat>() << std::endl;
   * std::cout << toRepositoryId<RTC::RTObject>() << std::endl;
   * </pre>
   * gives the following results.
   * <pre>
   * IDL:RTC/TimedFloat:1.0
   * IDL:omg.org/RTC/RTObject:1.0
   * </pre>
   *
   * @endif
   */
  template <class T>
  const char* toRepositoryId()
  {
    return typecode<is_corba_object<T>::value, T>::id();
  }

  template <class T>
  const char* toRepositoryIdOfStruct()
  {
    return typecode<false, T>::id();
  }

  template <class T>
  const char* toRepositoryIdOfObject()
  {
    return typecode<true, T>::id();
  }
  
}; // namespace CORBA_Util

template <class T>
const char* toTypename()
{
  std::cerr << "toTypename() is obsolete." << std::endl;
  std::cerr << "Please use CORBA_Util::toTypename() instead." << std::endl;
  return CORBA_Util::typecode<CORBA_Util::is_corba_object<T>::value, T>::name();
}

#endif // RTC_TYPENAME_H
