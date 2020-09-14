// -*-C++-*-
/*!
 * @file  AutoTestServiceSVC_impl.h
 * @brief Service implementation header of AutoTestService.idl
 *
 */

#include <iostream>
#include "AutoTestServiceSkel.h"

#ifndef AUTOTESTSERVICESVC_IMPL_H
#define AUTOTESTSERVICESVC_IMPL_H
 
/*!
 * @class MyServiceSVC_impl
 * Example class implementing IDL interface AutoTest::MyService
 */
class MyServiceSVC_impl
 : public virtual POA_AutoTest::MyService,
   public virtual PortableServer::RefCountServantBase
{
private:
  // Make sure all instances are built on the heap by making the
  // destructor non-public
  //virtual ~MyServiceSVC_impl();
  
public:
  /*!
   * @brief standard constructor
   */
  MyServiceSVC_impl();
  /*!
   * @brief destructor
   */
  virtual ~MyServiceSVC_impl();

  // attributes and operations
  char* echo(const char* msg);
  AutoTest::EchoList* get_echo_history();
  void set_value(CORBA::Float value);
  CORBA::Float get_value();
  AutoTest::ValueList* get_value_history();

  void reset_message()
  {
    m_isNew = false;
    m_msg ="";
  }

  std::string get_echo_message() {
    if (m_isNew) {
      m_isNew = false;
      return m_msg;
    }
    return "";
  }

private:
  std::string m_msg;
  bool m_isNew;
};



#endif // AUTOTESTSERVICESVC_IMPL_H


