// -*-C++-*-
/*!
 * @file  MyServiceSVC_impl.h
 * @brief Service implementation header of MyService.idl
 *
 */

#include "MyServiceSkel.h"

#ifndef MYSERVICESVC_IMPL_H
#define MYSERVICESVC_IMPL_H
 
/*
 * Example class implementing IDL interface MyService
 */
class MyServiceSVC_impl
  : public virtual POA_SimpleService::MyService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~MyServiceSVC_impl();

 public:
   // standard constructor
   MyServiceSVC_impl();
   virtual ~MyServiceSVC_impl();

   // attributes and operations
   char* echo(const char* msg)
     throw (CORBA::SystemException);
  SimpleService::EchoList* get_echo_history()
     throw (CORBA::SystemException);
   void set_value(CORBA::Float value)
     throw (CORBA::SystemException);
   CORBA::Float get_value()
     throw (CORBA::SystemException);
  SimpleService::ValueList* get_value_history()
     throw (CORBA::SystemException);

private:
  CORBA::Float m_value;
  SimpleService::EchoList m_echoList;
  SimpleService::ValueList m_valueList;
};



#endif // MYSERVICESVC_IMPL_H


