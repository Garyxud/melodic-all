// -*-C++-*-
/*!
 * @file  AutoTestServiceSVC_impl.cpp
 * @brief Service implementation code of AutoTestService.idl
 *
 */

#include "AutoTestServiceSVC_impl.h"

/*
 * Example implementational code for IDL interface AutoTest::MyService
 */
MyServiceSVC_impl::MyServiceSVC_impl() : 
  m_msg(""),
  m_isNew(false)
{
  // Please add extra constructor code here.
}


MyServiceSVC_impl::~MyServiceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
char* MyServiceSVC_impl::echo(const char* msg)
{
  // Please insert your code here and remove the following warning pragma
  m_msg = msg;
  if (m_isNew)
    std::cout << "echo's message was overwritten !!!" << std::endl;
  m_isNew = true;
  return CORBA::string_dup(msg);
}

AutoTest::EchoList* MyServiceSVC_impl::get_echo_history()
{
  // Please insert your code here and remove the following warning pragma
  return 0;
}

void MyServiceSVC_impl::set_value(CORBA::Float value)
{
  // Please insert your code here and remove the following warning pragma
}

CORBA::Float MyServiceSVC_impl::get_value()
{
  // Please insert your code here and remove the following warning pragma
  return 0;
}

AutoTest::ValueList* MyServiceSVC_impl::get_value_history()
{
  // Please insert your code here and remove the following warning pragma
  return 0;
}



// End of example implementational code



