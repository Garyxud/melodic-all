// -*- C++ -*-
/*!
 * @file   AsyncInvokerTests.cpp
 * @brief  AsyncInvoker test class
 * @date   $Date$ 
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id$ 
 *
 */

/*
 * $Log$
 *
 */

#ifndef AsyncInvoker_cpp
#define AsyncInvoker_cpp

#include <iostream>
#include <string>
#include <functional>

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/Async.h>
#include <coil/Time.h>

class A
{
public:
  A()
    : m_hoge(false), m_munya(false), m_addone(false)
  {}

  void hoge()
  {
    for (int i(0); i < 4; ++i)
      {
        std::cout << "," << std::flush;
        coil::usleep(500000);
      }
    m_hoge = true;
  }

  bool hoge_invoked() { return m_hoge; }

  void munya(const char* msg)
  {
    for (int i(0); i < 4; ++i)
      {
        std::cout << "," << std::flush;
        coil::usleep(500000);
      }
    m_munya = true;
  }

  bool munya_invoked() { return m_munya; }

  int add_one(int val)
  {
    for (int i(0); i < 4; ++i)
      {
        std::cout << "," << std::flush;
        coil::usleep(500000);
      }
    m_addone = true;
    return val + 1;
  }

  bool add_one_invoked() { return m_addone; }

private:
  bool m_hoge;
  bool m_munya;
  bool m_addone;
};

class add_one_functor
{
  int m_val, m_ret;
public:
  add_one_functor(int val) : m_val(val), m_ret(0)
  {
  }
  void operator()(A* obj)
  {
    m_ret = obj->add_one(m_val);
  }
  int get_ret()
  {
    return m_ret;
  }
private:
  add_one_functor(const add_one_functor& x);
};


/*!
 * @class AsyncInvokerTests class
 * @brief AsyncInvoker test
 */
namespace AsyncInvoker
{
  class AsyncInvokerTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(AsyncInvokerTests);
    CPPUNIT_TEST(test_memfun);
    CPPUNIT_TEST(test_memfun_oneway);
    CPPUNIT_TEST(test_bind2nd);
    CPPUNIT_TEST(test_bind2nd_oneway);
    CPPUNIT_TEST(test_myfunctor);
    CPPUNIT_TEST(test_myfunctor_oneway);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    AsyncInvokerTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~AsyncInvokerTests()
    {
    }
  
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
    }
    
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    { 
    }
  
    /* test case */
    void test_memfun()
    {
      A a;
      coil::Async* invoker(coil::AsyncInvoker(&a, std::mem_fun(&A::hoge)));
      
      invoker->invoke();
      CPPUNIT_ASSERT(a.hoge_invoked() == false);
      invoker->wait();
      CPPUNIT_ASSERT(a.hoge_invoked() == true);
    }

    void test_memfun_oneway()
    {
      {
        A a;
        coil::AsyncInvoker(&a, std::mem_fun(&A::hoge), true)->invoke();
        
        CPPUNIT_ASSERT(a.hoge_invoked() == false);
        coil::sleep(3);
        CPPUNIT_ASSERT(a.hoge_invoked() == true);
      }

      {
        A a;
        coil::AsyncInvoker(&a, std::mem_fun(&A::hoge), true)->invoke();
        
        CPPUNIT_ASSERT(a.hoge_invoked() == false);
        coil::sleep(3);
        CPPUNIT_ASSERT(a.hoge_invoked() == true);
      }
    }

    void test_bind2nd()
    {
      A a;
      coil::Async*
        invoker(coil::AsyncInvoker(&a,
                                   std::bind2nd(std::mem_fun(&A::munya),
                                                "daradara")));
      invoker->invoke();
      CPPUNIT_ASSERT(a.munya_invoked() == false);
      invoker->wait();
      CPPUNIT_ASSERT(a.munya_invoked() == true);
    }

    void test_bind2nd_oneway()
    {
      A a;
      coil::AsyncInvoker(&a,
                         std::bind2nd(std::mem_fun(&A::munya),
                                      "daradara"),
                         true)->invoke();
      
      CPPUNIT_ASSERT(a.munya_invoked() == false);
      coil::sleep(3);
      CPPUNIT_ASSERT(a.munya_invoked() == true);
    }

    void test_myfunctor()
    {
      const int val(100);
      A a;
      add_one_functor aof(100);
      coil::Async* invoker(coil::AsyncInvoker(&a, &aof));

      invoker->invoke();
      CPPUNIT_ASSERT(a.add_one_invoked() == false);
      invoker->wait();
      CPPUNIT_ASSERT(a.add_one_invoked() == true);
      CPPUNIT_ASSERT(aof.get_ret() == (val + 1));
    }

    void test_myfunctor_oneway()
    {
      const int val(100);
      A a;
      add_one_functor aof(100);
      coil::AsyncInvoker(&a, &aof)->invoke();

      CPPUNIT_ASSERT(a.add_one_invoked() == false);
      coil::sleep(3);
      CPPUNIT_ASSERT(a.add_one_invoked() == true);
      CPPUNIT_ASSERT(aof.get_ret() == (val + 1));
    }
  };
}; // namespace AsyncInvoker

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(AsyncInvoker::AsyncInvokerTests);

#ifdef LOCAL_MAIN
int main(int argc, char* argv[])
{
    CppUnit::TextUi::TestRunner runner;
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry().makeTest());
    CppUnit::Outputter* outputter = 
      new CppUnit::TextOutputter(&runner.result(), std::cout);
    runner.setOutputter(outputter);
    bool retcode = runner.run();
    return !retcode;
}
#endif // MAIN
#endif // AsyncInvoker_cpp
