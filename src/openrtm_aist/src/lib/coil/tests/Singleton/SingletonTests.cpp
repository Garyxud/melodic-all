// -*- C++ -*-
/*!
 * @file   SingletonTests.cpp
 * @brief  Singleton test class
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

#ifndef Singleton_cpp
#define Singleton_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <iostream>

#include <coil/Singleton.h>
#include <coil/Task.h>
#include <coil/Time.h>

class A
{
public:
  A()
  {
    std::cout << "A ctor entered" << std::endl;
    coil::usleep(100000);
    ++count;
    std::cout << "A ctor done" << std::endl;
  }
  int get_count()
  {
    return count;
  }
private:
  static int count;
};

int A::count = 0;

class B
{
public:
  B()
  {
    std::cout << "B ctor entered" << std::endl;
    coil::usleep(100000);
    ++count;
    std::cout << "B ctor done" << std::endl;
  }
  int get_count()
  {
    return count;
  }
private:
  static int count;
};

int B::count = 0;

class C
  : public coil::Singleton<C>
{
public:
  int get_count()
  {
    return count;
  }
private:
  static int count;
  C()
  {
    std::cout << "C ctor entered" << std::endl;
    coil::usleep(100000);
    ++count;
    std::cout << "C ctor done" << std::endl;
  }
  friend class coil::Singleton<C>;
};

int C::count = 0;

class Threaded
  : public coil::Task
{
public:
  typedef coil::Singleton<A> A_;
  typedef coil::Singleton<B> B_;
  virtual int svc()
  {
    std::cout << "thread entered" << std::endl;
    for (int i(0); i < 10; ++i)
      {
        std::cout << "Getting A in thread" << std::endl;;
        A& a(A_::instance());
        std::cout << "Getting A done in thread" << std::endl;;
        B& b(B_::instance());
        CPPUNIT_ASSERT(a.get_count() == 1);
        CPPUNIT_ASSERT(b.get_count() == 1);
        coil::usleep(1000);
      }
    return 0;
  }
};

/*!
 * @class SingletonTests class
 * @brief Singleton test
 */
namespace Singleton
{
  class SingletonTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(SingletonTests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
    typedef coil::Singleton<A> A_;
    typedef coil::Singleton<B> B_;
    /*!
     * @brief Constructor
     */
    SingletonTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~SingletonTests()
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
    void test_case0()
    {
      Threaded t;
      t.activate();
      for (int i(0); i < 10; ++i)
        {
          std::cout << "Getting A in main" << std::endl;
          A& a(A_::instance());
          std::cout << "Getting A done in main" << std::endl;

          std::cout << "Getting B in main" << std::endl;
          B& b(B_::instance());
          std::cout << "Getting B done in main" << std::endl;
          
          std::cout << "Getting C in main" << std::endl;
          C& c(C::instance());
          std::cout << "Getting C done in main" << std::endl;

          CPPUNIT_ASSERT(a.get_count() == 1);
          CPPUNIT_ASSERT(b.get_count() == 1);
          CPPUNIT_ASSERT(c.get_count() == 1);
          coil::usleep(1000);
        }



    }
  };
}; // namespace Singleton

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Singleton::SingletonTests);

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
#endif // Singleton_cpp
