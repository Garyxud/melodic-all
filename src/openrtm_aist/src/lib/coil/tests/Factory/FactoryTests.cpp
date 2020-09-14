// -*- C++ -*-
/*!
 * @file   FactoryTests.cpp
 * @brief  Factory test class
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

#ifndef Factory_cpp
#define Factory_cpp

#define LIBRARY_EXPORTS
#include <string>

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/Factory.h>
#include <coil/DynamicLib.h>

#include "MyFactory.h"

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  const char * LibName = "FactoryTest_libPluginC.dll";
#else 
  const char * LibName = "./.libs/libPluginC.so";
#endif /* Windows */

//class Base
//{
//public:
//  virtual ~Base(){}
//  virtual std::string name() = 0;
//};

class A
  : public Base
{
public:
  virtual std::string name()
  {
    return "A";
  }
};

class B
  : public Base
{
public:
  virtual std::string name()
  {
    return "B";
  }
};

//typedef coil::GlobalFactory<Base> g_factory;

/*!
 * @class FactoryTests class
 * @brief Factory test
 */
namespace Factory
{
  class FactoryTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(FactoryTests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST(test_case1);
    CPPUNIT_TEST(test_case2);
    CPPUNIT_TEST_SUITE_END();
  
  private:
    coil::Factory<Base> m_factory;
  public:
  
    /*!
     * @brief Constructor
     */
    FactoryTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~FactoryTests()
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
      std::string Astr("A");
      std::string Bstr("B");
      m_factory.addFactory("A",
                           coil::Creator<Base, A>,
                           coil::Destructor<Base, A>);
      m_factory.addFactory("B",
                           coil::Creator<Base, B>,
                           coil::Destructor<Base, B>);



      for (int i(0); i < 1000; ++i)
        {
          Base* a = m_factory.createObject("A");
          CPPUNIT_ASSERT(a->name() == "A");

          Base* b = m_factory.createObject("B");
          CPPUNIT_ASSERT(b->name() == "B");
          
          m_factory.deleteObject(a);
          CPPUNIT_ASSERT(a == 0);

          m_factory.deleteObject(b);
          CPPUNIT_ASSERT(b == 0);
        }
    }

    void test_case1()
    {
      std::string Astr("A");
      std::string Bstr("B");
      MyFactory::instance().addFactory("A",
                           coil::Creator<Base, A>,
                           coil::Destructor<Base, A>);
      MyFactory::instance().addFactory("B",
                           coil::Creator<Base, B>,
                           coil::Destructor<Base, B>);

      coil::DynamicLib dl;
      if (dl.open(LibName) < 0)
        {
          std::cout << "dl.open error" << std::endl;
        }

      typedef void (*InitFunc)();
      InitFunc init_func = (InitFunc)dl.symbol("PluginCInit");
      if (init_func == NULL)
        {
          std::cout << "synbol is NULL" << std::endl;
        }
      (*init_func)();

      if (MyFactory::instance().hasFactory("C"))
        {
          std::cout << "Factory C exists" << std::endl;
        }
      else
        {
          std::cout << "Factory C does not exist" << std::endl;
        }

      std::vector<std::string> ids(MyFactory::instance().getIdentifiers());
      std::cout << "IDs -> ";
      for (int i(0), len(ids.size()); i < len; ++i)
        {
          std::cout << ids[i] << ", ";
        }
      MyFactory& g(MyFactory::instance());
      for (int i(0); i < 1000; ++i)
        {
          Base* a = MyFactory::instance().createObject("A");
          CPPUNIT_ASSERT(a->name() == "A");

          Base* b = MyFactory::instance().createObject("B");
          CPPUNIT_ASSERT(b->name() == "B");
                    
          Base* c = MyFactory::instance().createObject("C");
          CPPUNIT_ASSERT(c->name() == "C");

          MyFactory::instance().deleteObject(a);
          CPPUNIT_ASSERT(a == 0);

          MyFactory::instance().deleteObject(b);
          CPPUNIT_ASSERT(b == 0);

          MyFactory::instance().deleteObject(c);
          CPPUNIT_ASSERT(c == 0);

          CPPUNIT_ASSERT(&g == &MyFactory::instance());
        }
    }

    /*!
     *
     */
    void test_case2()
    {
      std::string Astr("A");
      std::string Bstr("B");
      MyFactory::instance().addFactory("A",
                           coil::Creator<Base, A>,
                           coil::Destructor<Base, A>);
      //Factory A exists//
      CPPUNIT_ASSERT(MyFactory::instance().hasFactory("A"));

      MyFactory::instance().addFactory("B",
                           coil::Creator<Base, B>,
                           coil::Destructor<Base, B>);
      //Factory B exists//
      CPPUNIT_ASSERT(MyFactory::instance().hasFactory("B"));

      coil::DynamicLib dl;
      if (dl.open(LibName) < 0)
        {
          std::cout << "dl.open error" << std::endl;
        }

      typedef void (*InitFunc)();
      InitFunc init_func = (InitFunc)dl.symbol("PluginCInit");
      if (init_func == NULL)
        {
          std::cout << "synbol is NULL" << std::endl;
        }
      (*init_func)();

      //Factory C exists//
      CPPUNIT_ASSERT(MyFactory::instance().hasFactory("C"));

      std::vector<std::string> ids(MyFactory::instance().getIdentifiers());
      for (int i(0), len(ids.size()); i < len; ++i)
        {
          MyFactory::ReturnCode ret;
          ret = MyFactory::instance().removeFactory(ids[i]);
          CPPUNIT_ASSERT(ret == MyFactory::FACTORY_OK);
          CPPUNIT_ASSERT(!MyFactory::instance().hasFactory(ids[i]));
          ret = MyFactory::instance().removeFactory(ids[i]);
          CPPUNIT_ASSERT(ret == MyFactory::NOT_FOUND);
        }

    }
  };
}; // namespace Factory

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Factory::FactoryTests);

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
#endif // Factory_cpp
