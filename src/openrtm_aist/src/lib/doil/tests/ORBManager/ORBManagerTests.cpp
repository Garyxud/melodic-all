// -*- C++ -*-
/*!
 * @file   ORBManagerTests.cpp
 * @brief  ORBManager test class
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

#ifndef ORBManager_cpp
#define ORBManager_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <coil/DynamicLib.h>
#include <doil/ORBManager.h>
#include "EchoImpl.h"
#include "IEcho.h"
#include "EchoCORBASkel.h"
#include "EchoIceSkel.h"
#include <doil/corba/CORBAManager.h>
#include <doil/ice/IceManager.h>
/*!
 * @class ORBManagerTests class
 * @brief ORBManager test
 */
typedef void (*ModuleInitFunc)(coil::Properties&);

namespace ORBManager
{
  class ORBManagerTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ORBManagerTests);
    CPPUNIT_TEST(test_init);
    CPPUNIT_TEST(test_instance);
    CPPUNIT_TEST(test_load_and_add);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    ORBManagerTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~ORBManagerTests()
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
    void test_init()
    {
      coil::Properties prop;
      doil::ORBManager* mgr = doil::ORBManager::init(prop);
      for (int i(0); i < 100; ++i)
        {
          doil::ORBManager* tmp;
          tmp = doil::ORBManager::init(prop);
          CPPUNIT_ASSERT_MESSAGE("ORBManager's pointer should always be same",
                                 mgr == tmp);
          
        }
    }
    /* test case */
    void test_instance()
    {
      coil::Properties prop;
      doil::ORBManager* mgr = &doil::ORBManager::instance();
      for (int i(0); i < 100; ++i)
        {
          doil::ORBManager* tmp;
          tmp = &doil::ORBManager::instance();
          CPPUNIT_ASSERT_MESSAGE("ORBManager's pointer should always be same",
                                 mgr == tmp);
        }
    }
    /* test case */
    void test_load_and_add()
    {
      doil::ORBManager::init(coil::Properties());
      coil::Properties* prop = new coil::Properties();

      // CORBA
      coil::DynamicLib dll0;
      if (dll0.open("../../corba/.libs/libdoilcorba.so",
                    RTLD_LAZY | RTLD_GLOBAL) < 0)
        {
          std::cout << dll0.error() << std::endl;
          CPPUNIT_ASSERT_MESSAGE("doilcorba load faild", false);
        }

      void* func0;
      func0 = dll0.symbol("DoilCORBAInit");
      CPPUNIT_ASSERT_MESSAGE("invalid symbol for corba init", func0 != NULL);
      ModuleInitFunc init_func0;
      init_func0 = (ModuleInitFunc)(func0);
      init_func0(*prop);

      std::vector<doil::IORB*> orbs;
      orbs = doil::ORBManager::instance().getORBs();
      std::cout << "number of ORB: " << orbs.size() << std::endl;
      for (int i(0), len(orbs.size()); i < len; ++i)
        {
          std::cout << "ORB name: " << orbs[i]->name() << std::endl;
        }

      // Ice
      coil::DynamicLib dll1;
      if (dll1.open("../../ice/.libs/libdoilice.so",
                    RTLD_LAZY | RTLD_GLOBAL) < 0)
        {
          std::cout << dll1.error() << std::endl;
          CPPUNIT_ASSERT_MESSAGE("doilice load faild", false);
        }

      void* func1;
      func1 = dll1.symbol("DoilIceInit");
      CPPUNIT_ASSERT_MESSAGE("invalid symbol for ice init", func0 != NULL);
      ModuleInitFunc init_func1;
      init_func1 = (ModuleInitFunc)(func1);
      init_func1(*prop);

      orbs = doil::ORBManager::instance().getORBs();
      std::cout << "number of ORB: " << orbs.size() << std::endl;
      for (int i(0), len(orbs.size()); i < len; ++i)
        {
          std::cout << "ORB name: " << orbs[i]->name() << std::endl;
        }

      // libEchoCORBA.so
      coil::DynamicLib dll2;
      if (dll2.open(".libs/libEchoCORBA.so", RTLD_LAZY | RTLD_GLOBAL) < 0)
        {
          std::cout << dll2.error() << std::endl;
          CPPUNIT_ASSERT_MESSAGE("libEchoCORBA load faild", false);
        }
      ModuleInitFunc init_func2;
      init_func2 = (ModuleInitFunc)dll2.symbol("EchoCORBAInit");
      if (init_func2 == NULL)
        {
          std::cout << dll2.error() << std::endl;
          CPPUNIT_ASSERT_MESSAGE("symbol not found: EchoCORBAInit", false);
        }
      init_func2(*prop);


      // libEchoIce.so
      coil::DynamicLib dll3;
      if (dll3.open(".libs/libEchoIce.so", RTLD_LAZY | RTLD_GLOBAL) < 0)
        {
          std::cout << dll3.error() << std::endl;
          CPPUNIT_ASSERT_MESSAGE("libEchoIce load faild", false);
        }
      ModuleInitFunc init_func3;
      init_func3 = (ModuleInitFunc)dll3.symbol("EchoIceInit");
      if (init_func3 == NULL)
        {
          std::cout << dll3.error() << std::endl;
          CPPUNIT_ASSERT_MESSAGE("symbol not found: EchoIceInit", false);
        }
      init_func3(*prop);

      EchoImpl* ie0 = new EchoImpl();
      if (ie0 == NULL) throw "ErrorE";
      doil::ReturnCodes ret = doil::ORBManager::instance().activateObject(ie0);

      for (int i(0), l(ret.retcodes_.size()); i < l; ++i)
        {
          std::cout << i << ": " << ret.retcodes_[i].key_ << ": ";
          std::cout << doil::return_codes[ret.retcodes_[i].ret_] << std::endl;
        }

      // access corba object
      doil::CORBA::CORBAManager* corba(NULL);
      doil::Ice::IceManager* ice(NULL);
      for (int i(0), l(orbs.size()); i < l; ++i)
        {
          if (strcmp(orbs[i]->name(), "corba") == 0)
            corba = dynamic_cast<doil::CORBA::CORBAManager*>(orbs[i]);
          if (strcmp(orbs[i]->name(), "ice") == 0)
            ice = dynamic_cast<doil::Ice::IceManager*>(orbs[i]);
        }

      CORBA::Object_var obj = corba->toReference(ie0);
      EchoCORBA::EchoSample_var ec0 = EchoCORBA::EchoSample::_narrow(obj);
      if (CORBA::is_nil(ec0)) throw "Objectref is nil";
      ec0->echo("this is corba object");

      Ice::ObjectPrx base = ice->toReference(ie0);
      Demo::EchoSamplePrx eobj = Demo::EchoSamplePrx::checkedCast(base);
      eobj->echo("This is Ice object");

      ie0->echo("hogehoge");
      //      std::cout << "hoge" << std::endl;
    }
  };
}; // namespace ORBManager

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ORBManager::ORBManagerTests);

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
#endif // ORBManager_cpp
