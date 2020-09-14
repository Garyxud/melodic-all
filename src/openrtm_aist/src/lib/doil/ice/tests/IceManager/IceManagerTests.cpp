// -*- C++ -*-
/*!
 * @file   IceManagerTests.cpp
 * @brief  IceManager test class
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

#ifndef IceManager_cpp
#define IceManager_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <doil/ice/IceManager.h>
#include <doil/ice/IceServantBase.h>
#include <doil/ImplBase.h>
#include <iostream>
#include "EchoSample.h"

class EchoImpl
  : public doil::ImplBase
{
public:
  EchoImpl()
  {
    sprintf(m_name, "EchoSample%d", count);
    ++count;
  }
  virtual ~EchoImpl()
  {
    std::cout << "EchoImpl: " << name() << " deleted." << std::endl;
  }
  const char* id() {return "EchoSample";}
  const char* name() {return m_name;}
  void incRef(){}
  void decRef(){}
  void echo(std::string msg)
  {
    std::cout << name() <<  " -> Message is: " << msg << std::endl;
    return;
  }
  static int count;
  char m_name[16];
};
int EchoImpl::count = 0;


class EchoServant
  : public virtual Demo::EchoSample,
    public virtual doil::Ice::IceServantBase
{
public:
  EchoServant(doil::ImplBase* impl)
    : doil::Ice::IceServantBase(impl)
  {
    m_impl = dynamic_cast<EchoImpl*>(impl);
    if (m_impl == NULL) throw std::bad_alloc();
  }
  virtual ~EchoServant()
  {
    std::cout << "EchoServant: " << name() << " deleted." << std::endl;
  }
  virtual void echo(const std::string& msg, const Ice::Current&)
  {
    std::cout << "servant" << name() << "::echo()  was called" << std::endl;
    m_impl->echo(msg);
  }
private:
  EchoImpl* m_impl;
};



/*!
 * @class IceManagerTests class
 * @brief IceManager test
 */
namespace IceManager
{
  class IceManagerTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(IceManagerTests);
    CPPUNIT_TEST(test_init);
    CPPUNIT_TEST(test_instance);
    CPPUNIT_TEST(test_name);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    IceManagerTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~IceManagerTests()
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
      doil::Ice::IceManager* mgr;
      mgr = doil::Ice::IceManager::init(coil::Properties());

      for (int i(0); i < 1000; ++i)
        {
          doil::Ice::IceManager* tmp;
          tmp = doil::Ice::IceManager::init(coil::Properties());
          CPPUNIT_ASSERT_MESSAGE("IceManager's pointer should always be same",
                                 mgr == tmp);
        }
    }

    void test_instance()
    {
      doil::Ice::IceManager* mgr;
      mgr = &(doil::Ice::IceManager::instance());
      for (int i(0); i < 1000; ++i)
        {
          doil::Ice::IceManager* tmp;
          tmp = &(doil::Ice::IceManager::instance());
          CPPUNIT_ASSERT_MESSAGE("IceManager's pointer should always be same",
                                 mgr == tmp);
        }
      
    }

    void test_name()
    {
      for (int i(100); i < 1000; ++i)
        {
          std::string name(doil::Ice::IceManager::instance().name());
          CPPUNIT_ASSERT_MESSAGE("IceManager's name should be corba",
                                 name == "ice");
        }
    }


    /* test case */
    void test_case0()
    {
      doil::Ice::IceManager& mgr(doil::Ice::IceManager::instance());
      
      // name()
      std::cout << mgr.name() << std::endl;

      // activateObject
      EchoImpl* eimpl = new EchoImpl();
      doil::ReturnCode_t ret;
      ret = mgr.activateObject(eimpl);
      if (ret == doil::OK) std::cout << "register: OK" << std::endl;
      else if (ret == doil::NOT_FOUND) std::cout << "register: NOT_FOUND" << std::endl;
      else if (ret == doil::UNKNOWN) std::cout << "register: UNKNOWN" << std::endl;
      else std::cout << "other error" << std::endl;


      // regsiterFactory
      mgr.registerFactory(eimpl->id(), doil::New<EchoServant>,
                          doil::Delete<EchoServant>);
      ret = mgr.activateObject(eimpl);
      
      if (ret == doil::OK) std::cout << "register: OK" << std::endl;
      else if (ret == doil::NOT_FOUND) std::cout << "register: NOT_FOUND" << std::endl;
      else if (ret == doil::UNKNOWN) std::cout << "register: UNKNOWN" << std::endl;
      else std::cout << "other error" << std::endl;


      // toServant
      doil::ServantBase* svt = mgr.toServant(eimpl);
      if (svt == NULL)
        {
          std::cout << "servant not found" << std::endl;
          return;
        }
      EchoServant* esvt = dynamic_cast<EchoServant*>(svt);
      if (esvt == NULL) 
        {
          std::cout << "dynamic_cast failed" << std::endl;
      return;
        }
      //      esvt->echo("daradara");
      //  EchoServant* esvt = new EchoServant(eimpl);
      //  mgr.activateObject(eimpl, esvt);
      
      //  esvt->echo("hogehoge");
      std::cout << "ID  : " << esvt->id() << std::endl;
      std::cout << "name: " << esvt->name() << std::endl;
      
      // getORB/getPOA
      Ice::CommunicatorPtr orb = mgr.getORB();
      Ice::ObjectAdapterPtr adapter = mgr.getAdapter();
      
      Ice::ObjectPrx base = mgr.getORB()->stringToProxy("EchoSample0:default -p 10000");
      std::cout << "Proxy base was created" << std::endl;
      Demo::EchoSamplePrx eobj = Demo::EchoSamplePrx::checkedCast(base);
      std::cout << "base was casted to EchoSample proxy" << std::endl;
      if (!eobj)
        {
          std::cout << "Invalud Object reference" << std::endl;
          throw "Invalid Proxy!";
        }
      eobj->echo("munyamunya");


      // toImpl
      doil::ImplBase* tmpi = mgr.toImpl(esvt);
      if (tmpi == NULL)
        {
          std::cout << "not found impl" << std::endl;
          return;
        }
      
      EchoImpl* tmpe = dynamic_cast<EchoImpl*>(tmpi);
      tmpe->echo("gggggg");
      
      printf("eimpl 0x%lx\n", (unsigned long int)eimpl);
      printf("tmpe  0x%lx\n", (unsigned long int)tmpe);

      
      EchoImpl* eimpl2 = new EchoImpl();
      mgr.activateObject(eimpl2);

      base = mgr.getORB()->stringToProxy("EchoSample1:default -p 10000");
      eobj = Demo::EchoSamplePrx::checkedCast(base);
      eobj->echo("hogehogehoge");
      
      mgr.shutdown();
    }
  };
}; // namespace IceManager

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(IceManager::IceManagerTests);

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
#endif // IceManager_cpp
