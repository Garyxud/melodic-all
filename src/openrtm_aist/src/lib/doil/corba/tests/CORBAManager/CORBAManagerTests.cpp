// -*- C++ -*-
/*!
 * @file   CORBAManagerTests.cpp
 * @brief  CORBAManager test class
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

#ifndef CORBAManager_cpp
#define CORBAManager_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <doil/corba/CORBAManager.h>
#include <doil/corba/CORBAServantBase.h>
#include <doil/ImplBase.h>
#include <iostream>
#include "Echo.hh"

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
  : public virtual doil::CORBA::CORBAServantBase,
    public virtual POA_EchoSample
{
public:
  EchoServant(doil::ImplBase* impl)
    : doil::CORBA::CORBAServantBase(impl)
  {
    m_impl = dynamic_cast<EchoImpl*>(impl);
    if (m_impl == NULL) throw std::bad_alloc();
  }
  virtual ~EchoServant()
  {
    std::cout << "EchoServant: " << name() << " deleted." << std::endl;
  }
  void echo(const char* msg)
  {
    m_impl->echo(msg);
  }
private:
  EchoImpl* m_impl;
};



/*!
 * @class CORBAManagerTests class
 * @brief CORBAManager test
 */
namespace CORBAManager
{
  class CORBAManagerTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(CORBAManagerTests);
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
    CORBAManagerTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~CORBAManagerTests()
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
      doil::CORBA::CORBAManager* mgr;
      mgr = doil::CORBA::CORBAManager::init(coil::Properties());

      for (int i(0); i < 1000; ++i)
        {
          doil::CORBA::CORBAManager* tmp;
          tmp = doil::CORBA::CORBAManager::init(coil::Properties());
          CPPUNIT_ASSERT_MESSAGE("CORBAManager's pointer should always be same",
                                 mgr == tmp);
        }
    }

    void test_instance()
    {
      doil::CORBA::CORBAManager* mgr;
      mgr = &(doil::CORBA::CORBAManager::instance());
      for (int i(0); i < 1000; ++i)
        {
          doil::CORBA::CORBAManager* tmp;
          tmp = &(doil::CORBA::CORBAManager::instance());
          CPPUNIT_ASSERT_MESSAGE("CORBAManager's pointer should always be same",
                                 mgr == tmp);
        }
      
    }
    void test_name()
    {
      for (int i(100); i < 1000; ++i)
        {
          std::string name(doil::CORBA::CORBAManager::instance().name());
          CPPUNIT_ASSERT_MESSAGE("CORBAManager's name should be corba",
                                 name == "corba");
        }
    }

    // ごちゃまぜのテスト
    void test_case0()
    {
      doil::CORBA::CORBAManager& mgr(doil::CORBA::CORBAManager::instance());
      
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
      esvt->echo("daradara");
      //  EchoServant* esvt = new EchoServant(eimpl);
      //  mgr.activateObject(eimpl, esvt);
      
      //  esvt->echo("hogehoge");
      std::cout << "ID  : " << esvt->id() << std::endl;
      std::cout << "name: " << esvt->name() << std::endl;
      
      // getORB/getPOA
      CORBA::ORB_ptr orb = mgr.getORB();
      PortableServer::POA_ptr poa = mgr.getPOA();
      
      CORBA::Object_ptr obj = poa->servant_to_reference(esvt);
      EchoSample_ptr eptr = EchoSample::_narrow(obj);
      eptr->echo("munyamunya");

      // toImpl
      doil::ImplBase* tmpi = mgr.toImpl(eptr);
      if (tmpi == NULL)
        {
          std::cout << "not found impl" << std::endl;
          return;
        }
      
      EchoImpl* tmpe = dynamic_cast<EchoImpl*>(tmpi);
      tmpe->echo("gggggg");
      
      printf("eimpl 0x%lx\n", (unsigned long int)eimpl);
      printf("tmpe  0x%lx\n", (unsigned long int)tmpe);

      std::cout << "objref from eimpl: " << orb->object_to_string(mgr.toReference(eimpl)) << std::endl;
      std::cout << "objref from obj  : " << orb->object_to_string(obj) << std::endl;
      

      
      EchoImpl* eimpl2 = new EchoImpl();
      mgr.activateObject(eimpl2);
      std::cout << "objref from eimpl2:" << orb->object_to_string(mgr.toReference(eimpl2)) << std::endl;
      
      std::string ior0, ior1;
      ior0 = orb->object_to_string(obj);
      ior1 = orb->object_to_string(mgr.toReference(eimpl2));
      
      if (ior0 == ior1) std::cout << "same!!!!!!!!!!" << std::endl;
      else std::cout << "different" << std::endl;
      
      //  mgr.deactivateObject(eimpl);
      for (int j(0); j < 100; ++j)
        {
          CORBA::Object_var tobj;
          if (j % 2 == 0)
            {
              tobj = orb->string_to_object(ior0.c_str());
            }
          else
            {
              tobj = orb->string_to_object(ior1.c_str());
            }
          EchoSample_var ec = EchoSample::_narrow(tobj);
          char msg[100];
          sprintf(msg, "hoge%06d", j);
          ec->echo(msg);
        }
      std::cout << "deactivated" << std::endl;
      doil::ImplBase* dimpl = mgr.toImpl(svt);
      std::cout << "toImpl" << std::endl;
      if (dimpl == NULL) 
        {
          std::cout << "not found" << std::endl;
        }
      
      mgr.shutdown();
    }

  };
}; // namespace CORBAManager

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(CORBAManager::CORBAManagerTests);

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
#endif // CORBAManager_cpp
