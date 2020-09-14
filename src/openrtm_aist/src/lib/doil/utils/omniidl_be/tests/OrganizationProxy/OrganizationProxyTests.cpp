// -*- C++ -*-
/*!
 * @file   OrganizationProxyTests.cpp
 * @brief  OrganizationProxy test class
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

#ifndef OrganizationProxy_cpp
#define OrganizationProxy_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <string>
#include <iostream>
#include <idl/SDOPackageSkel.h>
#include <OrganizationProxy.h>
#include <ISDOSystemElement.h>
#include <doil/corba/CORBAManager.h>
#include <doil/corba/CORBAProxyBase.h>

/*!
 * @class OrganizationProxyTests class
 * @brief OrganizationProxy test
 */
namespace OrganizationProxy
{
  /*!
   * 
   * 
   *
   */
  class Logger
  {
  public:
    void log(const std::string& msg)
    {
      m_log.push_back(msg);
    }

    int countLog(const std::string& msg)
    {
      int count = 0;
      for (int i = 0; i < (int) m_log.size(); ++i)
        {
          if (m_log[i] == msg) ++count;
        }
     return count;
    }
		
  private:
    std::vector<std::string> m_log;
  };

  /*!
   * 
   * 
   *
   */
  class SDOSystemElementServantMock
   : public virtual ::POA_SDOPackage::SDOSystemElement,
     public virtual ::doil::CORBA::CORBAServantBase
  {
  public:
    SDOSystemElementServantMock(doil::ImplBase* impl)
     : ::doil::CORBA::CORBAServantBase(impl) 
    {
    }
    virtual ~SDOSystemElementServantMock()
    {
    }

    virtual ::SDOPackage::OrganizationList* get_owned_organizations()
    {
    }
  private:
    ::SDOPackage::Local::ISDOSystemElement* m_impl;
  };
  /*!
   * 
   * 
   *
   */
  class ISDOSystemElementMock
    : public virtual ::SDOPackage::Local::ISDOSystemElement
  {
  public:
    ISDOSystemElementMock()
      : m_refcount(0)
    {}
    virtual ~ISDOSystemElementMock(){}
    const char* id() {return "ISDOSystemElementMock";}
    const char* name() {return "ISDOSystemElementMock";}
    void incRef()
    {
      ++m_refcount;
    }
    void decRef()
    {
      --m_refcount;
      if (m_refcount == 0)
        delete this;
    }
    virtual ::SDOPackage::Local::OrganizationList get_owned_organizations()
      throw (::SDOPackage::Local::NotAvailable)
    {
        ::SDOPackage::Local::OrganizationList  list;
        return list;
    }
  private:
    std::string m_name;
    int m_refcount;
  };
  /*!
   * 
   * 
   *
   */
  class OrganizationRtmMock
    : public virtual ::POA_SDOPackage::Organization
  {
  protected:
      std::vector<std::string> m_log;
  public :
      OrganizationRtmMock(){}
      virtual ~OrganizationRtmMock(){}

    /*! 
     *
     */
      void setLogger(Logger* logger)
      {
        m_logger = logger;
      }
    /*! 
     *
     */
    virtual char* get_organization_id()
    {
      return "get_organization_id";
    }
    /*! 
     *
     */
    virtual SDOPackage::OrganizationProperty* get_organization_property()
    {
      return NULL;
    }
		
    /*! 
     *
     */
    virtual CORBA::Any* get_organization_property_value(const char* name)
    {
      return NULL;
    }
		
    /*! 
     *
     */
    virtual CORBA::Boolean set_organization_property(const SDOPackage::OrganizationProperty& organization_property)
    {
      return false;
    }
		
    /*! 
     *
     */
    virtual CORBA::Boolean set_organization_property_value(const char* name, const CORBA::Any& value)
    {
      return false;
    }
		
    /*! 
     *
     */
    virtual CORBA::Boolean remove_organization_property(const char* name)
    {
      return false;
    }
		
    /*! 
     *
     */
    virtual SDOPackage::SDOSystemElement_ptr get_owner()
    {
      return NULL;
    }
		
    /*! 
     *
     */
    virtual CORBA::Boolean set_owner(SDOPackage::SDOSystemElement_ptr sdo)
    {
        if (m_logger != NULL)
        {
            m_logger->log("set_owner");
        }
        return false;
    }
		
    /*! 
     *
     */
    virtual SDOPackage::SDOList* get_members()
    {
      return NULL;
    }
		
    /*! 
     *
     */
    virtual CORBA::Boolean set_members(const SDOPackage::SDOList& sdos)
    {
      return false;
    }
		
    /*! 
     *
     */
    virtual CORBA::Boolean add_members(const SDOPackage::SDOList& sdo_list)
    {
      return false;
    }
		
    /*! 
     *
     */
    virtual CORBA::Boolean remove_member(const char* id)
    {
      return false;
    }
		
    /*! 
     *
     */
    virtual SDOPackage::DependencyType get_dependency()
    {
      return SDOPackage::NO_DEPENDENCY;
    }
		
    /*! 
     *
     */
    virtual CORBA::Boolean set_dependency(SDOPackage::DependencyType dependency)
    {
      return false;
    }
  private:
    Logger* m_logger;

  };

  class OrganizationProxyTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(OrganizationProxyTests);
    CPPUNIT_TEST(test_set_owner);
//    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
  
  public:
  
    /*!
     * @brief Constructor
     */
    OrganizationProxyTests()
    {
      int argc = 0;
      char** argv = NULL;
      m_pORB = CORBA::ORB_init(argc, argv);
      m_pPOA = PortableServer::POA::_narrow(
                       m_pORB->resolve_initial_references("RootPOA"));
      m_pPOA->the_POAManager()->activate();
    }
    
    /*!
     * @brief Destructor
     */
    ~OrganizationProxyTests()
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
    /*! 
     *
     *
     *
     */
    void test_set_owner()
    {
      doil::CORBA::CORBAManager& 
                            mgr(doil::CORBA::CORBAManager::instance());
      std::cout <<"Manager Name==>"<< mgr.name() << std::endl;

      OrganizationRtmMock* obj = new OrganizationRtmMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::OrganizationProxy* ap 
                 = new ::SDOPackage::CORBA::OrganizationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

       
      bool ret; 
      ISDOSystemElementMock* sdo = new ISDOSystemElementMock();
      mgr.registerFactory(sdo->id(), 
                          doil::New<SDOSystemElementServantMock>,
                          doil::Delete<SDOSystemElementServantMock>);
      mgr.activateObject(sdo);
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("set_owner"));
      ret = ap->set_owner(sdo);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("set_owner"));
      CPPUNIT_ASSERT_EQUAL(false, ret);

      delete ap;
      CORBA::release(ref);

    }
  
    /* test case */
    void test_case0()
    {
    }
  };
}; // namespace OrganizationProxy

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(OrganizationProxy::OrganizationProxyTests);

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
#endif // OrganizationProxy_cpp
