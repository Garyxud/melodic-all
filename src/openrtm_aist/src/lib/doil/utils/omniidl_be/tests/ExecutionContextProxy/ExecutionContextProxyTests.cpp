// -*- C++ -*-
/*!
 * @file   ExecutionContextProxyTests.cpp
 * @brief  ExecutionContextProxy test class
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

#ifndef ExecutionContextProxy_cpp
#define ExecutionContextProxy_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <string>
#include <iostream>
#include <sstream>
#include <idl/RTCSkel.h>
#include <ExecutionContextProxy.h>
#include <ILightweightRTObject.h>
/*!
 * @class ExecutionContextProxyTests class
 * @brief ExecutionContextProxy test
 */
namespace ExecutionContextProxy
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
  class LightweightRTObjectServantMock
   : public virtual ::POA_RTC::LightweightRTObject,
     public virtual ::doil::CORBA::CORBAServantBase
  {
  public:
    LightweightRTObjectServantMock(doil::ImplBase* impl)
     : ::doil::CORBA::CORBAServantBase(impl) 
    {
    }
    virtual ~LightweightRTObjectServantMock()
    {
    }
    virtual ::RTC::ReturnCode_t initialize()
    {
    }
    virtual ::RTC::ReturnCode_t finalize()
    {
    }
    virtual ::CORBA::Boolean is_alive(::RTC::ExecutionContext_ptr exec_context)
    {
    }
    virtual ::RTC::ReturnCode_t exit()
    {
    }
    virtual ::RTC::ExecutionContextHandle_t attach_context(::RTC::ExecutionContext_ptr exec_context)
    {
    }
    virtual ::RTC::ReturnCode_t detach_context(::RTC::ExecutionContextHandle_t exec_handle)
    {
    }
    virtual ::RTC::ExecutionContext_ptr get_context(::RTC::ExecutionContextHandle_t exec_handle)
    {
    }
    virtual ::RTC::ExecutionContextList* get_owned_contexts()
    {
    }
    virtual ::RTC::ExecutionContextList* get_participating_contexts()
    {
    }
    virtual ::RTC::ExecutionContextHandle_t get_context_handle(::RTC::ExecutionContext_ptr cxt)
    {
    }
    virtual ::RTC::ReturnCode_t on_initialize()
    {
    }
    virtual ::RTC::ReturnCode_t on_finalize()
    {
    }
    virtual ::RTC::ReturnCode_t on_startup(::RTC::ExecutionContextHandle_t exec_handle)
    {
    }
    virtual ::RTC::ReturnCode_t on_shutdown(::RTC::ExecutionContextHandle_t exec_handle)
    {
    }
    virtual ::RTC::ReturnCode_t on_activated(::RTC::ExecutionContextHandle_t exec_handle)
    {
    }
    virtual ::RTC::ReturnCode_t on_deactivated(::RTC::ExecutionContextHandle_t exec_handle)
    {
    }
    virtual ::RTC::ReturnCode_t on_aborting(::RTC::ExecutionContextHandle_t exec_handle)
    {
    }
    virtual ::RTC::ReturnCode_t on_error(::RTC::ExecutionContextHandle_t exec_handle)
    {
    }
    virtual ::RTC::ReturnCode_t on_reset(::RTC::ExecutionContextHandle_t exec_handle)
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
  class ILightweightRTObjectMock
    : public virtual ::RTC::Local::ILightweightRTObject
  {
  public:
    ILightweightRTObjectMock()
      : m_refcount(0)
    {}
    virtual ~ILightweightRTObjectMock(){}
    const char* id() {return "ILightweightRTObjectMock";}
    const char* name() {return "ILightweightRTObjectMock";}
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
    virtual ::RTC::Local::ReturnCode_t initialize()
      throw ()
    {
    }
    virtual ::RTC::Local::ReturnCode_t finalize()
      throw ()
    {
    }
    virtual bool is_alive(const ::RTC::Local::IExecutionContext* exec_context)
      throw ()
    {
    }

    virtual ::RTC::Local::ReturnCode_t exit()
      throw ()
    {
    }

    virtual ::RTC::Local::ExecutionContextHandle_t attach_context(const ::RTC::Local::IExecutionContext* exec_context)
      throw ()
    {
    }

    virtual ::RTC::Local::ReturnCode_t detach_context(::RTC::Local::ExecutionContextHandle_t exec_handle)
      throw ()
    {
    }

    virtual ::RTC::Local::IExecutionContext* get_context(::RTC::Local::ExecutionContextHandle_t exec_handle)
      throw ()
    {
    }

    virtual ::RTC::Local::ExecutionContextList get_owned_contexts()
      throw ()
    {
    }

    virtual ::RTC::Local::ExecutionContextList get_participating_contexts()
      throw ()
    {
    }

    virtual ::RTC::Local::ExecutionContextHandle_t get_context_handle(const ::RTC::Local::IExecutionContext* cxt)
      throw ()
    {
    }
    virtual ::RTC::Local::ReturnCode_t on_initialize()
      throw ()
    {
    }

    virtual ::RTC::Local::ReturnCode_t on_finalize()
      throw ()
    {
    }

    virtual ::RTC::Local::ReturnCode_t on_startup(::RTC::Local::ExecutionContextHandle_t exec_handle)
      throw ()
    {
    }

    virtual ::RTC::Local::ReturnCode_t on_shutdown(::RTC::Local::ExecutionContextHandle_t exec_handle)
      throw ()
    {
    }

    virtual ::RTC::Local::ReturnCode_t on_activated(::RTC::Local::ExecutionContextHandle_t exec_handle)
      throw ()
    {
    }

    virtual ::RTC::Local::ReturnCode_t on_deactivated(::RTC::Local::ExecutionContextHandle_t exec_handle)
      throw ()
    {
    }

    virtual ::RTC::Local::ReturnCode_t on_aborting(::RTC::Local::ExecutionContextHandle_t exec_handle)
      throw ()
    {
    }

    virtual ::RTC::Local::ReturnCode_t on_error(::RTC::Local::ExecutionContextHandle_t exec_handle)
      throw ()
    {
    }

    virtual ::RTC::Local::ReturnCode_t on_reset(::RTC::Local::ExecutionContextHandle_t exec_handle)
      throw ()
    {
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
  class ExecutionContextRtmMock
    : public virtual ::POA_RTC::ExecutionContext  
  {
  protected:
      std::vector<std::string> m_log;
  public :
      ExecutionContextRtmMock(){}
      virtual ~ExecutionContextRtmMock(){}

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
  ::CORBA::Boolean is_running()
  {
    return false;
  }
  

    /*! 
     *
     */
  ::RTC::ReturnCode_t start()
  {
    return RTC::RTC_OK;
  }
  

    /*! 
     *
     */
  ::RTC::ReturnCode_t stop()
  {
    return RTC::RTC_OK;
  }
  
  
    /*! 
     *
     */
    ::CORBA::Double get_rate()
    {
        if (m_logger != NULL)
        {
            m_logger->log("get_rate");
        }
        return (CORBA::Double)3.14159;
    }
  
  
    /*! 
     *
     */
    ::RTC::ReturnCode_t set_rate(::CORBA::Double rate)
    {
        if (m_logger != NULL)
        {
            m_logger->log("set_rate");
            ::std::ostringstream os;
            os<<rate; 
            m_logger->log(os.str());
        }

        return RTC::BAD_PARAMETER;
    }
  

    /*! 
     *
     */
  ::RTC::ReturnCode_t activate_component(::RTC::LightweightRTObject_ptr comp)
  {
    return RTC::RTC_OK;
  }
  

    /*! 
     *
     */
  ::RTC::ReturnCode_t deactivate_component(::RTC::LightweightRTObject_ptr comp)
  {
    return RTC::RTC_OK;
  }
  

    /*! 
     *
     */
  ::RTC::ReturnCode_t reset_component(::RTC::LightweightRTObject_ptr comp)
  {
    return RTC::RTC_OK;
  }
  
  
    /*! 
     *
     */
  ::RTC::LifeCycleState get_component_state(::RTC::LightweightRTObject_ptr comp)
  {
    return RTC::INACTIVE_STATE;
  }
  
  
    /*! 
     *
     */
  ::RTC::ExecutionKind get_kind()
  {
//    return m_profile.kind;
  }
    /*! 
     *
     */
    ::RTC::ReturnCode_t add_component(::RTC::LightweightRTObject_ptr comp)
    {
        if (m_logger != NULL)
        {
            m_logger->log("add_component");
//            ::std::ostringstream os;
//            os<<rate; 
//            m_logger->log(os.str());
        }
        return RTC::BAD_PARAMETER;
    }
  
  
    /*! 
     *
     */
  ::RTC::ReturnCode_t remove_component(::RTC::LightweightRTObject_ptr comp)
  {
    CORBA::ULong index;
//    index = CORBA_SeqUtil::find(m_profile.participants,
//		find_objref<RTObject_ptr>(RTC::RTObject::_narrow(comp)));
				
    if (index < 0) return RTC::BAD_PARAMETER;
//    CORBA_SeqUtil::erase(m_profile.participants, index);
    return RTC::RTC_OK;
  }
  
  private:
    Logger* m_logger;

  };

  /*!
   * 
   * 
   *
   */
  class ExecutionContextProxyTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ExecutionContextProxyTests);
//    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST(test_get_rate);
    CPPUNIT_TEST(test_set_rate);
    CPPUNIT_TEST(test_add_component);
    CPPUNIT_TEST_SUITE_END();
  
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
  
  public:
  
    /*!
     * @brief Constructor
     */
    ExecutionContextProxyTests()
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
    ~ExecutionContextProxyTests()
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
    void test_get_rate()
    {

      ExecutionContextRtmMock* obj = new ExecutionContextRtmMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::RTC::CORBA::ExecutionContextProxy* ap 
                 = new ::RTC::CORBA::ExecutionContextProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      double ret;
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("get_rate"));
      ret = ap->get_rate(); 
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("get_rate"));
      CPPUNIT_ASSERT_EQUAL(3.14159, ret);

      delete ap;
      CORBA::release(ref);

    }
  
    /*! 
     *
     *
     *
     */
    void test_set_rate()
    {

      ExecutionContextRtmMock* obj = new ExecutionContextRtmMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::RTC::CORBA::ExecutionContextProxy* ap 
                 = new ::RTC::CORBA::ExecutionContextProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      double dbuf = 2.71828;
      ::RTC::Local::ReturnCode_t ret;
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("2.71828"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("set_rate"));
      ret = ap->set_rate(dbuf); 
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("set_rate"));
      CPPUNIT_ASSERT_EQUAL(::RTC::Local::BAD_PARAMETER, ret);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("2.71828"));

      delete ap;
      CORBA::release(ref);

    }
    /*! 
     *
     *
     *
     */
    void test_add_component()
    {
      doil::CORBA::CORBAManager& 
                            mgr(doil::CORBA::CORBAManager::instance());
      std::cout <<"Manager Name==>"<< mgr.name() << std::endl;


      ExecutionContextRtmMock* obj = new ExecutionContextRtmMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::RTC::CORBA::ExecutionContextProxy* ap 
                 = new ::RTC::CORBA::ExecutionContextProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      ::RTC::Local::ReturnCode_t ret;
      ILightweightRTObjectMock* comp = new ILightweightRTObjectMock();
      mgr.registerFactory(comp->id(), 
                          doil::New<LightweightRTObjectServantMock>,
                          doil::Delete<LightweightRTObjectServantMock>);
      mgr.activateObject(comp);
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("add_component"));
      ret = ap->add_component(comp); 
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("add_component"));
      CPPUNIT_ASSERT_EQUAL(::RTC::Local::BAD_PARAMETER, ret);

      delete ap;
      CORBA::release(ref);

    }
    /* test case */
    void test_case0()
    {
    }
  };
}; // namespace ExecutionContextProxy

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ExecutionContextProxy::ExecutionContextProxyTests);

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
#endif // ExecutionContextProxy_cpp
