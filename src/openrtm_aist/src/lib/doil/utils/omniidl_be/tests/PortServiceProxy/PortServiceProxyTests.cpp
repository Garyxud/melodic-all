// -*- C++ -*-
/*!
 * @file   PortServiceProxyTests.cpp
 * @brief  PortServiceProxy test class
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

#ifndef PortServiceProxy_cpp
#define PortServiceProxy_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <string>
#include <sstream>
#include <iostream>
#include <idl/RTCSkel.h>
#include <PortServiceProxy.h>
#include <PortServiceServant.h>
#include <IPortService.h>
#include <doil/corba/CORBAManager.h>

/*!
 * @class PortServiceProxyTests class
 * @brief PortServiceProxy test
 */
namespace PortServiceProxy
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
  class PortServiceServantMock 
   : public virtual ::POA_RTC::PortService,
     public virtual ::doil::CORBA::CORBAServantBase
  {
  public:
    PortServiceServantMock(doil::ImplBase* impl)
    : ::doil::CORBA::CORBAServantBase(impl), m_impl(NULL)
    {
      m_impl = dynamic_cast< ::RTC::Local::IPortService* >(impl);
    }
    virtual ~PortServiceServantMock(){}

    virtual ::RTC::PortProfile* get_port_profile()
    {
        ::RTC::PortProfile* corba_ret = new ::RTC::PortProfile ();
        return corba_ret;
    }
    virtual ::RTC::ConnectorProfileList* get_connector_profiles()
    {
        ::RTC::ConnectorProfileList* corba_ret 
                            = new ::RTC::ConnectorProfileList ();
        return corba_ret;
    }
    virtual ::RTC::ConnectorProfile* get_connector_profile(
                             const char* connector_id)
    {
        ::RTC::ConnectorProfile* corba_ret = new ::RTC::ConnectorProfile ();
        return corba_ret;
    }
    virtual ::RTC::ReturnCode_t connect(
                             ::RTC::ConnectorProfile& connector_profile)
    {
	return ::RTC::RTC_OK;
    }
    virtual ::RTC::ReturnCode_t disconnect(const char* connector_id)
    {
	return ::RTC::RTC_OK;
    }
    virtual ::RTC::ReturnCode_t disconnect_all()
    {
	return ::RTC::RTC_OK;
    }
    virtual ::RTC::ReturnCode_t notify_connect(
                             ::RTC::ConnectorProfile& connector_profile)
    {
	return ::RTC::RTC_OK;
    }
    virtual ::RTC::ReturnCode_t notify_disconnect(const char* connector_id)
    {
	return ::RTC::RTC_OK;
    }

  private:
    ::RTC::Local::IPortService* m_impl;
  };
  /*!
   * 
   * 
   *
   */
  class IPortServiceMock
    : public virtual ::RTC::Local::IPortService
  {
  public:
    IPortServiceMock()
      : m_refcount(0)
    {}
    virtual ~IPortServiceMock(){}
    const char* id() {return "IPortServiceMock";}
    const char* name() {return "IPortServiceMock";}
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
    virtual ::RTC::Local::PortProfile get_port_profile()
      throw ()
    {
        ::RTC::Local::PortProfile corba_ret;
        return corba_ret;
    }
    virtual ::RTC::Local::ConnectorProfileList get_connector_profiles()
      throw ()
    {
        ::RTC::Local::ConnectorProfileList corba_ret; 
        return corba_ret;
    }
    virtual ::RTC::Local::ConnectorProfile get_connector_profile(
                                        const ::std::string& connector_id)
      throw ()
    {
        ::RTC::Local::ConnectorProfile corba_ret;
        return corba_ret;
    }


    virtual ::RTC::Local::ReturnCode_t connect(
                        ::RTC::Local::ConnectorProfile& connector_profile)
      throw ()
    {
	return ::RTC::Local::RTC_OK;
    }

    virtual ::RTC::Local::ReturnCode_t disconnect(
                                      const ::std::string& connector_id)
      throw ()
    {
	return ::RTC::Local::RTC_OK;
    }

    virtual ::RTC::Local::ReturnCode_t disconnect_all()
      throw ()
    {
	return ::RTC::Local::RTC_OK;
    }

    virtual ::RTC::Local::ReturnCode_t notify_connect(
                       ::RTC::Local::ConnectorProfile& connector_profile)
      throw ()
    {
	return ::RTC::Local::RTC_OK;
    }

    virtual ::RTC::Local::ReturnCode_t notify_disconnect(
                                    const ::std::string& connector_id)
      throw()
    {
	return ::RTC::Local::RTC_OK;
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
  class PortServiceRtmMock
    : public virtual ::POA_RTC::PortService  {
  protected:
      std::vector<std::string> m_log;
      ::RTC::PortProfile m_profile;
  public :
      PortServiceRtmMock(){}
      virtual ~PortServiceRtmMock(){}

      void setLogger(Logger* logger)
      {
        m_logger = logger;
      }
    /*! 
     *
     */
    ::RTC::PortProfile* get_port_profile()
      throw (CORBA::SystemException)
    {
        if (m_logger != NULL)
        {
            m_logger->log("get_port_profile");
        }
        ::RTC::PortProfile_var prof;
        m_profile.name = "name of port";
        m_profile.interfaces.length(1);
        m_profile.interfaces[0].instance_name = "MyService (provided)";
        m_profile.interfaces[0].type_name = "Generic (provided)";
        m_profile.interfaces[0].polarity = ::RTC::PROVIDED;
        prof = new ::RTC::PortProfile(m_profile);
        return prof._retn();
    }
    /*! 
     *
     */
    ::RTC::ConnectorProfileList* get_connector_profiles()
      throw (CORBA::SystemException)
    {
        if (m_logger != NULL)
        {
            m_logger->log("get_connector_profiles");
        }
        m_profile.connector_profiles.length(1);
        m_profile.connector_profiles[0].name = "ConnectorProfile-name";
        m_profile.connector_profiles[0].connector_id = "connect_id0";
        m_profile.connector_profiles[0].properties.length(1);
        m_profile.connector_profiles[0].properties[0].name 
                                  = "ConnectorProfile-properties0-name";
        m_profile.connector_profiles[0].properties[0].value  
                                  <<= (::CORBA::Float)1.1;
        m_profile.connector_profiles[0].ports.length(1);  
//        PortServiceRtmMock* obj = new PortServiceRtmMock();
//        m_profile.connector_profiles[0].ports[0] = obj->_this();

        ::RTC::ConnectorProfileList_var conn_prof;
        conn_prof 
          = new ::RTC::ConnectorProfileList(m_profile.connector_profiles);
        return conn_prof._retn();
    }
    /*! 
     *
     */
    ::RTC::ConnectorProfile* get_connector_profile(const char* connector_id)
      throw (CORBA::SystemException)
    {
        if (m_logger != NULL)
        {
            m_logger->log("get_connector_profile");
            m_logger->log(connector_id);
        }
        m_profile.connector_profiles.length(1);
        m_profile.connector_profiles[0].name = "ConnectorProfile-name";
        m_profile.connector_profiles[0].connector_id = "connect_id0";
        m_profile.connector_profiles[0].properties.length(1);
        m_profile.connector_profiles[0].properties[0].name 
                                  = "ConnectorProfile-properties0-name";
        m_profile.connector_profiles[0].properties[0].value  
                                  <<= (::CORBA::Float)1.1;
        m_profile.connector_profiles[0].ports.length(1);  
        ::RTC::ConnectorProfile_var conn_prof;
//        PortServiceRtmMock* obj = new PortServiceRtmMock();
//        m_profile.connector_profiles[0].ports[0] = obj->_this();
        conn_prof = new ::RTC::ConnectorProfile(
                                     m_profile.connector_profiles[0]);
        return conn_prof._retn();
    }
    /*! 
     *
     */
    ::RTC::ReturnCode_t connect(::RTC::ConnectorProfile& connector_profile)
      throw (CORBA::SystemException)
    {
        if (m_logger != NULL)
        {

            m_logger->log("connect");
            m_logger->log(::std::string(connector_profile.name));
            m_logger->log(::std::string(connector_profile.connector_id));
            m_logger->log(
                   ::std::string(connector_profile.properties[0].name));
            const char* ch;
            ::CORBA::ULong ul = 3;
            ::CORBA::Any::to_string to_str(ch, ul);
            connector_profile.properties[0].value >>= to_str;
            m_logger->log(to_str.val);
        }
        connector_profile.name = "ConnectorProfile-name";
        connector_profile.connector_id = "connect_id0";
        connector_profile.properties.length(1);
        connector_profile.properties[0].name 
                                  = "ConnectorProfile-properties0-name";
        connector_profile.properties[0].value  
                                  <<= (::CORBA::Float)1.1;
        connector_profile.ports.length(1);  
//        PortServiceRtmMock* obj = new PortServiceRtmMock();
//        m_profile.connector_profiles[0].ports[0] = obj->_this();
        return RTC::RTC_OK;
    }
    /*! 
     *
     */
    ::RTC::ReturnCode_t notify_connect(::RTC::ConnectorProfile& connector_profile)
      throw (CORBA::SystemException)
    {
        if (m_logger != NULL)
        {
            m_logger->log("notify_connect");
            m_logger->log(::std::string(connector_profile.name));
            m_logger->log(::std::string(connector_profile.connector_id));
            m_logger->log(
                   ::std::string(connector_profile.properties[0].name));
            const char* ch;
            ::CORBA::ULong ul = 3;
            ::CORBA::Any::to_string to_str(ch, ul);
            connector_profile.properties[0].value >>= to_str;
            m_logger->log(to_str.val);
        }
        connector_profile.name = "ConnectorProfile-name";
        connector_profile.connector_id = "connect_id0";
        connector_profile.properties.length(1);
        connector_profile.properties[0].name 
                                  = "ConnectorProfile-properties0-name";
        connector_profile.properties[0].value  
                                  <<= (::CORBA::Float)1.1;
        connector_profile.ports.length(1);  
//        PortServiceRtmMock* obj = new PortServiceRtmMock();
//        m_profile.connector_profiles[0].ports[0] = obj->_this();
        return RTC::RTC_OK;
    }
    /*! 
     *
     */
    ::RTC::ReturnCode_t disconnect(const char* connector_id)
      throw (CORBA::SystemException)
    {
        if (m_logger != NULL)
        {
            m_logger->log("disconnect");
            m_logger->log(connector_id);
        }
        return RTC::RTC_OK;
    }
    /*! 
     *
     */
    ::RTC::ReturnCode_t notify_disconnect(const char* connector_id)
      throw (CORBA::SystemException)
    {
        if (m_logger != NULL)
        {
            m_logger->log("notify_disconnect");
            m_logger->log(connector_id);
        }
        return RTC::RTC_OK;
    }
    /*! 
     *
     */
    ::RTC::ReturnCode_t disconnect_all()
      throw (CORBA::SystemException)
    {
        if (m_logger != NULL) 
        {
            m_logger->log("disconnect_all");
        }
        return RTC::RTC_OK;
    }
  
  private:
    Logger* m_logger;

  };

  class PortServiceProxyTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(PortServiceProxyTests);
    CPPUNIT_TEST(test_get_port_profile);
    CPPUNIT_TEST(test_get_connector_profiles);
    CPPUNIT_TEST(test_get_connector_profile);
    CPPUNIT_TEST(test_connect);
    CPPUNIT_TEST(test_disconnect);
    CPPUNIT_TEST(test_disconnect_all);
    CPPUNIT_TEST(test_notify_connect);
    CPPUNIT_TEST(test_notify_disconnect);
    CPPUNIT_TEST_SUITE_END();
  
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
  
  public:
  
    /*!
     * @brief Constructor
     */
    PortServiceProxyTests()
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
    ~PortServiceProxyTests()
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
    void test_get_port_profile()
    {
      PortServiceRtmMock* obj = new PortServiceRtmMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::RTC::CORBA::PortServiceProxy* ap 
                 = new ::RTC::CORBA::PortServiceProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("get_port_profile"));
      ::RTC::Local::PortProfile prof;
      prof = ap->get_port_profile();
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("get_port_profile"));
      CPPUNIT_ASSERT_EQUAL(::std::string("name of port"), 
                           prof.name);
      CPPUNIT_ASSERT_EQUAL((::std::string::size_type)1, 
                           prof.interfaces.size());
      CPPUNIT_ASSERT_EQUAL(::std::string("MyService (provided)"), 
                           prof.interfaces[0].instance_name);
      CPPUNIT_ASSERT_EQUAL(::RTC::Local::PROVIDED, 
                           prof.interfaces[0].polarity);

      delete ap;
      CORBA::release(ref);


    }
    /*! 
     *
     *
     *
     */
    void test_get_connector_profiles()
    {
      PortServiceRtmMock* obj = new PortServiceRtmMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::RTC::CORBA::PortServiceProxy* ap 
                 = new ::RTC::CORBA::PortServiceProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("get_connector_profiles"));
      ::RTC::Local::ConnectorProfileList proflist;
      proflist = ap->get_connector_profiles();
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("get_connector_profiles"));

      CPPUNIT_ASSERT_EQUAL((::std::string::size_type)1, 
                           proflist.size());

      CPPUNIT_ASSERT_EQUAL(::std::string("ConnectorProfile-name"),
			   proflist[0].name);
			
      CPPUNIT_ASSERT_EQUAL(::std::string("connect_id0"),
			   proflist[0].connector_id);

::std::cout<<proflist[0].properties.size()<<::std::endl;
      CPPUNIT_ASSERT_EQUAL((::std::string::size_type)1, 
                           proflist[0].properties.size());
      CPPUNIT_ASSERT_EQUAL(::std::string(
                                    "ConnectorProfile-properties0-name"),
			   proflist[0].properties[0].name);
      CPPUNIT_ASSERT_EQUAL(::std::string("1.1"),
			   proflist[0].properties[0].value);
      CPPUNIT_ASSERT_EQUAL((::std::string::size_type)1, 
                           proflist[0].ports.size());

      delete ap;
      CORBA::release(ref);


    }
    /*! 
     *
     *
     *
     */
    void test_get_connector_profile()
    {
      PortServiceRtmMock* obj = new PortServiceRtmMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::RTC::CORBA::PortServiceProxy* ap 
                 = new ::RTC::CORBA::PortServiceProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      ::std::string str = "test_get_connector_profile";
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("get_connector_profile"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str));
      ::RTC::Local::ConnectorProfile prof;
      prof = ap->get_connector_profile(str);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("get_connector_profile"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str));

      CPPUNIT_ASSERT_EQUAL(::std::string("ConnectorProfile-name"),
			   prof.name);
			
      CPPUNIT_ASSERT_EQUAL(::std::string("connect_id0"),
			   prof.connector_id);

      CPPUNIT_ASSERT_EQUAL((::std::string::size_type)1, 
                           prof.properties.size());
      CPPUNIT_ASSERT_EQUAL(::std::string(
                                    "ConnectorProfile-properties0-name"),
			   prof.properties[0].name);
      CPPUNIT_ASSERT_EQUAL(::std::string("1.1"),
			   prof.properties[0].value);
      CPPUNIT_ASSERT_EQUAL((::std::string::size_type)1, 
                           prof.ports.size());
      

      delete ap;
      CORBA::release(ref);


    }
    /*! 
     *
     *
     *
     */
    void test_connect()
    {
      doil::CORBA::CORBAManager& 
                            mgr(doil::CORBA::CORBAManager::instance());
      std::cout <<"Manager Name==>"<< mgr.name() << std::endl;

      PortServiceRtmMock* obj = new PortServiceRtmMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::RTC::CORBA::PortServiceProxy* ap 
                 = new ::RTC::CORBA::PortServiceProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      ::RTC::Local::ConnectorProfile prof;
      ::std::string str1 = "in_ConnectorProfile-name";
      ::std::string str2 = "in_connect_id0";
      ::std::string str3 = "in_ConnectorProfile-properties0-name";
      ::std::string str4 = "2.2";
      prof.name = str1;
      prof.connector_id = str2;
      ::SDOPackage::Local::NameValue nv; 
      nv.name  = str3;
      nv.value = str4;
      prof.properties.push_back(nv);
      IPortServiceMock portservice;

      mgr.registerFactory(portservice.id(), 
                          doil::New<PortServiceServantMock>,
                          doil::Delete<PortServiceServantMock>);
      mgr.activateObject(&portservice);
      prof.ports.push_back(&portservice);


      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("connect"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str1));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str2));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str3));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str4));
      ::RTC::Local::ReturnCode_t ret;
      ret = ap->connect(prof);
      mgr.deactivateObject(&portservice);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("connect"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str1));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str2));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str3));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str4));
      CPPUNIT_ASSERT_EQUAL(::RTC::Local::RTC_OK, ret);
      CPPUNIT_ASSERT_EQUAL(::std::string("ConnectorProfile-name"),
			   prof.name);
			
      CPPUNIT_ASSERT_EQUAL(::std::string("connect_id0"),
			   prof.connector_id);

      CPPUNIT_ASSERT_EQUAL((::std::string::size_type)1, 
                           prof.properties.size());
      CPPUNIT_ASSERT_EQUAL(::std::string(
                                    "ConnectorProfile-properties0-name"),
			   prof.properties[0].name);
      CPPUNIT_ASSERT_EQUAL(::std::string("1.1"),
			   prof.properties[0].value);
      CPPUNIT_ASSERT_EQUAL((::std::string::size_type)1, 
                           prof.ports.size());
      

      delete ap;
      CORBA::release(ref);


    }
    /*! 
     *
     *
     *
     */
    void test_disconnect()
    {
      PortServiceRtmMock* obj = new PortServiceRtmMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::RTC::CORBA::PortServiceProxy* ap 
                 = new ::RTC::CORBA::PortServiceProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      ::std::string str = "test_disconnect";
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("disconnect"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str));
      ::RTC::Local::ReturnCode_t ret;
      ret = ap->disconnect(str);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("disconnect"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str));
      CPPUNIT_ASSERT_EQUAL(::RTC::Local::RTC_OK, ret);

      delete ap;
      CORBA::release(ref);


    }
    /*! 
     *
     *
     *
     */
    void test_disconnect_all()
    {
      PortServiceRtmMock* obj = new PortServiceRtmMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::RTC::CORBA::PortServiceProxy* ap 
                 = new ::RTC::CORBA::PortServiceProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("disconnect_all"));
      ::RTC::Local::ReturnCode_t ret;
      ret = ap->disconnect_all();
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("disconnect_all"));
      CPPUNIT_ASSERT_EQUAL(::RTC::Local::RTC_OK, ret);
      

      delete ap;
      CORBA::release(ref);


    }
    /*! 
     *
     *
     *
     */
    void test_notify_connect()
    {
      doil::CORBA::CORBAManager& 
                            mgr(doil::CORBA::CORBAManager::instance());
      std::cout <<"Manager Name==>"<< mgr.name() << std::endl;
      PortServiceRtmMock* obj = new PortServiceRtmMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::RTC::CORBA::PortServiceProxy* ap 
                 = new ::RTC::CORBA::PortServiceProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      ::RTC::Local::ConnectorProfile prof;
      ::std::string str1 = "in_ConnectorProfile-name";
      ::std::string str2 = "in_connect_id0";
      ::std::string str3 = "in_ConnectorProfile-properties0-name";
      ::std::string str4 = "2.2";
      prof.name = str1;
      prof.connector_id = str2;
      ::SDOPackage::Local::NameValue nv; 
      nv.name  = str3;
      nv.value = str4;
      prof.properties.push_back(nv);
      IPortServiceMock portservice;

      mgr.registerFactory(portservice.id(), 
                          doil::New<PortServiceServantMock>,
                          doil::Delete<PortServiceServantMock>);
      mgr.activateObject(&portservice);
      prof.ports.push_back(&portservice);
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("notify_connect"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str1));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str2));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str3));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str4));
      ::RTC::Local::ReturnCode_t ret;
      ret = ap->notify_connect(prof);
      mgr.deactivateObject(&portservice);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("notify_connect"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str1));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str2));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str3));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str4));
      CPPUNIT_ASSERT_EQUAL(::RTC::Local::RTC_OK, ret);
      CPPUNIT_ASSERT_EQUAL(::std::string("ConnectorProfile-name"),
			   prof.name);
			
      CPPUNIT_ASSERT_EQUAL(::std::string("connect_id0"),
			   prof.connector_id);

      CPPUNIT_ASSERT_EQUAL((::std::string::size_type)1, 
                           prof.properties.size());
      CPPUNIT_ASSERT_EQUAL(::std::string(
                                    "ConnectorProfile-properties0-name"),
			   prof.properties[0].name);
      CPPUNIT_ASSERT_EQUAL(::std::string("1.1"),
			   prof.properties[0].value);
      CPPUNIT_ASSERT_EQUAL((::std::string::size_type)1, 
                           prof.ports.size());

      


      delete ap;
      CORBA::release(ref);


    }
    /*! 
     *
     *
     *
     */
    void test_notify_disconnect()
    {
      PortServiceRtmMock* obj = new PortServiceRtmMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::RTC::CORBA::PortServiceProxy* ap 
                 = new ::RTC::CORBA::PortServiceProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      ::std::string str = "test_notify_disconnect";
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("notify_disconnect"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str));
      ::RTC::Local::ReturnCode_t ret;
      ret = ap->notify_disconnect(str);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("notify_disconnect"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str));
      CPPUNIT_ASSERT_EQUAL(::RTC::Local::RTC_OK, ret);

      

      delete ap;
      CORBA::release(ref);


    }
    /* test case */
    void test_case0()
    {
    }
  };
}; // namespace PortServiceProxy

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(PortServiceProxy::PortServiceProxyTests);

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
#endif // PortServiceProxy_cpp
