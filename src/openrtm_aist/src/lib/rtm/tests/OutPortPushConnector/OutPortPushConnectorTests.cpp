// -*- C++ -*-
/*!
 * @file   OutPortPushConnectorTests.cpp
 * @brief  OutPortPushConnector test class
 * @date   $Date: 2008/03/13 13:12:25 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 *
 * Copyright (C) 2006
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id: InPortTests.cpp 817 2008-08-06 02:54:26Z n-ando $
 *
 */


#ifndef OutPortPushConnector_cpp
#define OutPortPushConnector_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/Properties.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/DataPortSkel.h>
#include <rtm/Typename.h>
#include <rtm/OutPortPushConnector.h>
#include <rtm/CdrBufferBase.h>
#include <rtm/CORBA_SeqUtil.h>
#include <rtm/NVUtil.h>
#include <rtm/ConnectorBase.h>
#include <rtm/DataPortStatus.h>
#include <rtm/InPortBase.h>
#include <rtm/InPortConsumer.h>
#include <rtm/OutPortBase.h>
#include <rtm/PortAdmin.h>
#include <rtm/CorbaConsumer.h>
#include <rtm/PublisherBase.h>

/*!
 * @class OutPortPushConnectorTests class
 * @brief OutPortPushConnector test
 */
namespace OutPortPushConnector
{

  class DataListener
    : public RTC::ConnectorDataListenerT<RTC::TimedLong>
  {
  public:
    DataListener(const char* name) : m_name(name) {}
    virtual ~DataListener()
    {
      //std::cout << "dtor of " << m_name << std::endl;
    }

    virtual void operator()(const RTC::ConnectorInfo& info,
                            const RTC::TimedLong& data)
    {
      //std::cout << "------------------------------"   << std::endl;
      //std::cout << "Data Listener: " << m_name       << std::endl;
      //std::cout << "Profile::name: " << info.name    << std::endl;
      //std::cout << "------------------------------"   << std::endl;
    };
    std::string m_name;
  };


  class ConnListener
    : public RTC::ConnectorListener
  {
  public:
    ConnListener(const char* name) : m_name(name) {}
    virtual ~ConnListener()
    {
      //std::cout << "dtor of " << m_name << std::endl;
    }

    virtual void operator()(const RTC::ConnectorInfo& info)
    {
      std::cout << "------------------------------"   << std::endl;
      std::cout << "Connector Listener: " << m_name       << std::endl;
      std::cout << "Profile::name:      " << info.name    << std::endl;
      std::cout << "------------------------------"   << std::endl;
    };
    std::string m_name;
  };

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
  class InPortCorbaCdrConsumerMock
    : public RTC::InPortConsumer,
      public RTC::CorbaConsumer< ::OpenRTM::InPortCdr >
  {

  public:
      InPortCorbaCdrConsumerMock(void)
       {
          m_logger = NULL;
       }
      virtual ~InPortCorbaCdrConsumerMock(void)
      {
      }
      /*!
       *
       *
       */
      void init(coil::Properties& prop)
      {
          if (m_logger != NULL)
          {
              m_logger->log("InPortCorbaCdrConsumerMock::init");
          }
      }
      /*!
       *
       *
       */
      RTC::InPortConsumer::ReturnCode put(const cdrMemoryStream& data)
      {
          return PORT_OK;
      }
      /*!
       *
       *
       */
      void publishInterfaceProfile(SDOPackage::NVList& properties)
      {
          return;
      }

      /*!
       *
       *
       */
      bool subscribeInterface(const SDOPackage::NVList& properties)
      {
    
          return true;;
      }
  
      /*!
       *
       *
       */
      void unsubscribeInterface(const SDOPackage::NVList& properties)
      {
      }
  
  
      /*!
       *
       *
       */
      void setLogger(Logger* logger)
      {
          m_logger = logger;
      }
  private:
    Logger* m_logger;

  };
  /*!
   * 
   * 
   *
   */
  class PublisherFlushMock
    : public RTC::PublisherBase
  {
  public:
      PublisherFlushMock()
      {
          m_logger = NULL;
      }
      ~PublisherFlushMock()
      {
      }
      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode write(const cdrMemoryStream& data,
                                           unsigned long sec,
                                           unsigned long usec)
      {
          cdrMemoryStream cdr(data);
          CORBA::ULong inlen = cdr.bufSize();

          RTC::TimedLong td;
          td <<= cdr;
          std::stringstream ss;
          ss << td.data;
          if (m_logger != NULL)
          {
              m_logger->log("PublisherFlushMock::write");
              m_logger->log(ss.str());
          }
          logger.log("PublisherFlushMock::write");
          logger.log(ss.str());
          return PORT_OK;
      }
      /*!
       *
       *
       */
      bool isActive()
      {
          return true;
      }

      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode activate()
      {
          if (m_logger != NULL)
          {
              m_logger->log("PublisherFlushMock::activate");
          }
          logger.log("PublisherFlushMock::activate");
          return PORT_OK;
      }

      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode deactivate()
      {
          if (m_logger != NULL)
          {
              m_logger->log("PublisherFlushMock::deactivate");
          }
          logger.log("PublisherFlushMock::deactivate");
          return PORT_OK;
      }
      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode init(coil::Properties& prop)
      {
          return PORT_OK;
      }
      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode setConsumer(RTC::InPortConsumer* consumer)
      {
          if (m_logger != NULL)
          {
              m_logger->log("PublisherFlushMock::setConsumer");
          }
          logger.log("PublisherFlushMock::setConsumer");
          return PORT_OK;
      }

      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode setBuffer(RTC::CdrBufferBase* buffer)
      {
          if (m_logger != NULL)
          {
              m_logger->log("PublisherFlushMock::setBuffer");
              if(buffer == NULL)
              {
                  m_logger->log("buffer NG");
              }
              else
              {
                  m_logger->log("buffer OK");
              }
          }
          logger.log("PublisherFlushMock::setBuffer");
          if(buffer == NULL)
          {
              logger.log("buffer NG");
          }
          else
          {
              logger.log("buffer OK");
          }
          return PORT_OK;
      }

      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode setListener(RTC::ConnectorInfo& info,
                                                 RTC::ConnectorListeners* listeners)
      {
          if (m_logger != NULL)
          {
              m_logger->log("PublisherFlushMock::setListener");
              if (listeners == 0)
              {
                  m_logger->log("listeners NG");
              }
              else
              {
                  m_logger->log("listeners OK");
              }
          }
          logger.log("PublisherFlushMock::setListener");
          if (listeners == 0)
          {
              logger.log("listeners NG");
          }
          else
          {
              logger.log("listeners OK");
          }
          return PORT_OK;
      }

      /*!
       *
       *
       */
      void setLogger(Logger* logger)
      {
          m_logger = logger;
      }
    static Logger logger;
  private:
    Logger* m_logger;
  };

  Logger PublisherFlushMock::logger;

  /*!
   * 
   * 
   *
   */
  class PublisherNewMock
    : public RTC::PublisherBase
  {
  public:
      PublisherNewMock()
      {
          m_logger = NULL;
      }
      ~PublisherNewMock()
      {
      }
      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode write(const cdrMemoryStream& data,
                                           unsigned long sec,
                                           unsigned long usec)
      {
          cdrMemoryStream cdr(data);
          CORBA::ULong inlen = cdr.bufSize();

          RTC::TimedLong td;
          td <<= cdr;
          std::stringstream ss;
          ss << td.data;

          if (m_logger != NULL)
          {
              m_logger->log("PublisherNewMock::write");
              m_logger->log(ss.str());
          }
          logger.log("PublisherNewMock::write");
          logger.log(ss.str());
          return PORT_OK;
      }
      /*!
       *
       *
       */
      bool isActive()
      {
          return true;
      }

      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode activate()
      {
          if (m_logger != NULL)
          {
              m_logger->log("PublisherNewMock::activate");
          }
          logger.log("PublisherNewMock::activate");
          return PORT_OK;
      }

      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode deactivate()
      {
          if (m_logger != NULL)
          {
              m_logger->log("PublisherNewMock::deactivate");
          }
          logger.log("PublisherNewMock::deactivate");
          return PORT_OK;
      }

      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode init(coil::Properties& prop)
      {
          return PORT_OK;
      }

      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode setConsumer(RTC::InPortConsumer* consumer)
      {
          if (m_logger != NULL)
          {
              m_logger->log("PublisherNewMock::setConsumer");
          }
          logger.log("PublisherNewMock::setConsumer");
          return PORT_OK;
      }

      /*!
       *
       *
       */
      PublisherBase::ReturnCode setBuffer(RTC::CdrBufferBase* buffer)
      {
          if (m_logger != NULL)
          {
              m_logger->log("PublisherNewMock::setBuffer");
              if(buffer == NULL)
              {
                  m_logger->log("buffer NG");
              }
              else
              {
                  m_logger->log("buffer OK");
              }
          }
          logger.log("PublisherNewMock::setBuffer");
          if(buffer == NULL)
          {
              logger.log("buffer NG");
          }
          else
          {
              logger.log("buffer OK");
          }
          return PORT_OK;
      }

      /*!
       *
       *
       */
      RTC::PublisherBase::ReturnCode setListener(RTC::ConnectorInfo& info,
                                                 RTC::ConnectorListeners* listeners)
      {
          if (m_logger != NULL)
          {
              m_logger->log("PublisherNewMock::setListener");
              if (listeners == 0)
              {
                  m_logger->log("listeners NG");
              }
              else
              {
                  m_logger->log("listeners OK");
              }
          }
          logger.log("PublisherNewMock::setListener");
          if (listeners == 0)
          {
              logger.log("listeners NG");
          }
          else
          {
              logger.log("listeners OK");
          }
          return PORT_OK;
      }

      /*!
       *
       *
       */
      void setLogger(Logger* logger)
      {
          m_logger = logger;
      }
    static Logger logger;
  private:
    Logger* m_logger;
  };

  Logger PublisherNewMock::logger;

  class OutPortPushConnectorTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(OutPortPushConnectorTests);

    CPPUNIT_TEST(test_OutPortPushConnector);
    CPPUNIT_TEST(test_write);
    CPPUNIT_TEST(test_disconnect_getBuffer);
    CPPUNIT_TEST(test_activate_deactivate);

    CPPUNIT_TEST_SUITE_END();
		
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;


  public:
    RTC::ConnectorListeners m_listeners;
	
    /*!
     * @brief Constructor
     */
    OutPortPushConnectorTests()
    {

        int argc(0);
        char** argv(NULL);
        m_pORB = CORBA::ORB_init(argc, argv);
        m_pPOA = PortableServer::POA::_narrow(
		    m_pORB->resolve_initial_references("RootPOA"));
        m_pPOA->the_POAManager()->activate();

    }
		
    /*!
     * @brief Destructor
     */
    ~OutPortPushConnectorTests()
    {
    }
    
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
        //ConnectorDataListeners
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE].addListener(
                                   new DataListener("ON_BUFFER_WRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_FULL].addListener(
                                   new DataListener("ON_BUFFER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE_TIMEOUT].addListener(
                                   new DataListener("ON_BUFFER_WRITE_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_OVERWRITE].addListener(
                                   new DataListener("ON_BUFFER_OVERWRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_READ].addListener(
                                   new DataListener("ON_BUFFER_READ"), true);
        m_listeners.connectorData_[RTC::ON_SEND].addListener(
                                   new DataListener("ON_SEND"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVED].addListener(
                                   new DataListener("ON_RECEIVED"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_FULL].addListener(
                                   new DataListener("ON_RECEIVER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_TIMEOUT].addListener(
                                   new DataListener("ON_RECEIVER_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_ERROR].addListener(
                                   new DataListener("ON_RECEIVER_ERROR"), true);

        //ConnectorListeners
        m_listeners.connector_[RTC::ON_BUFFER_EMPTY].addListener(
                                    new ConnListener("ON_BUFFER_EMPTY"), true);
        m_listeners.connector_[RTC::ON_BUFFER_READ_TIMEOUT].addListener(
                                    new ConnListener("ON_BUFFER_READ_TIMEOUT"), true);
        m_listeners.connector_[RTC::ON_SENDER_EMPTY].addListener(
                                    new ConnListener("ON_SENDER_EMPTY"), true);
        m_listeners.connector_[RTC::ON_SENDER_TIMEOUT].addListener(
                                    new ConnListener("ON_SENDER_TIMEOUT"), true);
        m_listeners.connector_[RTC::ON_SENDER_ERROR].addListener(
                                    new ConnListener("ON_SENDER_ERROR"), true);
        m_listeners.connector_[RTC::ON_CONNECT].addListener(
                                    new ConnListener("ON_CONNECT"), true);
        m_listeners.connector_[RTC::ON_DISCONNECT].addListener(
                                    new ConnListener("ON_DISCONNECT"), true);
    }
		
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    {
    }
		
    /*!
     * @brief Constructor メソッドテスト
     * 
     */
    void test_OutPortPushConnector()
    {
        ::RTC::PublisherFactory::
        instance().addFactory("flush",
                              ::coil::Creator< ::RTC::PublisherBase,
                                                      PublisherFlushMock>,
                              ::coil::Destructor< ::RTC::PublisherBase,
                                                         PublisherFlushMock>);
        ::RTC::PublisherFactory::
        instance().addFactory("new",
                              ::coil::Creator< ::RTC::PublisherBase,
                                               PublisherNewMock>,
                              ::coil::Destructor< ::RTC::PublisherBase,
                                               PublisherNewMock>);
        RTC::ConnectorProfile prof;
        CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.interface_type",
					     "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.dataflow_type",
					     "push"));
        CORBA_SeqUtil::push_back(prof.properties,
	  		       NVUtil::newNV("dataport.subscription_type",
					     "new"));
        coil::Properties prop;
        {
            coil::Properties conn_prop;
            NVUtil::copyToProperties(conn_prop, prof.properties);
            prop << conn_prop.getNode("dataport"); // marge ConnectorProfile
        }
        InPortCorbaCdrConsumerMock* consumer = new InPortCorbaCdrConsumerMock();
        Logger logger;
        consumer->setLogger(&logger);
        RTC::ConnectorInfo profile_new(prof.name,
                                       prof.connector_id,
                                       CORBA_SeqUtil::refToVstring(prof.ports),
                                       prop); 
        RTC::OutPortConnector* connector(0);
        CPPUNIT_ASSERT_EQUAL(0, 
                           logger.countLog("InPortCorbaCdrConsumerMock::init"));
        CPPUNIT_ASSERT_EQUAL(0, 
            PublisherNewMock::logger.countLog("PublisherNewMock::setBuffer"));
        CPPUNIT_ASSERT_EQUAL(0, 
            PublisherNewMock::logger.countLog("buffer OK"));
        CPPUNIT_ASSERT_EQUAL(0, 
            PublisherNewMock::logger.countLog("PublisherNewMock::setConsumer"));
        connector = new RTC::OutPortPushConnector(profile_new, consumer, m_listeners);
        CPPUNIT_ASSERT_EQUAL(1, 
                           logger.countLog("InPortCorbaCdrConsumerMock::init"));
        CPPUNIT_ASSERT_EQUAL(1, 
            PublisherNewMock::logger.countLog("PublisherNewMock::setBuffer"));
        CPPUNIT_ASSERT_EQUAL(1, 
            PublisherNewMock::logger.countLog("buffer OK"));
        CPPUNIT_ASSERT_EQUAL(0, 
            PublisherNewMock::logger.countLog("buffer NG"));
        CPPUNIT_ASSERT_EQUAL(1, 
            PublisherNewMock::logger.countLog("PublisherNewMock::setConsumer"));

        delete connector;

        //subscription_type
        //Flush
        CORBA_SeqUtil::push_back(prof.properties,
	  		       NVUtil::newNV("dataport.subscription_type",
					     ""));
        {
            coil::Properties conn_prop;
            NVUtil::copyToProperties(conn_prop, prof.properties);
            prop << conn_prop.getNode("dataport"); // marge ConnectorProfile
        }
        RTC::ConnectorInfo profile_flush(prof.name,
                                         prof.connector_id,
                                         CORBA_SeqUtil::refToVstring(prof.ports),
                                         prop); 
        CPPUNIT_ASSERT_EQUAL(1, 
                           logger.countLog("InPortCorbaCdrConsumerMock::init"));
        CPPUNIT_ASSERT_EQUAL(0, 
        PublisherFlushMock::logger.countLog("PublisherFlushMock::setBuffer"));
        CPPUNIT_ASSERT_EQUAL(0, 
        PublisherFlushMock::logger.countLog("buffer OK"));
        CPPUNIT_ASSERT_EQUAL(0, 
        PublisherFlushMock::logger.countLog("PublisherFlushMock::setConsumer"));
        connector = new RTC::OutPortPushConnector(profile_flush, consumer, m_listeners);
        CPPUNIT_ASSERT_EQUAL(2, 
                           logger.countLog("InPortCorbaCdrConsumerMock::init"));
        CPPUNIT_ASSERT_EQUAL(1, 
        PublisherFlushMock::logger.countLog("PublisherFlushMock::setBuffer"));
        CPPUNIT_ASSERT_EQUAL(1, 
        PublisherFlushMock::logger.countLog("buffer OK"));
        CPPUNIT_ASSERT_EQUAL(1, 
        PublisherFlushMock::logger.countLog("PublisherFlushMock::setConsumer"));

        //consumer
        delete connector;
        

        //consumer
        RTC::OutPortConnector* connector_err(0);
        try {
            RTC::ConnectorProfile prof_err;
            {
                coil::Properties conn_prop;
                NVUtil::copyToProperties(conn_prop, prof_err.properties);
                prop << conn_prop.getNode("dataport"); // marge ConnectorProfile
            }
            RTC::ConnectorInfo profile_err(prof_err.name,
                                          prof_err.connector_id,
                                          CORBA_SeqUtil::refToVstring(prof_err.ports),
                                          prop); 
            connector_err = new RTC::OutPortPushConnector(profile_err, NULL, m_listeners);
            CPPUNIT_FAIL("The exception was not thrown. ");
        }
        catch(std::bad_alloc& e)
        {
        }
        catch(...)
        {
            CPPUNIT_FAIL("The exception not intended was thrown .");
        }
        delete connector_err;
        delete consumer;
    }
    /*!
     * @brief write メソッドテスト
     * 
     */
    void test_write()
    {
        ::RTC::PublisherFactory::
        instance().addFactory("new",
                              ::coil::Creator< ::RTC::PublisherBase,
                                                      PublisherNewMock>,
                              ::coil::Destructor< ::RTC::PublisherBase,
                                                         PublisherNewMock>);
        RTC::ConnectorProfile prof;
        CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.interface_type",
					     "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.dataflow_type",
					     "push"));
        CORBA_SeqUtil::push_back(prof.properties,
	  		       NVUtil::newNV("dataport.subscription_type",
					     "new"));
        coil::Properties prop;
        {
            coil::Properties conn_prop;
            NVUtil::copyToProperties(conn_prop, prof.properties);
            prop << conn_prop.getNode("dataport"); // marge ConnectorProfile
        }
        InPortCorbaCdrConsumerMock* consumer = new InPortCorbaCdrConsumerMock();
        Logger logger;
        consumer->setLogger(&logger);
        RTC::ConnectorInfo profile_new(prof.name,
                                       prof.connector_id,
                                       CORBA_SeqUtil::refToVstring(prof.ports),
                                       prop); 
        RTC::OutPortConnector* connector(0);
        connector = new RTC::OutPortPushConnector(profile_new, consumer, m_listeners);
        CPPUNIT_ASSERT_EQUAL(0, 
               PublisherNewMock::logger.countLog("PublisherNewMock::write"));
        CPPUNIT_ASSERT_EQUAL(0, 
               PublisherNewMock::logger.countLog("12345"));
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 12345;
        td >>= cdr;

        RTC::ConnectorBase::ReturnCode ret;
        ret = connector->write(cdr);
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::PORT_OK, ret);

        CPPUNIT_ASSERT_EQUAL(1, 
               PublisherNewMock::logger.countLog("PublisherNewMock::write"));
        CPPUNIT_ASSERT_EQUAL(1, 
               PublisherNewMock::logger.countLog("12345"));

        delete connector;
        delete consumer;
    }
    /*!
     * @brief disconnect メソッドテスト
     * 
     */
    void test_disconnect_getBuffer()
    {
        ::RTC::PublisherFactory::
        instance().addFactory("new",
                              ::coil::Creator< ::RTC::PublisherBase,
                                                      PublisherNewMock>,
                              ::coil::Destructor< ::RTC::PublisherBase,
                                                         PublisherNewMock>);
        RTC::ConnectorProfile prof;
        CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.interface_type",
					     "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.dataflow_type",
					     "push"));
        CORBA_SeqUtil::push_back(prof.properties,
	  		       NVUtil::newNV("dataport.subscription_type",
					     "new"));
        coil::Properties prop;
        {
            coil::Properties conn_prop;
            NVUtil::copyToProperties(conn_prop, prof.properties);
            prop << conn_prop.getNode("dataport"); // marge ConnectorProfile
        }
        InPortCorbaCdrConsumerMock* consumer = new InPortCorbaCdrConsumerMock();
        Logger logger;
        consumer->setLogger(&logger);
        RTC::ConnectorInfo profile_new(prof.name,
                                       prof.connector_id,
                                       CORBA_SeqUtil::refToVstring(prof.ports),
                                       prop); 
        RTC::OutPortConnector* connector(0);
        connector = new RTC::OutPortPushConnector(profile_new, consumer, m_listeners);
        CPPUNIT_ASSERT(connector->getBuffer());

        RTC::ConnectorBase::ReturnCode ret;
        ret = connector->disconnect();
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::PORT_OK, ret);

        CPPUNIT_ASSERT(!connector->getBuffer());

        delete connector;
        delete consumer;
    }
    /*!
     * @brief activate メソッドテスト
     * 
     */
    void test_activate_deactivate()
    {
        ::RTC::PublisherFactory::
        instance().addFactory("new",
                              ::coil::Creator< ::RTC::PublisherBase,
                                                      PublisherNewMock>,
                              ::coil::Destructor< ::RTC::PublisherBase,
                                                         PublisherNewMock>);
        RTC::ConnectorProfile prof;
        CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.interface_type",
					     "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.dataflow_type",
					     "push"));
        CORBA_SeqUtil::push_back(prof.properties,
	  		       NVUtil::newNV("dataport.subscription_type",
					     "new"));
        coil::Properties prop;
        {
            coil::Properties conn_prop;
            NVUtil::copyToProperties(conn_prop, prof.properties);
            prop << conn_prop.getNode("dataport"); // marge ConnectorProfile
        }
        InPortCorbaCdrConsumerMock* consumer = new InPortCorbaCdrConsumerMock();
        Logger logger;
        consumer->setLogger(&logger);
        RTC::ConnectorInfo profile_new(prof.name,
                                       prof.connector_id,
                                       CORBA_SeqUtil::refToVstring(prof.ports),
                                       prop); 
        RTC::OutPortConnector* connector(0);
        connector = new RTC::OutPortPushConnector(profile_new, consumer, m_listeners);
        CPPUNIT_ASSERT_EQUAL(0, 
               PublisherNewMock::logger.countLog("PublisherNewMock::activate"));
        connector->activate();
        CPPUNIT_ASSERT_EQUAL(1, 
               PublisherNewMock::logger.countLog("PublisherNewMock::activate"));

        delete connector;
        connector = new RTC::OutPortPushConnector(profile_new, consumer, m_listeners);
        CPPUNIT_ASSERT_EQUAL(0, 
             PublisherNewMock::logger.countLog("PublisherNewMock::deactivate"));

        connector->deactivate();
        CPPUNIT_ASSERT_EQUAL(1, 
             PublisherNewMock::logger.countLog("PublisherNewMock::deactivate"));

        delete connector;
        delete consumer;
    }
  };
}; // namespace OutPortPushConnector

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(OutPortPushConnector::OutPortPushConnectorTests);

#ifdef LOCAL_MAIN
int main(int argc, char* argv[])
{

  FORMAT format = TEXT_OUT;
  int target = 0;
  std::string xsl;
  std::string ns;
  std::string fname;
  std::ofstream ofs;

  int i(1);
  while (i < argc)
    {
      std::string arg(argv[i]);
      std::string next_arg;
      if (i + 1 < argc) next_arg = argv[i + 1];
      else              next_arg = "";

      if (arg == "--text") { format = TEXT_OUT; break; }
      if (arg == "--xml")
	{
	  if (next_arg == "")
	    {
	      fname = argv[0];
	      fname += ".xml";
	    }
	  else
	    {
	      fname = next_arg;
	    }
	  format = XML_OUT;
	  ofs.open(fname.c_str());
	}
      if ( arg == "--compiler"  ) { format = COMPILER_OUT; break; }
      if ( arg == "--cerr"      ) { target = 1; break; }
      if ( arg == "--xsl"       )
	{
	  if (next_arg == "") xsl = "default.xsl"; 
	  else                xsl = next_arg;
	}
      if ( arg == "--namespace" )
	{
	  if (next_arg == "")
	    {
	      std::cerr << "no namespace specified" << std::endl;
	      exit(1); 
	    }
	  else
	    {
	      xsl = next_arg;
	    }
	}
      ++i;
    }
  CppUnit::TextUi::TestRunner runner;
  if ( ns.empty() )
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry().makeTest());
  else
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry(ns).makeTest());
  CppUnit::Outputter* outputter = 0;
  std::ostream* stream = target ? &std::cerr : &std::cout;
  switch ( format )
    {
    case TEXT_OUT :
      outputter = new CppUnit::TextOutputter(&runner.result(),*stream);
      break;
    case XML_OUT :
      std::cout << "XML_OUT" << std::endl;
      outputter = new CppUnit::XmlOutputter(&runner.result(),
					    ofs, "shift_jis");
      static_cast<CppUnit::XmlOutputter*>(outputter)->setStyleSheet(xsl);
      break;
    case COMPILER_OUT :
      outputter = new CppUnit::CompilerOutputter(&runner.result(),*stream);
      break;
    }
  runner.setOutputter(outputter);
  runner.run();
  return 0; // runner.run() ? 0 : 1;
}
#endif // MAIN
#endif // InPort_cpp
