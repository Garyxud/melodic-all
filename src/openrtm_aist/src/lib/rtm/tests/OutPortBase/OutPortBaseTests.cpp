// -*- C++ -*-
/*!
 * @file   OutPortBaseTests.cpp
 * @brief  OutPortBase test class
 * @date   $Date: 2008/02/21 07:36:39 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id$
 *
 */

/*
 * $Log: OutPortBaseTests.cpp,v $
 * Revision 1.2  2008/02/21 07:36:39  arafune
 * Some tests were added.
 *
 * Revision 1.1  2007/12/20 07:50:17  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2006/12/02 18:55:54  n-ando
 * *** empty log message ***
 *
 *
 */

#ifndef OutPortBase_cpp
#define OutPortBase_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/DataPortSkel.h>
#include <rtm/Typename.h>
#include <rtm/PublisherBase.h>
#include <rtm/OutPortBase.h>
#include <rtm/InPort.h>
#include <rtm/CorbaConsumer.h>
#include <rtm/InPortConsumer.h>
#include <rtm/OutPort.h>
#include <rtm/OutPortConnector.h>
#include <rtm/PortAdmin.h>
#include <rtm/PublisherBase.h>
#include <rtm/PublisherFlush.h>
#include <rtm/PublisherNew.h>
#include <rtm/PublisherPeriodic.h>
#include <rtm/SystemLogger.h>
#include <rtm/OutPortPushConnector.h>
#include <rtm/OutPortProvider.h>
#include <rtm/OutPortPullConnector.h>
#include <rtm/ConnectorListener.h>

// ConnectorDataListenerType count
#define cdl_len 10
// ConnectorListenerType count
#define cl_len 7

/*!
 * @class OutPortBaseTests class
 * @brief OutPortBase test
 */
namespace OutPortBase
{

  // ConnectorDataListenerType
  static const char* str_cdl[] =
  {
    "ON_BUFFER_WRITE",
    "ON_BUFFER_FULL",
    "ON_BUFFER_WRITE_TIMEOUT",
    "ON_BUFFER_OVERWRITE",
    "ON_BUFFER_READ", 
    "ON_SEND", 
    "ON_RECEIVED",
    "ON_RECEIVER_FULL", 
    "ON_RECEIVER_TIMEOUT", 
    "ON_RECEIVER_ERROR"
  };

  // ConnectorListenerType
  static const char* str_cl[] =
  {
    "ON_BUFFER_EMPTY",
    "ON_BUFFER_READ_TIMEOUT",
    "ON_SENDER_EMPTY", 
    "ON_SENDER_TIMEOUT", 
    "ON_SENDER_ERROR", 
    "ON_CONNECT",
    "ON_DISCONNECT"
  };

  static int cdl_count;
  static int cl_count;

  class DataListener
    : public RTC::ConnectorDataListenerT<RTC::TimedLong>
  {
  public:
    DataListener(const char* name) : m_name(name)
    {
      ++cdl_count;
    }
    virtual ~DataListener()
    {
      --cdl_count;
    }

    virtual void operator()(const RTC::ConnectorInfo& info,
                            const RTC::TimedLong& data)
    {
      std::cout << "------------------------------"   << std::endl;
      std::cout << "Data Listener: " << m_name       << std::endl;
      std::cout << "Profile::name: " << info.name    << std::endl;
      std::cout << "------------------------------"   << std::endl;
    };
    std::string m_name;
  };


  class ConnListener
    : public RTC::ConnectorListener
  {
  public:
    ConnListener(const char* name) : m_name(name)
    {
      ++cl_count;
    }
    virtual ~ConnListener()
    {
      --cl_count;
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
  class PublisherA
    : public RTC::PublisherBase
  {
  public:
    PublisherA(std::string& footPrints) : m_footPrints(footPrints) {};
    virtual ~PublisherA() { 
      m_footPrints += "a"; 
    }

    void update() { m_footPrints += "A"; }
	  
    std::string& m_footPrints;
    // std::string m_footPrints;
  };
  /*!
   * 
   * 
   *
   */
  class PublisherB
    : public RTC::PublisherBase
  {
  public:
    std::string& m_footPrints;
    // std::string m_footPrints;
    PublisherB(std::string& footPrints) : m_footPrints(footPrints) {};
    virtual ~PublisherB() { 
      m_footPrints += "b"; 
    }
    void update() { m_footPrints += "B"; }
  };
  /*!
   * 
   * 
   *
   */
  class PublisherC
    : public RTC::PublisherBase
  {
  public:
    std::string& m_footPrints;
    //    std::string m_footPrints;
    PublisherC(std::string& footPrints) : m_footPrints(footPrints) {};
    virtual ~PublisherC() {
      m_footPrints += "c";
    }
    void update() { m_footPrints += "C"; }
  };
  /*!
   * 
   * 
   *
   */
  class PublisherD
    : public RTC::PublisherBase
  {
  public:
    std::string& m_footPrints;
    // std::string m_footPrints;
    PublisherD(std::string& footPrints) : m_footPrints(footPrints) {};
    virtual ~PublisherD() {
      m_footPrints += "d";
    }
    void update() { m_footPrints += "D"; }
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
		
    void clearLog(void)
    {
        m_log.clear();
    }
  private:
    std::vector<std::string> m_log;
  };
  /*!
   * 
   * 
   *
   */
  template <class DataType>
  class RingBufferMock
    : public RTC::BufferBase<DataType>
  {
  public:
    BUFFERSTATUS_ENUM
      RingBufferMock(long int length = 8)
      {
          m_logger = NULL;
          logger.log("RingBufferMock::Constructor");
          m_read_return_value = BUFFER_OK;
      }
      virtual ~RingBufferMock(void)
      {
      }
  
    
      /*!
       *
       *
       */
      void set_read_return_value(::RTC::BufferStatus::Enum value)
      {
          m_read_return_value = value;
      }
      /*!
       *
       *
       */
      virtual void init(const coil::Properties& prop)
      {
      }
      /*!
       *
       *
       */
      virtual size_t length(void) const
      {
          return 0;
      }
      /*!
       *
       *
       */
      virtual ReturnCode length(size_t n)
      {
          return ::RTC::BufferStatus::BUFFER_OK; //BUFFER_OK;
      }
      /*!
       *
       *
       */
      virtual ReturnCode reset()
      {
          return ::RTC::BufferStatus::BUFFER_OK; //BUFFER_OK;
      }
      /*!
       *
       *
       */
      virtual DataType* wptr(long int n = 0)
      {
          return &m_data;
      }
      /*!
       *
       *
       */
      virtual ReturnCode advanceWptr(long int n = 1)
      {
          return ::RTC::BufferStatus::BUFFER_OK; //BUFFER_OK;
      }
      /*!
       *
       *
       */
      virtual ReturnCode put(const DataType& value)
      {
          return ::RTC::BufferStatus::BUFFER_OK; //BUFFER_OK;
      }
      /*!
       *
       *
       */
      virtual ReturnCode write(const DataType& value,
                               long int sec = -1, long int nsec = -1)
      {
          return ::RTC::BufferStatus::BUFFER_OK; //BUFFER_OK;
      }
      /*!
       *
       *
       */
      virtual size_t writable() const
      {
          return 0;
      }
      /*!
       *
       *
       */
      virtual bool full(void) const
      {
          return true;
      }
      /*!
       *
       *
       */
      virtual DataType* rptr(long int n = 0)
      {
          return &m_data;
      }
      /*!
       *
       *
       */
      virtual ReturnCode advanceRptr(long int n = 1)
      {
          return ::RTC::BufferStatus::BUFFER_OK; //BUFFER_OK;
      }
      /*!
       *
       *
       */
      virtual ReturnCode get(DataType& value)
      {
          return ::RTC::BufferStatus::BUFFER_OK; //BUFFER_OK;
      }
      /*!
       *
       *
       */
      virtual DataType&  get()
      {
          return m_data;
      }
      /*!
       *
       *
       */
      virtual ReturnCode read(DataType& value,
                              long int sec = -1, long int nsec = -1)
      {
          if (m_logger != NULL)
          {
              m_logger->log("RingBufferMock::read");
          }
          logger.log("RingBufferMock::read");
          return m_read_return_value; //BUFFER_OK;
      }
      /*!
       *
       *
       */
      virtual size_t readable() const
      {
          return 0;
      }
      /*!
       *
       *
       */
      virtual bool empty(void) const
      {
          return true;
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
      DataType m_data;
      std::vector<DataType> m_buffer;
      Logger* m_logger;
      ::RTC::BufferStatus::Enum m_read_return_value;
  };
  template <class DataType>
  Logger RingBufferMock<DataType>::logger;
  typedef RingBufferMock<cdrMemoryStream> CdrRingBufferMock;
  /*!
   *
   *
   *
   */
  class OutPortBaseMock
    : public RTC::OutPortBase
  {
  public:
      /*!
       * 
       * 
       */
      OutPortBaseMock(const char* name, const char* data_type)
        : RTC::OutPortBase(name , data_type)
      {
      }
      /*!
       * 
       */
      coil::Properties get_m_properties()
      {
          return m_properties;
      }
      /*!
       * 
       */
      RTC::OutPortProvider* createProvider_public(RTC::ConnectorProfile& cprof,
                                      coil::Properties& prop)
      {
          return createProvider(cprof, prop);
      }
      /*!
       * 
       */
      RTC::InPortConsumer* createCondumer_public(RTC::ConnectorProfile& cprof,
                                      coil::Properties& prop)
      {
          return createConsumer(cprof, prop);
      }
      /*!
       * 
       */
      RTC::OutPortConnector* createConnector_public(const RTC::ConnectorProfile& cprof,
                                      coil::Properties& prop,
                                      RTC::OutPortProvider* provider)
      {
          return createConnector(cprof,prop,provider);
      }
      /*!
       * 
       */
      virtual RTC::ReturnCode_t
      publishInterfaces_public(RTC::ConnectorProfile& connector_profile)
       {
          return publishInterfaces(connector_profile);
       } 
      /*!
       * 
       */
      virtual RTC::ReturnCode_t
      subscribeInterfaces_public(RTC::ConnectorProfile& connector_profile)
       {
          return subscribeInterfaces(connector_profile);
       } 
      /*!
       * 
       * 
       */
      coil::vstring get_m_consumerTypes()
      {
          return m_consumerTypes;
      }
      /*!
       * 
       * 
       */
      coil::vstring get_m_providerTypes()
      {
          return m_providerTypes;
      }
      /*!
       * 
       */
      void initConsumers_public(void)
      {
          initConsumers();
      }
      /*!
       * 
       */
      void initProviders_public(void)
      {
          initProviders();
      }
      /*!
       * 
       * 
       */
      ConnectorList get_m_connectors()
      {
          return m_connectors;
      }
      /*!
       * 
       * 
       */

      bool write()
      {
          return true;
      }

     
  };
  /*!
   *
   *
   *
   */
  template <class DataType>
  class InPortMock
    : public RTC::InPortBase
  {
  public:
    InPortMock(const char* name, DataType& value) 
     : InPortBase(name,toTypename<DataType>()) {}
    /*!
     * 
     */
    virtual RTC::ReturnCode_t
    publishInterfaces_public(RTC::ConnectorProfile& connector_profile)
     {
        return publishInterfaces(connector_profile);
     } 
    /*!
     * 
     */
    virtual RTC::ReturnCode_t
    subscribeInterfaces_public(RTC::ConnectorProfile& connector_profile)
     {
        return subscribeInterfaces(connector_profile);
     } 
    /*!
     * 
     */
     bool read()
     {
        return true;
     }

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
  
      bool subscribeFromIor(const SDOPackage::NVList& properties)
      {
    
          return true;;
      }
      bool subscribeFromRef(const SDOPackage::NVList& properties)
      {
    
          return true;;
      }
      bool unsubscribeFromIor(const SDOPackage::NVList& properties)
      {
    
          return true;;
      }
      bool unsubscribeFromRef(const SDOPackage::NVList& properties)
      {
    
          return true;;
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
  class OutPortCorbaCdrProviderMock
    : public RTC::OutPortProvider,
      public virtual ::POA_OpenRTM::OutPortCdr,
      public virtual PortableServer::RefCountServantBase
  {

  public:
      OutPortCorbaCdrProviderMock(void)
       {
          setInterfaceType("corba_cdr");
          m_logger = NULL;
       }
      virtual ~OutPortCorbaCdrProviderMock(void)
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
              m_logger->log("OutPortCorbaCdrProviderMock::init");
          }
      }

      /*!
       *
       *
       */
      void setBuffer(RTC::CdrBufferBase* buffer)
      {
        if (m_logger != NULL)
  	{
  	  m_logger->log("OutPortCorbaCdrProviderMock::setBuffer");
  	}
      }
  
      void setListener(RTC::ConnectorInfo& info,
  		     RTC::ConnectorListeners* listeners)
      {
        // m_profile = info;
        // m_listeners = listeners;
      }
      void setConnector(RTC::OutPortConnector* connector)
      {
        // m_connector = connector;
      }

      /*!
       *
       *
       */
      virtual ::OpenRTM::PortStatus get(::OpenRTM::CdrData_out data)
      {
          return ::OpenRTM::PORT_OK;
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
      bool publishInterface(SDOPackage::NVList& prop)
      {
          return true;
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
};
namespace RTC 
{
  /*!
   *
   * for debug 
   *
   */
  ::OutPortBase::Logger logger;
};
namespace OutPortBase
{
  /*!
   * 
   * 
   *
   */
  class OutPortBaseTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(OutPortBaseTests);

    CPPUNIT_TEST(test_constructor);
    CPPUNIT_TEST(test_initConsumers);
    CPPUNIT_TEST(test_initConsumers2);//Consumers are not registered in Factory.
    CPPUNIT_TEST(test_initProviders);
    CPPUNIT_TEST(test_initProviders2);//Providers are not registered in Factory.
    CPPUNIT_TEST(test_init_properties);
    CPPUNIT_TEST(test_name);
    CPPUNIT_TEST(test_connectors_getConnectorXX);
    CPPUNIT_TEST(test_activateInterfaces_deactivateInterfaces);
    CPPUNIT_TEST(test_publishInterfaces);
    CPPUNIT_TEST(test_publishInterfaces2);//dataport.dataflow_type is "push" 
    CPPUNIT_TEST(test_publishInterfaces3);//dataport.dataflow_type is "else" 
    CPPUNIT_TEST(test_publishInterfaces4);//Provider is deleted.  
    CPPUNIT_TEST(test_publishInterfaces5);
    CPPUNIT_TEST(test_subscribeInterfaces);
    CPPUNIT_TEST(test_subscribeInterfaces2);//dataport.dataflow_type is "pull"
    CPPUNIT_TEST(test_subscribeInterfaces3);//dataport.dataflow_type is "else"
    CPPUNIT_TEST(test_subscribeInterfaces4);//Consumer is deleted.
    CPPUNIT_TEST(test_subscribeInterfaces5);
    CPPUNIT_TEST(test_ConnectorListener);

    CPPUNIT_TEST_SUITE_END();
	
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
    RTC::ConnectorListeners m_listeners;
    DataListener *m_datalisteners[cdl_len];
    ConnListener *m_connlisteners[cl_len];

  public:
    RTC::Logger rtclog;
	
    /*!
     * @brief Constructor
     */
    OutPortBaseTests()
    : rtclog("The unit test for OutPortBase")
    {
        int argc(0);
        char** argv(NULL);
        m_pORB = CORBA::ORB_init(argc, argv);
        m_pPOA = PortableServer::POA::_narrow(
		    m_pORB->resolve_initial_references("RootPOA"));
        m_pPOA->the_POAManager()->activate();
        rtclog.setLevel("PARANOID");
    }
		
    /*!
     * @brief Destructor
     */
    ~OutPortBaseTests()
    {
    }
		
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrConsumerMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);


        //既に "subscription_type" 登録されている場合は削除する。
        if( RTC::PublisherFactory::instance().hasFactory("new") )
        {
            RTC::PublisherFactory::instance().removeFactory("new");
        }
        ::RTC::PublisherFactory::
        instance().addFactory("new",
                              ::coil::Creator< ::RTC::PublisherBase,
                                               ::RTC::PublisherNew>,
                              ::coil::Destructor< ::RTC::PublisherBase,
                                                  ::RTC::PublisherNew>);

        if( RTC::PublisherFactory::instance().hasFactory("periodic") )
        {
            RTC::PublisherFactory::instance().removeFactory("periodic");
        }
        ::RTC::PublisherFactory::
        instance().addFactory("periodic",
                              ::coil::Creator< ::RTC::PublisherBase,
                                               ::RTC::PublisherPeriodic>,
                              ::coil::Destructor< ::RTC::PublisherBase,
                                                  ::RTC::PublisherPeriodic>);

        if( RTC::PublisherFactory::instance().hasFactory("flush") )
        {
            RTC::PublisherFactory::instance().removeFactory("flush");
        }
        ::RTC::PublisherFactory::
        instance().addFactory("flush",
                              ::coil::Creator< ::RTC::PublisherBase,
                                               ::RTC::PublisherFlush>,
                              ::coil::Destructor< ::RTC::PublisherBase,
                                                  ::RTC::PublisherFlush>);
    }
		
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    { 
    }
		
    /*!
     * @brief コンスラクタのテスト
     * 
     */
    void test_constructor()
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrConsumerMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);


        //既に "subscription_type" 登録されている場合は削除する。
        if( RTC::PublisherFactory::instance().hasFactory("new") )
        {
            RTC::PublisherFactory::instance().removeFactory("new");
        }
        if( RTC::PublisherFactory::instance().hasFactory("periodic") )
        {
            RTC::PublisherFactory::instance().removeFactory("periodic");
        }
        if( RTC::PublisherFactory::instance().hasFactory("flush") )
        {
            RTC::PublisherFactory::instance().removeFactory("flush");
        }

        {
            OutPortBaseMock outPort("OutPortBaseTest", 
                                    toTypename<RTC::TimedFloat>());

            RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
            portAdmin.registerPort(outPort); 
        
            RTC::PortProfile profile = outPort.getPortProfile();
            coil::Properties prop = NVUtil::toProperties(profile.properties);
            CPPUNIT_ASSERT_EQUAL(std::string("DataOutPort"), 
                                 prop["port.port_type"] );
            CPPUNIT_ASSERT_EQUAL(std::string(toTypename<RTC::TimedFloat>()),
                                 prop["dataport.data_type"] );
            CPPUNIT_ASSERT_EQUAL(std::string(""),
                                 prop["dataport.subscription_type"]);

            portAdmin.deletePort(outPort);
        }
        ::RTC::PublisherFactory::
        instance().addFactory("flush",
                              ::coil::Creator< ::RTC::PublisherBase,
                                               ::RTC::PublisherFlush>,
                              ::coil::Destructor< ::RTC::PublisherBase,
                                                  ::RTC::PublisherFlush>);
        {
            OutPortBaseMock outPort("OutPortBaseTest", 
                                    toTypename<RTC::TimedFloat>());

            RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
            portAdmin.registerPort(outPort); 
        
            RTC::PortProfile profile = outPort.getPortProfile();
            coil::Properties prop = NVUtil::toProperties(profile.properties);
            CPPUNIT_ASSERT_EQUAL(std::string("DataOutPort"), 
                                 prop["port.port_type"] );
            CPPUNIT_ASSERT_EQUAL(std::string(toTypename<RTC::TimedFloat>()),
                                 prop["dataport.data_type"] );
            CPPUNIT_ASSERT_EQUAL(std::string("flush"),
                                 prop["dataport.subscription_type"]);

            portAdmin.deletePort(outPort);
        }

        ::RTC::PublisherFactory::
        instance().addFactory("new",
                              ::coil::Creator< ::RTC::PublisherBase,
                                               ::RTC::PublisherNew>,
                              ::coil::Destructor< ::RTC::PublisherBase,
                                                  ::RTC::PublisherNew>);
        {
            OutPortBaseMock outPort("OutPortBaseTest", 
                                    toTypename<RTC::TimedFloat>());

            RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
            portAdmin.registerPort(outPort); 
        
            RTC::PortProfile profile = outPort.getPortProfile();
            coil::Properties prop = NVUtil::toProperties(profile.properties);
            CPPUNIT_ASSERT_EQUAL(std::string("DataOutPort"), 
                                 prop["port.port_type"] );
            CPPUNIT_ASSERT_EQUAL(std::string(toTypename<RTC::TimedFloat>()),
                                 prop["dataport.data_type"] );
            CPPUNIT_ASSERT_EQUAL(std::string("flush,new"),
                                 prop["dataport.subscription_type"]);

            portAdmin.deletePort(outPort);
        }
        ::RTC::PublisherFactory::
        instance().addFactory("periodic",
                              ::coil::Creator< ::RTC::PublisherBase,
                                               ::RTC::PublisherPeriodic>,
                              ::coil::Destructor< ::RTC::PublisherBase,
                                                  ::RTC::PublisherPeriodic>);
        {
            OutPortBaseMock outPort("OutPortBaseTest", 
                                    toTypename<RTC::TimedFloat>());

            RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
            portAdmin.registerPort(outPort); 
        
            RTC::PortProfile profile = outPort.getPortProfile();
            coil::Properties prop = NVUtil::toProperties(profile.properties);
            CPPUNIT_ASSERT_EQUAL(std::string("DataOutPort"), 
                                 prop["port.port_type"] );
            CPPUNIT_ASSERT_EQUAL(std::string(toTypename<RTC::TimedFloat>()),
                                 prop["dataport.data_type"] );
            CPPUNIT_ASSERT_EQUAL(std::string("flush,new,periodic"),
                                 prop["dataport.subscription_type"]);

            portAdmin.deletePort(outPort);
        }
    }
    /*!
     * @brief initConsumers()メソッドのテスト
     * 
     */
    void test_initConsumers()
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }

        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 

        //"corba_cdr" に InPortCorbaCdrConsumerMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);

        RTC::PortProfile profile = outPort.getPortProfile();
        coil::Properties prop = NVUtil::toProperties(profile.properties);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        coil::vstring cstr = outPort.get_m_consumerTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, cstr.size());

        outPort.initConsumers_public();

        profile = outPort.getPortProfile();
        prop = NVUtil::toProperties(profile.properties);

        //getPortProfileのpropertiesに以下が追加される
        CPPUNIT_ASSERT_EQUAL(std::string("push"),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             prop["dataport.interface_type"]);
 
        //ProviderTypes,ConsumerTypesが取得される
        cstr = outPort.get_m_consumerTypes();
        CPPUNIT_ASSERT((size_t)0!= cstr.size());
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             cstr[0]);

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief initConsumers()メソッドのテスト
     * 
     */
    void test_initConsumers2()
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }

        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 

        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);

        RTC::PortProfile profile = outPort.getPortProfile();
        coil::Properties prop = NVUtil::toProperties(profile.properties);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        coil::vstring cstr = outPort.get_m_consumerTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, cstr.size());

        outPort.initConsumers_public();

        profile = outPort.getPortProfile();
        prop = NVUtil::toProperties(profile.properties);

        //getPortProfileのpropertiesに以下が追加される
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);
 
        //ProviderTypes,ConsumerTypesが取得される
        cstr = outPort.get_m_consumerTypes();
        CPPUNIT_ASSERT((size_t)0== cstr.size());

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief initProviders()メソッドのテスト
     * 
     */
    void test_initProviders()
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }

        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 

        //"corba_cdr" に InPortCorbaCdrConsumerMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);

        RTC::PortProfile profile = outPort.getPortProfile();
        coil::Properties prop = NVUtil::toProperties(profile.properties);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        coil::vstring cstr = outPort.get_m_providerTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, cstr.size());

        outPort.initProviders_public();

        profile = outPort.getPortProfile();
        prop = NVUtil::toProperties(profile.properties);

        //getPortProfileのpropertiesに以下が追加される
        CPPUNIT_ASSERT_EQUAL(std::string("pull"),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             prop["dataport.interface_type"]);
 
        //ProviderTypes,ConsumerTypesが取得される
        cstr = outPort.get_m_providerTypes();
        CPPUNIT_ASSERT((size_t)0!= cstr.size());
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             cstr[0]);

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief initProviders()メソッドのテスト
     * 
     */
    void test_initProviders2()
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }

        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 

        //"corba_cdr" に InPortCorbaCdrConsumerMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);

        RTC::PortProfile profile = outPort.getPortProfile();
        coil::Properties prop = NVUtil::toProperties(profile.properties);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        coil::vstring cstr = outPort.get_m_providerTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, cstr.size());

        outPort.initProviders_public();

        profile = outPort.getPortProfile();
        prop = NVUtil::toProperties(profile.properties);

        //getPortProfileのpropertiesに以下が追加される
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);
 
        //ProviderTypes,ConsumerTypesが取得される
        cstr = outPort.get_m_providerTypes();
        CPPUNIT_ASSERT((size_t)0== cstr.size());

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief init(),properties()メソッドのテスト
     * 
     */
    void test_init_properties()
    {
        OutPortBaseMock outPort("OutPortBaseTest", toTypename<RTC::TimedDouble>());

        coil::Properties prop;
        prop["dataport.interface_type"] = "corba_cdr";
        prop["dataport.dataflow_type"] = "pull";
        prop["dataport.subscription_type"] = "new";

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 

        outPort.init(prop);

        coil::Properties prop2 = outPort.get_m_properties();
        CPPUNIT_ASSERT_EQUAL(prop.size(), prop2.size());
          
        CPPUNIT_ASSERT_EQUAL(prop["dataport.interface_type"],
                             prop2["dataport.interface_type"]);
        CPPUNIT_ASSERT_EQUAL(prop["dataport.dataflow_type"],
                             prop2["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(prop["dataport.subscription_type"],
                             prop2["dataport.subscription_type"]);

        prop2 = outPort.properties();
        CPPUNIT_ASSERT_EQUAL(prop.size(), prop2.size());
          
        CPPUNIT_ASSERT_EQUAL(prop["dataport.interface_type"],
                             prop2["dataport.interface_type"]);
        CPPUNIT_ASSERT_EQUAL(prop["dataport.dataflow_type"],
                             prop2["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(prop["dataport.subscription_type"],
                             prop2["dataport.subscription_type"]);

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief name()メソッドのテスト
     * 
     * - ポート名を正しく取得できるか？
     */
    void test_name()
    {
        OutPortBaseMock outPort("Hello, World!", toTypename<RTC::TimedDouble>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 

        CPPUNIT_ASSERT_EQUAL(std::string("unknown.Hello, World!"), std::string(outPort.getName()));
        portAdmin.deletePort(outPort);
    }
		
    /*!
     * @brief connectors(),getConnectorProfiles()メソッドのテスト
     * 
     */
    void test_connectors_getConnectorXX(void)
    {
        RTC::TimedDouble inbindValue;
        InPortMock<RTC::TimedDouble> inPort("in:OutPortBaseTest",inbindValue);

        OutPortBaseMock outPort("OutPortBaseTest", toTypename<RTC::TimedDouble>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 
        portAdmin.registerPort(inPort); 

        RTC::ConnectorProfile inprof;
        inprof.ports.length(1);
        inprof.ports[0] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(inprof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(inprof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "push"));
        CORBA_SeqUtil::push_back(inprof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "flush"));
        inprof.connector_id = "id0";
        inprof.name = CORBA::string_dup("bar");

        coil::Properties dummy;
        inPort.init(dummy);
        outPort.init(dummy);

        inPort.publishInterfaces_public(inprof);

        std::string vstrid[10] = {"id0","id1","id2","id3","id4",
                                  "id5","id6","id7","id8","id9"};
        std::string vstrname[10] = {"foo0","foo1","foo2","foo3","foo4",
                                    "foo5","foo6","foo7","foo8","foo9"};

        std::string vstrinterface[10] = {"corba_cdr","corba_cdr","corba_cdr",
                                         "corba_cdr","corba_cdr","corba_cdr",
                                         "corba_cdr","corba_cdr","corba_cdr",
                                         "corba_cdr"};
        std::string vstrdataflow[10] = {"push","push","push",
                                         "push","push","push",
                                         "pull","pull","pull","pull"};

        std::string vstrsubscription[10] = {"flush","flush","flush",
                                            "flush","flush","flush",
                                            "flush","flush","flush","flush"};
        
        //
        //connectors()
        //
        for(int ic(0);ic<10;++ic)
        {
            RTC::ConnectorProfile prof;
            prof.ports.length(1);
            prof.ports[0] = outPort.get_port_profile()->port_ref;
            CORBA_SeqUtil::push_back(prof.properties,
                                     NVUtil::newNV("dataport.interface_type",
                                                   vstrinterface[ic].c_str()));
            CORBA_SeqUtil::push_back(prof.properties,
                                     NVUtil::newNV("dataport.dataflow_type",
                                                   vstrdataflow[ic].c_str()));
            CORBA_SeqUtil::push_back(prof.properties,
                                     NVUtil::newNV("dataport.subscription_type",
                                                   vstrsubscription[ic].c_str()));
            CORBA_SeqUtil::push_back(prof.properties,
                                     NVUtil::newNV("dataport.corba_cdr.inport_ior",
                                     NVUtil::toString(inprof.properties,"dataport.corba_cdr.inport_ior").c_str()));
            prof.connector_id = vstrid[ic].c_str();
            prof.name = CORBA::string_dup(vstrname[ic].c_str());


            coil::Properties prop(outPort.properties());
            coil::Properties conn_prop;
            NVUtil::copyToProperties(conn_prop, prof.properties);
            prop << conn_prop.getNode("dataport"); // marge ConnectorProfile
            RTC::OutPortProvider* provider(outPort.createProvider_public(prof, prop));
            outPort.createConnector_public(prof,prop,provider);

            std::vector<RTC::OutPortConnector*> objs = outPort.connectors();

            CPPUNIT_ASSERT_EQUAL((size_t)(ic+1), objs.size());
            CPPUNIT_ASSERT_EQUAL(vstrid[ic], std::string(objs[ic]->id()));
            CPPUNIT_ASSERT_EQUAL(vstrname[ic], std::string(objs[ic]->name()));
        }

        //
        //getConnectorProfiles()
        //
        RTC::ConnectorInfoList list = outPort.getConnectorProfiles();
        CPPUNIT_ASSERT_EQUAL((size_t)10, list.size());
        for(int ic(0);ic<10;++ic)
        {
            CPPUNIT_ASSERT_EQUAL(vstrid[ic], std::string(list[ic].id));
            CPPUNIT_ASSERT_EQUAL(vstrname[ic], std::string(list[ic].name));
            CPPUNIT_ASSERT_EQUAL((size_t)1, list[ic].ports.size());
            
            CPPUNIT_ASSERT_EQUAL(vstrinterface[ic],
                                 list[ic].properties["interface_type"]);
            CPPUNIT_ASSERT_EQUAL(vstrdataflow[ic],
                                 list[ic].properties["dataflow_type"]);
            CPPUNIT_ASSERT_EQUAL(vstrsubscription[ic],
                                 list[ic].properties["subscription_type"]);

        }

        //
        //getConnectorIds()
        //
        coil::vstring ids = outPort.getConnectorIds();
        CPPUNIT_ASSERT_EQUAL((size_t)10, ids.size());
        for(int ic(0);ic<10;++ic)
        {
            CPPUNIT_ASSERT_EQUAL(vstrid[ic], ids[ic]);
        }

        //
        //getConnectorNames()
        //
        coil::vstring names = outPort.getConnectorNames();
        CPPUNIT_ASSERT_EQUAL((size_t)10, names.size());
        for(int ic(0);ic<10;++ic)
        {
            CPPUNIT_ASSERT_EQUAL(vstrname[ic], names[ic]);
        }

        //
        // getConnectorById()
        //
        RTC::OutPortConnector* oc = outPort.getConnectorById("unknown");
        CPPUNIT_ASSERT(oc == 0);
        oc = outPort.getConnectorById("id0");
        CPPUNIT_ASSERT(oc != 0);
        oc = outPort.getConnectorById("id1");
        CPPUNIT_ASSERT(oc != 0);

        //
        // getConnectorByName()
        //
        oc = outPort.getConnectorByName("unknown");
        CPPUNIT_ASSERT(oc == 0);
        oc = outPort.getConnectorByName("foo0");
        CPPUNIT_ASSERT(oc != 0);
        oc = outPort.getConnectorByName("foo1");
        CPPUNIT_ASSERT(oc != 0);

        //
        //getConnectorProfileById()
        //
        for(int ic(0);ic<10;++ic)
        {

            RTC::ConnectorInfo prof;
            bool ret = outPort.getConnectorProfileById(vstrid[ic].c_str(),prof);
            CPPUNIT_ASSERT(ret);
            CPPUNIT_ASSERT_EQUAL(vstrinterface[ic],
                                 prof.properties["interface_type"]);
            CPPUNIT_ASSERT_EQUAL(vstrdataflow[ic],
                                 prof.properties["dataflow_type"]);
            CPPUNIT_ASSERT_EQUAL(vstrsubscription[ic],
                                 prof.properties["subscription_type"]);
        }
        {
            RTC::ConnectorInfo prof;
            bool ret = outPort.getConnectorProfileById("foo",prof);
            CPPUNIT_ASSERT(!ret);
            ret = outPort.getConnectorProfileById("bar",prof);
            CPPUNIT_ASSERT(!ret);
        }

        //
        //getConnectorProfileByName()
        //
        for(int ic(0);ic<10;++ic)
        {
            RTC::ConnectorInfo prof;
            bool ret = outPort.getConnectorProfileByName(vstrname[ic].c_str(),
                                                         prof);
            CPPUNIT_ASSERT(ret);
            CPPUNIT_ASSERT_EQUAL(vstrinterface[ic],
                                 prof.properties["interface_type"]);
            CPPUNIT_ASSERT_EQUAL(vstrdataflow[ic],
                                 prof.properties["dataflow_type"]);
            CPPUNIT_ASSERT_EQUAL(vstrsubscription[ic],
                                 prof.properties["subscription_type"]);
        }
        {
            RTC::ConnectorInfo prof;
            bool ret = outPort.getConnectorProfileByName("foo",prof);
            CPPUNIT_ASSERT(!ret);
            ret = outPort.getConnectorProfileByName("bar",prof);
            CPPUNIT_ASSERT(!ret);
        }

        //
        //publishInterfaces()
        //
        {
            RTC::ConnectorProfile outprof;
            bool ret = outPort.publishInterfaces_public(outprof);
            CPPUNIT_ASSERT(ret);
            {
                const char* value;
                try {
                    NVUtil::find(outprof.properties, "dataport.data_type") >>= value;
	            CPPUNIT_FAIL("dataport.data_type fialure.");
                }
                catch(std::string ex) {
                }
                catch(...) {
	            CPPUNIT_FAIL("dataport.data_type failure.");
                }
            }
        }
        portAdmin.deletePort(outPort);
        portAdmin.deletePort(inPort);
    }

    /*!
     * @brief activateInterfaces(),deactivateInterfaces()メソッドのテスト
     * 
     */
    void test_activateInterfaces_deactivateInterfaces(void)
    {
        RTC::TimedDouble inbindValue;
        InPortMock<RTC::TimedDouble> inPort("in:OutPortBaseTest",inbindValue);

        OutPortBaseMock outPort("out:OutPortBaseTest", toTypename<RTC::TimedDouble>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 
        portAdmin.registerPort(inPort); 

        RTC::ConnectorProfile inprof;
        inprof.ports.length(1);
        inprof.ports[0] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(inprof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(inprof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "push"));
        CORBA_SeqUtil::push_back(inprof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "flush"));
        inprof.connector_id = "id0";
        inprof.name = CORBA::string_dup("bar");
        coil::Properties dummy;
        inPort.init(dummy);
        outPort.init(dummy);
        inPort.publishInterfaces_public(inprof);
        //std::cout<<NVUtil::toString(inprof.properties)<<std::endl;

        std::string vstrid[10] = {"id0","id1","id2","id3","id4",
                                  "id5","id6","id7","id8","id9"};
        std::string vstrname[10] = {"foo0","foo1","foo2","foo3","foo4",
                                    "foo5","foo6","foo7","foo8","foo9"};

        std::string vstrinterface[10] = {"corba_cdr","corba_cdr","corba_cdr",
                                         "corba_cdr","corba_cdr","corba_cdr",
                                         "corba_cdr","corba_cdr","corba_cdr",
                                         "corba_cdr"};
        std::string vstrdataflow[10] = {"push","push","push",
                                         "push","push","push",
                                         "push","push","push","push"};

        std::string vstrsubscription[10] = {"flush","flush","flush",
                                            "flush","flush","flush",
                                            "flush","flush","flush","flush"};
        //
        //
        for(int ic(0);ic<10;++ic)
        {
            RTC::ConnectorProfile prof;
            prof.ports.length(1);
            prof.ports[0] = outPort.get_port_profile()->port_ref;
            CORBA_SeqUtil::push_back(prof.properties,
                                     NVUtil::newNV("dataport.interface_type",
                                                   vstrinterface[ic].c_str()));
            CORBA_SeqUtil::push_back(prof.properties,
                                     NVUtil::newNV("dataport.dataflow_type",
                                                   vstrdataflow[ic].c_str()));
            CORBA_SeqUtil::push_back(prof.properties,
                                     NVUtil::newNV("dataport.subscription_type",
                                                   vstrsubscription[ic].c_str()));
            CORBA_SeqUtil::push_back(prof.properties,
                                     NVUtil::newNV("dataport.corba_cdr.inport_ior",
                                     NVUtil::toString(inprof.properties,"dataport.corba_cdr.inport_ior").c_str()));
            prof.connector_id = vstrid[ic].c_str();
            prof.name = CORBA::string_dup(vstrname[ic].c_str());


            outPort.subscribeInterfaces_public(prof);

        }
        int logcnt;
        logcnt = ::RTC::logger.countLog("OutPortPushConnector::activate"); 
        outPort.activateInterfaces();
        CPPUNIT_ASSERT_EQUAL(logcnt+10,
                  ::RTC::logger.countLog("OutPortPushConnector::activate"));


        logcnt = ::RTC::logger.countLog("OutPortPushConnector::deactivate"); 
        outPort.deactivateInterfaces();
        CPPUNIT_ASSERT_EQUAL(logcnt+10,
                  ::RTC::logger.countLog("OutPortPushConnector::deactivate"));

        portAdmin.deletePort(outPort);
        portAdmin.deletePort(inPort);

    }
    /*!
     * @brief publishInterfaces()メソッドのテスト
     * 
     */
    void test_publishInterfaces(void)
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);


        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("OutPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "pull"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "new"));
        coil::Properties dummy;
        outPort.init(dummy);

        RTC::ReturnCode_t retcode;
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        retcode = outPort.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(1,(int)outPort.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK,retcode);

        prof.connector_id = "id1";
        prof.name = CORBA::string_dup("OutPortBaseTest1");
        retcode = outPort.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(2,(int)outPort.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK,retcode);

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief publishInterfaces()メソッドのテスト
     * 
     */
    void test_publishInterfaces2(void)
    {
        //
        //dataport.dataflow_typeがpushでpublisherInterfaceをコール
        //

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);


        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("OutPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "push"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "new"));
        coil::Properties dummy;
        outPort.init(dummy);

        RTC::ReturnCode_t retcode;
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        retcode = outPort.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK,retcode);

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief publishInterfaces()メソッドのテスト
     * 
     */
    void test_publishInterfaces3(void)
    {
        //
        //dataport.dataflow_typeがelseでpublisherInterfaceをコール
        //

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);


        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("OutPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "else"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "new"));
        coil::Properties dummy;
        outPort.init(dummy);

        RTC::ReturnCode_t retcode;
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        retcode = outPort.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::BAD_PARAMETER,retcode);

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief publishInterfaces()メソッドのテスト
     * 
     */
    void test_publishInterfaces4(void)
    {
        //
        //ProviderなしでpublisherInterfaceをコール
        //

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);


        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("OutPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "pull"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "new"));
        coil::Properties dummy;
        outPort.init(dummy);

        RTC::ReturnCode_t retcode;
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        retcode = outPort.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::BAD_PARAMETER,retcode);


        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief publishInterfaces()メソッドのテスト
     * 
     */
    void test_publishInterfaces5(void)
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);


        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("OutPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "pull"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "new"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.OutPortBaseTests",
                                 "bad_alloc"));
        coil::Properties dummy;
        outPort.init(dummy);

        RTC::ReturnCode_t retcode;
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        retcode = outPort.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_ERROR,retcode);

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief subscribeInterfaces()メソッドのテスト
     * 
     */
    void test_subscribeInterfaces(void)
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrConsumerMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);


        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 

        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("OutPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "push"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "new"));
        coil::Properties dummy;
        outPort.init(dummy);

        RTC::ReturnCode_t retcode;
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        retcode = outPort.subscribeInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(1,(int)outPort.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK,retcode);

        prof.connector_id = "id1";
        prof.name = CORBA::string_dup("OutPortBaseTest1");
        retcode = outPort.subscribeInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(2,(int)outPort.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK,retcode);

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief subscribeInterfaces()メソッドのテスト
     * 
     */
    void test_subscribeInterfaces2(void)
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrConsumerMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);


        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 

        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("OutPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "pull"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "new"));
        RTC::ReturnCode_t retcode;
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        retcode = outPort.subscribeInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_ERROR,retcode);

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief subscribeInterfaces()メソッドのテスト
     * 
     */
    void test_subscribeInterfaces3(void)
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrConsumerMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);


        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 

        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("OutPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "else"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "new"));
        RTC::ReturnCode_t retcode;
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        retcode = outPort.subscribeInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::BAD_PARAMETER,retcode);

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief subscribeInterfaces()メソッドのテスト
     * 
     */
    void test_subscribeInterfaces4(void)
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }


        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 

        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("OutPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "push"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "new"));
        RTC::ReturnCode_t retcode;
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        retcode = outPort.subscribeInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::BAD_PARAMETER,retcode);

        portAdmin.deletePort(outPort);
    }
    /*!
     * @brief subscribeInterfaces()メソッドのテスト
     * 
     */
    void test_subscribeInterfaces5(void)
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に OutPortCorbaCdrProviderMock を登録する。
        RTC::OutPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortProvider, 
                                    OutPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::OutPortProvider, 
                                       OutPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrConsumerMock を登録する。
        RTC::InPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortConsumer, 
                                    InPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::InPortConsumer, 
                                       InPortCorbaCdrConsumerMock>);


        OutPortBaseMock outPort("OutPortBaseTest", 
                                toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 

        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("OutPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "push"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "new"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.OutPortBaseTests",
                                 "bad_alloc"));
        coil::Properties dummy;
        outPort.init(dummy);

        RTC::ReturnCode_t retcode;
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        retcode = outPort.subscribeInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)outPort.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_ERROR,retcode);

        portAdmin.deletePort(outPort);
    }

    /*!
     * @brief addConnectorDataListener(), removeConnectorDataListener(), addConnectorListener(), removeConnectorListener(), isLittleEndian(), connect() メソッドのテスト
     * 
     */
    void test_ConnectorListener(void)
    {
        RTC::TimedLong tdl;
        InPortMock<RTC::TimedLong> inPort("InPort", tdl);

        OutPortBaseMock outPort("OutPortBaseTest", toTypename<RTC::TimedLong>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(outPort); 
        portAdmin.registerPort(inPort); 

        RTC::ConnectorProfile prof;
        prof.ports.length(2);
        prof.ports[0] = inPort.get_port_profile()->port_ref;
        prof.ports[1] = outPort.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "push"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "flush"));
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("test");
        coil::Properties dummy;
        inPort.init(dummy);
        outPort.init(dummy);

        //ConnectorDataListeners settting
        for (int i(0); i<cdl_len; ++i)
          {
            m_datalisteners[i] = new DataListener(str_cdl[i]);
          }

        //ConnectorListeners settting
        for (int i(0); i<cl_len; ++i)
          {
            m_connlisteners[i] = new ConnListener(str_cl[i]);
          }

        // addConnectorDataListener()
        for (int i(0); i<cdl_len; ++i)
          {
            outPort.addConnectorDataListener((RTC::ConnectorDataListenerType)i, 
                                             m_datalisteners[i], true);
          }

        // addConnectorListener()
        for (int i(0); i<cl_len; ++i)
          {
            outPort.addConnectorListener((RTC::ConnectorListenerType)i, 
                                         m_connlisteners[i], true);
          }

        // Listener add count check
        CPPUNIT_ASSERT_EQUAL(10, cdl_count);
        CPPUNIT_ASSERT_EQUAL(7, cl_count);

        inPort.publishInterfaces_public(prof);
        outPort.subscribeInterfaces_public(prof);

        // connect()
        RTC::ReturnCode_t ret;
        ret = outPort.connect(prof);
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ret);

        // isLittleEndian()
        bool bret = outPort.isLittleEndian();
        CPPUNIT_ASSERT( bret );

        outPort.activateInterfaces();
        outPort.deactivateInterfaces();

        ret = outPort.disconnect_all();
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ret);

        portAdmin.deletePort(outPort);
        portAdmin.deletePort(inPort);

        // removeConnectorDataListener()
        for (int i(0); i<cdl_len; ++i)
          {
            outPort.removeConnectorDataListener((RTC::ConnectorDataListenerType)i, 
                                                m_datalisteners[i]);
          }

        // removeConnectorListener()
        for (int i(0); i<cl_len; ++i)
          {
            outPort.removeConnectorListener((RTC::ConnectorListenerType)i, 
                                            m_connlisteners[i]);
          }

        // Listener remove count check
        CPPUNIT_ASSERT_EQUAL(0, cdl_count);
        CPPUNIT_ASSERT_EQUAL(0, cl_count);
    }

  };
}; // namespace OutPortBase


/*!
 * @brief Mock RTC
 */
namespace RTC 
{
  /*!
   *
   * Mock OutPortPushConnector
   *
   */
  /*!
   *
   *
   */
  OutPortPushConnector::OutPortPushConnector(ConnectorInfo info, 
                                             InPortConsumer* consumer,
                                             ConnectorListeners& listeners,
                                             CdrBufferBase* buffer)
    : OutPortConnector(info),
      m_consumer(consumer), m_publisher(0), m_listeners(listeners), m_buffer(buffer)
  {
      if(info.properties["OutPortBaseTests"]=="bad_alloc")
      {
          throw std::bad_alloc();
      }
      m_publisher = createPublisher(info);
      m_publisher->init(info.properties);
      m_publisher->setListener(m_profile, &m_listeners);
      onConnect();
  }
  /*!
   *
   *
   */
  OutPortPushConnector::~OutPortPushConnector()
  {
      onDisconnect();
      disconnect();
  }
  /*!
   *
   *
   */
  ConnectorBase::ReturnCode OutPortPushConnector::disconnect()
  {
      if (m_publisher != 0)
        {
          PublisherFactory& pfactory(PublisherFactory::instance());
          pfactory.deleteObject(m_publisher);
        }
      m_publisher = 0;
      return PORT_OK;
  }
  /*!
   *
   *
   */
  CdrBufferBase* OutPortPushConnector::getBuffer()
  {
      return new ::OutPortBase::CdrRingBufferMock();
  }
  /*!
   *
   *
   */
  void OutPortPushConnector::activate()
  {
      RTC::logger.log("OutPortPushConnector::activate");
  }
  /*!
   *
   *
   */
  void OutPortPushConnector::deactivate()
  {
      RTC::logger.log("OutPortPushConnector::deactivate");
  }
  /*!
   *
   *
   */
  ConnectorBase::ReturnCode
  OutPortPushConnector::write(const cdrMemoryStream& data)
  {
      return PORT_OK;
  }
  /*!
   *
   *
   */
  PublisherBase* OutPortPushConnector::createPublisher(ConnectorInfo& info)
  {
      return new PublisherFlush(); 
  }
  /*!
   *
   *
   */
  CdrBufferBase* OutPortPushConnector::createBuffer(ConnectorInfo& info)
  {
      return new ::OutPortBase::CdrRingBufferMock();

  }
  /*!
   *
   *
   */
  void OutPortPushConnector::onConnect()
  {
    m_listeners.connector_[ON_CONNECT].notify(m_profile);
  }
  /*!
   *
   *
   */
  void OutPortPushConnector::onDisconnect()
  {
    m_listeners.connector_[ON_DISCONNECT].notify(m_profile);
  }

  /*!
   *
   * Mock OutPortPullConnector
   *
   */
  /*!
   *
   *
   */
  OutPortPullConnector::OutPortPullConnector(ConnectorInfo info,
                                             OutPortProvider* provider,
                                             ConnectorListeners& listeners,
                                             CdrBufferBase* buffer)
    : OutPortConnector(info), m_provider(provider), m_listeners(listeners),
      m_buffer(buffer)

  {
      if(info.properties["OutPortBaseTests"]=="bad_alloc")
      {
          throw std::bad_alloc();
      }
  }
  /*!
   *
   *
   */
  OutPortPullConnector::~OutPortPullConnector()
  {
  }
  /*!
   *
   *
   */
  ConnectorBase::ReturnCode
  OutPortPullConnector::write(const cdrMemoryStream& data)
  {
      return PORT_OK;
  }
  /*!
   *
   *
   */
  CdrBufferBase* OutPortPullConnector::getBuffer()
  {
      return new ::OutPortBase::CdrRingBufferMock();
  }
  /*!
   *
   *
   */
  ConnectorBase::ReturnCode OutPortPullConnector::disconnect()
  {
      return PORT_OK;
  }
};


/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(OutPortBase::OutPortBaseTests);

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
#endif // OutPortBase_cpp
