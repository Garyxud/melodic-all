// -*- C++ -*-
/*!
 * @file   InPortBaseTests.cpp
 * @brief  InPortBase test class
 * @date   $Date: 2008/02/21 07:36:39 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id$
 *
 */

/*
 * $Log: InPortBaseTests.cpp,v $
 *
 *
 */

#ifndef InPortBase_cpp
#define InPortBase_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/DataPortSkel.h>
#include <rtm/RTC.h>
#include <rtm/Typename.h>
#include <rtm/InPortBase.h>
#include <rtm/CdrBufferBase.h>
#include <rtm/PortAdmin.h>
#include <rtm/InPortConnector.h>
#include <rtm/InPortProvider.h>
#include <rtm/InPortConsumer.h>
#include <rtm/OutPortProvider.h>
#include <rtm/OutPortConsumer.h>
#include <rtm/CorbaConsumer.h>
#include <rtm/InPortPushConnector.h>
#include <rtm/InPortProvider.h>
#include <rtm/ConnectorListener.h>
#include <rtm/OutPort.h>
#include <rtm/OutPortBase.h>
#include <rtm/InPortCorbaCdrProvider.h>
#include <rtm/OutPortCorbaCdrConsumer.h>

// ConnectorDataListenerType count
#define cdl_len 10
// ConnectorListenerType count
#define cl_len 7

namespace InPortBase
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
  class InPortCorbaCdrProviderMock
    : public RTC::InPortProvider,
      public virtual POA_OpenRTM::InPortCdr,
      public virtual PortableServer::RefCountServantBase
  {

  public:
      InPortCorbaCdrProviderMock(void)
       {
          m_logger = NULL;
          // PortProfile setting
          setInterfaceType("corba_cdr");
    
          // ConnectorProfile setting
          m_objref = this->_this();
    
          // set InPort's reference
          CORBA::ORB_ptr orb = ::RTC::Manager::instance().getORB();
          CORBA_SeqUtil::
              push_back(m_properties,
                NVUtil::newNV("dataport.corba_cdr.inport_ior",
                              orb->object_to_string(m_objref.in())));
          CORBA_SeqUtil::
              push_back(m_properties,
                NVUtil::newNV("dataport.corba_cdr.inport_ref",
                              m_objref));
       }
      virtual ~InPortCorbaCdrProviderMock(void)
      {
          PortableServer::ObjectId_var oid;
          oid = _default_POA()->servant_to_id(this);
          _default_POA()->deactivate_object(oid);
      }
      /*!
       *
       *
       */
      void setBuffer(RTC::BufferBase<cdrMemoryStream>* buffer)
      {
          if (m_logger != NULL)
          {
              m_logger->log("InPortCorbaCdrProviderMock::setBuffer");
          }
      }
      /*!
       *
       *
       */
      ::OpenRTM::PortStatus put(const ::OpenRTM::CdrData& data)
         throw (CORBA::SystemException)
      {
          return ::OpenRTM::PORT_OK;
      }
      /*!
       *
       *
       */
      void init(coil::Properties& prop)
      {
          if (m_logger != NULL)
          {
              m_logger->log("InPortCorbaCdrProviderMock::init");
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
/*
      bool publishInterface(SDOPackage::NVList& prop)
      {
          return true;
      }
*/
      /*!
       *
       *
       */
      void setLogger(Logger* logger)
      {
          m_logger = logger;
      }

      void setListener(RTC::ConnectorInfo& info,
  		     RTC::ConnectorListeners* listeners)
      {
        // m_profile = info;
        // m_listeners = listeners;
      }
      void setConnector(RTC::InPortConnector* connector)
      {
        // m_connector = connector;
      }

  private:
    Logger* m_logger;
    ::OpenRTM::InPortCdr_var m_objref;

  };

  /*!
   * 
   * 
   *
   */
  class OutPortCorbaCdrConsumerMock
    : public RTC::OutPortConsumer,
      public RTC::CorbaConsumer< ::OpenRTM::OutPortCdr >
  {

  public:
      OutPortCorbaCdrConsumerMock(void)
       {
          m_logger = NULL;
       }
      virtual ~OutPortCorbaCdrConsumerMock(void)
      {
      }
      /*!
       *
       *
       */
      void setBuffer(RTC::BufferBase<cdrMemoryStream>* buffer)
      {
          if (m_logger != NULL)
          {
              m_logger->log("OutPortCorbaCdrConsumerMock::setBuffer");
          }
      }
      /*!
       *
       *
       */
      ::OpenRTM::PortStatus put(const ::OpenRTM::CdrData& data)
         throw (CORBA::SystemException)
      {
          return ::OpenRTM::PORT_OK;
      }
      /*!
       *
       *
       */
      void init(coil::Properties& prop)
      {
          if (m_logger != NULL)
          {
              m_logger->log("OutPortCorbaCdrConsumerMock::init");
          }
      }
      /*!
       *
       *
       */
      RTC::OutPortConsumer::ReturnCode put(const cdrMemoryStream& data)
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
      virtual ReturnCode get(cdrMemoryStream& data)
      {
          return PORT_OK;
      }
  
      void setListener(RTC::ConnectorInfo& info,
  		       RTC::ConnectorListeners* listeners)
      {
        // m_profile = info;
        // m_listeners = listeners;
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
   */
  class InPortBaseMock
    : public RTC::InPortBase
  {
  public:
      /*!
       * 
       * 
       */
      InPortBaseMock(const char* name, const char* data_type)
        : RTC::InPortBase(name , data_type)
      {
          rtclog.setLevel("PARANOID");
      }
      /*!
       * 
       * 
       */
      virtual ~InPortBaseMock()
      {
      }
      /*!
       * 
       * 
       */
      RTC::CdrBufferBase* getThebuffer()
      {
          return m_thebuffer;
      }
      /*!
       * 
       * 
       */
      coil::vstring getProviderTypes()
      {
          return m_providerTypes;
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
      void initConsumers_public()
      {
          initConsumers();
      }
      /*!
       * 
       * 
       */
      void initProviders_public()
      {
          initProviders();
      }
      /*!
       * 
       * 
       */
      void setSinglebuffer()
      {
          m_singlebuffer = true;
      }
      /*!
       * 
       * 
       */
      void setMultibuffer()
      {
          m_singlebuffer = false;
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
      virtual RTC::ReturnCode_t publishInterfaces_public(RTC::ConnectorProfile& connector_profile)
      {
          RTC::ReturnCode_t ret = RTC::InPortBase::publishInterfaces(connector_profile);
          return ret;
      }
      /*!
       * 
       * 
       */
      virtual RTC::ReturnCode_t subscribeInterfaces_public(RTC::ConnectorProfile& connector_profile)
      {
          RTC::ReturnCode_t ret = RTC::InPortBase::subscribeInterfaces(connector_profile);
          return ret;
      }
      /*!
       * 
       * 
       */
      RTC::InPortConnector*
      createConnector_public(RTC::ConnectorProfile& cprof, 
                            coil::Properties& prop,
                            RTC::InPortProvider* provider)
      {
         return RTC::InPortBase::createConnector(cprof, prop, provider);
      }
      /*!
       * 
       * 
       */
      RTC::InPortProvider* createProvider_public(RTC::ConnectorProfile& cprof,
                                      coil::Properties& prop)
      {
          return RTC::InPortBase::createProvider(cprof, prop);
      }
      /*!
       * 
       * 
       */
      coil::Properties get_m_properties()
      {
          return m_properties;
      }
      /*!
       * 
       * 
       */
      void init(coil::Properties& prop)
      {
          RTC::InPortBase::init(prop);
      }
      /*!
       * 
       */
      bool read()
      {
         return true;
      }
  };

  template <class DataType>
  class OutPortMock
    : public RTC::OutPort<DataType>
  {
  public:
      OutPortMock(const char* name, DataType& value)
       : RTC::OutPort<DataType>(name, value)
      {
      }

      virtual ~OutPortMock()
      {
      }
  };

  class InPortBaseTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(InPortBaseTests);

    CPPUNIT_TEST(test_constructor);
    CPPUNIT_TEST(test_init);
    CPPUNIT_TEST(test_init2);         //multibuffer
    CPPUNIT_TEST(test_init3);         //Buffers are not registered in Factory.
    CPPUNIT_TEST(test_initProviders);
    CPPUNIT_TEST(test_initProviders2);//Providers are not registered in Factory.
    CPPUNIT_TEST(test_initConsumers);
    CPPUNIT_TEST(test_initConsumers2);//Consumers are not registered in Factory.
    CPPUNIT_TEST(test_activateInterfaces_deactivateInterfaces);
    CPPUNIT_TEST(test_publishInterfaces);
    CPPUNIT_TEST(test_publishInterfaces2);//dataport.dataflow_type is "pull" 
    CPPUNIT_TEST(test_publishInterfaces3);//dataport.dataflow_type is "else" 
    CPPUNIT_TEST(test_publishInterfaces4);//Provider is deleted.  
    CPPUNIT_TEST(test_publishInterfaces5);
    CPPUNIT_TEST(test_subscribeInterfaces);
    CPPUNIT_TEST(test_subscribeInterfaces2);//dataport.dataflow_type is "push"
    CPPUNIT_TEST(test_subscribeInterfaces3);//dataport.dataflow_type is "else"
    CPPUNIT_TEST(test_subscribeInterfaces4);//Consumer is deleted.
    CPPUNIT_TEST(test_subscribeInterfaces5);//Buffer is deleted.
    CPPUNIT_TEST(test_createConnector);
    CPPUNIT_TEST(test_createConnector2);
    CPPUNIT_TEST(test_ConnectorListener);

    CPPUNIT_TEST_SUITE_END();
	
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
    RTC::ConnectorListeners m_listeners;
    DataListener *m_datalisteners[cdl_len];
    ConnListener *m_connlisteners[cl_len];

  public:
	
    /*!
     * @brief Constructor
     */
    InPortBaseTests()
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
    ~InPortBaseTests()
    {
    }
		
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }
        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);
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
        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        
        RTC::PortProfile profile = inport.getPortProfile();
        coil::Properties prop = NVUtil::toProperties(profile.properties);
        CPPUNIT_ASSERT_EQUAL(std::string("DataInPort"), prop["port.port_type"] );
        CPPUNIT_ASSERT_EQUAL(std::string(toTypename<RTC::TimedFloat>()),
                             prop["dataport.data_type"] );
        CPPUNIT_ASSERT_EQUAL(std::string("Any"),
                             prop["dataport.subscription_type"]);

        portAdmin.deletePort(inport);


    }
    /*!
     * @brief init()メソッドのテスト
     * 
     */
    void test_init()
    {
        //
        //Condition
        //  m_singlebuffer is true(singlebuffer).
        //  m_thebuffer is CdrRingBufferMock.
        //Check
        //  Check that buffer is generated.
        //  
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);


        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        
        CPPUNIT_ASSERT(0==inport.getThebuffer());
        RTC::PortProfile profile = inport.getPortProfile();
        coil::Properties prop = NVUtil::toProperties(profile.properties);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        coil::vstring pstr = inport.getProviderTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, pstr.size());
        coil::vstring cstr = inport.get_m_consumerTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, cstr.size());
        CdrRingBufferMock::logger.clearLog();
        CPPUNIT_ASSERT_EQUAL(0,
             CdrRingBufferMock::logger.countLog("RingBufferMock::Constructor"));

        coil::Properties dummy;
        inport.init(dummy);

        //m_singlebufferがtrueの場合m_thebufferが取得される
        CPPUNIT_ASSERT(0!=inport.getThebuffer());
        CPPUNIT_ASSERT_EQUAL(1,
             CdrRingBufferMock::logger.countLog("RingBufferMock::Constructor"));

        profile = inport.getPortProfile();
        prop = NVUtil::toProperties(profile.properties);

        //getPortProfileのpropertiesに以下が追加される
        CPPUNIT_ASSERT_EQUAL(std::string("push,pull"),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             prop["dataport.interface_type"]);
 
        //ProviderTypes,ConsumerTypesが取得される
        pstr = inport.getProviderTypes();
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             pstr[0]);
        cstr = inport.get_m_consumerTypes();
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             cstr[0]);

        portAdmin.deletePort(inport);
    }
    /*!
     * @brief init()メソッドのテスト
     * 
     */
    void test_init2()
    {
        //
        //Condition
        //  m_singlebuffer is false(multibuffer).
        //  m_thebuffer is CdrRingBufferMock.
        //Check
        //  Check that buffer is not generated.
        //
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }
        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);
        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        
        CPPUNIT_ASSERT(0==inport.getThebuffer());
        RTC::PortProfile profile = inport.getPortProfile();
        coil::Properties prop = NVUtil::toProperties(profile.properties);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        coil::vstring pstr = inport.getProviderTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, pstr.size());
        coil::vstring cstr = inport.get_m_consumerTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, cstr.size());
        CdrRingBufferMock::logger.clearLog();
        CPPUNIT_ASSERT_EQUAL(0,
             CdrRingBufferMock::logger.countLog("RingBufferMock::Constructor"));

        inport.setMultibuffer();
        coil::Properties dummy;
        inport.init(dummy);

        CPPUNIT_ASSERT(0==inport.getThebuffer());
        CPPUNIT_ASSERT_EQUAL(0,
             CdrRingBufferMock::logger.countLog("RingBufferMock::Constructor"));

        profile = inport.getPortProfile();
        prop = NVUtil::toProperties(profile.properties);

        //getPortProfileのpropertiesに以下が追加される
        CPPUNIT_ASSERT_EQUAL(std::string("push,pull"),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             prop["dataport.interface_type"]);
 
        //ProviderTypes,ConsumerTypesが取得される
        pstr = inport.getProviderTypes();
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             pstr[0]);
        cstr = inport.get_m_consumerTypes();
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             cstr[0]);

        portAdmin.deletePort(inport);
    }
    /*!
     * @brief init()メソッドのテスト
     * 
     */
    void test_init3()
    {
        //
        //Condition
        //  m_singlebuffer is true(singlebuffer).
        //  m_thebuffer is unset.(The buffer is not registered in Factory.)
        //Check
        //  Check that buffer is not generated.
        //  
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        
        CPPUNIT_ASSERT(0==inport.getThebuffer());
        RTC::PortProfile profile = inport.getPortProfile();
        coil::Properties prop = NVUtil::toProperties(profile.properties);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        coil::vstring pstr = inport.getProviderTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, pstr.size());
        coil::vstring cstr = inport.get_m_consumerTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, cstr.size());
        CdrRingBufferMock::logger.clearLog();
        CPPUNIT_ASSERT_EQUAL(0,
             CdrRingBufferMock::logger.countLog("RingBufferMock::Constructor"));

        coil::Properties dummy;
        inport.init(dummy);

        //m_singlebufferがtrueの場合m_thebufferが取得される
        CPPUNIT_ASSERT(0==inport.getThebuffer());
        CPPUNIT_ASSERT_EQUAL(0,
             CdrRingBufferMock::logger.countLog("RingBufferMock::Constructor"));

        profile = inport.getPortProfile();
        prop = NVUtil::toProperties(profile.properties);

        //getPortProfileのpropertiesに以下が追加される
        CPPUNIT_ASSERT_EQUAL(std::string("push,pull"),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             prop["dataport.interface_type"]);
 
        //ProviderTypes,ConsumerTypesが取得される
        pstr = inport.getProviderTypes();
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             pstr[0]);
        cstr = inport.get_m_consumerTypes();
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             cstr[0]);


        portAdmin.deletePort(inport);
    }
    /*!
     * @brief initProviders()メソッドのテスト
     * 
     */
    void test_initProviders()
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);


        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        
        RTC::PortProfile profile = inport.getPortProfile();
        coil::Properties prop = NVUtil::toProperties(profile.properties);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        coil::vstring pstr = inport.getProviderTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, pstr.size());
        inport.initProviders_public();


        profile = inport.getPortProfile();
        prop = NVUtil::toProperties(profile.properties);

        //getPortProfileのpropertiesに以下が追加される
        CPPUNIT_ASSERT_EQUAL(std::string("push"),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             prop["dataport.interface_type"]);
 
        //ProviderTypes,ConsumerTypesが取得される
        pstr = inport.getProviderTypes();
        CPPUNIT_ASSERT((size_t)0!= pstr.size());
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             pstr[0]);

        portAdmin.deletePort(inport);
    }
    /*!
     * @brief initProviders()メソッドのテスト
     * The provider is not registered in Factory.
     * 
     */
    void test_initProviders2()
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }
        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);


        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        
        RTC::PortProfile profile = inport.getPortProfile();
        coil::Properties prop = NVUtil::toProperties(profile.properties);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        coil::vstring pstr = inport.getProviderTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, pstr.size());
        inport.initProviders_public();


        profile = inport.getPortProfile();
        prop = NVUtil::toProperties(profile.properties);

        //getPortProfileのpropertiesに以下が追加される
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        //ProviderTypes,ConsumerTypesが取得される
        pstr = inport.getProviderTypes();
        CPPUNIT_ASSERT((size_t)0== pstr.size());

        portAdmin.deletePort(inport);
    }
    /*!
     * @brief initConsumers()メソッドのテスト
     * 
     */
    void test_initConsumers()
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }
        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 

        RTC::PortProfile profile = inport.getPortProfile();
        coil::Properties prop = NVUtil::toProperties(profile.properties);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        coil::vstring cstr = inport.get_m_consumerTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, cstr.size());

        inport.initConsumers_public();

        profile = inport.getPortProfile();
        prop = NVUtil::toProperties(profile.properties);

        //getPortProfileのpropertiesに以下が追加される
        CPPUNIT_ASSERT_EQUAL(std::string("pull"),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             prop["dataport.interface_type"]);
 
        //ProviderTypes,ConsumerTypesが取得される
        cstr = inport.get_m_consumerTypes();
        CPPUNIT_ASSERT((size_t)0!= cstr.size());
        CPPUNIT_ASSERT_EQUAL(std::string("corba_cdr"),
                             cstr[0]);

        portAdmin.deletePort(inport);

    }
    /*!
     * @brief initConsumers()メソッドのテスト
     * 
     */
    void test_initConsumers2()
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }
        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 

        RTC::PortProfile profile = inport.getPortProfile();
        coil::Properties prop = NVUtil::toProperties(profile.properties);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        coil::vstring cstr = inport.get_m_consumerTypes();
        CPPUNIT_ASSERT_EQUAL((size_t)0, cstr.size());

        inport.initConsumers_public();

        profile = inport.getPortProfile();
        prop = NVUtil::toProperties(profile.properties);

        //getPortProfileのpropertiesに以下が追加される
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.dataflow_type"]);
        CPPUNIT_ASSERT_EQUAL(std::string(""),
                             prop["dataport.interface_type"]);

        //ProviderTypes,ConsumerTypesが取得される
        cstr = inport.get_m_consumerTypes();
        CPPUNIT_ASSERT((size_t)0== cstr.size());

        portAdmin.deletePort(inport);

    }
    /*!
     * @brief activateInterfaces(),deactivateInterfaces()メソッドのテスト
     * 
     */
    void test_activateInterfaces_deactivateInterfaces(void)
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());
        coil::Properties dummy;
        inport.init(dummy);

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("InPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = inport.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "push"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "new"));
        inport.publishInterfaces_public(prof);
        prof.connector_id = "id1";
        prof.name = CORBA::string_dup("InPortBaseTest1");
        inport.publishInterfaces_public(prof);

        // getConnectorProfiles()
        RTC::ConnectorInfoList cilist = inport.getConnectorProfiles();
        CPPUNIT_ASSERT(cilist.size() == 2);
        CPPUNIT_ASSERT_EQUAL(std::string("id0"), std::string(cilist[0].id));
        CPPUNIT_ASSERT_EQUAL(std::string("InPortBaseTest0"), std::string(cilist[0].name));
        CPPUNIT_ASSERT_EQUAL((size_t)1, cilist[0].ports.size());
        CPPUNIT_ASSERT_EQUAL(std::string("id1"), std::string(cilist[1].id));
        CPPUNIT_ASSERT_EQUAL(std::string("InPortBaseTest1"), std::string(cilist[1].name));
        CPPUNIT_ASSERT_EQUAL((size_t)1, cilist[1].ports.size());

        // getConnectorIds()
        coil::vstring ids = inport.getConnectorIds();
        CPPUNIT_ASSERT(ids.size() == 2);
        CPPUNIT_ASSERT_EQUAL(std::string("id0"), std::string(ids[0]));
        CPPUNIT_ASSERT_EQUAL(std::string("id1"), std::string(ids[1]));

        // getConnectorNames()
        coil::vstring names = inport.getConnectorNames();
        CPPUNIT_ASSERT_EQUAL(std::string("InPortBaseTest0"), std::string(names[0]));
        CPPUNIT_ASSERT_EQUAL(std::string("InPortBaseTest1"), std::string(names[1]));

        // getConnectorById()
        RTC::InPortConnector* ic = inport.getConnectorById("unknown");
        CPPUNIT_ASSERT(ic == 0);
        ic = inport.getConnectorById("id0");
        CPPUNIT_ASSERT(ic != 0);
        ic = inport.getConnectorById("id1");
        CPPUNIT_ASSERT(ic != 0);

        // getConnectorByName()
        ic = inport.getConnectorByName("unknown");
        CPPUNIT_ASSERT(ic == 0);
        ic = inport.getConnectorByName("InPortBaseTest0");
        CPPUNIT_ASSERT(ic != 0);
        ic = inport.getConnectorByName("InPortBaseTest1");
        CPPUNIT_ASSERT(ic != 0);

        // getConnectorProfileById()
        RTC::ConnectorInfo info;
        bool bret = inport.getConnectorProfileById("unknown", info);
        CPPUNIT_ASSERT( !bret );
        bret = inport.getConnectorProfileById("id0", info);
        CPPUNIT_ASSERT( bret );
        bret = inport.getConnectorProfileById("id1", info);
        CPPUNIT_ASSERT( bret );

        // getConnectorProfileByName()
        bret = inport.getConnectorProfileByName("unknown", info);
        CPPUNIT_ASSERT( !bret );
        bret = inport.getConnectorProfileByName("InPortBaseTest0", info);
        CPPUNIT_ASSERT( bret );
        bret = inport.getConnectorProfileByName("InPortBaseTest1", info);
        CPPUNIT_ASSERT( bret );

        CPPUNIT_ASSERT_EQUAL(2,(int)inport.get_m_connectors().size());
        std::vector<RTC::InPortConnector*> list =  inport.get_m_connectors();
        inport.activateInterfaces();
        inport.deactivateInterfaces();

        portAdmin.deletePort(inport);

        ic = NULL;
        delete ic;
    }
    /*!
     * @brief publishInterfaces()メソッドのテスト
     * 
     */
    void test_publishInterfaces(void)
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());
        coil::Properties dummy;
        inport.init(dummy);

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("InPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = inport.get_port_profile()->port_ref;
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
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        retcode = inport.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(1,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK,retcode);

        prof.connector_id = "id1";
        prof.name = CORBA::string_dup("InPortBaseTest1");
        retcode = inport.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(2,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK,retcode);

        portAdmin.deletePort(inport);
    }
    /*!
     * @brief publishInterfaces()メソッドのテスト
     * dataport.dataflow_typeがpullでpublisherInterfaceをコール
     *
     * 
     */
    void test_publishInterfaces2(void)
    {
        //
        //dataport.dataflow_typeがpullでpublisherInterfaceをコール
        //

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());
        coil::Properties dummy;
        inport.init(dummy);

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("InPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = inport.get_port_profile()->port_ref;
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
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        retcode = inport.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK,retcode);

        prof.connector_id = "id1";
        prof.name = CORBA::string_dup("InPortBaseTest1");
        retcode = inport.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK,retcode);

        portAdmin.deletePort(inport);
    }
    /*!
     * @brief publishInterfaces()メソッドのテスト
     * 
     */
    void test_publishInterfaces3(void)
    {
        //
        //dataport.dataflow_typeが不正な状態でpublisherInterfaceをコール
        //

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());
        coil::Properties dummy;
        inport.init(dummy);

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("InPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = inport.get_port_profile()->port_ref;
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
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        retcode = inport.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::BAD_PARAMETER,retcode);

        prof.connector_id = "id1";
        prof.name = CORBA::string_dup("InPortBaseTest1");
        retcode = inport.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::BAD_PARAMETER,retcode);

        portAdmin.deletePort(inport);
    }
    /*!
     * @brief publishInterfaces(),deactivateInterfaces()メソッドのテスト
     * 
     */
    void test_publishInterfaces4(void)
    {
        //
        //ProviderなしでpublisherInterfaceをコール
        //

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());
        coil::Properties dummy;
        inport.init(dummy);

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("InPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = inport.get_port_profile()->port_ref;
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
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        retcode = inport.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::BAD_PARAMETER,retcode);

        portAdmin.deletePort(inport);
    }
    /*!
     * @brief publishInterfaces()メソッドのテスト
     * 
     */
    void test_publishInterfaces5(void)
    {
        //
        //BufferなしでpublisherInterfaceをコール
        //

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }


        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());
        coil::Properties dummy;
        inport.init(dummy);

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("InPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = inport.get_port_profile()->port_ref;
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
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        retcode = inport.publishInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::BAD_PARAMETER,retcode);


        portAdmin.deletePort(inport);
    }
    /*!
     * @brief subscribeInterfaces()メソッドのテスト
     * 
     */
    void test_subscribeInterfaces(void)
    {
        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());
        coil::Properties dummy;
        inport.init(dummy);

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("InPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = inport.get_port_profile()->port_ref;
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
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        retcode = inport.subscribeInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(1,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK,retcode);

        prof.connector_id = "id1";
        prof.name = CORBA::string_dup("InPortBaseTest1");
        retcode = inport.subscribeInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(2,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK,retcode);

        portAdmin.deletePort(inport);
    }
    /*!
     * @brief subscribeInterfaces()メソッドのテスト
     * dataport.dataflow_typeがpullでsubscribeInterfaceをコール
     *
     */
    void test_subscribeInterfaces2(void)
    {
        //
        //dataport.dataflow_typeがpullでsubscribeInterfaceをコール
        //

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());
        coil::Properties dummy;
        inport.init(dummy);

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("InPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = inport.get_port_profile()->port_ref;
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
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        retcode = inport.subscribeInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_ERROR,retcode);


        portAdmin.deletePort(inport);
    }
    /*!
     * @brief subscribeInterfaces()メソッドのテスト
     * 
     */
    void test_subscribeInterfaces3(void)
    {
        //
        //dataport.dataflow_typeが不正な状態でsubscrtibeInterfaceをコール
        //

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());
        coil::Properties dummy;
        inport.init(dummy);

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("InPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = inport.get_port_profile()->port_ref;
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
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        retcode = inport.subscribeInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::BAD_PARAMETER,retcode);


        portAdmin.deletePort(inport);
    }
    /*!
     * @brief subscribeInterfaces()メソッドのテスト
     * 
     */
    void test_subscribeInterfaces4(void)
    {
        //
        //ConsumerなしでsubscribeInterfaceをコール
        //

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());
        coil::Properties dummy;
        inport.init(dummy);

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("InPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = inport.get_port_profile()->port_ref;
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
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        retcode = inport.subscribeInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::BAD_PARAMETER,retcode);

        portAdmin.deletePort(inport);
    }
    /*!
     * @brief subscribeInterfaces()メソッドのテスト
     * 
     */
    void test_subscribeInterfaces5(void)
    {
        //
        //BufferなしでsubscribeInterfaceをコール
        //

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    InPortCorbaCdrProviderMock>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       InPortCorbaCdrProviderMock>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProviderMock を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    OutPortCorbaCdrConsumerMock>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       OutPortCorbaCdrConsumerMock>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedFloat>());
        coil::Properties dummy;
        inport.init(dummy);

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("InPortBaseTest0");
        prof.ports.length(1);
        prof.ports[0] = inport.get_port_profile()->port_ref;
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
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        retcode = inport.subscribeInterfaces_public(prof);
        CPPUNIT_ASSERT_EQUAL(0,(int)inport.get_m_connectors().size());
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_ERROR,retcode);

        portAdmin.deletePort(inport);
    }
    /*!
     * @brief createConnector()メソッドのテスト
     * 
     */
    void test_createConnector(void)
    {
        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedDouble>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 

        RTC::ConnectorProfile prof;

        coil::Properties prop(inport.properties());
        RTC::InPortProvider* provider = new InPortCorbaCdrProviderMock();
        RTC::InPortConnector* connector 
              = inport.createConnector_public(prof, prop, provider);
        CPPUNIT_ASSERT(0!= connector);
            
        portAdmin.deletePort(inport);
//        delete provider;
    }
    /*!
     * @brief createConnector()メソッドのテスト
     * 
     */
    void test_createConnector2(void)
    {
        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedDouble>());

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 

        RTC::ConnectorProfile prof;

        coil::Properties prop(inport.properties());
        prop.setProperty("InPortBaseTests","bad_alloc");
        RTC::InPortProvider* provider = new InPortCorbaCdrProviderMock();
        RTC::InPortConnector* connector 
              = inport.createConnector_public(prof, prop, provider);
        CPPUNIT_ASSERT(0 != connector);
            
        portAdmin.deletePort(inport);
//        delete provider;
    }

    /*!
     * @brief addConnectorDataListener(), removeConnectorDataListener(), addConnectorListener(), removeConnectorListener(), isLittleEndian(), connect() メソッドのテスト
     * 
     */
    void test_ConnectorListener(void)
    {
        RTC::TimedLong tdl;
        OutPortMock<RTC::TimedLong> outport("OutPort", tdl);
        coil::Properties dummy;
        outport.init(dummy);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::InPortProviderFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::InPortProviderFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProvider を登録する。
        RTC::InPortProviderFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::InPortProvider, 
                                    ::RTC::InPortCorbaCdrProvider>,
                   ::coil::Destructor< ::RTC::InPortProvider, 
                                       ::RTC::InPortCorbaCdrProvider>);

        //既に "corba_cdr" で登録されている場合は削除する。
        if( RTC::OutPortConsumerFactory::instance().hasFactory("corba_cdr") )
        {
            RTC::OutPortConsumerFactory::instance().removeFactory("corba_cdr");
        }
        //"corba_cdr" に InPortCorbaCdrProvider を登録する。
        RTC::OutPortConsumerFactory::instance().
        addFactory("corba_cdr",
                   ::coil::Creator< ::RTC::OutPortConsumer, 
                                    ::RTC::OutPortCorbaCdrConsumer>,
                   ::coil::Destructor< ::RTC::OutPortConsumer, 
                                       ::RTC::OutPortCorbaCdrConsumer>);

        //既に "ring_buffer" で登録されている場合は削除する。
        if( RTC::CdrBufferFactory::instance().hasFactory("ring_buffer") )
        {
            RTC::CdrBufferFactory::instance().removeFactory("ring_buffer");
        }

        //"ring_buffer" に CdrRingBufferMock を登録する。
        RTC::CdrBufferFactory::instance().
        addFactory("ring_buffer",
                   coil::Creator<RTC::CdrBufferBase, CdrRingBufferMock>,
                   coil::Destructor<RTC::CdrBufferBase, CdrRingBufferMock>);

        InPortBaseMock inport("InPortBaseTest", toTypename<RTC::TimedLong>());
        inport.init(dummy);

        RTC::PortAdmin portAdmin(m_pORB,m_pPOA);
        portAdmin.registerPort(inport); 
        RTC::ConnectorProfile prof;
        prof.connector_id = "id0";
        prof.name = CORBA::string_dup("connectTest0");
        prof.ports.length(2);
        prof.ports[0] = inport.get_port_profile()->port_ref;
        prof.ports[1] = outport.get_port_profile()->port_ref;
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                 "corba_cdr"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
                                 "push"));
        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
                                 "flush"));

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
            inport.addConnectorDataListener((RTC::ConnectorDataListenerType)i, 
                                             m_datalisteners[i], true);
          }

        // addConnectorListener()
        for (int i(0); i<cl_len; ++i)
          {
            inport.addConnectorListener((RTC::ConnectorListenerType)i, 
                                         m_connlisteners[i], true);
          }

        // Listener add count check
        CPPUNIT_ASSERT_EQUAL(10, cdl_count);
        CPPUNIT_ASSERT_EQUAL(7, cl_count);

        inport.publishInterfaces_public(prof);

        // connect()
        RTC::ReturnCode_t ret;
        ret = inport.connect(prof);
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ret);

        // isLittleEndian()
        // little set & check
        bool bret = inport.isLittleEndian();
        CPPUNIT_ASSERT( bret );

        tdl.data = 100;
        outport.write(tdl);

        inport.activateInterfaces();
        inport.deactivateInterfaces();
        ret = inport.disconnect_all();
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ret);

        portAdmin.deletePort(inport);
        // removeConnectorDataListener()
        for (int i(0); i<cdl_len; ++i)
          {
            inport.removeConnectorDataListener((RTC::ConnectorDataListenerType)i, 
                                                m_datalisteners[i]);
          }

        // removeConnectorListener()
        for (int i(0); i<cl_len; ++i)
          {
            inport.removeConnectorListener((RTC::ConnectorListenerType)i, 
                                            m_connlisteners[i]);
          }

        // Listener remove count check
        CPPUNIT_ASSERT_EQUAL(0, cdl_count);
        CPPUNIT_ASSERT_EQUAL(0, cl_count);

    }

  };
}; // namespace OutPortBase


/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(InPortBase::InPortBaseTests);

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
