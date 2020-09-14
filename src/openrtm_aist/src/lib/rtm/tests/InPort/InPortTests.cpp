// -*- C++ -*-
/*!
 * @file   InPortTests.cpp
 * @brief  InPort test class
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
 * $Id$
 *
 */

/*
 * $Log: InPortTests.cpp,v $
 * Revision 1.2  2008/03/13 13:12:25  arafune
 * Added some new tests.
 *
 * Revision 1.1  2007/12/20 07:50:19  arafune
 * *** empty log message ***
 *
 * Revision 1.2  2007/01/12 14:51:20  n-ando
 * Callback functions' namespace were changed.
 *
 * Revision 1.1  2006/11/27 08:32:39  n-ando
 * TestSuites are devided into each directory.
 *
 *
 */

#ifndef InPort_cpp
#define InPort_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/DataPortSkel.h>
#include <rtm/InPortConsumer.h>
#include <rtm/InPortProvider.h>
#include <rtm/InPort.h>
#include <rtm/InPortBase.h>
#include <rtm/CorbaConsumer.h>
#include <rtm/OutPortConsumer.h>
#include <rtm/CdrBufferBase.h>

#include <string>
#include <vector>
#include <coil/Guard.h>
#include <coil/Mutex.h>
#include <rtm/idl/RTCSkel.h>
#include <rtm/CORBA_SeqUtil.h>
#include <rtm/NVUtil.h>
#include <assert.h>
#include <coil/UUID.h>
#include <memory>
#include <rtm/RTC.h>

#include <coil/Logger.h>
#include <rtm/SystemLogger.h>
#include <rtm/Manager.h>
#include <rtm/PortCallback.h>
#include <rtm/InPortPushConnector.h>
#include <rtm/ConnectorBase.h>
#include <rtm/OutPortConnector.h>

#define WTIMEOUT_USEC 1000000
#define USEC_PER_SEC 1000000

/*!
 * @class InPortTests class
 * @brief InPort test
 */
namespace InPort
{
  /*!
   * 
   * 
   */
  template <class DataType>
  class InPortMock
    : public RTC::InPort<DataType>
  {
  public:
      /*!
       * 
       * 
       */
      InPortMock(const char* name, DataType& value,
	   int bufsize=64, 
	   bool read_block = false, bool write_block = false,
	   int read_timeout = 0, int write_timeout = 0)
        : RTC::InPort<DataType>(name , value)
      {
      }
      /*!
       * 
       * 
       */
      virtual ~InPortMock()
      {
      }
      /*!
       * 
       * 
       */
      RTC::CdrBufferBase* get_m_thebuffer()
      {
          return RTC::InPort<DataType>::m_thebuffer;
      }
      /*!
       * 
       * 
       */
      coil::vstring get_m_providerTypes()
      {
          return RTC::InPort<DataType>::m_providerTypes;
      }
      /*!
       * 
       * 
       */
      coil::vstring get_m_consumerTypes()
      {
          return RTC::InPort<DataType>::m_consumerTypes;
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
	// return RTC::InPort<DataType>::createConnector(cprof, prop, provider);
         return RTC::InPortBase::createConnector(cprof, prop, provider);
      }
      /*!
       * 
       * 
       */
      RTC::InPortBase::ConnectorList get_m_connectors()
      {
          return RTC::InPort<DataType>::m_connectors;
      }
      /*!
       * 
       * 
       */
      coil::Properties get_m_properties()
      {
          return RTC::InPort<DataType>::m_properties;
      }
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
      std::vector<std::string> args(coil::split("-ORBendPoint giop:tcp:2809", " "));
      // TAO's ORB_init needs argv[0] as command name.
      args.insert(args.begin(), "manager");
      char** argv = coil::toArgv(args);
      int argc(args.size());
      
      // ORB initialization
      CORBA::ORB_ptr orb = CORBA::ORB_init(argc, argv);
      
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
      return RTC::InPortConsumer::PORT_OK;
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
  private:
    Logger* m_logger;
    ::OpenRTM::InPortCdr_var m_objref;
    
  };
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
          m_readable_return_value = 0;

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
          return m_readable_return_value;
      }
      /*!
       *
       *
       */
      void set_readable_return_value(size_t value)
      {
          m_readable_return_value = value;
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
      size_t m_readable_return_value;
  };
  template <class DataType>
  Logger RingBufferMock<DataType>::logger;
  typedef RingBufferMock<cdrMemoryStream> CdrRingBufferMock;
};
namespace RTC
{
  /*!
   * Global variable for test
   */
  ::InPort::Logger RTC_logger;
  ConnectorBase::ReturnCode InPortPushConnector_read_return_value;
  int InPortPushConnector_read;
  /*!
   *
   * Mock InPortPushConnector
   *
   */
  /*!
   *
   *
   */
  InPortPushConnector::InPortPushConnector(ConnectorInfo profile, 
                                           InPortProvider* provider,
					   ConnectorListeners& listeners,
                                           CdrBufferBase* buffer)
    : InPortConnector(profile, buffer),
      m_listeners(listeners)
  {

      InPortPushConnector_read = 0;
      InPortPushConnector_read_return_value = PORT_OK;
      if(profile.properties["InPortBaseTests"]=="bad_alloc")
      {
          throw std::bad_alloc();
      }

  }
  /*!
   *
   *
   */
  InPortPushConnector::~InPortPushConnector()
  {
  }
  /*!
   *
   *
   */
  ConnectorBase::ReturnCode InPortPushConnector::disconnect()
  {
      return PORT_OK;
  }
  /*!
   *
   *
   */
  ConnectorBase::ReturnCode
  InPortPushConnector::read(cdrMemoryStream& data)
  {
      RTC_logger.log("InPortPushConnector::read");
      RTC::TimedLong td;
      td.data = 777;;
      td >>= data;
      return InPortPushConnector_read_return_value;

     
  }
  /*!
   *
   *
   */
  CdrBufferBase* InPortPushConnector::createBuffer(ConnectorInfo& profile)
  {
      return new ::InPort::CdrRingBufferMock();
  }
 /*!
  * Mock Manager
  *
  *
  *
  *
  */
  Manager* Manager::manager = NULL;
  coil::Mutex Manager::mutex;
 /*!
  * 
  * 
  */
  Manager::Manager()
    : m_initProc(NULL),
      m_logStreamBuf(), rtclog(&m_logStreamBuf),
      m_runner(NULL), m_terminator(NULL)
  {
  }
 /*!
  * 
  * 
  */
  Manager& Manager::instance()
  {
    // DCL for singleton
    if (!manager)
      {
	Guard guard(mutex);
	if (!manager)
	  {
	    manager = new Manager();
	    manager->initORB();
	  }
      }
    return *manager;
  }
 /*!
  * 
  * 
  */
  CORBA::ORB_ptr Manager::getORB()
  {
    return m_pORB;
  }
 /*!
  * 
  * 
  */
  bool Manager::initORB()
  {
    // Initialize ORB
    try
      {
	std::vector<std::string> args(coil::split(NULL, " "));
	// TAO's ORB_init needs argv[0] as command name.
	args.insert(args.begin(), "manager");
	char** argv = coil::toArgv(args);
	int argc(args.size());
	
	// ORB initialization
	m_pORB = CORBA::ORB_init(argc, argv);
	// Get the RootPOA
	CORBA::Object_var obj = m_pORB->resolve_initial_references("RootPOA");
	PortableServer::POA_var rootPOA = PortableServer::POA::_narrow(obj);
	CORBA::PolicyList policies;
	policies.length(2);
	policies[(CORBA::ULong)0] = rootPOA->create_lifespan_policy(PortableServer::PERSISTENT);
	policies[(CORBA::ULong)1] = rootPOA->create_id_assignment_policy(PortableServer::USER_ID);
	PortableServer::POAManager_var rootManager = rootPOA->the_POAManager();
	m_pPOA = rootPOA->create_POA("persistent", rootManager, policies);
	if (CORBA::is_nil(m_pPOA))
	  {
	    return false;
	  }
	// Get the POAManager
	m_pPOAManager = m_pPOA->the_POAManager();
      }
    catch (...)
      {
	RTC_ERROR(("Exception: Caught unknown exception in initORB()." ));
	return false;
      }
    return true;
  }
  
 /*!
  *
  *
  *
  *
  *
  */
  LogStreamBuf m_logStreamBuf;
  const char* Logger::m_levelString[] =
    {
      " SILENT: ",
      " FATAL: ",
      " ERROR: ",
      " WARNING: ",
      " INFO: ",
      " DEBUG: ",
      " TRACE: ",
      " VERBOSE: ",
      " PARANOID: "
    };

  Logger::Logger(const char* name)
    : ::coil::LogStream(&m_logStreamBuf, 0, 8, 0)
  {
  }

  Logger::Logger(LogStreamBuf* streambuf)
    : ::coil::LogStream(&m_logStreamBuf, 0, 8, 0)
  {
  }

  Logger::~Logger(void)
  {
  }

  /*!
   * @if jp
   * @brief ログレベルを文字列で設定する
   * @else
   * @brief Set log level by string
   * @endif
   */
  bool Logger::setLevel(const char* level)
  {
    return true; 
  }

  /*!
   * @if jp
   * @brief ヘッダに付加する日時フォーマットを指定する。
   * @else
   * @brief Set date/time format for adding the header
   * @endif
   */
  void Logger::setDateFormat(const char* format)
  {
  }

  /*!
   * @if jp
   * @brief ヘッダの日時の後に付加する文字列を設定する。
   * @else
   * @brief Set suffix of date/time string of header.
   * @endif
   */
  void Logger::setName(const char* name)
  {
  }

  /*!
   * @if jp
   * @brief メッセージのプリフィックス追加関数
   * @else
   * @brief Message prefix appender function
   * @endif
   */
  void Logger::header(int level)
  {
  }

  /*!
   * @if jp
   * @brief フォーマットされた現在日時文字列を取得する。
   * @else
   * @brief Get the current formatted date/time string
   * @endif
   */
  std::string Logger::getDate(void)
  {
    const int maxsize = 256;
    char buf[maxsize];

    return std::string(buf);
  }

  /*!
   * @if jp
   * @brief ログレベル設定
   * @else
   * @brief Set the log level
   * @endif
   */
  int Logger::strToLevel(const char* level)
  {
      return 0;
  }

#if 1
 /*
  *
  *
  *
  *
  *
  */
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  InPortProvider::InPortProvider()
  {
  }
  
  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  InPortProvider::~InPortProvider()
  {
  }
  
  /*!
   * @if jp
   * @brief InterfaceProfile情報を公開する
   * @else
   * @brief Publish InterfaceProfile information
   * @endif
   */
  void InPortProvider::publishInterfaceProfile(SDOPackage::NVList& prop)
  {
  }
  
  /*!
   * @if jp
   * @brief Interface情報を公開する
   * @else
   * @brief Publish Interface information
   * @endif
   */
  bool InPortProvider::publishInterface(SDOPackage::NVList& prop)
  {
    return true;
  }

  //----------------------------------------------------------------------
  // protected functions
  
  /*!
   * @if jp
   * @brief インターフェースタイプを設定する
   * @else
   * @brief Set the interface type
   * @endif
   */
  void InPortProvider::setInterfaceType(const char* interface_type)
  {
  }
  
  /*!
   * @if jp
   * @brief データフロータイプを設定する
   * @else
   * @brief Set the dataflow type
   * @endif
   */
  void InPortProvider::setDataFlowType(const char* dataflow_type)
  {
  }
  
  /*!
   * @if jp
   * @brief サブスクリプションタイプを設定する
   * @else
   * @brief Set the subscription type
   * @endif
   */
  void InPortProvider::setSubscriptionType(const char* subs_type)
  {
  }
#endif 
}; // namespace RTC
namespace InPort
{
  /*!
   * 
   * 
   */
  template <class DataType>
  class OnReadMock
    : public RTC::OnRead<DataType>
  {
  public:
    /*!
     * 
     * 
     */
    OnReadMock(void)
    {
        m_logger = NULL;
    }
    /*!
     * 
     * 
     */
    virtual void operator()()
    {
      if (m_logger != NULL)
      {
         m_logger->log("OnReadMock::operator");
      }
      
    }
    DataType m_value;
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
  template <class DataType>
  struct OnReadConvertMock
    : public RTC::OnReadConvert<DataType>
  {
    /*!
     *
     *
     */
    OnReadConvertMock(void)
    {
        m_logger = NULL;
    }
    /*!
     *
     *
     */
    virtual DataType operator()(const DataType& value)
    {
      if (m_logger != NULL)
      {
         m_logger->log("OnReadConvertMock::operator");
      }
      return value;
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

/*
  template <class DataType>
  class FullBuffer
    : public RTC::NullBuffer<DataType>
  {
  public:
    FullBuffer(long int length) {};
    virtual bool isFull() const
    {
      return true;
    }
  };
*/
  template <class DataType>
  class OnWriteMock
    : public RTC::OnWrite<DataType>
  {
  public:
    virtual void operator()(const DataType& value)
    {
      m_value = value;
    }
    DataType m_value;
  };

  /*
  template <class DataType>
  class OnOverflowMock
    : public RTC::OnOverflow<DataType>
  {
  public:
    virtual void operator()(const DataType& value)
    {
      m_value = value;
    }
    DataType m_value;
  };
  */
  /*
  class OnWriteConvertMock
    : public RTC::OnWriteConvert<double>
  {
  public:
    OnWriteConvertMock(double amplitude)
      : m_amplitude(amplitude) {}
    virtual double operator()(const double& value)
    {
      return m_amplitude * value;
    }
    double m_amplitude;
  };
  */

  /*!
   * 
   * 
   */
  class InPortTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(InPortTests);

    CPPUNIT_TEST(test_name);
    CPPUNIT_TEST(test_isNew);
    CPPUNIT_TEST(test_isEmpty);
    CPPUNIT_TEST(test_read);
//    CPPUNIT_TEST(test_setOnWrite);     //The callback is not used. 
//    CPPUNIT_TEST(test_setOnUnderflow); //The callback is not used. 


/*
    CPPUNIT_TEST(test_write_and_read);
    CPPUNIT_TEST(test_write_OnWrite);
    CPPUNIT_TEST(test_write_OnWrite_full);
    CPPUNIT_TEST(test_write_OnOverflow);
    CPPUNIT_TEST(test_write_OnOverflow_not_full);
    CPPUNIT_TEST(test_write_OnWriteConvert);
    CPPUNIT_TEST(test_write_timeout);
*/
    CPPUNIT_TEST_SUITE_END();
		
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
    Logger* m_logger;
		

  public:
	
    /*!
     * @brief Constructor
     */
    InPortTests()
    {
      int argc(0);
      char** argv(NULL);
      m_pORB = CORBA::ORB_init(argc, argv);
      m_pPOA = PortableServer::POA::_narrow(
		    m_pORB->resolve_initial_references("RootPOA"));
      m_pPOA->the_POAManager()->activate();
      m_logger = NULL;
    }
		
    /*!
     * @brief Destructor
     */
    ~InPortTests()
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
     * @brief name()メソッドのテスト
     * 
     */
    void test_name(void)
    {
        RTC::TimedLong tl;
        RTC::InPort<RTC::TimedLong> inport("test_name0",tl);
        CPPUNIT_ASSERT_EQUAL(std::string("test_name0"), 
                             std::string(inport.name()));

	m_pPOA->deactivate_object(*m_pPOA->servant_to_id(&inport));
    }

    /*!
     * @brief isNew()メソッドのテスト
     * 
     */
    void test_isNew(void)
    {
        RTC::TimedLong tl;
        InPortMock<RTC::TimedLong> inport("test_name0",tl);
        coil::Properties prop_;
        inport.init(prop_);

        //no connectors
        CPPUNIT_ASSERT_EQUAL(false, inport.isNew());

        RTC::ConnectorProfile prof;
        coil::Properties prop(inport.properties());
        RTC::InPortProvider* provider = new InPortCorbaCdrProviderMock();
        RTC::InPortConnector* connector 
              = inport.createConnector_public(prof, prop, provider);
        CPPUNIT_ASSERT(0!= connector);
        //no readable data
        CPPUNIT_ASSERT_EQUAL(false, inport.isNew());

        /* Dynamic_cast is done for the test.. */
        /* Please do not do usual.             */
        CdrRingBufferMock* pbuffer 
                 = dynamic_cast<CdrRingBufferMock*>(inport.get_m_thebuffer());

        CPPUNIT_ASSERT(0!= pbuffer);
        //no readable data
        CPPUNIT_ASSERT_EQUAL(false, inport.isNew());
        pbuffer->set_readable_return_value((size_t)5);
        //readable data
        CPPUNIT_ASSERT_EQUAL(true, inport.isNew());


	m_pPOA->deactivate_object(*m_pPOA->servant_to_id(&inport));
        delete provider;
    }
    /*!
     * @brief isEmpty()メソッドのテスト
     * 
     */
    void test_isEmpty(void)
    {
        RTC::TimedLong tl;
        InPortMock<RTC::TimedLong> inport("test_name0",tl);
        coil::Properties prop_;
        inport.init(prop_);

        //no connectors
        CPPUNIT_ASSERT_EQUAL(true, inport.isEmpty());

        RTC::ConnectorProfile prof;
        coil::Properties prop(inport.properties());
        RTC::InPortProvider* provider = new InPortCorbaCdrProviderMock();
        RTC::InPortConnector* connector 
              = inport.createConnector_public(prof, prop, provider);
        CPPUNIT_ASSERT(0!= connector);

        //no readable data
        CPPUNIT_ASSERT_EQUAL(true, inport.isEmpty());

        /* Dynamic_cast is done for the test.. */
        /* Please do not do usual.             */
        CdrRingBufferMock* pbuffer 
                 = dynamic_cast<CdrRingBufferMock*>(inport.get_m_thebuffer());

        CPPUNIT_ASSERT(0!= pbuffer);
        //no readable data
        CPPUNIT_ASSERT_EQUAL(true, inport.isEmpty());
        pbuffer->set_readable_return_value((size_t)5);

        //readable data
        CPPUNIT_ASSERT_EQUAL(false, inport.isEmpty());


	m_pPOA->deactivate_object(*m_pPOA->servant_to_id(&inport));
        delete provider;
    }
    /*!
     * @brief read(),operator>>(DataType&),setOnRead(),setOnReadConvert()メソッドのテスト
     * 
     */
    void test_read(void)
    {
        RTC::TimedLong tl;
        tl.data = 123;
        InPortMock<RTC::TimedLong> inport("test_read0",tl);
        coil::Properties prop_;
        inport.init(prop_);

        OnReadMock<RTC::TimedLong> onRead;
        Logger logger;
        onRead.setLogger(&logger);
        inport.setOnRead(&onRead);

        bool ret;
        CPPUNIT_ASSERT_EQUAL(0,logger.countLog("OnReadMock::operator"));
        ret = inport.read();
        //no connectors
        CPPUNIT_ASSERT_EQUAL(false, ret);
        CPPUNIT_ASSERT_EQUAL((CORBA::Long)123, tl.data);
        CPPUNIT_ASSERT_EQUAL(1,logger.countLog("OnReadMock::operator"));

        RTC::ConnectorProfile prof;
        coil::Properties prop(inport.properties());
        RTC::InPortProvider* provider = new InPortCorbaCdrProviderMock();
        RTC::InPortConnector* connector 
              = inport.createConnector_public(prof, prop, provider);
        CPPUNIT_ASSERT(0!= connector);
        //
        //
        //
        OnReadConvertMock<RTC::TimedLong> onReadConvert;
        onReadConvert.setLogger(&logger);

        RTC::InPortPushConnector_read_return_value 
                                           = RTC::ConnectorBase::PORT_OK;
        RTC::TimedLong tl_;
	inport.operator>>(tl_);
        CPPUNIT_ASSERT_EQUAL((CORBA::Long)777, tl_.data);

        int logcount;
        CPPUNIT_ASSERT_EQUAL(0,logger.countLog("OnReadConvertMock::operator"));
        logcount = ::RTC::RTC_logger.countLog("InPortPushConnector::read");
        ret = inport.read();
        //data read succeeded
        CPPUNIT_ASSERT_EQUAL(true, ret);
        CPPUNIT_ASSERT_EQUAL((CORBA::Long)777, tl.data);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("InPortPushConnector::read"));
        CPPUNIT_ASSERT_EQUAL(0,logger.countLog("OnReadConvertMock::operator"));

        inport.setOnReadConvert(&onReadConvert);
        CPPUNIT_ASSERT_EQUAL(0,logger.countLog("OnReadConvertMock::operator"));
        logcount = ::RTC::RTC_logger.countLog("InPortPushConnector::read");
        ret = inport.read();
        //data read succeeded
        CPPUNIT_ASSERT_EQUAL(true, ret);
        CPPUNIT_ASSERT_EQUAL((CORBA::Long)777, tl.data);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("InPortPushConnector::read"));
        CPPUNIT_ASSERT_EQUAL(1,logger.countLog("OnReadConvertMock::operator"));

        //
        //
        //
        RTC::InPortPushConnector_read_return_value 
                                           = RTC::ConnectorBase::PORT_OK;
        inport.setOnRead(&onRead);

        logcount = ::RTC::RTC_logger.countLog("InPortPushConnector::read");
        ret = inport.read();
        //data read succeeded
        CPPUNIT_ASSERT_EQUAL(true, ret);
        CPPUNIT_ASSERT_EQUAL((CORBA::Long)777, tl.data);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("InPortPushConnector::read"));
        //
        //
        //
        RTC::InPortPushConnector_read_return_value 
                                           = RTC::ConnectorBase::BUFFER_EMPTY;
        logcount = ::RTC::RTC_logger.countLog("InPortPushConnector::read");
        ret = inport.read();
        //data read failed, because InPortPushConnectorMock::read() return BUFFER_EMPTY.
        CPPUNIT_ASSERT_EQUAL(false, ret);
        CPPUNIT_ASSERT_EQUAL((CORBA::Long)777, tl.data);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("InPortPushConnector::read"));

        //
        //
        //
        RTC::InPortPushConnector_read_return_value 
                                           = RTC::ConnectorBase::BUFFER_TIMEOUT;
        logcount = ::RTC::RTC_logger.countLog("InPortPushConnector::read");
        ret = inport.read();
        //data read failed, because InPortPushConnectorMock::read() return BUFFER_TIMEOUT
        CPPUNIT_ASSERT_EQUAL(false, ret);
        CPPUNIT_ASSERT_EQUAL((CORBA::Long)777, tl.data);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("InPortPushConnector::read"));

        //
        //
        //
        RTC::InPortPushConnector_read_return_value 
                                           = RTC::ConnectorBase::UNKNOWN_ERROR;
        logcount = ::RTC::RTC_logger.countLog("InPortPushConnector::read");
        ret = inport.read();
        //data read failed, because InPortPushConnectorMock::read() return UNKNOWN_ERROR
        CPPUNIT_ASSERT_EQUAL(false, ret);
        CPPUNIT_ASSERT_EQUAL((CORBA::Long)777, tl.data);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("InPortPushConnector::read"));

	m_pPOA->deactivate_object(*m_pPOA->servant_to_id(&inport));
        delete provider;
    }
    /*!
     * @brief setOnWrite()メソッドのテスト
     * 
     */
    void test_setOnWrite(void)
    {
    }
    /*!
     * @brief setOnUnderflow()メソッドのテスト
     * 
     */
    void test_setOnUnderflow()
    {
    }

    /*!
     * @brief write()メソッドとread()メソッドのテスト
     * 
     * - write()で書き込んだ値が、read()で正しく読み出されるか？
     */
/*
    void test_write_and_read()
    {
      double bindValue;
      std::auto_ptr<RTC::InPort<double> > inPort(
						 new RTC::InPort<double>("InPort", bindValue, 8));
			
      for (int i = 0; i < 100; ++i)
	{
	  double writeValue = i * 3.14159265;
	  CPPUNIT_ASSERT(inPort->write(writeValue));
				
	  // write()で書き込んだ値が、read()で正しく読み出されるか？
	  double readValue = inPort->read();
	  CPPUNIT_ASSERT_EQUAL(writeValue, readValue);
	}
    }
*/
		
    /*!
     * @brief write()メソッドのOnWriteコールバック呼出テスト
     * 
     * - あらかじめ設定されたOnWriteコールバックが正しく呼び出されるか？
     */
/*
    void test_write_OnWrite()
    {
      double bindValue;
      std::auto_ptr<RTC::InPort<double> > inPort(
						 new RTC::InPort<double>("InPort", bindValue, 8));
			
      OnWriteMock<double> onWrite;
      onWrite.m_value = 0;
      inPort->setOnWrite(&onWrite);
			
      // write()メソッドは成功するか？
      double writeValue = 3.14159265;
      CPPUNIT_ASSERT(inPort->write(writeValue));
			
      // あらかじめ設定されたOnWriteコールバックが正しく呼び出されたか？
      CPPUNIT_ASSERT_EQUAL(writeValue, onWrite.m_value);
    }
*/

    /*!
     * @brief バッファフル時のwrite()メソッドのOnWriteコールバック呼出テスト
     * 
     * - あらかじめ設定されたOnWriteコールバックが正しく呼び出されるか？
     */
/* 
   void test_write_OnWrite_full()
    {
      double bindValue;
      std::auto_ptr<RTC::InPort<double, FullBuffer> > inPort(
							     new RTC::InPort<double, FullBuffer>("InPort", bindValue, 8));
			
      OnWriteMock<double> onWrite;
      onWrite.m_value = 0;
      inPort->setOnWrite(&onWrite);
			
      // バッファフルによりwrite()メソッドは意図どおり失敗するか？
      double writeValue = 3.14159265;
      CPPUNIT_ASSERT(! inPort->write(writeValue));
			
      // あらかじめ設定されたOnWriteコールバックが正しく呼び出されたか？
      CPPUNIT_ASSERT_EQUAL(writeValue, onWrite.m_value);
    }
*/

    /*!
     * @brief write()メソッドのOnOverflowコールバック呼出テスト
     * 
     * - バッファがフルの場合に、あらかじめ設定されたOnOverflowコールバックが正しく呼び出されるか？
     */
/* 
   void test_write_OnOverflow()
    {
      // 常にフル状態であるバッファを用いてInPortオブジェクトを生成する
      double bindValue;
      std::auto_ptr<RTC::InPort<double, FullBuffer> > inPort(
							     new RTC::InPort<double, FullBuffer>("InPort", bindValue, 8));
			
      OnOverflowMock<double> onOverflow;
      onOverflow.m_value = 0;
      inPort->setOnOverflow(&onOverflow);

      // バッファフルによりwrite()メソッドは意図どおり失敗するか？
      double writeValue = 3.14159265;
      CPPUNIT_ASSERT(! inPort->write(writeValue));
			
      // OutPortに割り当てされたバッファがフルの場合に、あらかじめ設定されたOnOverflowコールバックが正しく呼び出されたか？
      CPPUNIT_ASSERT_EQUAL(writeValue, onOverflow.m_value);
    }
*/

    /*!
     * @brief バッファフルでない時の、write()メソッドのOnOverflowコールバック呼出テスト
     * 
     * - バッファフルでない場合、OnOverflowコールバックが意図どおり未呼出のままか？
     */
/*
    void test_write_OnOverflow_not_full()
    {
      double bindValue;
      std::auto_ptr<RTC::InPort<double> > inPort(
						 new RTC::InPort<double>("InPort", bindValue, 8));

      OnOverflowMock<double> onOverflow;
      onOverflow.m_value = 0;
      inPort->setOnOverflow(&onOverflow);

      // write()メソッドは成功するか？
      double writeValue = 3.14159265;
      CPPUNIT_ASSERT(inPort->write(writeValue));
			
      // バッファフルでない場合、OnOverflowコールバックが意図どおり未呼出のままか？
      CPPUNIT_ASSERT_EQUAL((double) 0, onOverflow.m_value);
    }
*/
		
    /*!
     * @brief write()メソッドのOnWriteConvertコールバック呼出テスト
     * 
     * - 意図したとおり、OnWriteConvertコールバックで変換した値がバッファに書き込まれるか？
     */
/*
    void test_write_OnWriteConvert()
    {
      double bindValue;
      std::auto_ptr<RTC::InPort<double> > inPort(
						 new RTC::InPort<double>("InPort", bindValue, 8));
			
      double amplitude = 1.41421356;
      OnWriteConvertMock onWriteConvert(amplitude);
      inPort->setOnWriteConvert(&onWriteConvert);
			
      for (int i = 0; i < 100; ++i)
	{
	  double writeValue = i * 3.14159265;
	  CPPUNIT_ASSERT(inPort->write(writeValue));
				
	  double readValue = inPort->read();
				
	  // write()で書き込んだ値が、read()で正しく読み出されるか？
	  double expectedValue = amplitude * writeValue;
	  CPPUNIT_ASSERT_EQUAL(expectedValue, readValue);
	}
    }
*/

    /*!
     * @brief write()メソッドのタイムアウト処理テスト
     * 
     * - バッファがフルの場合に、指定した時間どおりにwrite()メソッドがタイムアウトするか？
     * - バッファがフルの場合に、write()メソッドが意図どおり失敗するか？
     */
/*
    void test_write_timeout()
    {
      // 常にフル状態であるバッファを用いてInPortオブジェクトを生成する
      bool readBlock = false;
      bool writeBlock = true; // ブロッキングモードON
      int readTimeout = 0;
      int writeTimeout = WTIMEOUT_USEC; // タイムアウト値を指定する

      double bindValue;
      std::auto_ptr<RTC::InPort<double, FullBuffer> > inPort(
							     new RTC::InPort<double, FullBuffer>(
												 "InPort", bindValue, 8, readBlock, writeBlock, readTimeout, writeTimeout));
			
      timeval tm_pre;
      gettimeofday(&tm_pre, 0);
			
      for (int i = 0; i < 10; ++i) {
				
	double writeValue = i * 3.14159265;
				
	// OutPortに割り当てられたバッファがフルの場合に、write()メソッドが意図どおり失敗するか？
	CPPUNIT_ASSERT(! inPort->write(writeValue));
				
	// OutPortに割り当てされたバッファがフルの場合に、指定した時間どおりにwrite()メソッドがタイムアウトしているか？
	timeval tm_cur;
	gettimeofday(&tm_cur, 0);
				
	timeval tm_diff;
	timersub(&tm_cur, &tm_pre, &tm_diff);
				
	double interval = (double) tm_diff.tv_sec
	  + (double) tm_diff.tv_usec / USEC_PER_SEC;
					
	tm_pre = tm_cur;
				
	CPPUNIT_ASSERT_DOUBLES_EQUAL(
				     (double) WTIMEOUT_USEC / USEC_PER_SEC, interval,
				     0.1 * WTIMEOUT_USEC/USEC_PER_SEC);
      }
    }
*/
		
  };
}; // namespace InPort

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(InPort::InPortTests);

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
