// -*- C++ -*-
/*!
 * @file   OutPortTests.cpp
 * @brief  OutPort test class
 * @date   $Date: 2008/02/19 08:13:36 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id$
 *
 */

/*
 * $Log: OutPortTests.cpp,v $
 * Revision 1.2  2008/02/19 08:13:36  arafune
 * Some tests were added and orthogonalized.
 *
 * Revision 1.1  2007/12/20 07:50:18  arafune
 * *** empty log message ***
 *
 * Revision 1.2  2006/12/02 18:52:54  n-ando
 * Some tests were added.
 *
 * Revision 1.1  2006/11/29 04:26:28  n-ando
 * CppUnit tests for OutPort.
 *
 *
 */

#ifndef OutPort_cpp
#define OutPort_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/DataPortSkel.h>
#include <rtm/DataPortStatus.h>
#include <rtm/OutPort.h>
#include <rtm/OutPortBase.h>
#include <rtm/OutPortPushConnector.h>
#include <rtm/InPort.h>
#include <rtm/CorbaConsumer.h>

#define WTIMEOUT_USEC 1000000
#define USEC_PER_SEC 1000000

/*!
 * @class OutPortTests class
 * @brief OutPort test
 */
namespace OutPort
{
  /*!
   * 
   * 
   */
  template <class DataType>
  class OutPortMock
    : public RTC::OutPort<DataType>
  {
  public:
      /*!
       * 
       * 
       */
      OutPortMock(const char* name, DataType& value)
       : RTC::OutPort<DataType>(name, value)
      {
      }
      /*!
       * 
       * 
       */
      virtual ~OutPortMock()
      {
      }
      /*!
       * 
       * 
       */
      RTC::OutPortConnector*
      createConnector_public(RTC::ConnectorProfile& cprof, 
                            coil::Properties& prop,
                            RTC::OutPortProvider* provider)
      {
         return RTC::OutPort<DataType>::createConnector(cprof, prop, provider);
      }
      /*!
       * 
       * 
       */
      RTC::OutPortConnector*
      createConnector_public(RTC::ConnectorProfile& cprof, 
                            coil::Properties& prop,
                            RTC::InPortConsumer* consumer)
      {
         return RTC::OutPort<DataType>::createConnector(cprof, prop, consumer);
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
  class OutPortCorbaCdrProviderMock
    : public RTC::OutPortProvider,
      public virtual ::POA_OpenRTM::OutPortCdr,
      public virtual PortableServer::RefCountServantBase
  {

  public:
      OutPortCorbaCdrProviderMock(void)
       {
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
      void setLogger(Logger* logger)
      {
          m_logger = logger;
      }
  private:
    Logger* m_logger;

  };
/****
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
****/

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

          CORBA::Octet oct[8];
          cdr.get_octet_array (oct, (int)inlen);
          long lval(0);
          for(int ic(0);ic<(int)inlen;++ic)
          {
              lval = lval+(int)(oct[ic]<<(ic*8));
          }
          std::stringstream ss;
          ss << lval;
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
      ::RTC::DataPortStatus::Enum setListener(RTC::ConnectorInfo& info,
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
	
};
/*!
 * 
 *
 *
 *
 *
 */
namespace RTC
{
  /*!
   *
   *
   */
  /*!
   * Global variable for test
   */
  OutPort::Logger RTC_logger;
  ConnectorBase::ReturnCode OutPortPushConnector_write_return_value;
  /*!
   *
   * Mock OutPortPushConnector
   *
   */
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  OutPortPushConnector::OutPortPushConnector(ConnectorInfo info, 
                                             InPortConsumer* consumer,
                                             ConnectorListeners& listeners,
                                             CdrBufferBase* buffer)
    : OutPortConnector(info),
      m_consumer(consumer), m_publisher(0), m_listeners(listeners), m_buffer(buffer)
  {

  }
  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  OutPortPushConnector::~OutPortPushConnector()
  {
  }
  /*!
   * @if jp
   * @brief 接続解除
   * @else
   * @brief disconnect
   * @endif
   */
  ConnectorBase::ReturnCode OutPortPushConnector::disconnect()
  {
    return PORT_OK;
  }
 /*!
  * 
  * 
  */
 CdrBufferBase* OutPortPushConnector::getBuffer()
 {
   return m_buffer;
 }
  /*!
   * @if jp
   * @brief データの書き込み
   * @else
   * @brief Writing data
   * @endif
   */
  ConnectorBase::ReturnCode
  OutPortPushConnector::write(const cdrMemoryStream& data)
  {
      RTC_logger.log("OutPortPushConnector::write");
      RTC::TimedDouble td; 
      cdrMemoryStream cdr(data);
      td <<= cdr;
      std::ostringstream os;
      os<<td.data;
      RTC_logger.log(os.str());
//      ::CORBA::Any any;
//      any <<= cdr;
//      std::cout<<any.type()->kind()<<std::endl;

      return OutPortPushConnector_write_return_value;
  }
 /*!
  * 
  * 
  */
    void OutPortPushConnector::activate()
    {
    }
 /*!
  * 
  * 
  */
    void OutPortPushConnector::deactivate()
    {
    }
 /*!
  * 
  * 
  */
  CdrBufferBase* OutPortPushConnector::createBuffer(ConnectorInfo& info)
  {
      return new ::OutPort::CdrRingBufferMock();
  }
 /*!
  * 
  * 
  */
  PublisherBase* OutPortPushConnector::createPublisher(ConnectorInfo& info)
  {
      return new ::OutPort::PublisherFlushMock();
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
  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  OutPortProvider::~OutPortProvider(void)
  {
  }

  void OutPortProvider::init(coil::Properties& prop)
  {
  }

  void OutPortProvider::setBuffer(BufferBase<cdrMemoryStream>* buffer)
  {
  }

  /*!
   * @if jp
   * @brief InterfaceProfile情報を公開する
   * @else
   * @brief Publish InterfaceProfile information
   * @endif
   */
  void OutPortProvider::publishInterfaceProfile(SDOPackage::NVList& prop)
  {
    NVUtil::appendStringValue(prop, "dataport.interface_type",
			      m_interfaceType.c_str());
    NVUtil::append(prop, m_properties);
  }
  
  /*!
   * @if jp
   * @brief Interface情報を公開する
   * @else
   * @brief Publish interface information
   * @endif
   */
  bool OutPortProvider::publishInterface(SDOPackage::NVList& prop)
  {

    if (!NVUtil::isStringValue(prop,
			       "dataport.interface_type",
			       m_interfaceType.c_str()))
      {
	return false;
      }
    
    NVUtil::append(prop, m_properties);
    return true;
  }
  
  /*!
   * @if jp
   * @brief ポートタイプを設定する
   * @else
   * @brief Set the port type
   * @endif
   */
  void OutPortProvider::setPortType(const char* port_type)
  {
    m_portType = port_type;
  }
  
  /*!
   * @if jp
   * @brief データタイプを設定する
   * @else
   * @brief Set the data type
   * @endif
   */
  void OutPortProvider::setDataType(const char* data_type)
  {
    m_dataType = data_type;
  }
  
  /*!
   * @if jp
   * @brief インターフェースタイプを設定する
   * @else
   * @brief Set the interface type
   * @endif
   */
  void OutPortProvider::setInterfaceType(const char* interface_type)
  {
    m_interfaceType = interface_type;
  }
  
  /*!
   * @if jp
   * @brief データフロータイプを設定する
   * @else
   * @brief Set the data flow type
   * @endif
   */
  void OutPortProvider::setDataFlowType(const char* dataflow_type)
  {
    m_dataflowType = dataflow_type;
  }
  
  /*!
   * @if jp
   * @brief サブスクリプションタイプを設定する
   * @else
   * @brief Set the subscription type
   * @endif
   */
  void OutPortProvider::setSubscriptionType(const char* subs_type)
  {
    m_subscriptionType = subs_type;
  }


}; // namespace RTC
/*!
 * 
 *
 *
 *
 *
 */
namespace OutPort
{	
  /*!
   * 
   * 
   * 
   */
  template <class DataType>
  class OnWriteMock
    : public RTC::OnWrite<DataType>
  {
  public:
    /*!
     * 
     * 
     */
    OnWriteMock(void)
    {
        m_logger = NULL;
    }
    /*!
     * 
     * 
     */
    virtual void operator()(const DataType& value)
    {
      m_value = value;
      if (m_logger != NULL)
      {
         m_logger->log("OnWriteMock::operator");
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
  struct OnWriteConvertMock
    : public RTC::OnWriteConvert<DataType>
  {
  public:
    /*!
     *
     *
     */
    OnWriteConvertMock(RTC::TimedDouble amplitude)
      : m_amplitude(amplitude)
    {
        m_logger = NULL;
    }
    /*!
     *
     *
     */
    virtual DataType operator()(const DataType& value)
    {
      DataType td;   
      td.data = m_amplitude.data * value.data;
      if (m_logger != NULL)
      {
         m_logger->log("OnWriteConvertMock::operator");
      }
      return td;
    }
    RTC::TimedDouble m_amplitude;
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
  class OutPortTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(OutPortTests);
    CPPUNIT_TEST(test_write);  // include onWrite and OnWriteConver
/*
    CPPUNIT_TEST(test_write_OnWrite);
    CPPUNIT_TEST(test_write_OnWriteConvert);
    CPPUNIT_TEST(test_write_OnWrite_full);
    CPPUNIT_TEST(test_write_OnOverflow);
    CPPUNIT_TEST(test_write_OnOverflow_not_full);
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
    OutPortTests()
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
    ~OutPortTests()
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
     * @brief write()メソッドのテスト
     * 
     */
    void test_write(void)
    {
        RTC::TimedDouble td;
        td.data = 123;
        OutPortMock<RTC::TimedDouble> outport("testi_write0",td);

        OnWriteMock<RTC::TimedDouble> onWrite;
        Logger logger;
        onWrite.setLogger(&logger);
        outport.setOnWrite(&onWrite);

        bool ret;
        CPPUNIT_ASSERT_EQUAL(0,logger.countLog("OnWriteMock::operator"));
        ret = outport.write();
        //no connectors was return false
        CPPUNIT_ASSERT_EQUAL(false, ret);
        CPPUNIT_ASSERT_EQUAL(1,logger.countLog("OnWriteMock::operator"));

        RTC::ConnectorProfile prof;
        coil::Properties prop(outport.properties());
        RTC::InPortConsumer* consumer = new InPortCorbaCdrConsumerMock();
        RTC::OutPortConnector* connector 
              = outport.createConnector_public(prof, prop, consumer);
        CPPUNIT_ASSERT(0!= connector);
        //
        //
        //
        RTC::TimedDouble amplitude;
//        amplitude.data = 1.41421356;
        amplitude.data = 1;
        OnWriteConvertMock<RTC::TimedDouble> onWriteConvert(amplitude);
        onWriteConvert.setLogger(&logger);

        RTC::OutPortPushConnector_write_return_value 
                                           = RTC::ConnectorBase::PORT_OK;
        int logcount;
        int writedatacount;

        CPPUNIT_ASSERT_EQUAL(0,logger.countLog("OnWriteConvertMock::operator"));
        logcount = ::RTC::RTC_logger.countLog("OntPortPushConnector::write");
        ret = outport.write();
        //data read succeeded
        CPPUNIT_ASSERT_EQUAL(true, ret);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("OutPortPushConnector::write"));
        CPPUNIT_ASSERT_EQUAL(0,logger.countLog("OnReadConvertMock::operator"));

        outport.setOnWriteConvert(&onWriteConvert);
        CPPUNIT_ASSERT_EQUAL(0,logger.countLog("OnReadConvertMock::operator"));
        logcount = ::RTC::RTC_logger.countLog("OutPortPushConnector::write");
        ret = outport.write();
        //data read succeeded
        CPPUNIT_ASSERT_EQUAL(true, ret);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("OutPortPushConnector::write"));
        CPPUNIT_ASSERT_EQUAL(1,logger.countLog("OnWriteConvertMock::operator"));

        //
        //
        //
        RTC::OutPortPushConnector_write_return_value 
                                           = RTC::ConnectorBase::PORT_OK;
        outport.setOnWrite(&onWrite);

        logcount = ::RTC::RTC_logger.countLog("OutPortPushConnector::write");
        writedatacount = ::RTC::RTC_logger.countLog("123");
        ret = outport.write();
        //data read succeeded
        CPPUNIT_ASSERT_EQUAL(true, ret);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("OutPortPushConnector::write"));
        CPPUNIT_ASSERT_EQUAL(writedatacount+1,
                      ::RTC::RTC_logger.countLog("123"));
        //
        //
        //
/*** check OK
        RTC::OutPortPushConnector_write_return_value 
                                           = RTC::ConnectorBase::PRECONDITION_NOT_MET;
        logcount = ::RTC::RTC_logger.countLog("OutPortPushConnector::write");
        ret = outport.write();
        //data read succeeded
        CPPUNIT_ASSERT_EQUAL(false, ret);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("OutPortPushConnector::write"));
***/

        //
        //
        //
/*** check OK
        RTC::OutPortPushConnector_write_return_value 
                                           = RTC::ConnectorBase::CONNECTION_LOST;
        logcount = ::RTC::RTC_logger.countLog("OutPortPushConnector::write");
        ret = outport.write();
        //data read succeeded
        CPPUNIT_ASSERT_EQUAL(false, ret);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("OutPortPushConnector::write"));
***/

        //
        //
        //
/*** check OK
        RTC::OutPortPushConnector_write_return_value 
                                           = RTC::ConnectorBase::BUFFER_FULL;
        logcount = ::RTC::RTC_logger.countLog("OutPortPushConnector::write");
        ret = outport.write();
        //data read succeeded
        CPPUNIT_ASSERT_EQUAL(false, ret);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("OutPortPushConnector::write"));
***/

        //
        //
        //

        RTC::OutPortPushConnector_write_return_value 
                                           = RTC::ConnectorBase::PORT_OK;

        outport.setOnWrite(&onWrite);

        logcount = ::RTC::RTC_logger.countLog("OutPortPushConnector::write");
        writedatacount = ::RTC::RTC_logger.countLog("55");
        RTC::TimedDouble doublevalue;
        doublevalue.data = 55;

        ret = outport.write(doublevalue);
        //data write succeeded
        CPPUNIT_ASSERT_EQUAL(true, ret);
        CPPUNIT_ASSERT_EQUAL(logcount+1,
                     ::RTC::RTC_logger.countLog("OutPortPushConnector::write"));
        CPPUNIT_ASSERT_EQUAL(writedatacount+1,
                      ::RTC::RTC_logger.countLog("55"));

	m_pPOA->deactivate_object(*m_pPOA->servant_to_id(&outport));
        delete consumer;
    }

    /*!
     * @brief write()メソッドのOnWriteコールバック呼出テスト
     * 
     * - あらかじめ設定されたOnWriteコールバックが正しく呼び出されるか？
     */
    void test_write_OnWrite()
    {
      RTC::TimedDouble bindValue;
      OutPortMock<RTC::TimedDouble>* outPort
	= new OutPortMock<RTC::TimedDouble>("OutPort", bindValue);

			
      OnWriteMock<RTC::TimedDouble> onWrite;
      onWrite.m_value.data = 0;
      outPort->setOnWrite(&onWrite);
			
      // write()メソッドは成功するか？
      RTC::TimedDouble writeValue;
      writeValue.data = 3.14159265;
      CPPUNIT_ASSERT(outPort->write(writeValue));
			
      // あらかじめ設定されたOnWriteコールバックが正しく呼び出されたか？
      CPPUNIT_ASSERT_EQUAL(writeValue.data, onWrite.m_value.data);

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(outPort));
      delete outPort;
    }
		
    /*!
     * @brief バッファフル時のwrite()メソッドのOnWriteコールバック呼出テスト
     * 
     * - あらかじめ設定されたOnWriteコールバックが正しく呼び出されるか？
     */
    void test_write_OnWrite_full()
    {
      RTC::TimedDouble bindValue;
      RTC::OutPort<RTC::TimedDouble>* outPort
	= new RTC::OutPort<RTC::TimedDouble>("OutPort", bindValue);
			
      OnWriteMock<RTC::TimedDouble> onWrite;
      onWrite.m_value.data = 0;
      outPort->setOnWrite(&onWrite);
			
      RTC::PortService_var oportref = outPort->get_port_profile()->port_ref;

      RTC::TimedDouble InbindValue;
      RTC::InPort<RTC::TimedDouble>* inPort
	= new RTC::InPort<RTC::TimedDouble>("InPort", InbindValue);

      coil::Properties dummy;
      inPort->init(dummy);
      RTC::PortService_var iportref = inPort->get_port_profile()->port_ref;

      RTC::ConnectorProfile prof;
      prof.connector_id = "";
      prof.name = CORBA::string_dup("connector0");
      prof.ports.length(2);
      prof.ports[0] = oportref;
      prof.ports[1] = iportref;

      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.interface_type",
					     "corba_cdr"));

      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.dataflow_type",
					     "push"));
      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.subscription_type",
					     "new"));
      inPort->connect(prof);

      // バッファフルによりwrite()メソッドは意図どおり失敗するか？
      RTC::TimedDouble writeValue;
      writeValue.data = 3.14159265;
      for(int ic(0);ic<8;++ic)
        {
          outPort->write(writeValue);
        }
      CPPUNIT_ASSERT(! outPort->write(writeValue));
			
      // あらかじめ設定されたOnWriteコールバックが正しく呼び出されたか？
      CPPUNIT_ASSERT_EQUAL(writeValue.data, onWrite.m_value.data);
      delete inPort;
      delete outPort;
    }
		
    /*!
     * @brief write()メソッドのOnOverflowコールバック呼出テスト
     * 
     * - OutPortに割り当てされたバッファがフルの場合に、あらかじめ設定されたOnOverflowコールバックが正しく呼び出されるか？
     */
    void test_write_OnOverflow()
    {
      // 常にフル状態であるバッファを用いてOutPortオブジェクトを生成する
      RTC::TimedDouble bindValue;
      RTC::OutPort<RTC::TimedDouble>* outPort
	= new RTC::OutPort<RTC::TimedDouble>("OutPort", bindValue);
			
//      OnOverflowMock<RTC::TimedDouble> onOverflow;
//      onOverflow.m_value.data = 0;
//      outPort->setOnOverflow(&onOverflow);

      RTC::PortService_var oportref = outPort->get_port_profile()->port_ref;
      RTC::TimedDouble InbindValue;
      RTC::InPort<RTC::TimedDouble>* inPort
	= new RTC::InPort<RTC::TimedDouble>("InPort", InbindValue);

      coil::Properties dummy;
      inPort->init(dummy);
      RTC::PortService_var iportref = inPort->get_port_profile()->port_ref;

      RTC::ConnectorProfile prof;
      prof.connector_id = "";
      prof.name = CORBA::string_dup("connector0");
      prof.ports.length(2);
      prof.ports[0] = oportref;
      prof.ports[1] = iportref;

      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.interface_type",
					     "corba_cdr"));

      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.dataflow_type",
					     "push"));
      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.subscription_type",
					     "new"));
      inPort->connect(prof);

      // バッファフルによりwrite()メソッドは意図どおり失敗するか？
      RTC::TimedDouble writeValue;
      writeValue.data = 3.14159265;
      for(int ic(0);ic<8;++ic)
        {
          outPort->write(writeValue);
        }
      CPPUNIT_ASSERT(! outPort->write(writeValue));
			
      // OutPortに割り当てされたバッファがフルの場合に、あらかじめ設定されたOnOverflowコールバックが正しく呼び出されたか？
//      CPPUNIT_ASSERT_EQUAL(writeValue.data, onOverflow.m_value.data);
      delete inPort;
      delete outPort;
    }

    /*!
     * @brief バッファフルでない時の、write()メソッドのOnOverflowコールバック呼出テスト
     * 
     * - バッファフルでない場合、OnOverflowコールバックが意図どおり未呼出のままか？
     */
    void test_write_OnOverflow_not_full()
    {
      RTC::TimedDouble bindValue;
      RTC::OutPort<RTC::TimedDouble>* outPort
	= new RTC::OutPort<RTC::TimedDouble>("OutPort", bindValue);

//      OnOverflowMock<RTC::TimedDouble> onOverflow;
//      onOverflow.m_value.data = 0;
//      outPort->setOnOverflow(&onOverflow);

      RTC::PortService_var oportref = outPort->get_port_profile()->port_ref;

      RTC::TimedDouble InbindValue;
      RTC::InPort<RTC::TimedDouble>* inPort
	= new RTC::InPort<RTC::TimedDouble>("InPort", InbindValue);

      coil::Properties dummy;
      inPort->init(dummy);
      RTC::PortService_var iportref = inPort->get_port_profile()->port_ref;

      RTC::ConnectorProfile prof;
      prof.connector_id = "";
      prof.name = CORBA::string_dup("connector0");
      prof.ports.length(2);
      prof.ports[0] = oportref;
      prof.ports[1] = iportref;

      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.interface_type",
					     "corba_cdr"));

      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.dataflow_type",
					     "push"));
      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.subscription_type",
					     "new"));
      inPort->connect(prof);
      // write()メソッドは成功するか？
      RTC::TimedDouble writeValue;
      writeValue.data = 3.14159265;
      CPPUNIT_ASSERT(outPort->write(writeValue));
			
      // バッファフルでない場合、OnOverflowコールバックが意図どおり未呼出のままか？
//      CPPUNIT_ASSERT_EQUAL((double) 0, onOverflow.m_value.data);
      delete inPort;
      delete outPort;
    }
		

    void test_write_OnWriteConvert()
    {
      RTC::TimedDouble bindValue;
      OutPortMock<RTC::TimedDouble>* outPort
	= new OutPortMock<RTC::TimedDouble>("OutPort", bindValue);
			
        RTC::ConnectorProfile prof;
        coil::Properties prop(outPort->properties());
        RTC::InPortConsumer* consumer = new InPortCorbaCdrConsumerMock();
        RTC::OutPortConnector* connector 
              = outPort->createConnector_public(prof, prop, consumer);

      RTC::TimedDouble amplitude;
      amplitude.data = 1.41421356;
      OnWriteConvertMock<RTC::TimedDouble> onWriteConvert(amplitude);
      outPort->setOnWriteConvert(&onWriteConvert);
			
      for (int i = 0; i < 100; ++i)
	{
	  RTC::TimedDouble writeValue;
	  writeValue.data = i * 3.14159265;
	  RTC::TimedDouble expectedValue;
	  expectedValue.data = amplitude.data * writeValue.data;

          std::ostringstream os;
          os << expectedValue.data;
          int logcount;
          logcount = ::RTC::RTC_logger.countLog(os.str());
	  CPPUNIT_ASSERT(outPort->write(writeValue));
          CPPUNIT_ASSERT_EQUAL(logcount+1,
                        ::RTC::RTC_logger.countLog(os.str()));
				
				
	}
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(outPort));
      delete consumer;
      delete outPort;
    }

		
    /*!
     * @brief write()メソッドのタイムアウト処理テスト
     * 
     * - OutPortに割り当てされたバッファがフルの場合に、指定した時間どおりにwrite()メソッドがタイムアウトするか？
     * - OutPortに割り当てられたバッファがフルの場合に、write()メソッドが意図どおり失敗するか？
     */
    void test_write_timeout()
    {
      // 常にフル状態であるバッファを用いてOutPortオブジェクトを生成する
      RTC::TimedDouble bindValue;
      RTC::OutPort<RTC::TimedDouble>* outPort
	= new RTC::OutPort<RTC::TimedDouble>("OutPort", bindValue);
			
      RTC::PortService_var oportref = outPort->get_port_profile()->port_ref;

      RTC::TimedDouble InbindValue;
      RTC::InPort<RTC::TimedDouble>* inPort
	= new RTC::InPort<RTC::TimedDouble>("InPort", InbindValue);

      coil::Properties dummy;
      inPort->init(dummy);
      RTC::PortService_var iportref = inPort->get_port_profile()->port_ref;

      RTC::ConnectorProfile prof;
      prof.connector_id = "";
      prof.name = CORBA::string_dup("connector0");
      prof.ports.length(2);
      prof.ports[0] = oportref;
      prof.ports[1] = iportref;

      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.interface_type",
					     "corba_cdr"));

      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.dataflow_type",
					     "push"));
      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.subscription_type",
					     "new"));
      inPort->connect(prof);

      // OutPortオブジェクトに対して、ブロッキングモードONを指定する
//      outPort->setWriteBlock(true);
			
      // OutPortオブジェクトに対して、タイムアウト値を指定する
//      outPort->setWriteTimeout(WTIMEOUT_USEC);
			
      timeval tm_pre;
      gettimeofday(&tm_pre, 0);
			
      for(int ic(0);ic<8;++ic)
        {
          RTC::TimedDouble writeValue;
          writeValue.data = 3.14159265;
          outPort->write(writeValue);
        }
      for (int i = 0; i < 10; ++i) {
				
	RTC::TimedDouble writeValue;
	writeValue.data = i * 3.14159265;
				
	// OutPortに割り当てられたバッファがフルの場合に、write()メソッドが意図どおり失敗するか？
	CPPUNIT_ASSERT(! outPort->write(writeValue));
				
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
      delete inPort;
      delete outPort;
    }
		
  };
}; // namespace OutPort

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(OutPort::OutPortTests);

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
#endif // OutPort_cpp
