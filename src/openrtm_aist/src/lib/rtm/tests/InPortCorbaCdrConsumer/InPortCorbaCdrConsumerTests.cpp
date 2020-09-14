// -*- C++ -*-
/*!
 * @file   InPortCorbaCdrConsumerTests.cpp
 * @brief  InPortCorbaCdrConsumer test class
 * @date   $Date: 2008/02/21 07:36:39 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id: InPortCorbaCdrConsumerTests.cpp 1007 2008-10-31 01:35:29Z fsi-katami $
 *
 */

/*
 * $Log: InPortCorbaCdrConsumerTests.cpp,v $
 *
 *
 */

#ifndef InPortCorbaCdrConsumer_cpp
#define InPortCorbaCdrConsumer_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/DataPortSkel.h>
#include <rtm/RTC.h>
#include <rtm/Typename.h>
#include <rtm/InPortCorbaCdrConsumer.h>
#include <rtm/InPortCorbaCdrProvider.h>
#include <rtm/CdrBufferBase.h>
#include <rtm/PortAdmin.h>
#include <rtm/CORBA_SeqUtil.h>
#include <rtm/NVUtil.h>
#include <rtm/ConnectorListener.h>
#include <rtm/InPortPushConnector.h>

/*!
 * @class InPortCorbaCdrConsumerTests class
 * @brief InPortCorbaCdrConsumer test
 */
namespace InPortCorbaCdrConsumer
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
   */
  class InPortCorbaCdrConsumerMock
    : public RTC::InPortCorbaCdrConsumer
  {
  public:
      /*!
       * 
       * 
       */
      InPortCorbaCdrConsumerMock(void)
      {
      }
      /*!
       * 
       * 
       */
      virtual ~InPortCorbaCdrConsumerMock()
      {
      }
      /*!
       *  確認用 
       * 
       */
      CORBA::Object_var get_m_objre()
      {
          return m_objref;
      }
  };
  class InPortCorbaCdrConsumerTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(InPortCorbaCdrConsumerTests);

    CPPUNIT_TEST(test_case0);

    CPPUNIT_TEST_SUITE_END();
	
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
    RTC::ConnectorListeners m_listeners;
    RTC::InPortConnector* connector;

  public:
	
    /*!
     * @brief Constructor
     */
    InPortCorbaCdrConsumerTests()
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
    ~InPortCorbaCdrConsumerTests()
    {
      delete connector;
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
     * @brief 
     * 
     */
    void test_case0()
    {
        InPortCorbaCdrConsumerMock consumer;

        //ConnectorInfo
        coil::Properties prop;
        coil::vstring ports;
        RTC::ConnectorInfo info("name", "id", ports, prop);

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

        RTC::ConnectorProfile prof;
        bool ret;
        int testdata[8] = { 12,34,56,78,90,23,45,99 };

        consumer.init(prop);
        ret = consumer.subscribeInterface(prof.properties);
        //ior を設定していないのでfalseとなることを確認する。
        CPPUNIT_ASSERT_EQUAL(false, ret);

        RTC::InPortCorbaCdrProvider provider;
        provider.setListener(info, &m_listeners);
        consumer.publishInterfaceProfile(prof.properties);

        CORBA_SeqUtil::push_back(prof.properties,
                                 NVUtil::newNV("dataport.interface_type",
                                               "corba_cdr"));
        provider.publishInterface(prof.properties);

        ret = consumer.subscribeInterface(prof.properties);
        CPPUNIT_ASSERT_EQUAL(true, ret);

        cdrMemoryStream indata;
        indata.setByteSwapFlag(true);
        ::RTC::DataPortStatus::Enum retcode;

        for(int ic(0);ic<8;++ic)
        {
            RTC::TimedLong td;
            td.data = testdata[ic];
            td >>= indata;
        }

        //provider 側の buffer がない状態でコール(error)
        retcode = consumer.put(indata);
        CPPUNIT_ASSERT_EQUAL((::RTC::DataPortStatus::Enum)1, retcode);

        RTC::CdrBufferBase* buffer;
        buffer = RTC::CdrBufferFactory::instance().createObject("ring_buffer");
        provider.setBuffer(buffer);

        connector = new RTC::InPortPushConnector(info, &provider, m_listeners, buffer);
        if (connector == 0)
          {
            std::cout << "ERROR: PushConnector creation failed." << std::endl;
          }
        provider.setConnector(connector);

        for(int ic(0);ic<8;++ic)
        {
            retcode = consumer.put(indata);
            CPPUNIT_ASSERT_EQUAL((::RTC::DataPortStatus::Enum)0, retcode);
         }

        //full の状態でコール(full)
        retcode = consumer.put(indata);
        CPPUNIT_ASSERT_EQUAL((::RTC::DataPortStatus::Enum)0, retcode);

        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream cdr;
            buffer->read(cdr);

            CORBA::ULong inlen = cdr.bufSize();
            CPPUNIT_ASSERT_EQUAL(96,(int)inlen);
            RTC::TimedLong rtd;
            rtd <<= cdr;
            CPPUNIT_ASSERT_EQUAL(testdata[0], (int)rtd.data);
        }

        CPPUNIT_ASSERT(!CORBA::is_nil(consumer.get_m_objre()));
        consumer.unsubscribeInterface(prof.properties);
        CPPUNIT_ASSERT(CORBA::is_nil(consumer.get_m_objre()));

        int index;
        index = NVUtil::find_index(prof.properties,
                                   "dataport.corba_cdr.inport_ior");
         const char* ior;
         if (prof.properties[index].value >>= ior)
         {
             CORBA::Object_ptr var = m_pORB->string_to_object(ior);
             PortableServer::Servant ser = m_pPOA->reference_to_servant(var);
	     m_pPOA->deactivate_object(*m_pPOA->servant_to_id(ser));
         }

        // delete connector;
        // ここで delete すると落ちるので、デストラクタで行う。
    }
    
  };
}; // namespace OutPortBase

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(InPortCorbaCdrConsumer::InPortCorbaCdrConsumerTests);

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
