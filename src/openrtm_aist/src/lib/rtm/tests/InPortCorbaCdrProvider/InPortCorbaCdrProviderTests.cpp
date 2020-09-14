// -*- C++ -*-
/*!
 * @file   InPortCorbaCdrProviderTests.cpp
 * @brief  InPortCorbaCdrProvider test class
 * @date   $Date: 2008/02/21 07:36:39 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id: InPortCorbaCdrProviderTests.cpp 1007 2008-10-31 01:35:29Z fsi-katami $
 *
 */

/*
 * $Log: InPortCorbaCdrProviderTests.cpp,v $
 *
 *
 */

#ifndef InPortCorbaCdrProvider_cpp
#define InPortCorbaCdrProvider_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/RTC.h>
#include <rtm/Typename.h>
#include <rtm/InPortCorbaCdrProvider.h>
#include <rtm/CdrBufferBase.h>
#include <rtm/PortAdmin.h>
#include <rtm/CORBA_SeqUtil.h>
#include <rtm/ConnectorListener.h>
#include <rtm/InPortPushConnector.h>

/*!
 * @class InPortCorbaCdrProviderTests class
 * @brief InPortCorbaCdrProvider test
 */
namespace InPortCorbaCdrProvider
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
  class InPortCorbaCdrProviderMock
    : public RTC::InPortCorbaCdrProvider
  {
  public:
      /*!
       * 
       * 
       */
      InPortCorbaCdrProviderMock(void)
      {
      }
      /*!
       * 
       * 
       */
      virtual ~InPortCorbaCdrProviderMock()
      {
      }
      /*!
       *  確認用 
       * 
       */
      SDOPackage::NVList get_m_properties()
      {
          return m_properties;
      }
  };
  class InPortCorbaCdrProviderTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(InPortCorbaCdrProviderTests);

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
    InPortCorbaCdrProviderTests()
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
    ~InPortCorbaCdrProviderTests()
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
        InPortCorbaCdrProviderMock provider;
        CORBA::Long index;

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

        //IOR をプロパティに追加することを確認
        index = NVUtil::find_index(provider.get_m_properties(),"dataport.corba_cdr.inport_ior");
        CPPUNIT_ASSERT(0<=index);

        //ref をプロパティに追加することを確認
        index = NVUtil::find_index(provider.get_m_properties(),"dataport.corba_cdr.inport_ref");
        CPPUNIT_ASSERT(0<=index);

        provider.init(prop);
        provider.setListener(info, &m_listeners);

        ::OpenRTM::PortStatus ret;
        ::OpenRTM::CdrData data;
        int testdata[10] = { 0,1,2,3,4,5,6,7,8,9 };
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        CORBA::ULong len;

        td.data = testdata[0];
        td >>= cdr;
        len = (CORBA::ULong)cdr.bufSize();
        data.length(len);
        cdr.get_octet_array(&(data[0]), len);
        ret = provider.put(data);

        // buffer 未設定によるPORT_ERROR
        CPPUNIT_ASSERT_EQUAL(::OpenRTM::PORT_ERROR,ret);

        RTC::CdrBufferBase* buffer;
        buffer = RTC::CdrBufferFactory::instance().createObject("ring_buffer");
        provider.setBuffer(buffer);

        connector = new RTC::InPortPushConnector(info, &provider, m_listeners, buffer);
        if (connector == 0)
          {
            std::cout << "ERROR: Connector creation failed." << std::endl;
          }
        provider.setConnector(connector);

        ret = provider.put(data);
        CPPUNIT_ASSERT_EQUAL(::OpenRTM::PORT_OK,ret);

        for( int i(1); i<7; ++i )
          {
            OpenRTM::PortStatus ret;
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            CORBA::ULong len;
            td.data = testdata[i];
            td >>= cdr;
            len = (CORBA::ULong)cdr.bufSize();
            data.length(len);
            cdr.get_octet_array(&(data[0]), len);
            ret = provider.put(data);
            CPPUNIT_ASSERT_EQUAL(::OpenRTM::PORT_OK,ret);

          }
        cdrMemoryStream cdr2;
        td.data = testdata[7];
        td >>= cdr2;
        len = (CORBA::ULong)cdr2.bufSize();
        data.length(len);
        cdr2.get_octet_array(&(data[0]), len);
        ret = provider.put(data);
        CPPUNIT_ASSERT_EQUAL(::OpenRTM::PORT_OK,ret);

        // delete connector;
        // ここで delete すると落ちるので、デストラクタで行う。
    }
    
  };
}; // namespace InPortCorbaCdrProvider

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(InPortCorbaCdrProvider::InPortCorbaCdrProviderTests);

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
#endif // InPortCorbaCdrProvider_cpp
