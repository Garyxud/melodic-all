// -*- C++ -*-
/*!
 * @file   OutPortProviderTests.cpp
 * @brief  OutPortProvider test class
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
 * $Log: OutPortProviderTests.cpp,v $
 * Revision 1.1  2008/02/21 12:51:22  arafune
 * The first commitment.
 *
 */

#ifndef OutPortProvider_cpp
#define OutPortProvider_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <rtm/OutPortProvider.h>

/*!
 * @class OutPortProviderTests class
 * @brief OutPortProvider test
 */
/*
 *
 *
 * Mock RTC
 *
 *
 */
namespace RTC
{
 /*!
  *
  * Mock SystemLogger
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
   */
  bool Logger::setLevel(const char* level)
  {
    return true; 
  }

  /*!
   */
  void Logger::setDateFormat(const char* format)
  {
  }

  /*!
   */
  void Logger::setName(const char* name)
  {
  }

  /*!
   */
  void Logger::header(int level)
  {
  }

  /*!
   */
  std::string Logger::getDate(void)
  {
    const int maxsize = 256;
    char buf[maxsize];

    return std::string(buf);
  }

  /*!
   */
  int Logger::strToLevel(const char* level)
  {
      return 0;
  }
};
/*
 *
 *
 *
 *
 *
 */
namespace OutPortProvider
{
  class OutPortProviderMock
    : public RTC::OutPortProvider
  {
  public:
    OutPortProviderMock(
			const std::string& portType,
			const std::string& dataType,
			const std::string& interfaceType,
			const std::string& dataFlowType,
			const std::string& subscriptionType)
    {
      setPortType(portType.c_str());
      setDataType(dataType.c_str());
      setInterfaceType(interfaceType.c_str());
      setDataFlowType(dataFlowType.c_str());
      setSubscriptionType(subscriptionType.c_str());
			
    }

    void setDummydataInProperties(void)
    {
      NVUtil::appendStringValue(m_properties, "PROPERTY_NAME1", "PROPERTY_VALUE1");
      NVUtil::appendStringValue(m_properties, "PROPERTY_NAME2", "PROPERTY_VALUE2");
    }

    void setBuffer(RTC::CdrBufferBase* buffer)
    {
    }

    void setListener(RTC::ConnectorInfo& info, RTC::ConnectorListeners* listeners)
    {
    }
    void setConnector(RTC::OutPortConnector* connector)
    {
    }

  };
	
  int g_argc;
  std::vector<std::string> g_argv;

  class OutPortProviderTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(OutPortProviderTests);
    CPPUNIT_TEST(test_publishInterfaceProfile);
    CPPUNIT_TEST(test_publishInterfaceProfile2);
    CPPUNIT_TEST(test_publishInterface);
    CPPUNIT_TEST_SUITE_END();
		
  public:
	
    /*!
     * @brief Constructor
     */
    OutPortProviderTests()
    {
      char* argv[g_argc];
      for (int i = 0; i < g_argc; i++) {
	argv[i] = (char *)g_argv[i].c_str();
      }
	      
      CORBA::ORB_var orb = CORBA::ORB_init(g_argc, argv);
    }
		
    /*!
     * @brief Destructor
     */
    ~OutPortProviderTests()
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
     * @brief publishInterfaceProfile()メソッドのテスト。
     * 
     * - dataport.interface_typeプロパティを正しく取得できるか？
     * - dataport.data_typeプロパティは空か？
     * - dataport.dataflow_typeプロパティは空か？
     * - dataport.subscription_typeプロパティは空か？
     */
    void test_publishInterfaceProfile()
    {
      std::auto_ptr<RTC::OutPortProvider> provider(
          new OutPortProviderMock("PORT_TYPE", "DATA_TYPE", "INTERFACE_TYPE",
                                      "DATA_FLOW_TYPE", "SUBSCRIPTION_TYPE"));

      SDOPackage::NVList prop;
      provider->publishInterfaceProfile(prop);

      // dataport.data_typeプロパティは空か？
      {
	const char* value;
        try {
            NVUtil::find(prop, "dataport.data_type") >>= value;
	    CPPUNIT_FAIL("dataport.data_type fialure.");
        }
        catch(std::string ex) {
        }
        catch(...) {
	    CPPUNIT_FAIL("dataport.data_type failure.");
        }
      }
			
      // dataport.interface_typeプロパティを正しく取得できるか？
      {
	const char* value;
	NVUtil::find(prop, "dataport.interface_type") >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("INTERFACE_TYPE"), std::string(value));
      }
			
      // dataport.dataflow_typeプロパティは空か？
      {
	const char* value;
        try {
            NVUtil::find(prop, "dataport.dataflow_type") >>= value;
	    CPPUNIT_FAIL("dataport.dataflow_type failure.");
        }
        catch(std::string ex) {
        }
        catch(...) {
	    CPPUNIT_FAIL("dataport.dataflow_type failure.");
        }
      }
			
      // dataport.subscription_typeプロパティは空か？
      {
	const char* value;
        try {
            NVUtil::find(prop, "dataport.subscription_type") >>= value;
	    CPPUNIT_FAIL("dataport.subscription_type failure.");
        }
        catch(std::string ex) {
        }
        catch(...) {
	    CPPUNIT_FAIL("dataport.subscription_type failure.");
        }
      }
    }

    /*!
     * @brief publishInterface()メソッドのテスト
     * 
     * - 引数で渡したNameValueオブジェクトのインタフェースタイプが、ポートのそれと一致しない場合に、
     * Interface情報が取得されないことを確認する。
     * - 引数で渡したNameValueオブジェクトのインタフェースタイプが、ポートのそれと一致する場合に、
     * Interface情報を取得でき、それが期待値と一致することを確認する。
     */
    void test_publishInterface()
    {
      std::auto_ptr<OutPortProviderMock> provider(
          new OutPortProviderMock("PORT_TYPE", "DATA_TYPE", "INTERFACE_TYPE",
                                  "DATA_FLOW_TYPE", "SUBSCRIPTION_TYPE"));

      SDOPackage::NVList prop;
      provider->publishInterfaceProfile(prop);
			        
      // (1) インタフェースタイプが不一致の場合：
      SDOPackage::NVList prop_dummy = prop;
      for (CORBA::ULong i(0); i < prop_dummy.length(); ++i)
	{
	  if (std::string(prop_dummy[i].name) == std::string("dataport.interface_type"))
	    {
	      // インタフェースタイプが一致しないように、書き換える
	      prop_dummy[i].value <<= "DUMMY";
	    }
	}

      //m_propertiesへ"PROPERTY_NAME1",”PROPERTY_NAME2”を設定
      provider->setDummydataInProperties();
			
      provider->publishInterface(prop_dummy);
			
      // インタフェース情報が取得されないことを確認する
      CPPUNIT_ASSERT_EQUAL(CORBA::Long(-1), NVUtil::find_index(prop_dummy, "PROPERTY_NAME1"));
      CPPUNIT_ASSERT_EQUAL(CORBA::Long(-1), NVUtil::find_index(prop_dummy, "PROPERTY_NAME2"));
			
      // (2) インタフェースタイプ一致の場合：
      provider->publishInterface(prop);
			
      // インタフェース情報が取得されることを確認する
      CORBA::Long index1 = NVUtil::find_index(prop, "PROPERTY_NAME1");
      CORBA::Long index2 = NVUtil::find_index(prop, "PROPERTY_NAME2");
      CPPUNIT_ASSERT(CORBA::Long(-1) != index1);
      CPPUNIT_ASSERT(CORBA::Long(-1) != index2);
			
      const char* value1;
      prop[index1].value >>= value1;
      CPPUNIT_ASSERT_EQUAL(std::string("PROPERTY_VALUE1"), std::string(value1));

      const char* value2;			
      prop[index2].value >>= value2;
      CPPUNIT_ASSERT_EQUAL(std::string("PROPERTY_VALUE2"), std::string(value2));
    }
    /*!
     * @brief publishInterfaceProfile()メソッドのテスト
     * 
     * - publishInterfaceProfileメソッドでm_propertiesを読み込んでいることを確認する。
     */
    void test_publishInterfaceProfile2()
    {
      std::auto_ptr<OutPortProviderMock> provider(
          new OutPortProviderMock("PORT_TYPE", "DATA_TYPE", "INTERFACE_TYPE",
                                  "DATA_FLOW_TYPE", "SUBSCRIPTION_TYPE"));


      //m_propertiesへ"PROPERTY_NAME1",”PROPERTY_NAME2”を設定
      provider->setDummydataInProperties();

      SDOPackage::NVList prop;
      provider->publishInterfaceProfile(prop);
      // インタフェース情報が取得されることを確認する
      CORBA::Long index1 = NVUtil::find_index(prop, "PROPERTY_NAME1");
      CORBA::Long index2 = NVUtil::find_index(prop, "PROPERTY_NAME2");
      CPPUNIT_ASSERT(CORBA::Long(-1) != index1);
      CPPUNIT_ASSERT(CORBA::Long(-1) != index2);
			
      const char* value1;
      prop[index1].value >>= value1;
      CPPUNIT_ASSERT_EQUAL(std::string("PROPERTY_VALUE1"), std::string(value1));

      const char* value2;			
      prop[index2].value >>= value2;
      CPPUNIT_ASSERT_EQUAL(std::string("PROPERTY_VALUE2"), std::string(value2));

    }

  };
}; // namespace OutPortProvider

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(OutPortProvider::OutPortProviderTests);

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
#endif // OutPortProvider_cpp
