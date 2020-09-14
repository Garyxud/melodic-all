// -*- C++ -*-
/*!
 * @file   InPortProviderTests.cpp
 * @brief  InPortProvider test class
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

/*
 * $Log: InPortProviderTests.cpp,v $
 * Revision 1.1  2008/03/10 11:28:31  arafune
 * The first commitment.
 *
 * Revision 1.1  2008/02/21 12:51:22  arafune
 * The first commitment.
 *
 */

#ifndef InPortProvider_cpp
#define InPortProvider_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <rtm/InPortProvider.h>
#include <rtm/BufferBase.h>

/*!
 * @class InPortProviderTests class
 * @brief InPortProvider test
 */
namespace InPortProvider
{
  class InPortProviderMock
    : public RTC::InPortProvider
  {
  public:
    InPortProviderMock(
		       const std::string& interfaceType,
		       const std::string& dataFlowType,
		       const std::string& subscriptionType)
    {
      setInterfaceType(interfaceType.c_str());
      setDataFlowType(dataFlowType.c_str());
      setSubscriptionType(subscriptionType.c_str());
    }
    void setDummydataInProperties(void)
    {
      NVUtil::appendStringValue(m_properties, "KEY1", "VALUE1");
      NVUtil::appendStringValue(m_properties, "KEY2", "VALUE2");
    }
    void init(coil::Properties& prop)
      {
      }
    void setBuffer(RTC::BufferBase<cdrMemoryStream>* buffer) 
      {
      }
    void setListener(RTC::ConnectorInfo& info,
                     RTC::ConnectorListeners* listeners)
      {
      }
    void setConnector(RTC::InPortConnector* connector)
      {
      }
   };
	
  int g_argc;
  std::vector<std::string> g_argv;

  class InPortProviderTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(InPortProviderTests);
    CPPUNIT_TEST(test_publishInterfaceProfile);
    CPPUNIT_TEST(test_publishInterface_with_interfaceType_matched);
    CPPUNIT_TEST(test_publishInterface_with_interfaceType_unmatched);
    CPPUNIT_TEST_SUITE_END();
		
  public:
	
    /*!
     * @brief Constructor
     */
    InPortProviderTests()
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
    ~InPortProviderTests()
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
     * @brief publishInterfaceProfile()
     * 
     * - "dataport.interface_type"
     * - "dataport.dataflow_type"
     * - "dataport.subscription_type"
     */
    void test_publishInterfaceProfile()
    {
      std::auto_ptr<InPortProviderMock> provider(
          new InPortProviderMock("INTERFACE_TYPE", "DATA_FLOW_TYPE", "SUBSCRIPTION_TYPE"));

      //m_propertiesへ設定
      provider->setDummydataInProperties();

      SDOPackage::NVList prop;
      provider->publishInterfaceProfile(prop);
			
      // "dataport.interface_type"
      CPPUNIT_ASSERT_EQUAL(std::string("INTERFACE_TYPE"),
			   NVUtil::toString(prop, "dataport.interface_type"));
			
      // 
      CPPUNIT_ASSERT_EQUAL(std::string("VALUE1"), NVUtil::toString(prop, "KEY1"));
      CPPUNIT_ASSERT_EQUAL(std::string("VALUE2"), NVUtil::toString(prop, "KEY2"));
    }
		
    /*!
     * @brief publishInterface()
     * 
     * - 
     */
    void test_publishInterface_with_interfaceType_matched()
    {
      std::auto_ptr<InPortProviderMock> provider(
          new InPortProviderMock("INTERFACE_TYPE", "DATA_FLOW_TYPE", "SUBSCRIPTION_TYPE"));

      //m_propertiesへ設定
      provider->setDummydataInProperties();

      SDOPackage::NVList props;
      NVUtil::appendStringValue(props, "dataport.interface_type", "INTERFACE_TYPE");
      CPPUNIT_ASSERT_EQUAL(true, provider->publishInterface(props));

      // 
      CPPUNIT_ASSERT_EQUAL(std::string("VALUE1"), NVUtil::toString(props, "KEY1"));
      CPPUNIT_ASSERT_EQUAL(std::string("VALUE2"), NVUtil::toString(props, "KEY2"));
    }
		
    /*!
     * @brief publishInterface()
     * 
     * - 
     */
    void test_publishInterface_with_interfaceType_unmatched()
    {
      std::auto_ptr<InPortProviderMock> provider(
          new InPortProviderMock("INTERFACE_TYPE", "DATA_FLOW_TYPE", "SUBSCRIPTION_TYPE"));

      //m_propertiesへ設定
      provider->setDummydataInProperties();

      SDOPackage::NVList props;
      NVUtil::appendStringValue(props, "dataport.interface_type", "UNMATCHED_INTERFACE_TYPE");
      CPPUNIT_ASSERT_EQUAL(false, provider->publishInterface(props));

      // 
      CPPUNIT_ASSERT_EQUAL(std::string(""), NVUtil::toString(props, "KEY1"));
      CPPUNIT_ASSERT_EQUAL(std::string(""), NVUtil::toString(props, "KEY2"));
    }
		
  };
}; // namespace InPortProvider

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(InPortProvider::InPortProviderTests);

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
#endif // InPortProvider_cpp
