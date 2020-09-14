// -*- C++ -*-
/*!
 * @file   InPortConnectorTests.cpp
 * @brief  InPortConnector test class
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
 * $Id: InPortConnectorTests.cpp 817 2008-08-06 02:54:26Z n-ando $
 *
 */


#ifndef InPortConnector_cpp
#define InPortConnector_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/Properties.h>

#include <rtm/InPortConnector.h>
#include <rtm/CdrBufferBase.h>
#include <rtm/CORBA_SeqUtil.h>
#include <rtm/NVUtil.h>
#include <rtm/ConnectorBase.h>
#include <rtm/DataPortStatus.h>

/*!
 * @class InPortConnectorTests class
 * @brief InPortConnector test
 */
namespace InPortConnector
{
  class InPortConnectorMock
    : public RTC::InPortConnector
  {
  public:
      InPortConnectorMock(RTC::ConnectorInfo& info,
                          RTC::CdrBufferBase* buffer)
        : RTC::InPortConnector(info, buffer)
      {
      }
      virtual ~InPortConnectorMock()
      {
      }
      virtual ReturnCode disconnect()
      {
          return ::RTC::DataPortStatus::PORT_OK;
      }
      virtual ReturnCode read(cdrMemoryStream& data)
      {
          return ::RTC::DataPortStatus::PORT_OK;
      }
      virtual void activate()
      {
      }
      virtual void deactivate()
      {
      }
  };

  class InPortConnectorTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(InPortConnectorTests);

    CPPUNIT_TEST(test_case0);

    CPPUNIT_TEST_SUITE_END();
		
  private:

  public:
	
    /*!
     * @brief Constructor
     */
    InPortConnectorTests()
    {
        int argc(0);
        char** argv(NULL);
        CORBA::ORB_ptr m_pORB = CORBA::ORB_init(argc, argv);
        PortableServer::POA_ptr m_pPOA = PortableServer::POA::_narrow(
		    m_pORB->resolve_initial_references("RootPOA"));
        m_pPOA->the_POAManager()->activate();
    }
		
    /*!
     * @brief Destructor
     */
    ~InPortConnectorTests()
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
     * @brief profile(),id(),name(),getBuffer(),setEndian(),isLittleEndian() メソッドのテスト
     * 
     */
    void test_case0()
    {

        RTC::ConnectorProfile cprof;
        cprof.connector_id = "id";
        cprof.name = CORBA::string_dup("InPortConnectorTest");

        CORBA_SeqUtil::push_back(cprof.properties,
                                 NVUtil::newNV("dataport.interface_type",
					       "corba_cdr"));
        CORBA_SeqUtil::push_back(cprof.properties,
                                 NVUtil::newNV("dataport.dataflow_type",
					       "push"));
        CORBA_SeqUtil::push_back(cprof.properties,
                                 NVUtil::newNV("dataport.subscription_type",
					       "new"));


        coil::Properties prop;
        NVUtil::copyToProperties(prop, cprof.properties);
        RTC::ConnectorInfo info(cprof.name,
                                cprof.connector_id,
                                CORBA_SeqUtil::refToVstring(cprof.ports),
                                prop); 
        RTC::CdrBufferBase* m_thebuffer;
        m_thebuffer = RTC::CdrBufferFactory::instance().createObject("ring_buffer");

        InPortConnector::InPortConnectorMock connector(info, m_thebuffer);
        CPPUNIT_ASSERT_EQUAL(std::string(cprof.connector_id), std::string(connector.id()));
        CPPUNIT_ASSERT_EQUAL(std::string(cprof.name), std::string(connector.name()));
        CPPUNIT_ASSERT_EQUAL(m_thebuffer, connector.getBuffer());

        connector.setEndian(false);
        CPPUNIT_ASSERT(!connector.isLittleEndian());
        connector.setEndian(true);
        CPPUNIT_ASSERT(connector.isLittleEndian());

        RTC::ConnectorInfo info2 = connector.profile();
        CPPUNIT_ASSERT_EQUAL(info.name, info2.name);
        CPPUNIT_ASSERT_EQUAL(info.id, info2.id);
    }

  };
}; // namespace InPortConnector

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(InPortConnector::InPortConnectorTests);

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
#endif // InPortConnector_cpp
