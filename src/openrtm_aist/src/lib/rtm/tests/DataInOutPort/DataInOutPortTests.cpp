// -*- C++ -*-
/*!
 * @file   DataInOutPortTests.cpp
 * @brief  DataInOutPort test class
 * @date   $Date: 2008/07/18 04:32:00 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id$
 *
 */

/*
 * $Log: DataInOutPortTests.cpp,v $
 * Revision 1.2  2008/07/18 04:32:00  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2007/12/20 07:50:18  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2007/01/06 18:04:51  n-ando
 * The first commitment.
 *
 *
 */

#ifndef DataInOutPort_cpp
#define DataInOutPort_cpp

#include <iostream>

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/RTCSkel.h>
#include <rtm/DataOutPort.h>
#include <rtm/OutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/InPort.h>
#include <coil/Properties.h>

/*!
 * @class DataInOutPortTests class
 * @brief DataInOutPort test
 */
namespace DataInOutPort
{  
  template <class DataType>
  struct HogeCovnert : public RTC::OnReadConvert<DataType>
  {
    DataType operator()(const DataType& value)
    {
      DataType d(value);
      d.data = value.data * value.data;
      return d;
    }
  };

  class DataInOutPortTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(DataInOutPortTests);
    CPPUNIT_TEST(test_connect);
    CPPUNIT_TEST_SUITE_END();
  
  private:
    RTC::OutPort<RTC::TimedFloat> m_outport;
    RTC::TimedFloat m_ofloat;
    RTC::PortService_var m_oportref;

    RTC::InPort<RTC::TimedFloat> m_inport;
    RTC::TimedFloat m_ifloat;
    RTC::PortService_var m_iportref;

    HogeCovnert<RTC::TimedFloat>* m_conv;
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
  public:
  
    /*!
     * @brief Constructor
     */
    DataInOutPortTests()
      : m_outport("fout", m_ofloat),
	m_inport("fin", m_ifloat)
    {
      m_conv = new HogeCovnert<RTC::TimedFloat>();

      int argc(0);
      char** argv(NULL);
      m_pORB = CORBA::ORB_init(argc, argv);
      m_pPOA = PortableServer::POA::_narrow(
		    m_pORB->resolve_initial_references("RootPOA"));
      m_pPOA->the_POAManager()->activate();

      coil::Properties dummy;
      m_inport.init(dummy);
      m_outport.init(dummy);
      m_oportref = m_outport.get_port_profile()->port_ref;
      m_iportref = m_inport.get_port_profile()->port_ref;

    }
    
    /*!
     * @brief Destructor
     */
    ~DataInOutPortTests()
    {
      delete m_conv;
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
  
    /* test case */
    void test_connect()
    {
      RTC::ConnectorProfile prof;
      prof.connector_id = "";
      prof.name = CORBA::string_dup("connector0");
      prof.ports.length(2);
      prof.ports[0] = m_oportref;
      prof.ports[1] = m_iportref;

      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.interface_type",
					     "corba_cdr"));

      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.dataflow_type",
					     "push"));
      CORBA_SeqUtil::push_back(prof.properties,
			       NVUtil::newNV("dataport.subscription_type",
					     "flush"));
      RTC::ReturnCode_t ret= m_inport.connect(prof);
      CPPUNIT_ASSERT(ret == RTC::RTC_OK);
      
      RTC::ConnectorProfileList* iprof;
      iprof = m_inport.get_connector_profiles();

      RTC::ConnectorProfileList* oprof;
      oprof = m_outport.get_connector_profiles();

#ifdef DEBUG
      std::cout << "Returned  connector ID"
		<< prof.connector_id << std::endl;
      std::cout << "InPort's  connector ID" 
		<< (*iprof)[0].connector_id << std::endl;
      std::cout << "OutPort's connector ID"
		<< (*oprof)[0].connector_id << std::endl;
#endif
      std::string c_id, i_id, o_id;
      c_id = prof.connector_id;
      i_id = (*iprof)[0].connector_id;
      o_id = (*oprof)[0].connector_id;

      CPPUNIT_ASSERT(c_id == o_id);
      CPPUNIT_ASSERT(c_id == i_id);
      CPPUNIT_ASSERT(o_id == i_id);

      for (int i = 0; i < 100; ++i)
	{
	  m_ofloat.data = 1.234567 * i;
	  m_outport.write();

	  m_inport.read();
#ifdef DEBUG
	  sleep(1);
	  std::cout <<  m_ofloat.data << " <=> " << m_ifloat.data << std::endl;
#endif
	  CPPUNIT_ASSERT(m_ofloat.data == m_ifloat.data);
	}

	m_pPOA->deactivate_object(*m_pPOA->servant_to_id(&m_inport));
	m_pPOA->deactivate_object(*m_pPOA->servant_to_id(&m_outport));

    }
  };
}; // namespace DataInOutPort

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(DataInOutPort::DataInOutPortTests);

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
#endif // DataInOutPort_cpp
