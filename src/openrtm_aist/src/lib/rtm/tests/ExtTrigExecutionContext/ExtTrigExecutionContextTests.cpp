// -*- C++ -*-
/*!
 * @file   ExtTrigExecutionContextTests.cpp
 * @brief  ExtTrigExecutionContext test class
 * @date   $Date: 2008/04/15 05:03:53 $
 *
 * $Id: ExtTrigExecutionContextTests.cpp,v 1.1 2008/04/15 05:03:53 arafune Exp $
 *
 */

/*
 * $Log: ExtTrigExecutionContextTests.cpp,v $
 * Revision 1.1  2008/04/15 05:03:53  arafune
 * The first commitment.
 *
 *
 */

#ifndef ExtTrigExecutionContext_cpp
#define ExtTrigExecutionContext_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/idl/RTCSkel.h>
#include <rtm/RTObject.h>
#include <rtm/ExtTrigExecutionContext.h>
#include <rtm/CORBA_SeqUtil.h>

/*!
 * @class ExtTrigExecutionContextTests class
 * @brief ExtTrigExecutionContext test
 */
namespace ExtTrigExecutionContext
{
  class LightweightRTObjectMock
    : public virtual POA_RTC::LightweightRTObject,
      public virtual PortableServer::RefCountServantBase
  {
  protected:
    typedef std::map<RTC::UniqueId, RTC::ExecutionContext_ptr> ExecContexts;
    CORBA::Long m_nextUniqueId;
    ExecContexts m_execContexts;
    std::vector<std::string> m_log;
    bool m_alive;
    bool m_error;
	
  public:
    LightweightRTObjectMock()
      : m_alive(true), m_error(false)
    {
    }
		
    // RTC::_impl_ComponentAction
    virtual RTC::UniqueId attach_context(RTC::ExecutionContext_ptr exec_context)
    {
      m_log.push_back("attach_executioncontext");
      m_execContexts.insert(
			    std::pair<RTC::UniqueId, RTC::ExecutionContext_ptr>(m_nextUniqueId++, exec_context));
      return m_nextUniqueId;
    }
    virtual RTC::ReturnCode_t detach_context(RTC::UniqueId ec_id)
    {
      m_log.push_back("detach_executioncontext");
      m_execContexts.erase(ec_id);
      return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_initialize()
    {
      m_log.push_back("on_initialize");
      return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_finalize()
    {
      m_log.push_back("on_finalize");
      return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_startup(RTC::UniqueId ec_id)
    {
      m_log.push_back("on_startup");
      return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_shutdown(RTC::UniqueId ec_id)
    {
      m_log.push_back("on_shutdown");
      return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_activated(RTC::UniqueId ec_id)
    {
      m_log.push_back("on_activated");
      return returnCode(RTC::RTC_OK);
    }
    virtual RTC::ReturnCode_t on_deactivated(RTC::UniqueId ec_id)
    {
      m_log.push_back("on_deactivated");
      return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_aborting(RTC::UniqueId ec_id)
    {
      m_log.push_back("on_aborting");
      return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_error(RTC::UniqueId ec_id)
    {
      m_log.push_back("on_error");
      return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_reset(RTC::UniqueId ec_id)
    {
      m_log.push_back("on_reset");
      return RTC::RTC_OK;
    }
    
    // RTC::_impl_LightweightRTObject
    virtual RTC::ReturnCode_t initialize()
    {
      m_log.push_back("initialize");
      return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t finalize()
    {
      m_log.push_back("finalize");
      return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t exit()
    {
      m_log.push_back("exit");
      return RTC::RTC_OK;
    }
    virtual CORBA::Boolean is_alive(RTC::ExecutionContext_ptr exec_context)
    {
      m_log.push_back("is_alive");
      return CORBA::Boolean(m_alive);
    }
    virtual RTC::ExecutionContextList* get_owned_contexts()
    {
      m_log.push_back("get_contexts");
      return 0;
    }
    virtual RTC::ExecutionContextList* get_participating_contexts()
    {
      m_log.push_back("get_context");
      return 0;
    }
    virtual RTC::_objref_ExecutionContext* get_context(RTC::ExecutionContextHandle_t)
    {
      return 0;
    }
    virtual RTC::ExecutionContextHandle_t get_context_handle(RTC::_objref_ExecutionContext*)
    {
      return 0;
    }
    
  public: // helper methods
    int countLog(std::string line)
    {
      int count = 0;
      for (int i = 0; i < (int) m_log.size(); ++i)
	{
	  if (m_log[i] == line) ++count;
	}
      return count;
    }
		
    void setAlive(bool alive)
    {
      m_alive = alive;
    }
		
    void setError(bool error)
    {
      m_error = error;
    }
	
  private:
    RTC::ReturnCode_t returnCode(RTC::ReturnCode_t rc)
    {
      return m_error ? RTC::RTC_ERROR : rc;
    }
  };
	
  class DataFlowComponentMock
//    : public virtual POA_RTC::DataFlowComponent,
    : public virtual POA_OpenRTM::DataFlowComponent,
      public virtual LightweightRTObjectMock
  {
  public:
    // SDOPackage::_impl_SDOSystemElement
    virtual SDOPackage::OrganizationList* get_owned_organizations()
      throw (SDOPackage::NotAvailable)
    {
      m_log.push_back("get_owned_organizations");
      return 0; // dummy
    }
    virtual char* get_sdo_id()
      throw (SDOPackage::NotAvailable, SDOPackage::InternalError)
    {
      m_log.push_back("get_sdo_id");
      return 0; // dummy
    }
    virtual char* get_sdo_type()
      throw (SDOPackage::NotAvailable, SDOPackage::InternalError)
    {
      m_log.push_back("get_sdo_type");
      return 0; // dummy
    }
    virtual SDOPackage::DeviceProfile* get_device_profile()
      throw (SDOPackage::NotAvailable, SDOPackage::InternalError)
    {
      m_log.push_back("get_device_profile");
      return 0; // dummy
    }
    virtual SDOPackage::ServiceProfileList* get_service_profiles()
      throw (SDOPackage::InvalidParameter, SDOPackage::NotAvailable, SDOPackage::InternalError)
    {
      m_log.push_back("get_service_profiles");
      return 0; // dummy
    }
    virtual SDOPackage::ServiceProfile* get_service_profile(const char* id)
      throw (SDOPackage::InvalidParameter, SDOPackage::NotAvailable, SDOPackage::InternalError)
    {
      m_log.push_back("get_service_profile");
      return 0; // dummy
    }
    virtual SDOPackage::SDOService_ptr get_sdo_service(const char* id)
      throw (SDOPackage::InvalidParameter, SDOPackage::NotAvailable, SDOPackage::InternalError)
    {
      m_log.push_back("get_sdo_service");
      return SDOPackage::SDOService::_nil(); // dummy
    }
    virtual SDOPackage::Configuration_ptr get_configuration()
      throw (SDOPackage::InterfaceNotImplemented, SDOPackage::NotAvailable, SDOPackage::InternalError)
    {
      m_log.push_back("get_configuration");
      return SDOPackage::Configuration::_nil(); // dummy
    }
    virtual SDOPackage::Monitoring_ptr get_monitoring()
      throw (SDOPackage::InterfaceNotImplemented, SDOPackage::NotAvailable, SDOPackage::InternalError)
    {
      m_log.push_back("get_monitoring");
      return SDOPackage::Monitoring::_nil(); // dummy
    }
    virtual SDOPackage::OrganizationList* get_organizations()
      throw (SDOPackage::NotAvailable, SDOPackage::InternalError)
    {
      m_log.push_back("get_organizations");
      return 0; // dummy
    }
    virtual SDOPackage::NVList* get_status_list()
      throw (SDOPackage::NotAvailable, SDOPackage::InternalError)
    {
      m_log.push_back("get_status_list");
      return 0; // dummy
    }
    virtual CORBA::Any* get_status(const char* name)
      throw (SDOPackage::InvalidParameter, SDOPackage::NotAvailable, SDOPackage::InternalError)
    {
      m_log.push_back("get_status");
      return 0; // dummy
    }

    // RTC::_impl_RTObject
    virtual RTC::ComponentProfile* get_component_profile()
    {
      m_log.push_back("get_component_profile");
      // dummy
      RTC::ComponentProfile_var prof(new RTC::ComponentProfile());
      return prof._retn();
    }
    virtual RTC::PortServiceList* get_ports()
    {
      m_log.push_back("get_ports");
      // dummy
      RTC::PortServiceList_var ports(new RTC::PortServiceList());
      ports->length(0);
      return ports._retn();
    }
/*
    virtual RTC::ExecutionContextServiceList* get_execution_context_services()
    {
      m_log.push_back("get_execution_context_services");
      // dummy
      RTC::ExecutionContextServiceList_var ec = new RTC::ExecutionContextServiceList();
      ec->length(0);
      return ec._retn();
    }
*/
    // RTC::_impl_DataFlowComponentAction
    virtual RTC::ReturnCode_t on_execute(RTC::UniqueId ec_id)
    {
      m_log.push_back("on_execute");
      return RTC::RTC_OK; // dummy
    }
    virtual RTC::ReturnCode_t on_state_update(RTC::UniqueId ec_id)
    {
      m_log.push_back("on_state_update");
      return RTC::RTC_OK; // dummy
    }
    virtual RTC::ReturnCode_t on_rate_changed(RTC::UniqueId ec_id)
    {
      m_log.push_back("on_rate_changed");
      return RTC::RTC_OK; // dummy
    }
  };
	
  class ExtTrigExecutionContextTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ExtTrigExecutionContextTests);
    CPPUNIT_TEST(test_tick);
    CPPUNIT_TEST_SUITE_END();
	
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
	
  public:
    /*!
     * @brief Constructor
     */
    ExtTrigExecutionContextTests()
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
    virtual ~ExtTrigExecutionContextTests()
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
		
    void test_tick()
    {
      // RTObjectを生成する
      POA_RTC::LightweightRTObject* rto
	= new DataFlowComponentMock(); // will be deleted automatically
      DataFlowComponentMock* mock
	= dynamic_cast<DataFlowComponentMock*>(rto);

      // ExecutionContextを生成する
      RTC::ExtTrigExecutionContext* ec
	= new RTC::ExtTrigExecutionContext(); // will be deleted automatically

      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ec->start());
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ec->add_component(rto->_this()));
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ec->activate_component(rto->_this()));
			
      // tick()呼出を行い、その回数とon_execute()の呼出回数が一致していることを確認する
      ec->tick();
      for (int tickCalledCount = 0; tickCalledCount < 10; tickCalledCount++)
	{
          usleep(1000);
	  CPPUNIT_ASSERT_EQUAL(tickCalledCount, mock->countLog("on_execute"));
          usleep(1000);
	  ec->tick();
	}

      ec->stop();
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(ec));
      delete ec;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(rto));
      delete rto;
    }
		
  };
}; // namespace ExtTrigExecutionContext

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ExtTrigExecutionContext::ExtTrigExecutionContextTests);

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
#endif // ExtTrigExecutionContext_cpp
