// -*- C++ -*-
/*!
 * @file   RTCUtilTests.cpp
 * @brief  RTCUtil test class
 * @date   $Date: 2008/05/02 11:29:13 $
 *
 * $Id$
 *
 */

/*
 * $Log: RTCUtilTests.cpp,v $
 * Revision 1.1  2008/05/02 11:29:13  arafune
 * The first commitment.
 *
 *
 */

#ifndef RTCUtil_cpp
#define RTCUtil_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <idl/SDOPackageSkel.h>
#include <idl/RTCSkel.h>
#include <idl/OpenRTMSkel.h>
#include <rtm/RTCUtil.h>

/*!
 * @class RTCUtilTests class
 * @brief RTCUtil test
 */
namespace Tests
{
  class DataFlowComponentMock
    : public virtual POA_OpenRTM::DataFlowComponent,
      public virtual PortableServer::RefCountServantBase
  {
  public:
    DataFlowComponentMock() {}
    virtual ~DataFlowComponentMock() {}
    // _impl_SDOSystemElement
    virtual SDOPackage::OrganizationList* get_owned_organizations() { return NULL; }
    
    // SDOPackage::_impl_SDO
    virtual char* get_sdo_id() { return NULL; }
    virtual char* get_sdo_type() { return NULL; }
    virtual SDOPackage::DeviceProfile* get_device_profile() { return NULL; }
    virtual SDOPackage::ServiceProfileList* get_service_profiles() { return NULL; }
    virtual SDOPackage::ServiceProfile* get_service_profile(const char*) { return NULL; }
    virtual SDOPackage::_objref_SDOService* get_sdo_service(const char*) { return NULL; }
    virtual SDOPackage::_objref_Configuration* get_configuration() { return NULL; }
    virtual SDOPackage::_objref_Monitoring* get_monitoring() { return NULL; }
    virtual SDOPackage::OrganizationList* get_organizations() { return NULL; }
    virtual SDOPackage::NVList* get_status_list() { return NULL; }
    virtual CORBA::Any* get_status(const char*) { return NULL; }
		
    // RTC::_impl_DataFlowComponentAction
    virtual RTC::ReturnCode_t on_execute(RTC::ExecutionContextHandle_t) { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t on_state_update(RTC::ExecutionContextHandle_t) { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t on_rate_changed(RTC::ExecutionContextHandle_t) { return RTC::RTC_OK; }
    virtual RTC::ExecutionContextHandle_t attach_context(RTC::_objref_ExecutionContext*) { return RTC::ExecutionContextHandle_t(0); }
    virtual RTC::ReturnCode_t detach_context(RTC::ExecutionContextHandle_t) { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t on_initialize() { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t on_finalize() { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t on_startup(RTC::ExecutionContextHandle_t) { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t on_shutdown(RTC::ExecutionContextHandle_t) { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t on_activated(RTC::ExecutionContextHandle_t) { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t on_deactivated(RTC::ExecutionContextHandle_t) { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t on_aborting(RTC::ExecutionContextHandle_t) { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t on_error(RTC::ExecutionContextHandle_t) { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t on_reset(RTC::ExecutionContextHandle_t) { return RTC::RTC_OK; }
    virtual RTC::ExecutionContextList* get_owned_contexts()
    {
        return 0;
    }
    virtual RTC::ExecutionContextList* get_participating_contexts()
    {
        return 0;
    }
    virtual RTC::ExecutionContextHandle_t get_context_handle(RTC::_objref_ExecutionContext*)	
    {
        return 0;
    }
    virtual RTC::ReturnCode_t send_stimulus(const char*, RTC::ExecutionContextHandle_t)
    {
        return RTC::RTC_OK;
    }
    // RTC::_impl_LightweightRTObjec:t
    virtual RTC::ReturnCode_t initialize() { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t finalize() { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t exit() { return RTC::RTC_OK; }
    virtual CORBA::Boolean is_alive(RTC::_objref_ExecutionContext*) { return true; }
    virtual RTC::_objref_ExecutionContext* get_context(RTC::ExecutionContextHandle_t) { return NULL; }
		
    // RTC::_impl_RTObject
    virtual RTC::ComponentProfile* get_component_profile() { return NULL; }
    virtual RTC::PortServiceList* get_ports() { return NULL; }
  };

  class FsmObjectMock
    : public virtual POA_RTC::FsmObject
  {
    // RTC::_impl_FsmObject
    virtual RTC::ReturnCode_t stimulate(const char*, RTC::ExecutionContextHandle_t) { return RTC::RTC_OK; }
    virtual RTC::ReturnCode_t send_stimulus(const char*, RTC::ExecutionContextHandle_t)
    {
        return RTC::RTC_OK;
    }
  };
	
  class MultiModeObjectMock
    : public virtual POA_RTC::MultiModeObject
  {
    // RTC::_impl_ModeCapable
    virtual RTC::_objref_Mode* get_default_mode() { return NULL; }
    virtual RTC::_objref_Mode* get_current_mode() { return NULL; }
    virtual RTC::_objref_Mode* get_current_mode_in_context(RTC::_objref_ExecutionContext*) { return NULL; } 
    virtual RTC::_objref_Mode* get_pending_mode() { return NULL; }
    virtual RTC::_objref_Mode* get_pending_mode_in_context(RTC::_objref_ExecutionContext*) { return NULL; }
    virtual RTC::ReturnCode_t set_mode(RTC::_objref_Mode*, CORBA::Boolean) { return RTC::RTC_OK; }

    // RTC::_impl_MultiModeComponentAction
    virtual RTC::ReturnCode_t on_mode_changed(RTC::ExecutionContextHandle_t)
    {
      return RTC::RTC_OK;
    }

    virtual RTC::ReturnCode_t send_stimulus(const char*, RTC::ExecutionContextHandle_t)
    {
        return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_initialize()
    {
        return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_startup(RTC::ExecutionContextHandle_t)
    {
        return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_shutdown(RTC::ExecutionContextHandle_t)
    {
        return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_activated(RTC::ExecutionContextHandle_t)
    {
        return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_deactivated(RTC::ExecutionContextHandle_t)
    {
        return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_aborting(RTC::ExecutionContextHandle_t)
    {
        return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_error(RTC::ExecutionContextHandle_t)
    {
        return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_reset(RTC::ExecutionContextHandle_t)
    {
        return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t on_finalize()
    {
        return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t initialize()
    {
        return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t finalize()
    {
        return RTC::RTC_OK;
    }
    virtual CORBA::Boolean is_alive(RTC::_objref_ExecutionContext*)
    {
        return 0;
    }
    virtual RTC::ReturnCode_t exit()
    {
        return RTC::RTC_OK;
    }
    virtual RTC::ExecutionContextHandle_t attach_context(RTC::_objref_ExecutionContext*) 
    { 
      return RTC::ExecutionContextHandle_t(0); 
    }
    virtual RTC::ReturnCode_t detach_context(RTC::ExecutionContextHandle_t) 
    { 
      return RTC::RTC_OK; 
    }
    virtual RTC::_objref_ExecutionContext* get_context(RTC::ExecutionContextHandle_t)
    {
      return 0;
    }
    virtual RTC::ExecutionContextList* get_owned_contexts()
    {
      return NULL;
    }
    virtual RTC::ExecutionContextList* get_participating_contexts()
    {
      return NULL;
    }
    virtual RTC::ExecutionContextHandle_t get_context_handle(RTC::_objref_ExecutionContext*)
    {
      return 0;
    }

  };
	
  class RTCUtilTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(RTCUtilTests);
		
    CPPUNIT_TEST(test_isDataFlowComponent_DataFlowComponent);
    CPPUNIT_TEST(test_isDataFlowComponent_FsmObject);
    CPPUNIT_TEST(test_isDataFlowComponent_MultiModeObject);

    CPPUNIT_TEST(test_isFsmParticipant_DataFlowComponent);
    CPPUNIT_TEST(test_isFsmParticipant_FsmObject);
    CPPUNIT_TEST(test_isFsmParticipant_MultiModeObject);

    CPPUNIT_TEST(test_isFsmObject_DataFlowComponent);
    CPPUNIT_TEST(test_isFsmObject_FsmObject);
    CPPUNIT_TEST(test_isFsmObject_MultiModeObject);
		
    CPPUNIT_TEST(test_isMultiModeObject_DataFlowComponent);
    CPPUNIT_TEST(test_isMultiModeObject_FsmObject);
    CPPUNIT_TEST(test_isMultiModeObject_MultiModeObject);
		
    CPPUNIT_TEST_SUITE_END();
	
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
	
  public:
    /*!
     * @brief Constructor
     */
    RTCUtilTests()
    {
      int argc = 0;
      char** argv = NULL;
      m_pORB = CORBA::ORB_init(argc, argv);
      m_pPOA = PortableServer::POA::_narrow(
					    m_pORB->resolve_initial_references("RootPOA"));
      m_pPOA->the_POAManager()->activate();
    }
		    
    /*!
     * @brief Destructor
     */
    virtual ~RTCUtilTests()
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
		
    void test_isDataFlowComponent_DataFlowComponent()
    {
      DataFlowComponentMock* obj = new DataFlowComponentMock();
      CORBA::Object_ptr ref = obj->_this();
      CPPUNIT_ASSERT(! CORBA::is_nil(ref));
			
      CPPUNIT_ASSERT(RTC_Utils::isDataFlowComponent(ref));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(obj));
      delete obj;
    }
		
    void test_isDataFlowComponent_FsmObject()
    {
      FsmObjectMock* obj = new FsmObjectMock();
      CORBA::Object_ptr ref = obj->_this();
      CPPUNIT_ASSERT(! CORBA::is_nil(ref));
			
      CPPUNIT_ASSERT(! RTC_Utils::isDataFlowComponent(ref));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(obj));
      delete obj;
    }
		
    void test_isDataFlowComponent_MultiModeObject()
    {
      MultiModeObjectMock* obj = new MultiModeObjectMock();
			
      CORBA::Object_ptr ref = obj->_this();
      CPPUNIT_ASSERT(! CORBA::is_nil(ref));
			
      CPPUNIT_ASSERT(! RTC_Utils::isDataFlowComponent(ref));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(obj));
      delete obj;
    }
		
    void test_isFsmParticipant_DataFlowComponent()
    {
      DataFlowComponentMock* obj = new DataFlowComponentMock();
      CORBA::Object_ptr ref = obj->_this();
      CPPUNIT_ASSERT(! CORBA::is_nil(ref));
			
      CPPUNIT_ASSERT(! RTC_Utils::isFsmParticipant(ref));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(obj));
      delete obj;
    }
		
    void test_isFsmParticipant_FsmObject()
    {
      FsmObjectMock* obj = new FsmObjectMock();
      CORBA::Object_ptr ref = obj->_this();
      CPPUNIT_ASSERT(! CORBA::is_nil(ref));
			
      CPPUNIT_ASSERT(! RTC_Utils::isFsmParticipant(ref));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(obj));
      delete obj;
    }
		
    void test_isFsmParticipant_MultiModeObject()
    {
      MultiModeObjectMock* obj = new MultiModeObjectMock();
			
      CORBA::Object_ptr ref = obj->_this();
      CPPUNIT_ASSERT(! CORBA::is_nil(ref));
			
      CPPUNIT_ASSERT(! RTC_Utils::isFsmParticipant(ref));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(obj));
      delete obj;
    }
		
    void test_isFsmObject_DataFlowComponent()
    {
      DataFlowComponentMock* obj = new DataFlowComponentMock();
      CORBA::Object_ptr ref = obj->_this();
      CPPUNIT_ASSERT(! CORBA::is_nil(ref));
			
      CPPUNIT_ASSERT(! RTC_Utils::isFsmObject(ref));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(obj));
      delete obj;
    }
		
    void test_isFsmObject_FsmObject()
    {
      FsmObjectMock* obj = new FsmObjectMock();
      CORBA::Object_ptr ref = obj->_this();
      CPPUNIT_ASSERT(! CORBA::is_nil(ref));
			
      CPPUNIT_ASSERT(RTC_Utils::isFsmObject(ref));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(obj));
      delete obj;
    }

    void test_isFsmObject_MultiModeObject()
    {
      MultiModeObjectMock* obj = new MultiModeObjectMock();
			
      CORBA::Object_ptr ref = obj->_this();
      CPPUNIT_ASSERT(! CORBA::is_nil(ref));
			
      CPPUNIT_ASSERT(! RTC_Utils::isFsmObject(ref));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(obj));
      delete obj;
    }
		
    void test_isMultiModeObject_DataFlowComponent()
    {
      DataFlowComponentMock* obj = new DataFlowComponentMock();
      CORBA::Object_ptr ref = obj->_this();
      CPPUNIT_ASSERT(! CORBA::is_nil(ref));
			
      CPPUNIT_ASSERT(! RTC_Utils::isMultiModeObject(ref));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(obj));
      delete obj;
    }

    void test_isMultiModeObject_FsmObject()
    {
      FsmObjectMock* obj = new FsmObjectMock();
      CORBA::Object_ptr ref = obj->_this();
      CPPUNIT_ASSERT(! CORBA::is_nil(ref));
			
      CPPUNIT_ASSERT(! RTC_Utils::isMultiModeObject(ref));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(obj));
      delete obj;
    }

    void test_isMultiModeObject_MultiModeObject()
    {
      MultiModeObjectMock* obj = new MultiModeObjectMock();
			
      CORBA::Object_ptr ref = obj->_this();
      CPPUNIT_ASSERT(! CORBA::is_nil(ref));
			
      CPPUNIT_ASSERT(RTC_Utils::isMultiModeObject(ref));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(obj));
      delete obj;
    }
		
  };
}; // namespace RTCUtil

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Tests::RTCUtilTests);

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
#endif // RTCUtil_cpp
