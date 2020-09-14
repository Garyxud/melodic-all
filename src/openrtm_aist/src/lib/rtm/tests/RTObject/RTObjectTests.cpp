// -*- C++ -*-
/*!
 * @file   RTObjectTests.cpp
 * @brief  RTObject test class
 * @date   $Date: 2008/04/24 08:49:57 $
 *
 * $Id: RTObjectTests.cpp,v 1.1 2008/04/24 08:49:57 arafune Exp $
 *
 */

/*
 * $Log: RTObjectTests.cpp,v $
 * Revision 1.1  2008/04/24 08:49:57  arafune
 * The first commitment.
 *
 */

#ifndef RTObject_cpp
#define RTObject_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/RTC.h>
#include <rtm/PeriodicExecutionContext.h>
#include <rtm/RTObject.h>

/*!
 * @class RTObjectTests class
 * @brief RTObject test
 */
namespace RTObject
{
  class RTObjectMock
    : public RTC::RTObject_impl
  {
  public:
    RTObjectMock(CORBA::ORB_ptr orb, PortableServer::POA_ptr poa)
      : RTC::RTObject_impl(orb, poa)
    {
    }

    virtual ~RTObjectMock()
    {
    }
		
    virtual RTC::ReturnCode_t on_initialize()
      throw (CORBA::SystemException)
    {
      log("on_initialize");
      return RTC::RTObject_impl::on_initialize();
    }
		
    virtual RTC::ReturnCode_t on_finalize()
      throw (CORBA::SystemException)
    {
      log("on_finalize");
      return RTC::RTObject_impl::on_finalize();
    }

    virtual RTC::UniqueId bindContext(RTC::ExecutionContext_ptr exec_context)
    {
      RTC::UniqueId id;
      id = RTC::RTObject_impl::bindContext(exec_context);
      ecMine = RTC::RTObject_impl::m_ecMine;
      return id;
    }
    RTC::ExecutionContextServiceList ecMine;

    // protected: m_ecMineの設定
    void set_ecMine()
    {
      RTC::RTObject_impl::m_ecMine = ecMine;
    }

    // protected: m_ecMineの判定
    bool chk_ecMine(int id, RTC::ExecutionContext_ptr exec_context)
    {
      RTC::ExecutionContextService_var ecs;
      ecs = RTC::ExecutionContextService::_narrow(exec_context);
      if (RTC::RTObject_impl::m_ecMine[id] == ecs)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    std::vector<RTC::ExecutionContextBase*> eclist;
    // protected: m_eclistの取得
    int get_eclist()
    {
      int len(RTC::RTObject_impl::m_eclist.size());
      eclist = RTC::RTObject_impl::m_eclist;
      return len;
    }

		
  public: // helper for test
    int countLog(std::string line)
    {
      int count = 0;
      for (int i = 0; i < (int) m_log.size(); ++i)
	{
	  if (m_log[i] == line) ++count;
	}
      return count;
    }
		
    void set_status(const char* name, const CORBA::Any& value)
    {
      CORBA::Long idx = NVUtil::find_index(m_sdoStatus, name);
      if (idx < 0)
	{
	  SDOPackage::NameValue nv = NVUtil::newNVAny(name, value);
	  CORBA_SeqUtil::push_back(m_sdoStatus, nv);
	}
      else
	{
	  m_sdoStatus[idx].value <<= value;
	}
    }
    
    void shutdown() 
    {
      RTObject_impl::shutdown();
    }
    
		
  private:
    void log(const std::string& msg)
    {
      m_log.push_back(msg);
    }
		
  private:
    std::vector<std::string> m_log;
  };
	
  class PortMock
    : public RTC::PortBase
  {
  protected:
    virtual RTC::ReturnCode_t publishInterfaces(RTC::ConnectorProfile& connector_profile)
    {
      return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t subscribeInterfaces(const RTC::ConnectorProfile& connector_profile)
    {
      return RTC::RTC_OK;
    }
    virtual void unsubscribeInterfaces(const RTC::ConnectorProfile& connector_profile)
    {
    }
    virtual void activateInterfaces()
    {
    }
    virtual void deactivateInterfaces()
    {
    }
  };

  class InPortMock
    : public RTC::InPortBase
  {
  public:
    InPortMock(const char* name, const char* data_type)
      :RTC::InPortBase(name, data_type),
       m_return(true),
       m_cnt(0)
    {
      return;
    }
    
    bool read(){
      ++m_cnt;
      return m_return;
    }

    void set_return(bool ret)
    {
      m_return = ret;
    }

    int get_counter()
    {
      return m_cnt;
    }

  private:
    bool m_return;
    int m_cnt;
  };
	
  class OutPortMock
    : public RTC::OutPortBase
  {
  public:
    OutPortMock(const char* name, const char* data_type)
      : OutPortBase(name, data_type),
       m_return(true),
       m_cnt(0)
    {
      return;
    }

    bool write() {
      ++m_cnt;
      return m_return;
    }
    void set_return(bool ret)
    {
      m_return = ret;
    }

    int get_counter()
    {
      return m_cnt;
    }
  private:
    bool m_return;
    int m_cnt;
  };
	
  class SDOServiceMock
    : public POA_SDOPackage::SDOService,
      public virtual PortableServer::RefCountServantBase
  {
  };
	
  struct PortFinder
  {
    PortFinder(const RTC::PortService_ptr& port) : m_port(port) {}
    bool operator()(const RTC::PortService_ptr& port)
    {
      return m_port->_is_equivalent(port);
    }
		
    const RTC::PortService_ptr& m_port;
  };
	
  class SDOSystemElementMock
    : public POA_SDOPackage::SDOSystemElement,
      public virtual PortableServer::RefCountServantBase
  {
  };
	
  class OrganizationMock
    : public POA_SDOPackage::Organization,
      public virtual PortableServer::RefCountServantBase
  {
  public:
    OrganizationMock(const char* id) : m_id(id)
    {
    }
		
    virtual char* get_organization_id()
    {
      return CORBA::string_dup(m_id);
    }
		
    virtual SDOPackage::OrganizationProperty* get_organization_property()
    {
      return NULL;
    }
		
    virtual CORBA::Any* get_organization_property_value(const char* name)
    {
      return NULL;
    }
		
    virtual CORBA::Boolean add_organization_property(const SDOPackage::OrganizationProperty& organization_property)
    {
      return false;
    }
		
    virtual CORBA::Boolean set_organization_property_value(const char* name, const CORBA::Any& value)
    {
      return false;
    }
		
    virtual CORBA::Boolean remove_organization_property(const char* name)
    {
      return false;
    }
		
    virtual SDOPackage::SDOSystemElement_ptr get_owner()
    {
      return NULL;
    }
		
    virtual CORBA::Boolean set_owner(SDOPackage::SDOSystemElement_ptr sdo)
    {
      return false;
    }
		
    virtual SDOPackage::SDOList* get_members()
    {
      return NULL;
    }
		
    virtual CORBA::Boolean set_members(const SDOPackage::SDOList& sdos)
    {
      return false;
    }
		
    virtual CORBA::Boolean add_members(const SDOPackage::SDOList& sdo_list)
    {
      return false;
    }
		
    virtual CORBA::Boolean remove_member(const char* id)
    {
      return false;
    }
		
    virtual SDOPackage::DependencyType get_dependency()
    {
      return SDOPackage::NO_DEPENDENCY;
    }
		
    virtual CORBA::Boolean set_dependency(SDOPackage::DependencyType dependency)
    {
      return false;
    }
		
  private:
    const char* m_id;
  };
	
  struct ExecutionContextServiceFinder
  {
    ExecutionContextServiceFinder(const RTC::ExecutionContextService_ptr& ecSvc)
      : m_ecSvc(ecSvc) {}
    bool operator()(const RTC::ExecutionContextService_ptr& ecSvc)
    {
      return m_ecSvc->_is_equivalent(ecSvc);
    }
    const RTC::ExecutionContextService_ptr& m_ecSvc;
  };
	
  struct ServiceProfileFinder
  {
    ServiceProfileFinder(const char* id) : m_id(id) {}
    bool operator()(const SDOPackage::ServiceProfile& svcProf)
    {
      return strcmp(m_id, (const char*)(svcProf.id)) == 0;
    }
    const char* m_id;
  };
	
  struct OrganizationFinder
  {
    OrganizationFinder(const char* id) : m_id(id) {}
    bool operator()(const SDOPackage::Organization_ptr& org)
    {
      return strcmp(m_id, org->get_organization_id()) == 0;
    }
    const char* m_id;
  };
	
  class RTObjectTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(RTObjectTests);

    CPPUNIT_TEST(test_finalizeContexts);
    CPPUNIT_TEST(test_bindContext);
    CPPUNIT_TEST(test_add_removePort);
    CPPUNIT_TEST(test_readAll);
    CPPUNIT_TEST(test_writeAll);
    CPPUNIT_TEST(test_initialize_invoking_on_initialize);
    CPPUNIT_TEST(test_initialize_in_Alive);
    CPPUNIT_TEST(test_finalize_invoking_on_finalize);
    CPPUNIT_TEST(test_finalize_participating_in_execution_context);
    CPPUNIT_TEST(test_finalize_in_Created);
    //		CPPUNIT_TEST(test_is_alive);
    CPPUNIT_TEST(test_exit);
    CPPUNIT_TEST(test_exit_in_Created);
    CPPUNIT_TEST(test_detach_executioncontext);
    CPPUNIT_TEST(test_detach_executioncontext_with_illegal_id);
    CPPUNIT_TEST(test_get_context);
    CPPUNIT_TEST(test_get_contexts);
    CPPUNIT_TEST(test_get_component_profile);
    CPPUNIT_TEST(test_get_ports);
//    CPPUNIT_TEST(test_get_execution_context_services);
    //		CPPUNIT_TEST(test_get_owned_organizations);
    CPPUNIT_TEST(test_get_sdo_id);
    CPPUNIT_TEST(test_get_sdo_type);
    //		CPPUNIT_TEST(test_get_device_profile);
    //		CPPUNIT_TEST(test_get_service_profile);
    CPPUNIT_TEST(test_get_service_profile_with_illegal_arguments);
    CPPUNIT_TEST(test_get_sdo_service);
    CPPUNIT_TEST(test_get_sdo_service_with_illegal_arguments);
    CPPUNIT_TEST(test_get_configuration_and_set_device_profile_and_get_device_profile);
    CPPUNIT_TEST(test_get_configuration_and_set_service_profile_and_get_service_profile);
    CPPUNIT_TEST(test_get_configuration_and_set_service_profile_and_get_service_profiles);
    CPPUNIT_TEST(test_get_configuration_and_set_service_profile_and_get_sdo_service);
    CPPUNIT_TEST(test_get_configuration_and_remove_service_profile);
    CPPUNIT_TEST(test_get_configuration_and_add_organization_and_get_organizations);
    CPPUNIT_TEST(test_get_configuration_and_remove_organization);
    //		CPPUNIT_TEST(test_get_monitoring);
    CPPUNIT_TEST(test_get_status);
    CPPUNIT_TEST(test_get_status_list);

    CPPUNIT_TEST_SUITE_END();
	
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
	
  public:
    /*!
     * @brief Constructor
     */
    RTObjectTests()
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
    virtual ~RTObjectTests()
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
     * @brief initialize()メソッドのテスト
     * 
     * - initialize()メソッド呼出により、on_initialize()コールバックが呼び出されるか？
     */
    void test_initialize_invoking_on_initialize()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // initialize()メソッド呼出により、on_initialize()コールバックが呼び出されるか？
      CPPUNIT_ASSERT_EQUAL(0, rto->countLog("on_initialize"));
      coil::Properties prop;
      prop.setProperty("exec_cxt.periodic.type","PeriodicExecutionContext");
      prop.setProperty("exec_cxt.periodic.rate","1000");
      rto->setProperties(prop);
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, rto->initialize());
      CPPUNIT_ASSERT_EQUAL(1, rto->countLog("on_initialize"));
      rto->exit();
      delete rto;
    }
		
    /*!
     * @brief initialize()メソッドのテスト
     * 
     * - Alive状態の時にinitialize()メソッドを呼出た場合、正常に動作するか？
     */
    void test_initialize_in_Alive()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // initialize()メソッド呼出しを行い、Alive状態に遷移させる
      coil::Properties prop;
      prop.setProperty("exec_cxt.periodic.type","PeriodicExecutionContext");
      prop.setProperty("exec_cxt.periodic.rate","1000");
      rto->setProperties(prop);
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, rto->initialize());

      RTC::ExecutionContext_ptr ec;
      ec = rto->get_context(0);
      CPPUNIT_ASSERT_EQUAL(true, rto->is_alive(ec));
			
      // Alive状態でinitialize()メソッド呼出しを行った場合、正常に動作するか？
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, rto->initialize());
      rto->exit();
      delete rto;
    }
		
    /*!
     * @brief finalize()メソッドのテスト
     * 
     * - finalize()呼出により、on_finalize()コールバックが呼び出されるか？
     */
    void test_finalize_invoking_on_finalize()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // initialize()メソッド呼出しを行い、Alive状態に遷移させる
      coil::Properties prop;
      prop.setProperty("exec_cxt.periodic.type","PeriodicExecutionContext");
      prop.setProperty("exec_cxt.periodic.rate","1000");
      rto->setProperties(prop);
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, rto->initialize());
      RTC::ExecutionContext_ptr ec;
      ec = rto->get_context(0);
      CPPUNIT_ASSERT_EQUAL(true, rto->is_alive(ec));
			
      // finalize()呼出しにより、on_finalize()コールバックが呼び出されるか？
      CPPUNIT_ASSERT_EQUAL(0, rto->countLog("on_finalize"));
			
      CPPUNIT_ASSERT_EQUAL(true, rto->is_alive(ec));
      // exit()呼び出しで、finalize()が有効となり実行される
      rto->exit();
      CPPUNIT_ASSERT_EQUAL(1, rto->countLog("on_finalize"));
      delete rto;
    }
		
    /*!
     * @brief finalize()メソッドのテスト
     * 
     * - ExecutionContextに登録された状態でfinalize()を呼び出した場合、意図どおりのエラーを返すか？
     */
    void test_finalize_participating_in_execution_context()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // initialize()メソッド呼出しを行い、Alive状態に遷移させる
      coil::Properties prop;
      prop.setProperty("exec_cxt.periodic.type","PeriodicExecutionContext");
      prop.setProperty("exec_cxt.periodic.rate","1000");
      rto->setProperties(prop);
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, rto->initialize());

      RTC::ExecutionContext_ptr ec;
      ec = rto->get_context(0);
      CPPUNIT_ASSERT_EQUAL(true, rto->is_alive(ec));
			
      // ExecutionContextに登録しておく
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ec->add_component(rto->_this()));
			
      // ExecutionContextに登録された状態でfinalize()を呼び出した場合、意図どおりのエラーを返すか？
      CPPUNIT_ASSERT_EQUAL(RTC::PRECONDITION_NOT_MET, rto->finalize());

      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ec->remove_component(rto->_this()));
      rto->exit();
      delete rto;
    }
		
    /*!
     * @brief finalize()メソッドのテスト
     * 
     * - Created状態でfinalize()を呼出した場合、意図どおりのエラーで返るか？
     */
    void test_finalize_in_Created()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // Created状態でfinalize()を呼出した場合、意図どおりのエラーで返るか？
      CPPUNIT_ASSERT_EQUAL(RTC::PRECONDITION_NOT_MET, rto->finalize());
      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief is_alive()メソッドのテスト
     */
    void test_is_alive()
    {
      // test_initialize_in_Alive()にてテストを兼ねている
    }
		
    /*!
     * @brief exit()メソッドのテスト
     * 
     * - exit()呼出しにより、当該コンポーネントがfinalize()されるか？
     * - exit()呼出しにより、当該コンポーネントが終状態に遷移するか？
     */
    void test_exit()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
      rto->setObjRef(rto->_this());
			
      // initialize()メソッド呼出しを行い、Alive状態に遷移させる
      coil::Properties prop;
      prop.setProperty("exec_cxt.periodic.type","PeriodicExecutionContext");
      prop.setProperty("exec_cxt.periodic.rate","1000");
      rto->setProperties(prop);
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, rto->initialize());

      RTC::ExecutionContext_ptr ec;
      ec = rto->get_context(0);
      CPPUNIT_ASSERT_EQUAL(true, rto->is_alive(ec));
			
      // コンポーネントをExecutionContextに登録してアクティブ化する
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ec->add_component(rto->_this()));
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ec->activate_component(rto->_this()));
      //Call start() for the state machine drive. 
      ec->start();
      coil::sleep(1);
			
      // exit()呼出しにより、当該コンポーネントがfinalize()されるか？
      // exit()呼出しにより、当該コンポーネントが終状態に遷移するか？
      CPPUNIT_ASSERT_EQUAL(0, rto->countLog("on_finalize"));
      CPPUNIT_ASSERT_EQUAL(RTC::ACTIVE_STATE, ec->get_component_state(rto->_this()));
      ec->stop();
      coil::sleep(1);
      //Call remove_component(),to cancel the registered component.
      ec->remove_component(rto->_this());
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, rto->exit());
      CPPUNIT_ASSERT_EQUAL(1, rto->countLog("on_finalize"));
      CPPUNIT_ASSERT_EQUAL(true, rto->is_alive(ec));

      delete rto;
    }

    /*!
     * @brief exit()メソッドのテスト
     * 
     * - Create状態でexit()を呼出した場合、意図どおりのエラーを返すか？
     */
    void test_exit_in_Created()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
      rto->setObjRef(rto->_this());
			
      // Create状態でexit()を呼出した場合、意図どおりのエラーを返すか？
      CPPUNIT_ASSERT_EQUAL(RTC::PRECONDITION_NOT_MET, rto->exit());

      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief detach_executioncontext()メソッドのテスト
     * 
     * - attach済みのRTCを正常にdetachできるか？
     */
    void test_detach_executioncontext()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // ExecutionContextを生成する
      RTC::PeriodicExecutionContext* ec
	= new RTC::PeriodicExecutionContext(); // will be deleted automatically
			
      // ExecutionContextにattachしておく
      RTC::UniqueId id = rto->attach_context(ec->_this());
      CPPUNIT_ASSERT(RTC::UniqueId(-1) != id);
			
      // 正常にdetachできるか？
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, rto->detach_context(id));
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(ec));
      delete ec;
      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief detach_executioncontext()メソッドのテスト
     * 
     * - 存在しないIDでRTCのdetachを試みた場合、意図どおりのエラーを返すか？
     */
    void test_detach_executioncontext_with_illegal_id()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // 存在しないIDでRTCのdetachを試みた場合、意図どおりのエラーを返すか？
      CPPUNIT_ASSERT_EQUAL(RTC::BAD_PARAMETER,
			   rto->detach_context(RTC::UniqueId(1)));
      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief get_context()メソッドのテスト
     * 
     * - 指定したIDのExecutionContextを正しく取得できるか？
     */
    void test_get_context()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // ExecutionContextを生成する
      RTC::PeriodicExecutionContext* ec1
	= new RTC::PeriodicExecutionContext(); // will be deleted automatically
      RTC::PeriodicExecutionContext* ec2
	= new RTC::PeriodicExecutionContext(); // will be deleted automatically
			
      // ExecutionContextにattachしておく
      RTC::UniqueId id1 = rto->attach_context(ec1->_this());
      CPPUNIT_ASSERT(RTC::UniqueId(-1) != id1);
      RTC::UniqueId id2 = rto->attach_context(ec2->_this());
      CPPUNIT_ASSERT(RTC::UniqueId(-1) != id2);
      CPPUNIT_ASSERT(id1 != id2);
			
      // 指定したIDのExecutionContextを正しく取得できるか？
      RTC::ExecutionContext_ptr ecPtr1 = rto->get_context(id1);
      CPPUNIT_ASSERT(ecPtr1->_is_equivalent(ec1->_this()));
      RTC::ExecutionContext_ptr ecPtr2 = rto->get_context(id2);
      CPPUNIT_ASSERT(ecPtr2->_is_equivalent(ec2->_this()));

      rto->detach_context(id2);
      rto->detach_context(id1);
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(ec2));
      delete ec2;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(ec1));
      delete ec1;

      rto->shutdown();
      delete rto;

    }
		
    /*!
     * @brief get_contexts()メソッドのテスト
     * 
     * - attachされているExecutionContextをすべて正しく取得できるか？
     */
    void test_get_contexts()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // ExecutionContextを生成する
      RTC::PeriodicExecutionContext* ec1
	= new RTC::PeriodicExecutionContext(); // will be deleted automatically
      RTC::PeriodicExecutionContext* ec2
	= new RTC::PeriodicExecutionContext(); // will be deleted automatically

      // ExecutionContextにattachしておく
      RTC::UniqueId id1 = rto->attach_context(ec1->_this());
      CPPUNIT_ASSERT(RTC::UniqueId(-1) != id1);
      RTC::UniqueId id2 = rto->attach_context(ec2->_this());
      CPPUNIT_ASSERT(RTC::UniqueId(-1) != id2);
			
      // attachされているExecutionContextをすべて正しく取得できるか？
      RTC::ExecutionContextList* ecList = rto->get_participating_contexts();
      CPPUNIT_ASSERT(ecList != NULL);
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2), ecList->length());
      CPPUNIT_ASSERT(! (*ecList)[0]->_is_equivalent((*ecList)[1]));
      CPPUNIT_ASSERT((*ecList)[0]->_is_equivalent(ec1->_this())
		     || (*ecList)[0]->_is_equivalent(ec2->_this()));
      CPPUNIT_ASSERT((*ecList)[1]->_is_equivalent(ec1->_this())
		     || (*ecList)[1]->_is_equivalent(ec2->_this()));


      rto->detach_context(id2);
      rto->detach_context(id1);

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(ec2));
      delete ec2;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(ec1));
      delete ec1;

      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief get_component_profile()メソッドのテスト
     * 
     * - ComponentProfileを正しく取得できるか？
     */
    void test_get_component_profile()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // ComponentProfileとして取得されるべき情報をあらかじめ設定しておく
      coil::Properties prop;
      prop.setProperty("instance_name", "INSTANCE_NAME");
      prop.setProperty("type_name", "TYPE_NAME");
      prop.setProperty("description", "DESCRIPTION");
      prop.setProperty("version", "VERSION");
      prop.setProperty("vendor", "VENDOR");
      prop.setProperty("category", "CATEGORY");
      rto->setProperties(prop);
			
      // ComponentProfileを正しく取得できるか？
      RTC::ComponentProfile* compProf = rto->get_component_profile();
      CPPUNIT_ASSERT(compProf != NULL);
			
      CPPUNIT_ASSERT_EQUAL(std::string("INSTANCE_NAME"),
			   std::string(compProf->instance_name));
      CPPUNIT_ASSERT_EQUAL(std::string("TYPE_NAME"),
			   std::string(compProf->type_name));
      CPPUNIT_ASSERT_EQUAL(std::string("DESCRIPTION"),
			   std::string(compProf->description));
      CPPUNIT_ASSERT_EQUAL(std::string("VERSION"),
			   std::string(compProf->version));
      CPPUNIT_ASSERT_EQUAL(std::string("VENDOR"),
			   std::string(compProf->vendor));
      CPPUNIT_ASSERT_EQUAL(std::string("CATEGORY"),
			   std::string(compProf->category));
      rto->shutdown();
      delete rto;
      
    }
		
    /*!
     * @brief add[In/Out]Port(),addPort()メソッドのテスト
     * 
     * - Portを正しく登録できるか？
     * - 既に登録済みのポートと同じポート名のPortを登録しようとした場合に失敗するか？
     * - 登録済みのPortを正しく削除できるか？
     */
    void test_add_removePort()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically

      PortMock* port0 = new PortMock();
      port0->setName("port0");
      // Portを正しく登録できるか？
      CPPUNIT_ASSERT_EQUAL(true, rto->addPort(*port0));
      // 既に登録済みのポートと同じポート名のPortを登録しようとした場合に失敗するか？
      // PortBase::updateConnectors()内、Guard guard(m_profile_mutex);でロックされ、
      // 処理が戻ってこない。(デッドロック???)
      // CPPUNIT_ASSERT_EQUAL(false, rto->addPort(*port0));

      PortMock* port1 = new PortMock();
      port1->setName("port1");
      CPPUNIT_ASSERT_EQUAL(true, rto->addPort(*port1));
			
      // 登録したPort参照をすべて正しく取得できるか？
      RTC::PortServiceList* portList = rto->get_ports();
      CPPUNIT_ASSERT(portList != NULL);
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2), portList->length());

      // 登録済みのPortを正しく削除できるか？
      CPPUNIT_ASSERT_EQUAL(true, rto->removePort(*port1));
      CPPUNIT_ASSERT_EQUAL(true, rto->removePort(*port0));

      InPortMock*  inport0  = new InPortMock("in","TimedLong");
      OutPortMock* outport0 = new OutPortMock("out","TimedLong");
      // InPort, OutPortを正しく登録できるか？
      CPPUNIT_ASSERT_EQUAL(true, rto->addInPort("in",*inport0));
      CPPUNIT_ASSERT_EQUAL(true, rto->addOutPort("out", *outport0));
      // 既に登録済みのポートと同じポート名のPortを登録しようとした場合に失敗するか？
      // PortBase::updateConnectors()内、Guard guard(m_profile_mutex);でロックされ、
      // 処理が戻ってこない。(デッドロック???)
      // CPPUNIT_ASSERT_EQUAL(false, rto->addInPort("in",*inport0));
      // CPPUNIT_ASSERT_EQUAL(false, rto->addOutPort("out", *outport0));

      // 登録したPort参照をすべて正しく取得できるか？
      portList = rto->get_ports();
      CPPUNIT_ASSERT(portList != NULL);
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2), portList->length());

      // 登録済みのPortを正しく削除できるか？
      CPPUNIT_ASSERT_EQUAL(true, rto->removeInPort(*inport0));
      CPPUNIT_ASSERT_EQUAL(true, rto->removeOutPort(*outport0));

      portList = rto->get_ports();
      CPPUNIT_ASSERT(portList != NULL);
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(0), portList->length());

      delete outport0;
      delete inport0;
      delete port1;
      delete port0;

      rto->shutdown();
      delete rto;

    }

    /*!
     * @brief readAll(),setReadAll()メソッドのテスト
     * 
     * - readAll()で全てのInPortのread()がコールされるか？
     * - setReadAll()で全てのInPortのread()がコールされるか？
     * - setReadAll()が正しく機能しているか？
     */
    void test_readAll()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically

      InPortMock*  inport0  = new InPortMock("in","TimedLong");
      InPortMock*  inport1  = new InPortMock("in2","TimedLong");
      OutPortMock* outport0 = new OutPortMock("out","TimedLong");

      rto->addInPort("in",*inport0);
      rto->addInPort("in2",*inport1);
      rto->addOutPort("out", *outport0);

      // readAll()で全てのInPortのread()がコールされるか？
      CPPUNIT_ASSERT_EQUAL(true, rto->readAll());
      CPPUNIT_ASSERT_EQUAL(1, inport0->get_counter());
      CPPUNIT_ASSERT_EQUAL(1, inport1->get_counter());

      // setReadAll()で全てのInPortのread()がコールされるか？
      rto->setReadAll();
      rto->on_execute(0);
      CPPUNIT_ASSERT_EQUAL(2, inport0->get_counter());
      CPPUNIT_ASSERT_EQUAL(2, inport1->get_counter());

      // setReadAll(),readAll()が正しく機能しているか？
      inport0->set_return(false);
      rto->setReadAll(true,false);
      CPPUNIT_ASSERT_EQUAL(false, rto->readAll());
      CPPUNIT_ASSERT_EQUAL(3, inport0->get_counter());
      // Because setReadAll(true,false) was called,
      // inport1.read() was not called.
      // inport0.read() return false.
      CPPUNIT_ASSERT_EQUAL(2, inport1->get_counter());

      rto->setReadAll(true,true);
      CPPUNIT_ASSERT_EQUAL(false, rto->readAll());
      CPPUNIT_ASSERT_EQUAL(4, inport0->get_counter());
      // Because setReadAll(true,true) was called,
      // inport0.read() return false,
      // but inport1.read() was called.
      CPPUNIT_ASSERT_EQUAL(3, inport1->get_counter());

      rto->setReadAll(false,true);
      rto->on_execute(0);
      // Because setReadAll(false,true) was called,
      // inport0.read() and inport1.read() was not called.
      CPPUNIT_ASSERT_EQUAL(4, inport0->get_counter());
      CPPUNIT_ASSERT_EQUAL(3, inport1->get_counter());
      
      rto->setReadAll(false,false);
      rto->on_execute(0);
      // Because setReadAll(false,true) was called,
      // inport0.read() and inport1.read() was not called.
      CPPUNIT_ASSERT_EQUAL(4, inport0->get_counter());
      CPPUNIT_ASSERT_EQUAL(3, inport1->get_counter());
      
      rto->removeInPort(*inport0);
      rto->removeInPort(*inport1);
      rto->removeOutPort(*outport0);
      
      delete outport0;
      delete inport0;
      delete inport1;

      rto->shutdown();
      delete rto;
    }


    /*!
     * @brief writeAll(),setWriteAll()メソッドのテスト
     * 
     * - setWriteAll()で全てのOutPortのwrite()がコールされるか？
     * - writeAll()で全てのOutPortのwrite()がコールされるか？
     * - setWriteAll()が正しく機能しているか？
     */
    void test_writeAll()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically

      OutPortMock*  outport0  = new OutPortMock("out","TimedLong");
      OutPortMock*  outport1  = new OutPortMock("out2","TimedLong");
      InPortMock*  inport0 = new InPortMock("in","TimedLong");

      rto->addOutPort("out",*outport0);
      rto->addOutPort("out2",*outport1);
      rto->addInPort("in", *inport0);

      // writeAll()で全てのOutPortのwrite()がコールされるか？
      CPPUNIT_ASSERT_EQUAL(true, rto->writeAll());
      CPPUNIT_ASSERT_EQUAL(1, outport0->get_counter());
      CPPUNIT_ASSERT_EQUAL(1, outport1->get_counter());

      // setWriteAll()で全てのOutPortのwrite()がコールされるか？
      rto->setWriteAll();
      rto->on_execute(0);
      CPPUNIT_ASSERT_EQUAL(2, outport0->get_counter());
      CPPUNIT_ASSERT_EQUAL(2, outport1->get_counter());

      // setWriteAll(),writeAll()が正しく機能しているか？
      outport0->set_return(false);
      rto->setWriteAll(true,false);
      CPPUNIT_ASSERT_EQUAL(false, rto->writeAll());
      CPPUNIT_ASSERT_EQUAL(3, outport0->get_counter());
      // Because setWriteAll(true,false) was called,
      // outport1.write() was not called.
      // outport0.write() return false.
      CPPUNIT_ASSERT_EQUAL(2, outport1->get_counter());

      rto->setWriteAll(true,true);
      CPPUNIT_ASSERT_EQUAL(false, rto->writeAll());
      CPPUNIT_ASSERT_EQUAL(4, outport0->get_counter());
      // Because setWriteAll(true,true) was called,
      // outport0.write() return false,
      // but outport1.write() was called.
      CPPUNIT_ASSERT_EQUAL(3, outport1->get_counter());

      rto->setWriteAll(false,true);
      rto->on_execute(0);
      // Because setWriteAll(false,true) was called,
      // outport0.write() and outport1.write() was not called.
      CPPUNIT_ASSERT_EQUAL(4, outport0->get_counter());
      CPPUNIT_ASSERT_EQUAL(3, outport1->get_counter());
      
      rto->setWriteAll(false,false);
      rto->on_execute(0);
      // Because setWriteAll(false,true) was called,
      // outport0.write() and outport1.write() was not called.
      CPPUNIT_ASSERT_EQUAL(4, outport0->get_counter());
      CPPUNIT_ASSERT_EQUAL(3, outport1->get_counter());
      
      rto->removeOutPort(*outport0);
      rto->removeOutPort(*outport1);
      rto->removeInPort(*inport0);
      
      delete outport0;
      delete outport1;
      delete inport0;

      rto->shutdown();
      delete rto;
    }


    /*!
     * @brief get_ports()メソッドのテスト
     * 
     * - 登録したPort参照をすべて正しく取得できるか？
     */
    void test_get_ports()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically

      // Portを登録しておく
      PortMock* port0 = new PortMock();
      port0->setName("port0");
      rto->addPort(*port0);

      PortMock* port1 = new PortMock();
      port1->setName("port1");
      rto->addPort(*port1);
			
      // 登録したPort参照をすべて正しく取得できるか？
      RTC::PortServiceList* portList = rto->get_ports();
      CPPUNIT_ASSERT(portList != NULL);
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2), portList->length());
      CPPUNIT_ASSERT(CORBA::Long(-1)
		     != CORBA_SeqUtil::find(*portList, PortFinder(port0->_this())));
      CPPUNIT_ASSERT(CORBA::Long(-1)
		     != CORBA_SeqUtil::find(*portList, PortFinder(port1->_this())));

      CPPUNIT_ASSERT_EQUAL(true, rto->removePort(*port1));
      CPPUNIT_ASSERT_EQUAL(true, rto->removePort(*port0));

      delete port1;
      delete port0;

      rto->shutdown();
      delete rto;

    }
		
    /*!
     * @brief get_execution_context_services()メソッドのテスト
     * 
     * - ExecutionContextServiceをすべて正しく取得できるか？
     */
/*
    void test_get_execution_context_services()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically

      // ExecutionContextを生成する
      RTC::PeriodicExecutionContext* ec1
	= new RTC::PeriodicExecutionContext(); // will be deleted automatically
      RTC::PeriodicExecutionContext* ec2
	= new RTC::PeriodicExecutionContext(); // will be deleted automatically

      // ExecutionContextにattachしておく
      RTC::UniqueId id1 = rto->attach_context(ec1->_this());
      CPPUNIT_ASSERT(RTC::UniqueId(-1) != id1);
      RTC::UniqueId id2 = rto->attach_context(ec2->_this());
      CPPUNIT_ASSERT(RTC::UniqueId(-1) != id2);

      // ExecutionContextServiceをすべて正しく取得できるか？
      // (注) RTC::PeriodicExecutionContextはExecutionContextServiceのサブクラスになっている。
      RTC::ExecutionContextServiceList* ecSvcList
//	= rto->get_execution_context_services();
	= rto->get_owned_contexts();
      CPPUNIT_ASSERT(ecSvcList != NULL);
      CPPUNIT_ASSERT(CORBA::Long(-1)
		     != CORBA_SeqUtil::find(*ecSvcList, ExecutionContextServiceFinder(ec1->_this())));
      CPPUNIT_ASSERT(CORBA::Long(-1)
		     != CORBA_SeqUtil::find(*ecSvcList, ExecutionContextServiceFinder(ec2->_this())));
    }
*/		
    void test_get_owned_organizations()
    {
      // テスト対象が未実装につき、テスト未実装
    }
		
    /*!
     * @brief get_sdo_id()メソッドのテスト
     * 
     * - SDO IDを取得できるか？
     * - 取得されたSDO IDは一意か？
     */
    void test_get_sdo_id()
    {
      RTObjectMock* rto1 = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
      rto1->setInstanceName("INSTANCE_NAME 1");
      std::string str1(rto1->getInstanceName());
      CPPUNIT_ASSERT("INSTANCE_NAME 1" == str1);
      RTObjectMock* rto2 = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
      rto2->setInstanceName("INSTANCE_NAME 2");
      std::string str2(rto2->getInstanceName());
      CPPUNIT_ASSERT("INSTANCE_NAME 2" == str2);

      // SDO IDを取得できるか？
      char* id1 = rto1->get_sdo_id();
      CPPUNIT_ASSERT(id1 != NULL);
      char* id2 = rto2->get_sdo_id();
      CPPUNIT_ASSERT(id2 != NULL);
			
      // 取得されたSDO IDは一意か？
      // (注) instance_nameがSDO IDとしてそのまま使用されることに注意。
      //      つまり、実装上、SDO IDの一意性はinstance_nameの一意性に基づいている。
      //      仕様上、instance_nameは一意でなければならないので、首尾一貫している。
      CPPUNIT_ASSERT(id1 != id2);

      rto2->shutdown();
      delete rto2;
      rto1->shutdown();
      delete rto1;
    }
		
    /*!
     * @brief get_sdo_type()メソッドのテスト
     * 
     * - SDOタイプを取得できるか？
     */
    void test_get_sdo_type()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
				
      // ※ 実装上、type_nameがSDOタイプとして使用されているため、ここで準備設定している
      coil::Properties prop;
      prop.setProperty("type_name", "TYPE_NAME");
      rto->setProperties(prop);
			
      // SDOタイプを取得できるか？
      char* sdoType = rto->get_sdo_type();
      CPPUNIT_ASSERT(sdoType != NULL);

      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief get_device_profile()メソッドのテスト
     * 
     * - DeviceProfileを正しく取得できるか？
     */
    void test_get_device_profile()
    {
      // test_get_configuration_and_set_device_profile_and_get_device_profileで兼ねる
    }
		
    /*!
     * @brief get_service_profile()メソッドのテスト
     * 
     * - 指定したIDのServiceProfileを正しく取得できるか？
     */
    void test_get_service_profile()
    {
      // test_get_configuration_and_set_service_profile_and_get_service_profileで兼ねる
    }
		
    /*!
     * @brief get_service_profile()メソッドのテスト
     * 
     * - 引数にNULLを指定した場合、意図どおりの例外がスローされるか？
     * - 引数に存在しないIDを指定した場合、意図どおりの例外がスローされるか？
     */
    void test_get_service_profile_with_illegal_arguments()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // 引数にNULLを指定した場合、意図どおりの例外がスローされるか？
      try
	{
	  rto->get_service_profile(NULL);
	  CPPUNIT_FAIL("Exception not thrown.");
	}
      catch (SDOPackage::InvalidParameter expected)
	{
	  // 意図どおりの例外をキャッチした
	}
      catch (...)
	{
	  // 意図しない例外をキャッチした
	  CPPUNIT_FAIL("Unexpected exception caught.");
	}
			
      // 引数に存在しないIDを指定した場合、意図どおりの例外がスローされるか？
      try
	{
	  rto->get_service_profile("INEXIST ID");
	  CPPUNIT_FAIL("Exception not thrown.");
	}
      catch (SDOPackage::InvalidParameter expected)
	{
	  // 意図どおりの例外をキャッチした
	}
      catch (...)
	{
	  // 意図しない例外をキャッチした
	  CPPUNIT_FAIL("Unexpected exception caught.");
	}
      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief get_sdo_service()メソッドのテスト
     * 
     * - 指定したIDのSDOServiceを正しく取得できるか？
     */
    void test_get_sdo_service()
    {
      // test_get_configuration_and_set_service_profile_and_get_sdo_serviceで兼ねる
    }
		
    void test_get_sdo_service_with_illegal_arguments()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // 引数にNULLを指定した場合、意図どおりの例外がスローされるか？
      try
	{
	  rto->get_sdo_service(NULL);
	  CPPUNIT_FAIL("Exception not thrown.");
	}
      catch (SDOPackage::InvalidParameter expected)
	{
	  // 意図どおりの例外をキャッチした
	}
      catch (...)
	{
	  // 意図しない例外をキャッチした
	  CPPUNIT_FAIL("Unexpected exception caught.");
	}
			
      // 引数に、存在しないIDを指定した場合、意図どおりの例外がスローされるか？
      try
	{
	  rto->get_sdo_service("INEXIST ID");
	  CPPUNIT_FAIL("Exception not thrown.");
	}
      catch (SDOPackage::InvalidParameter expected)
	{
	  // 意図どおりの例外をキャッチした
	}
      catch (...)
	{
	  // 意図しない例外をキャッチした
	  CPPUNIT_FAIL("Unexpected exception caught.");
	}

      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief get_configuration()メソッドのテスト
     * 
     * - Configuration::set_device_profile()を通して、DeviceProfileを正しく設定できるか？
     * - 設定されたDeviceProfileを、get_device_profile()で正しく取得できるか？
     */
    void test_get_configuration_and_set_device_profile_and_get_device_profile()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // DeviceProfileを準備しておく
      SDOPackage::DeviceProfile devProf;
      devProf.device_type = "DEVICE_TYPE";
      devProf.manufacturer = "MANUFACTURER";
      devProf.model = "MODEL";
      devProf.version = "VERSION";
      devProf.properties.length(1);
      devProf.properties[0].name = "PROPERTIES NAME";
      devProf.properties[0].value <<= "PROPERTIES VALUE";
			
      // Configurationインタフェースを取得し、DeviceProfileを設定する
      SDOPackage::Configuration_ptr cfg = rto->get_configuration();
      cfg->set_device_profile(devProf);
      // DeviceProfileを取得して、正しく設定されたことを確認する
      SDOPackage::DeviceProfile* devProfRet = rto->get_device_profile();
      CPPUNIT_ASSERT_EQUAL(std::string("DEVICE_TYPE"),
			   std::string(devProfRet->device_type));
      CPPUNIT_ASSERT_EQUAL(std::string("MANUFACTURER"),
			   std::string(devProfRet->manufacturer));
      CPPUNIT_ASSERT_EQUAL(std::string("MODEL"),
			   std::string(devProfRet->model));
      CPPUNIT_ASSERT_EQUAL(std::string("VERSION"),
			   std::string(devProfRet->version));
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(1), devProfRet->properties.length());
      CPPUNIT_ASSERT_EQUAL(std::string("PROPERTIES NAME"),
			   std::string(devProfRet->properties[0].name));
      {
	const char* value; devProfRet->properties[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("PROPERTIES VALUE"), std::string(value));
      }

      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief get_configuration()メソッドとget_serivce_profile()メソッドのテスト
     * 
     * - Configuration::set_service_profile()を通して、ServiceProfileを正しく設定できるか？
     * - 設定されたServiceProfileを、get_service_profile()で正しく取得できるか？
     */
    void test_get_configuration_and_set_service_profile_and_get_service_profile()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically

      // SDOServiceを準備する
      SDOServiceMock* sdoSvc1 = new SDOServiceMock();
      SDOServiceMock* sdoSvc2 = new SDOServiceMock();
			
      // ServiceProfileを準備しておく
      SDOPackage::ServiceProfile svcProf1;
      svcProf1.id = "ID 1";
      svcProf1.interface_type = "INTERFACE_TYPE 1";
      svcProf1.properties.length(1);
      svcProf1.properties[0].name = "PROPERTIES NAME 1";
      svcProf1.properties[0].value <<= "3.14159";
      svcProf1.service = sdoSvc1->_this();

      SDOPackage::ServiceProfile svcProf2;
      svcProf2.id = "ID 2";
      svcProf2.interface_type = "INTERFACE_TYPE 2";
      svcProf2.properties.length(1);
      svcProf2.properties[0].name = "PROPERTIES NAME 2";
      svcProf2.properties[0].value <<= "2.71828";
      svcProf2.service = sdoSvc2->_this();
			
      // Configurationインタフェースを取得し、ServiceProfileを設定する
      SDOPackage::Configuration_ptr cfg = rto->get_configuration();
      CPPUNIT_ASSERT(! CORBA::is_nil(cfg));
      cfg->add_service_profile(svcProf1);
      cfg->add_service_profile(svcProf2);
			
      // get_service_profile()を用いてServiceProfileを取得して、正しく設定されたことを確認する
      SDOPackage::ServiceProfile* svcProfRet1 = rto->get_service_profile("ID 1");
      CPPUNIT_ASSERT(svcProfRet1 != NULL);
      CPPUNIT_ASSERT_EQUAL(std::string("ID 1"),	std::string(svcProfRet1->id));
      CPPUNIT_ASSERT_EQUAL(std::string("INTERFACE_TYPE 1"),
			   std::string(svcProfRet1->interface_type));
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(1), svcProfRet1->properties.length());
      CPPUNIT_ASSERT_EQUAL(std::string("PROPERTIES NAME 1"),
			   std::string(svcProfRet1->properties[0].name));
      {
	const char* value; svcProfRet1->properties[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("3.14159"), std::string(value));
      }
			
      SDOPackage::ServiceProfile* svcProfRet2 = rto->get_service_profile("ID 2");
      CPPUNIT_ASSERT(svcProfRet2 != NULL);
      CPPUNIT_ASSERT_EQUAL(std::string("ID 2"), std::string(svcProfRet2->id));
      CPPUNIT_ASSERT_EQUAL(std::string("INTERFACE_TYPE 2"),
			   std::string(svcProfRet2->interface_type));
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(1), svcProfRet2->properties.length());
      CPPUNIT_ASSERT_EQUAL(std::string("PROPERTIES NAME 2"),
			   std::string(svcProfRet2->properties[0].name));
      {
	const char* value; svcProfRet2->properties[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("2.71828"), std::string(value));
      }

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(sdoSvc2));
      delete sdoSvc2;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(sdoSvc1));
      delete sdoSvc1;

      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief get_configuration()メソッドとget_service_profiles()メソッドのテスト
     * 
     * - 設定されたServiceProfileを、get_service_profiles()で正しく取得できるか？
     */
    void test_get_configuration_and_set_service_profile_and_get_service_profiles()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically

      // SDOServiceを準備する
      SDOServiceMock* sdoSvc1 = new SDOServiceMock();
      SDOServiceMock* sdoSvc2 = new SDOServiceMock();
			
      // ServiceProfileを準備しておく
      SDOPackage::ServiceProfile svcProf1;
      svcProf1.id = "ID 1";
      svcProf1.interface_type = "INTERFACE_TYPE 1";
      svcProf1.properties.length(1);
      svcProf1.properties[0].name = "PROPERTIES NAME 1";
      svcProf1.properties[0].value <<= "3.14159";
      svcProf1.service = sdoSvc1->_this();

      SDOPackage::ServiceProfile svcProf2;
      svcProf2.id = "ID 2";
      svcProf2.interface_type = "INTERFACE_TYPE 2";
      svcProf2.properties.length(1);
      svcProf2.properties[0].name = "PROPERTIES NAME 2";
      svcProf2.properties[0].value <<= "2.71828";
      svcProf2.service = sdoSvc2->_this();
			
      // Configurationインタフェースを取得し、ServiceProfileを設定する
      SDOPackage::Configuration_ptr cfg = rto->get_configuration();
      CPPUNIT_ASSERT(! CORBA::is_nil(cfg));
      cfg->add_service_profile(svcProf1);
      cfg->add_service_profile(svcProf2);
			
      // get_service_profiles()を使ってServiceProfile群を取得して、正しく設定されたことを確認する
      SDOPackage::ServiceProfileList* svcProfList = rto->get_service_profiles();
      CPPUNIT_ASSERT(svcProfList != NULL);
			
      CORBA::Long svcProfIdx1 = CORBA_SeqUtil::find(
						    *svcProfList, ServiceProfileFinder("ID 1"));
      CPPUNIT_ASSERT(CORBA::Long(-1) != svcProfIdx1);
      CPPUNIT_ASSERT_EQUAL(std::string("ID 1"),
			   std::string((*svcProfList)[svcProfIdx1].id));
      CPPUNIT_ASSERT_EQUAL(std::string("INTERFACE_TYPE 1"),
			   std::string((*svcProfList)[svcProfIdx1].interface_type));
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(1),
			   (*svcProfList)[svcProfIdx1].properties.length());
      CPPUNIT_ASSERT_EQUAL(std::string("PROPERTIES NAME 1"),
			   std::string((*svcProfList)[svcProfIdx1].properties[0].name));
      {
	const char* value; (*svcProfList)[svcProfIdx1].properties[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("3.14159"), std::string(value));
      }
			
      CORBA::Long svcProfIdx2 = CORBA_SeqUtil::find(
						    *svcProfList, ServiceProfileFinder("ID 2"));
      CPPUNIT_ASSERT(CORBA::Long(-1) != svcProfIdx2);
      CPPUNIT_ASSERT_EQUAL(std::string("ID 2"),
			   std::string((*svcProfList)[svcProfIdx2].id));
      CPPUNIT_ASSERT_EQUAL(std::string("INTERFACE_TYPE 2"),
			   std::string((*svcProfList)[svcProfIdx2].interface_type));
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(1),
			   (*svcProfList)[svcProfIdx2].properties.length());
      CPPUNIT_ASSERT_EQUAL(std::string("PROPERTIES NAME 2"),
			   std::string((*svcProfList)[svcProfIdx2].properties[0].name));
      {
	const char* value; (*svcProfList)[svcProfIdx2].properties[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("2.71828"), std::string(value));
      }
      
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(sdoSvc2));
      delete sdoSvc2;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(sdoSvc1));
      delete sdoSvc1;

      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief get_configuration()メソッドとget_sdo_service()メソッドのテスト
     * 
     * - Configuration::set_service_profile()を通して、SDOServiceを正しく設定できるか？
     * - get_sdo_service()を用いて、設定されているSDPServiceを正しく取得できるか？
     */
    void test_get_configuration_and_set_service_profile_and_get_sdo_service()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically

      // SDOServiceを準備する
      SDOServiceMock* sdoSvc1 = new SDOServiceMock();
      SDOServiceMock* sdoSvc2 = new SDOServiceMock();
			
      // ServiceProfileを準備しておく
      SDOPackage::ServiceProfile svcProf1;
      svcProf1.id = "ID 1";
      svcProf1.interface_type = "INTERFACE_TYPE 1";
      svcProf1.properties.length(1);
      svcProf1.properties[0].name = "PROPERTIES NAME 1";
      svcProf1.properties[0].value <<= "3.14159";
      svcProf1.service = sdoSvc1->_this();

      SDOPackage::ServiceProfile svcProf2;
      svcProf2.id = "ID 2";
      svcProf2.interface_type = "INTERFACE_TYPE 2";
      svcProf2.properties.length(1);
      svcProf2.properties[0].name = "PROPERTIES NAME 2";
      svcProf2.properties[0].value <<= "2.71828";
      svcProf2.service = sdoSvc2->_this();
			
      // Configurationインタフェースを取得し、ServiceProfileを設定する
      SDOPackage::Configuration_ptr cfg = rto->get_configuration();
      CPPUNIT_ASSERT(! CORBA::is_nil(cfg));
      cfg->add_service_profile(svcProf1);
      cfg->add_service_profile(svcProf2);
			
      // 指定したIDのSDOServiceを正しく取得できるか？
      SDOPackage::SDOService_ptr sdoSvcRet1 = rto->get_sdo_service("ID 1");
      CPPUNIT_ASSERT(! CORBA::is_nil(sdoSvcRet1));
      CPPUNIT_ASSERT(sdoSvcRet1->_is_equivalent(sdoSvc1->_this()));

      SDOPackage::SDOService_ptr sdoSvcRet2 = rto->get_sdo_service("ID 2");
      CPPUNIT_ASSERT(! CORBA::is_nil(sdoSvcRet2));
      CPPUNIT_ASSERT(sdoSvcRet2->_is_equivalent(sdoSvc2->_this()));

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(sdoSvc2));
      delete sdoSvc2;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(sdoSvc1));
      delete sdoSvc1;

      rto->shutdown();
      delete rto;

    }
		
    /*!
     * @brief Configuration::remove_service_profile()メソッドのテスト
     * 
     * - 指定したIDのServiceProfileを正しく削除できるか？
     */
    void test_get_configuration_and_remove_service_profile()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically

      // SDOServiceを準備する
      SDOServiceMock* sdoSvc1 = new SDOServiceMock();
      SDOServiceMock* sdoSvc2 = new SDOServiceMock();
			
      // ServiceProfileを準備しておく
      SDOPackage::ServiceProfile svcProf1;
      svcProf1.id = "ID 1";
      svcProf1.interface_type = "INTERFACE_TYPE 1";
      svcProf1.properties.length(1);
      svcProf1.properties[0].name = "PROPERTIES NAME 1";
      svcProf1.properties[0].value <<= "3.14159";
      svcProf1.service = sdoSvc1->_this();

      SDOPackage::ServiceProfile svcProf2;
      svcProf2.id = "ID 2";
      svcProf2.interface_type = "INTERFACE_TYPE 2";
      svcProf2.properties.length(1);
      svcProf2.properties[0].name = "PROPERTIES NAME 2";
      svcProf2.properties[0].value <<= "2.71828";
      svcProf2.service = sdoSvc2->_this();
			
      // Configurationインタフェースを取得し、ServiceProfileを設定する
      SDOPackage::Configuration_ptr cfg = rto->get_configuration();
      CPPUNIT_ASSERT(! CORBA::is_nil(cfg));
      cfg->add_service_profile(svcProf1);
      cfg->add_service_profile(svcProf2);
      CPPUNIT_ASSERT(rto->get_service_profile("ID 1") != NULL);
      CPPUNIT_ASSERT(rto->get_service_profile("ID 2") != NULL);
			
      // 設定したうち、片方のServiceProfileをremoveして、正しくremoveされたことを確認する
      CPPUNIT_ASSERT_EQUAL(true, cfg->remove_service_profile("ID 1"));
      try
	{
	  rto->get_service_profile("ID 1");
	  CPPUNIT_FAIL("Expected exception not thrown.");
	}
      catch (SDOPackage::InvalidParameter expected) {}
      CPPUNIT_ASSERT(rto->get_service_profile("ID 2") != NULL);

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(sdoSvc2));
      delete sdoSvc2;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(sdoSvc1));
      delete sdoSvc1;

      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief get_configuration()メソッドとget_organizations()メソッドのテスト
     * 
     * - Configuration::add_organization()でOrganizationを正しく追加できるか？
     * - get_organizations()で、登録されているOrganizationを正しく取得できるか？
     */
    void test_get_configuration_and_add_organization_and_get_organizations()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // Organizationを準備する
      OrganizationMock* org1 = new OrganizationMock("ORG 1");
      OrganizationMock* org2 = new OrganizationMock("ORG 2");

      // Configurationインタフェースを取得し、Organizationを追加する
      SDOPackage::Configuration_ptr cfg = rto->get_configuration();
      CPPUNIT_ASSERT(! CORBA::is_nil(cfg));
      CPPUNIT_ASSERT_EQUAL(true, cfg->add_organization(org1->_this()));
      CPPUNIT_ASSERT_EQUAL(true, cfg->add_organization(org2->_this()));
			
      // get_organizations()を用いてOrganization群を正しく取得できるか？
      SDOPackage::OrganizationList* orgList = rto->get_organizations();
      CPPUNIT_ASSERT(orgList != NULL);
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2), orgList->length());
			
      CORBA::Long orgIdx1 = CORBA_SeqUtil::find(*orgList, OrganizationFinder("ORG 1"));
      CPPUNIT_ASSERT(CORBA::Long(-1) != orgIdx1);
      CPPUNIT_ASSERT_EQUAL(std::string("ORG 1"),
			   std::string((*orgList)[orgIdx1]->get_organization_id()));

      CORBA::Long orgIdx2 = CORBA_SeqUtil::find(*orgList, OrganizationFinder("ORG 2"));
      CPPUNIT_ASSERT(CORBA::Long(-1) != orgIdx2);
      CPPUNIT_ASSERT_EQUAL(std::string("ORG 2"),
			   std::string((*orgList)[orgIdx2]->get_organization_id()));

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(org2));
      delete org2;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(org1));
      delete org1;

      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief Configuration::remove_organization()メソッドのテスト
     * 
     * - 指定したIDのOrganizationを正しく削除できるか？
     */
    void test_get_configuration_and_remove_organization()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // Organizationを準備する
      OrganizationMock* org1 = new OrganizationMock("ORG 1");
      OrganizationMock* org2 = new OrganizationMock("ORG 2");

      // Configurationインタフェースを取得し、Organizationを追加する
      SDOPackage::Configuration_ptr cfg = rto->get_configuration();
      CPPUNIT_ASSERT(! CORBA::is_nil(cfg));
      CPPUNIT_ASSERT_EQUAL(true, cfg->add_organization(org1->_this()));
      CPPUNIT_ASSERT_EQUAL(true, cfg->add_organization(org2->_this()));
			
      // 追加されていることを確認しておく
      SDOPackage::OrganizationList* orgList = rto->get_organizations();
      CPPUNIT_ASSERT(orgList != NULL);
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2), orgList->length());
			
      // 追加したうち、片方のOrganizationをremoveし、正しくremoveされていることを確認する
      CPPUNIT_ASSERT_EQUAL(true, cfg->remove_organization("ORG 1"));
      orgList = rto->get_organizations();
      CPPUNIT_ASSERT(orgList != NULL);
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(1), orgList->length());

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(org2));
      delete org2;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(org1));
      delete org1;

      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief get_monitoring()メソッドのテスト
     */
    void test_get_monitoring()
    {
      // テスト対象が未実装につき、テストも未実装
    }
		
    /*!
     * @brief get_status()メソッドのテスト
     * 
     * - 指定した名称のstatus値を正しく取得できるか？
     */
    void test_get_status()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // Mockの機能を用いてstatusを設定しておく
      CORBA::Any valueAny1; valueAny1 <<= CORBA::Float(3.14159);
      rto->set_status("STATUS 1", valueAny1);
			
      CORBA::Any valueAny2; valueAny2 <<= CORBA::Float(2.71828);
      rto->set_status("STATUS 2", valueAny2);
			
      // 設定したstatusを正しく取得できるか？
      CORBA::Any* valueAnyRet1 = rto->get_status("STATUS 1");
      CPPUNIT_ASSERT(valueAnyRet1 != NULL);
      {
	CORBA::Float value; *valueAnyRet1 >>= value;
	CPPUNIT_ASSERT_EQUAL(CORBA::Float(3.14159), value);
      }
			
      CORBA::Any* valueAnyRet2 = rto->get_status("STATUS 2");
      CPPUNIT_ASSERT(valueAnyRet2 != NULL);
      {
	CORBA::Float value; *valueAnyRet2 >>= value;
	CPPUNIT_ASSERT_EQUAL(CORBA::Float(2.71828), value);
      }
      
      rto->shutdown();
      delete rto;

    }
		
    /*!
     * @brief get_status_list()メソッドのテスト
     * 
     * - 設定されているすべてのstatusを正しく取得できるか？
     */
    void test_get_status_list()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA); // will be deleted automatically
			
      // Mockの機能を用いてstatusを設定しておく
      CORBA::Any valueAny1; valueAny1 <<= CORBA::Float(3.14159);
      rto->set_status("STATUS 1", valueAny1);
			
      CORBA::Any valueAny2; valueAny2 <<= CORBA::Float(2.71828);
      rto->set_status("STATUS 2", valueAny2);
			
      // 設定したstatusを正しく取得できるか？
      SDOPackage::NVList* statusList = rto->get_status_list();
      CPPUNIT_ASSERT(statusList != NULL);
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2), statusList->length());
			
      const CORBA::Any& valueAnyRet1 = NVUtil::find(*statusList, "STATUS 1");
      {
	CORBA::Float value; valueAnyRet1 >>= value;
	CPPUNIT_ASSERT_EQUAL(CORBA::Float(3.14159), value);
      }

      const CORBA::Any& valueAnyRet2 = NVUtil::find(*statusList, "STATUS 2");
      {
	CORBA::Float value; valueAnyRet2 >>= value;
	CPPUNIT_ASSERT_EQUAL(CORBA::Float(2.71828), value);
      }

      rto->shutdown();
      delete rto;
    }
		
    /*!
     * @brief finalizeContexts()メソッドのテスト
     * 
     * - 全コンテキストの登録を削除できるか？
     */
    void test_finalizeContexts()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA);
      coil::Properties prop;
      prop.setProperty("exec_cxt.periodic.type","PeriodicExecutionContext");
      prop.setProperty("exec_cxt.periodic.rate","1000");
      rto->setProperties(prop);
      // initialize()で、m_eclistへ登録し、m_ecMineをstart
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, rto->initialize());
      CPPUNIT_ASSERT_EQUAL(1, rto->get_eclist());
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, rto->initialize());
      CPPUNIT_ASSERT_EQUAL(2, rto->get_eclist());
      CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, rto->initialize());
      CPPUNIT_ASSERT_EQUAL(3, rto->get_eclist());

      RTC::ExecutionContext_ptr ec;
      ec = rto->get_context(0);
      CPPUNIT_ASSERT_EQUAL(true, rto->is_alive(ec));
      rto->finalizeContexts();

      // 全コンテキストが削除されたか？
      CPPUNIT_ASSERT_EQUAL(0, rto->get_eclist());
      rto->exit();
      delete rto;
    }
		
    /*!
     * @brief bindContext()メソッドのテスト
     * 
     * - ExecutionContextを正しく設定できるか？
     */
    void test_bindContext()
    {
      RTObjectMock* rto = new RTObjectMock(m_pORB, m_pPOA);
      coil::Properties prop;
      prop.setProperty("exec_cxt.periodic.type","PeriodicExecutionContext");
      prop.setProperty("exec_cxt.periodic.rate","1000");
      rto->setProperties(prop);

      RTC::ExecutionContext_ptr ec;

      // nilを設定した場合、-1を返すか？
      ec = RTC::ExecutionContext::_nil();
      int id = (int)(rto->bindContext(ec));
      CPPUNIT_ASSERT_EQUAL(-1, id);

      // m_ecMine 未登録の場合、m_ecMineの番号を返すか？
      RTC::PeriodicExecutionContext* pec = new RTC::PeriodicExecutionContext();
      ec = pec->getObjRef();
      id = (int)(rto->bindContext(ec));

      // [0]に登録されるか？
      CPPUNIT_ASSERT_EQUAL(0, id);

      // 正しく登録されているか？
      CPPUNIT_ASSERT(rto->chk_ecMine(id,ec));

      // m_ecMine 登録済みで nil の場合、m_ecMineの番号を返すか？
      rto->ecMine[0] = RTC::ExecutionContextService::_nil();
      rto->set_ecMine();
      RTC::PeriodicExecutionContext* pec2 = new RTC::PeriodicExecutionContext();
      ec = pec2->getObjRef();
      id = (int)(rto->bindContext(ec));

      // [0]に登録されるか？
      CPPUNIT_ASSERT_EQUAL(0, id);

      // 正しく登録されているか？
      CPPUNIT_ASSERT(rto->chk_ecMine(id,ec));

      rto->exit();

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(pec));
      delete pec;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(pec2));
      delete pec2;

      rto->shutdown();
      delete rto;

    }
		
  };
}; // namespace RTObject

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(RTObject::RTObjectTests);

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
#endif // RTObject_cpp
