// -*- C++ -*-
/*!
 * @file   CorbaPortTests.cpp
 * @brief  CorbaPort test class
 * @date   $Date: 2008/04/04 12:06:09 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id$
 *
 */

/*
 * $Log: CorbaPortTests.cpp,v $
 * Revision 1.2  2008/04/04 12:06:09  arafune
 * Refactored some tests. No new tests were added.
 *
 * Revision 1.1  2007/12/20 07:50:18  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2007/01/04 00:50:38  n-ando
 * *** empty log message ***
 *
 *
 */

#ifndef CorbaPort_cpp
#define CorbaPort_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/CorbaPort.h>
#include <rtm/CorbaConsumer.h>

#include "MyServiceSkel.h"

class MyService_impl
  : public virtual POA_MyService,
    public virtual PortableServer::RefCountServantBase
{
public:
  MyService_impl() : m_hello_world_called(false)
  {
  };

  virtual ~MyService_impl()
  {
  };
	
  void setName(const char* name)
  {
    m_name = name;
  }
	
  char* name()
  {
    return "MyService";
  }
	
  void hello_world()
  {
    m_hello_world_called = true;
  }
	
  void print_msg(const char* msg)
  {
    std::cout << m_name << ": " << msg << std::endl;
  }

  bool is_hello_world_called()
  {
    return m_hello_world_called;
  }
	
private:
  std::string m_name;
  bool m_hello_world_called;
};


/*!
 * @class CorbaPortMock class
 *
 */
class CorbaPortMock
 : public RTC::CorbaPort
{
  public:
    /*!
     *
     */
    CorbaPortMock(const char* name)
    : RTC::CorbaPort(name)
    {
    }
    /*!
     *
     */
    virtual ~CorbaPortMock(void)
    {
    }
    /*!
     *
     */
    void deactivateInterfaces_public()
    {
        deactivateInterfaces();
    }
    /*!
     *
     */
    void activateInterfaces_public()
    {
        activateInterfaces();
    }
};
/*!
 * @class CorbaPortTests class
 * @brief CorbaPort test
 */
namespace CorbaPort
{
  class CorbaPortTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(CorbaPortTests);
    CPPUNIT_TEST(test_get_port_profile);
    CPPUNIT_TEST(test_connect);
    CPPUNIT_TEST(test_disconnect);
    CPPUNIT_TEST(test_registerProvider);
    CPPUNIT_TEST(test_activateInterfaces_deactivateInterfaces);
    CPPUNIT_TEST_SUITE_END();
		
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
		
  public:
    /*!
     * @brief Constructor
     */
    CorbaPortTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~CorbaPortTests()
    {
    }
		
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
      int argc(0);
      char** argv(NULL);
			
      m_pORB = CORBA::ORB_init(argc, argv);
      m_pPOA = PortableServer::POA::_narrow(
					    m_pORB->resolve_initial_references("RootPOA"));
      m_pPOA->the_POAManager()->activate();
    }
		
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    { 
      try
	{
	  // m_pPOA->destroy(false, false);
	  // m_pPOA->the_POAManager()->deactivate(false, false);
	  // m_pORB->destroy(); 
	}
      catch (...)
	{
	  // do nothing
	}
    }
		
    /*!
     * @brief get_port_profile()メソッドのテスト
     * 
     * - プロファイル内の、ポート名称を正しく取得できるか？
     * - インタフェースプロファイル内の、instance_nameを正しく取得できるか？
     * - インタフェースプロファイル内の、type_nameを正しく取得できるか？
     * - インタフェースプロファイル内の、polarityを正しく取得できるか？
     */
    void test_get_port_profile()
    {
      // テスト対象となるCorbaPortを構成する
      MyService_impl* pMyServiceImpl
	= new MyService_impl(); // will be deleted automatically
      RTC::CorbaConsumer<MyService>* pMyServiceConsumer
	= new RTC::CorbaConsumer<MyService>(); // will be deleted automatically
			
      RTC::CorbaPort* port = new RTC::CorbaPort("name of port");
      coil::Properties dummy;
      port->init(dummy);
      port->registerProvider("MyService (provided)", "Generic (provided)", *pMyServiceImpl);
      port->registerConsumer("MyService (required)", "Generic (required)", *pMyServiceConsumer);
			
      RTC::PortService_var portRef = port->getPortRef();
      RTC::PortProfile* profile = portRef->get_port_profile();

      // ポート名称を正しく取得できるか？
      CPPUNIT_ASSERT_EQUAL(std::string("unknown.name of port"), std::string(profile->name));
			
      // インタフェースプロファイルを取得し、あらかじめ設定しておいた内容と一致することを確認する
      RTC::PortInterfaceProfileList& profiles = profile->interfaces;
      for (CORBA::ULong i = 0; i < profile->interfaces.length(); ++i)
	{
	  if (std::string(profiles[i].instance_name)
	      == std::string("MyService (provided)"))
	    {
	      // type_nameを正しく取得できるか？
	      CPPUNIT_ASSERT_EQUAL(std::string("Generic (provided)"),
				   std::string(profiles[i].type_name));
					
	      // polarityを正しく取得できるか？
	      CPPUNIT_ASSERT_EQUAL(RTC::PROVIDED, profiles[i].polarity);
	    }
	  else if (std::string(profiles[i].instance_name)
		   == std::string("MyService (required)"))
	    {
	      // type_nameを正しく取得できるか？
	      CPPUNIT_ASSERT_EQUAL(std::string("Generic (required)"),
				   std::string(profiles[i].type_name));

	      // polarityを正しく取得できるか？
	      CPPUNIT_ASSERT_EQUAL(RTC::REQUIRED, profiles[i].polarity);
	    }
	  else
	    {
	      // 予期しないinstance_nameが取得された場合
	      std::string msg("Unexpected instance_name:");
	      msg += std::string(profiles[i].instance_name);
	      CPPUNIT_FAIL(msg);
	    }
	}

      delete port;
      delete pMyServiceConsumer;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(pMyServiceImpl));
      delete pMyServiceImpl;
    }
		
    /*!
     * connect()メソッド、および、そこから呼び出される各protectedメソッドのテスト
     * 
     * - ポート1のコンシューマ側からメソッドを呼び出すと、ポート0のプロバイダ側が意図どおり呼び出されるか？
     * - ポート0のコンシューマ側からメソッドを呼び出すと、ポート1のプロバイダ側が意図どおり呼び出されるか？
     */
    void test_connect()
    {
      // ポート0を構成する
      MyService_impl* pMyServiceImplA
	= new MyService_impl(); // will be deleted automatically
      RTC::CorbaConsumer<MyService>* pMyServiceConsumerB
	= new RTC::CorbaConsumer<MyService>(); // will be deleted automatically
			
      CorbaPortMock* port0 = new CorbaPortMock("name of port0");
      port0->registerProvider("MyServiceA", "Generic", *pMyServiceImplA);
      port0->registerConsumer("MyServiceB", "Generic", *pMyServiceConsumerB);

      // ポート1を構成する
      MyService_impl* pMyServiceImplB
	= new MyService_impl(); // will be deleted automatically
      RTC::CorbaConsumer<MyService>* pMyServiceConsumerA
	= new RTC::CorbaConsumer<MyService>(); // will be deleted automatically
			
      CorbaPortMock* port1 = new CorbaPortMock("name of port1");
      port1->registerProvider("MyServiceB", "Generic", *pMyServiceImplB);
      port1->registerConsumer("MyServiceA", "Generic", *pMyServiceConsumerA);
			
      // 接続プロファイルを構成する
      RTC::ConnectorProfile connProfile;
      connProfile.connector_id = "";
      connProfile.name = CORBA::string_dup("name of connector profile");
      connProfile.ports.length(2);
      connProfile.ports[0] = port0->getPortRef();
      connProfile.ports[1] = port1->getPortRef();

      // 接続する
      port0->getPortRef()->connect(connProfile);

      port0->activateInterfaces_public();
      port1->activateInterfaces_public();

      // ポート1のコンシューマ側からメソッドを呼び出すと、ポート0のプロバイダ側が意図どおり呼び出されるか？
      CPPUNIT_ASSERT(! pMyServiceImplA->is_hello_world_called());
      (*pMyServiceConsumerA)->hello_world();
      CPPUNIT_ASSERT(pMyServiceImplA->is_hello_world_called());

      // ポート0のコンシューマ側からメソッドを呼び出すと、ポート1のプロバイダ側が意図どおり呼び出されるか？
      CPPUNIT_ASSERT(! pMyServiceImplB->is_hello_world_called());
      (*pMyServiceConsumerB)->hello_world();
      CPPUNIT_ASSERT(pMyServiceImplB->is_hello_world_called());

//      delete port1;
      delete port0;
      delete pMyServiceConsumerA;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(pMyServiceImplB));
      delete pMyServiceImplB;
      delete pMyServiceConsumerB;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(pMyServiceImplA));
      delete pMyServiceImplA;
    }
    
    /*!
     * @brief disconnect()メソッド、および、そこから呼び出される各protectedメソッドのテスト
     * 
     * - 接続が成功した後に切断を行い、リモートメソッドの呼出しが意図どおり失敗することを確認する
     */
    void test_disconnect()
    {
      // ポート0を構成する
      MyService_impl* pMyServiceImplA
	= new MyService_impl(); // will be deleted automatically
      RTC::CorbaConsumer<MyService>* pMyServiceConsumerB
	= new RTC::CorbaConsumer<MyService>(); // will be deleted automatically
			
      CorbaPortMock* port0 = new CorbaPortMock("name of port0-1");
      port0->registerProvider("MyServiceA-1", "Generic", *pMyServiceImplA);
      port0->registerConsumer("MyServiceB-1", "Generic", *pMyServiceConsumerB);

      // ポート1を構成する
      MyService_impl* pMyServiceImplB
	= new MyService_impl(); // will be deleted automatically
      RTC::CorbaConsumer<MyService>* pMyServiceConsumerA
	= new RTC::CorbaConsumer<MyService>(); // will be deleted automatically
			
      CorbaPortMock* port1 = new CorbaPortMock("name of port1-1");
      port1->registerProvider("MyServiceB-1", "Generic", *pMyServiceImplB);
      port1->registerConsumer("MyServiceA-1", "Generic", *pMyServiceConsumerA);
			
      // 接続プロファイルを構成する
      RTC::ConnectorProfile connProfile;
      connProfile.connector_id = "id1";
      connProfile.name = CORBA::string_dup("name of connector profile-1");
      connProfile.ports.length(2);
      connProfile.ports[0] = port0->getPortRef();
      connProfile.ports[1] = port1->getPortRef();

      // 接続する
      port0->getPortRef()->connect(connProfile);

      port0->activateInterfaces_public();
      port1->activateInterfaces_public();

      // ポート1のコンシューマ側からメソッドを呼び出すと、ポート0のプロバイダ側が意図どおり呼び出されるか？
      CPPUNIT_ASSERT(! pMyServiceImplA->is_hello_world_called());
      (*pMyServiceConsumerA)->hello_world();
      CPPUNIT_ASSERT(pMyServiceImplA->is_hello_world_called());
			
      // 切断する
      port0->getPortRef()->disconnect(connProfile.connector_id);
			
      // ポート0のコンシューマ側からメソッドを呼び出しを試みると、意図どおりに失敗するか？
      try
	{
	  CPPUNIT_ASSERT(! pMyServiceImplB->is_hello_world_called());
	  (*pMyServiceConsumerB)->hello_world();
							
	  CPPUNIT_FAIL("Couldn't catch no exceptions. Disconnection failed.");
	}
      catch(...)
	{
	  // Properly disconnected.
	}

//      delete port1;
      delete port0;
      delete pMyServiceConsumerA;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(pMyServiceImplB));
      delete pMyServiceImplB;
      delete pMyServiceConsumerB;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(pMyServiceImplA));
      delete pMyServiceImplA;
    }
    /*!
     * @brief registerProvider()メソッドのテスト
     * 
     */
    void test_registerProvider(void)
    {
      MyService_impl* pImpl0
	= new MyService_impl();

      MyService_impl* pImpl1
	= new MyService_impl();
			
			
      CorbaPortMock* port0 = new CorbaPortMock("name of port");
      bool ret;
      ret = port0->registerProvider("registerProvider0", "Generic", *pImpl0);
      CPPUNIT_ASSERT_EQUAL(true,ret);

      //既に登録してあるインスタンス名とオブジェクトを登録した場合、falseを返す。
      ret = port0->registerProvider("registerProvider0", "Generic", *pImpl0);
      CPPUNIT_ASSERT_EQUAL(false,ret);

      //既に登録してあるインスタンス名を登録した場合、falseを返す。
      ret = port0->registerProvider("registerProvider0", "Generic", *pImpl1);
      CPPUNIT_ASSERT_EQUAL(false,ret);
     
      //別なインスタンス名で既に登録してあるオブジェクトを登録した場合、trueを返す。
      //   falseを返すが登録される。
      ret = port0->registerProvider("registerProvider1", "Generic", *pImpl0);
      CPPUNIT_ASSERT_EQUAL(true,ret);

      //登録してないインスタンス名とオブジェクトを登録した場合、trueを返す。
      ret = port0->registerProvider("registerProvider2", "Generic", *pImpl1);
      CPPUNIT_ASSERT_EQUAL(true,ret);

      port0->deactivateInterfaces_public();

      delete port0;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(pImpl1));
      delete pImpl1;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(pImpl0));
      delete pImpl0;
    }
    /*!
     * @brief registerProvider()メソッドのテスト
     * 
     */
    void test_activateInterfaces_deactivateInterfaces(void)
    {
      // ポート0を構成する
      MyService_impl* pMyServiceImplA
	= new MyService_impl(); // will be deleted automatically
      RTC::CorbaConsumer<MyService>* pMyServiceConsumerB
	= new RTC::CorbaConsumer<MyService>(); // will be deleted automatically
			
      CorbaPortMock* port0 = new CorbaPortMock("name of port0");
      port0->registerProvider("MyServiceAA", "Generic", *pMyServiceImplA);
      port0->registerConsumer("MyServiceBB", "Generic", *pMyServiceConsumerB);

      // ポート1を構成する
      MyService_impl* pMyServiceImplB
	= new MyService_impl(); // will be deleted automatically
      RTC::CorbaConsumer<MyService>* pMyServiceConsumerA
	= new RTC::CorbaConsumer<MyService>(); // will be deleted automatically
			
      CorbaPortMock* port1 = new CorbaPortMock("name of port1");
      port1->registerProvider("MyServiceBB", "Generic", *pMyServiceImplB);
      port1->registerConsumer("MyServiceAA", "Generic", *pMyServiceConsumerA);
			
      // 接続プロファイルを構成する
      RTC::ConnectorProfile connProfile;
      connProfile.connector_id = "";
      connProfile.name = CORBA::string_dup("name of connector profile");
      connProfile.ports.length(2);
      connProfile.ports[0] = port0->getPortRef();
      connProfile.ports[1] = port1->getPortRef();

      // 接続する
      port0->getPortRef()->connect(connProfile);

      port0->activateInterfaces_public();
      port1->activateInterfaces_public();

      // ポート1のコンシューマ側からメソッドを呼び出すと、ポート0のプロバイダ側が意図どおり呼び出されるか？
      CPPUNIT_ASSERT(! pMyServiceImplA->is_hello_world_called());
      (*pMyServiceConsumerA)->hello_world();
      CPPUNIT_ASSERT(pMyServiceImplA->is_hello_world_called());
			
      // 切断する
      port0->getPortRef()->disconnect(connProfile.connector_id);

      //全てdeactivate obujectする
      port0->deactivateInterfaces_public();
      port1->deactivateInterfaces_public();

      port0->getPortRef()->connect(connProfile);

      //deactivate object が実行されてたため、エラーとなる。
      try
      {
         (*pMyServiceConsumerA)->hello_world();
	  CPPUNIT_FAIL("Couldn't catch no exceptions. Disconnection failed.");
      }
      catch(...)
      {
      }
			
      // 切断する
      port0->getPortRef()->disconnect(connProfile.connector_id);

      port0->activateInterfaces_public();
      port1->activateInterfaces_public();

      port0->getPortRef()->connect(connProfile);

      //activate object が実行されてたため、正常に動作する 。
      (*pMyServiceConsumerA)->hello_world();

      // 切断する
      port0->getPortRef()->disconnect(connProfile.connector_id);

//      delete port1;
      delete port0;
      delete pMyServiceConsumerA;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(pMyServiceImplB));
      delete pMyServiceImplB;
      delete pMyServiceConsumerB;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(pMyServiceImplA));
      delete pMyServiceImplA;
    }
    
  };
}; // namespace CorbaPort

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(CorbaPort::CorbaPortTests);

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
#endif // CorbaPort_cpp
