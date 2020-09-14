// -*- C++ -*-
/*!
 * @file   CorbaConsumerTests.cpp
 * @brief  CorbaConsumer test class
 * @date   $Date: 2008/02/24 09:05:52 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id$
 *
 */

/*
 * $Log: CorbaConsumerTests.cpp,v $
 * Revision 1.2  2008/02/24 09:05:52  arafune
 * Added some new tests.
 *
 * Revision 1.1  2007/12/20 07:50:17  arafune
 * *** empty log message ***
 *
 * Revision 1.2  2007/01/12 14:48:30  n-ando
 * The name of class to be tested was changed from Consumer to CorbaConsumer.
 *
 * Revision 1.1  2006/11/27 08:30:48  n-ando
 * TestSuites are devided into each directory.
 *
 *
 */

#ifndef CorbaConsumer_cpp
#define CorbaConsumer_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/config_rtc.h>
#include "HelloSkel.h"
#include <rtm/CorbaConsumer.h>

/*!
 * @class CorbaConsumerTests class
 * @brief CorbaConsumer test
 */
namespace CorbaConsumer
{
  class hello_impl
    : virtual public POA_hello,
      virtual public PortableServer::RefCountServantBase
  {
  public:
    hello_impl() : m_invokedCount(0) {};
    virtual ~hello_impl() {};
		
    void hello_world()
    {
      ++m_invokedCount;
    }
		
  public:
    int m_invokedCount;
  };
	
  class CorbaConsumerTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(CorbaConsumerTests);
    CPPUNIT_TEST(test_setObject_and__ptr);
    CPPUNIT_TEST(test_copy_constructor);
    CPPUNIT_TEST(test_pointer_operator);
    CPPUNIT_TEST(test_substitute_operator);
    CPPUNIT_TEST(test_releaseObject);
    CPPUNIT_TEST_SUITE_END();
	
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
	
  public:
	
    /*!
     * @brief Constructor
     */
    CorbaConsumerTests()
    {
      int argc = 0;
      char** argv = 0;
			
      m_pORB = CORBA::ORB_init(argc, argv);
      m_pPOA = PortableServer::POA::_narrow(
					    m_pORB->resolve_initial_references("RootPOA"));
      m_pPOA->the_POAManager()->activate();
    }
		
    /*!
     * @brief Destructor
     */
    ~CorbaConsumerTests()
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
     * @brief setObject()メソッドと_ptr()メソッドのテスト
     * 
     * - 生成したサーバントをConsumerにセットした後、Consumerを通してサーバントのメソッドを正しく呼び出せるか？
     */
    void test_setObject_and__ptr()
    {
      // Consumerに割り当てるオブジェクトを生成する
      hello_impl* helloImpl = new hello_impl();
      PortableServer::ObjectId_var objId = m_pPOA->activate_object(helloImpl);
			
      // 生成したオブジェクトをConsumerにセットする
      RTC::CorbaConsumer<hello> consumer;
      consumer.setObject(m_pPOA->id_to_reference(objId));
			
      // Consumerに割り当てたオブジェクトのメソッドを正しく呼び出せるか？
      CPPUNIT_ASSERT_EQUAL(0, helloImpl->m_invokedCount);
      consumer._ptr()->hello_world();
      CPPUNIT_ASSERT_EQUAL(1, helloImpl->m_invokedCount);

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(helloImpl));
      delete helloImpl;
    }
		
    /*!
     * @brief コピーコンストラクタのテスト
     * 
     * - 既存のConsumerオブジェクトを元に、別の新たなConsumerをコピーコンストラクトし、
     * その新たなConsumerオブジェクトに対してメソッドを正しく呼び出せるか？
     */
    void test_copy_constructor()
    {
      // Consumerに割り当てるオブジェクトを生成する
      hello_impl* helloImpl = new hello_impl();
      PortableServer::ObjectId_var objId = m_pPOA->activate_object(helloImpl);
			
      // 生成したオブジェクトをConsumerにセットする
      RTC::CorbaConsumer<hello> consumer;
      consumer.setObject(m_pPOA->id_to_reference(objId));
			
      // 作成したConsumerオブジェクトを元に、別の新たなConsumerをコピーコンストラクトする
      RTC::CorbaConsumer<hello> consumerNew(consumer);
			
      // 新たに作成したConsumerに対して、メソッドを正しく呼び出せるか？
      CPPUNIT_ASSERT_EQUAL(0, helloImpl->m_invokedCount);
      consumerNew._ptr()->hello_world();
      CPPUNIT_ASSERT_EQUAL(1, helloImpl->m_invokedCount);

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(helloImpl));
      delete helloImpl;
    }
		
    /*!
     * @brief ポインタ演算子(operator->())のテスト
     * 
     * - 生成したサーバントをConsumerにセットした後、
     * Consumerのポインタ演算子を通してサーバントのメソッドを正しく呼び出せるか？
     */
    void test_pointer_operator()
    {
      // Consumerに割り当てるオブジェクトを生成する
      hello_impl* helloImpl = new hello_impl();
      PortableServer::ObjectId_var objId = m_pPOA->activate_object(helloImpl);
			
      // 生成したオブジェクトをConsumerにセットする
      RTC::CorbaConsumer<hello> consumer;
      consumer.setObject(m_pPOA->id_to_reference(objId));
			
      // ポインタ演算子(->)を用いて、Consumerに割り当てたオブジェクトのメソッドを正しく呼び出せるか？
      CPPUNIT_ASSERT_EQUAL(0, helloImpl->m_invokedCount);
      consumer->hello_world();
      CPPUNIT_ASSERT_EQUAL(1, helloImpl->m_invokedCount);

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(helloImpl));
      delete helloImpl;
    }
		
    void test_substitute_operator()
    {
      // Consumerに割り当てるオブジェクトを生成する
      hello_impl* helloImpl = new hello_impl();
      PortableServer::ObjectId_var objId = m_pPOA->activate_object(helloImpl);
			
      // 生成したオブジェクトをConsumerにセットする
      RTC::CorbaConsumer<hello> consumer;
      consumer.setObject(m_pPOA->id_to_reference(objId));
			
      // 作成したConsumerオブジェクトを元に、別の新たなConsumerをコピーコンストラクトする
      RTC::CorbaConsumer<hello> consumerNew = consumer;
			
      // 新たに作成したConsumerに対して、メソッドを正しく呼び出せるか？
      CPPUNIT_ASSERT_EQUAL(0, helloImpl->m_invokedCount);
      consumerNew._ptr()->hello_world();
      CPPUNIT_ASSERT_EQUAL(1, helloImpl->m_invokedCount);

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(helloImpl));
      delete helloImpl;
    }
		
    /*!
     * @brief releaseObject()メソッドのテスト
     * 
     * - releaseObject()呼出によって、保持されているオブジェクト参照が正しくクリアされるか？
     */
    void test_releaseObject()
    {
      // Consumerに割り当てるオブジェクトを生成する
      hello_impl* helloImpl = new hello_impl();
      PortableServer::ObjectId_var objId = m_pPOA->activate_object(helloImpl);
			
      // 生成したオブジェクトをConsumerにセットする
      RTC::CorbaConsumer<hello> consumer;
      consumer.setObject(m_pPOA->id_to_reference(objId));

      // この時点では、オブジェクト参照は保持されているはず
      CPPUNIT_ASSERT(! CORBA::is_nil(consumer._ptr()));
			
      // releaseObject()呼出によって、保持されているオブジェクト参照が正しくクリアされるか？
      consumer.releaseObject();
      CPPUNIT_ASSERT(CORBA::is_nil(consumer._ptr()));

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(helloImpl));
      delete helloImpl;
    }
		
  };
}; // namespace CorbaConsumer

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(CorbaConsumer::CorbaConsumerTests);

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
#endif // CorbaConsumer_cpp
