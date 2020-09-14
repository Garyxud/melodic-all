// -*- C++ -*-
/*!
 * @file   ECFactoryTests.cpp
 * @brief  ECFactory test class
 * @date   $Date: 2008/04/15 08:38:41 $
 *
 * $Id: ECFactoryTests.cpp,v 1.1 2008/04/15 08:38:41 arafune Exp $
 *
 */

/*
 * $Log: ECFactoryTests.cpp,v $
 * Revision 1.1  2008/04/15 08:38:41  arafune
 * The first commitment.
 *
 *
 */

#ifndef ECFactory_cpp
#define ECFactory_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/ExtTrigExecutionContext.h>
#include <rtm/PeriodicExecutionContext.h>
#include <rtm/ECFactory.h>

/*!
 * @class ECFactoryTests class
 * @brief ECFactory test
 */
namespace ECFactory
{
  class ECFactoryTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ECFactoryTests);
    CPPUNIT_TEST(test_name);
    CPPUNIT_TEST(test_create_and_destroy);
    CPPUNIT_TEST_SUITE_END();
	
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
	
  public:
    /*!
     * @brief Constructor
     */
    ECFactoryTests()
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
    virtual ~ECFactoryTests()
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
     * @brief name()メソッドのテスト
     * 
     * - コンストラクタで指定した名称を、name()メソッドで正しく取得できるか？
     */
    void test_name()
    {
      std::string name = "name of execution context";
			
      std::auto_ptr<RTC::ECFactoryBase> factory(
		new RTC::ECFactoryCXX(
				      name.c_str(),
				      RTC::ECCreate<RTC::PeriodicExecutionContext>,
				      RTC::ECDelete<RTC::PeriodicExecutionContext>));
			
      // コンストラクタで指定した名称を、name()メソッドで正しく取得できるか？
      CPPUNIT_ASSERT_EQUAL(name, std::string(factory->name()));
    }
		
    /*!
     * @brief create()メソッドとdestroy()メソッドのテスト
     * 
     * - create()呼出しにより、所定のExcecutionContextのインスタンスが生成されるか？
     * - destroy()呼出しにより、所定のExecutionContextインスタンスが削除されるか？
     */
    void test_create_and_destroy()
    {
      std::auto_ptr<RTC::ECFactoryBase> factory(
		new RTC::ECFactoryCXX(
				      "name of execution context",
				      RTC::ECCreate<RTC::PeriodicExecutionContext>,
				      RTC::ECDelete<RTC::PeriodicExecutionContext>));
			
      // create()呼出しにより、所定のExcecutionContextのインスタンスが生成されるか？
      RTC::ExecutionContextBase* ec = factory->create();
      CPPUNIT_ASSERT(dynamic_cast<RTC::PeriodicExecutionContext*>(ec) != 0);
			
      // destroy()呼出しにより、所定のExecutionContextインスタンスが削除されるか？
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(ec));
      factory->destroy(ec);
      CPPUNIT_ASSERT(dynamic_cast<RTC::PeriodicExecutionContext*>(ec) == 0);
    }
		
  };
}; // namespace ECFactory

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ECFactory::ECFactoryTests);

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
#endif // ECFactory_cpp
