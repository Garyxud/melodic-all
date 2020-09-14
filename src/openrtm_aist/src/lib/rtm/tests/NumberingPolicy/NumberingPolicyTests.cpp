// -*- C++ -*-
/*!
 * @file   NumberingPolicyTests.cpp
 * @brief  NumberingPolicy test class
 * @date   $Date: 2008/05/02 13:44:01 $
 *
 * $Id: NumberingPolicyTests.cpp,v 1.1 2008/05/02 13:44:01 arafune Exp $
 *
 */

/*
 * $Log: NumberingPolicyTests.cpp,v $
 * Revision 1.1  2008/05/02 13:44:01  arafune
 * The first commitment.
 *
 *
 */

#ifndef NumberingPolicy_cpp
#define NumberingPolicy_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/NumberingPolicy.h>

namespace Tests
{
  class NumberingPolicyTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(NumberingPolicyTests);
    CPPUNIT_TEST(test_onCreate_and_onDelete);
    CPPUNIT_TEST_SUITE_END();

  private:
	
  public:
    /*!
     * @brief Constructor
     */
    NumberingPolicyTests()
    {
    }
		
    /*!
     * @brief Destructor
     */
    ~NumberingPolicyTests()
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
     * @brief DefaultNumberingPolicy::onCreate()とDefaultNumberingPolicy::onDelete()のテスト
     * 
     * - onCreate()は意図どおりに名称を生成して返すか？
     * - onDelete()で正しく登録解除されるか？
     * - 登録解除後に、onCreate()で登録した場合、解除されたオブジェクトの番号が再利用されるか？
     */
    void test_onCreate_and_onDelete()
    {

      std::string object1 = "apple";
      std::string object2 = "orange";
      std::string object3 = "banana";
			
      std::auto_ptr<NumberingPolicy> policy(new DefaultNumberingPolicy());
			
      // onCreate()は意図どおりに名称を生成して返すか？
      CPPUNIT_ASSERT_EQUAL(std::string("0"), policy->onCreate(&object1));
      CPPUNIT_ASSERT_EQUAL(std::string("1"), policy->onCreate(&object2));
      CPPUNIT_ASSERT_EQUAL(std::string("2"), policy->onCreate(&object3));
			
      // onDeleteで、いったん登録解除する
      policy->onDelete(&object1);
      policy->onDelete(&object2);
			
      // 登録順を入れ換えて再度onCreateを呼び出した場合、意図どおりの名称がアサインされるか？
      // （登録解除後に、onCreate()で登録した場合、解除されたオブジェクトの番号が再利用されるか？）
      CPPUNIT_ASSERT_EQUAL(std::string("0"), policy->onCreate(&object2));
      CPPUNIT_ASSERT_EQUAL(std::string("1"), policy->onCreate(&object1));
    }
		
  };
}; // namespace NumberingPolicy

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Tests::NumberingPolicyTests);

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
#endif // NumberingPolicy_cpp
