// -*- C++ -*-
/*!
 * @file   CORBA_IORUtilTests.cpp
 * @brief  CORBA_IORUtil test class
 * @date   $Date: 2010/02/09 04:32:00 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id: CORBA_IORUtilTests.cpp 1758 2010-01-22 14:04:54Z hakuta $
 *
 */

/*
 * $Log: CORBA_IORUtilTests.cpp,v $
 */

#ifndef CORBA_IORUtil_cpp
#define CORBA_IORUtil_cpp

#include <iostream>

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/CORBA_IORUtil.h>


/*!
 * @class CORBA_IORUtilTests class
 * @brief CORBA_IORUtil test
 */

namespace CORBA_IORUtil
{

  class CORBA_IORUtilTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(CORBA_IORUtilTests);

    CPPUNIT_TEST(test_case0);

    CPPUNIT_TEST_SUITE_END();
  
  private:

  public:
  
    /*!
     * @brief Constructor
     */
    CORBA_IORUtilTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~CORBA_IORUtilTests()
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
     * @brief toIOR(), toString(), replaceEndpoint(), formatIORinfo() メソッドテスト
     */
    void test_case0()
    {
      bool bret;
      IOP::IOR ior;
      std::string str_ior("IOR:010000001400000049444c3a52544d2f4d616e616765723a312e300001000000000000006000000001010200100000003139322e3136382e3130302e323430007acf0000070000006d616e61676572000200000000000000080000000100000000545441010000001c00000001000000010001000100000001000105090101000100000009010100");

      std::string str_ior2;
      char* str_ior3 = 0;
      std::string str_ior_ret;

      // toIOR(), return false check
      bret = CORBA_IORUtil::toIOR(str_ior3, ior);
      CPPUNIT_ASSERT( !bret );

      // toIOR(), return false check
      bret = CORBA_IORUtil::toIOR("IOR", ior);
      CPPUNIT_ASSERT( !bret );

      // toIOR(), return false check
      bret = CORBA_IORUtil::toIOR("ior:", ior);
      CPPUNIT_ASSERT( !bret );

      // toIOR(), return true check
      bret = CORBA_IORUtil::toIOR(str_ior.c_str(), ior);
      CPPUNIT_ASSERT( bret );

      // toString()
      bret = CORBA_IORUtil::toString(ior, str_ior2);
      CPPUNIT_ASSERT( bret );
      CPPUNIT_ASSERT_EQUAL(str_ior, str_ior2);

      // replaceEndpoint()
      bret = CORBA_IORUtil::replaceEndpoint(str_ior, "127.0.0.1");
      CPPUNIT_ASSERT( bret );

      // formatIORinfo()
      str_ior_ret = CORBA_IORUtil::formatIORinfo(str_ior3);
      CPPUNIT_ASSERT( str_ior_ret.size() > 0 );
    }

  };
}; // namespace CORBA_IORUtil

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(CORBA_IORUtil::CORBA_IORUtilTests);

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
#endif // CORBA_IORUtil_cpp
