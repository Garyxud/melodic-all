// -*- C++ -*-
/*!
 * @file   SdoServiceTests.cpp
 * @brief  SdoService test class
 * @date   $Date: 2007/12/20 07:50:16 $
 * @author Shinji Kurihara
 *         Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id$
 *
 */

/*
 * $Log: SdoServiceTests.cpp,v $
 * Revision 1.1  2007/12/20 07:50:16  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2006/11/27 08:38:37  n-ando
 * TestSuites are devided into each directory.
 *
 *
 */

#ifndef SdoService_cpp
#define SdoService_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

//#include <rtm/SdoService.h>

/*!
 * @class SdoServiceTests class
 * @brief SdoService test
 */
namespace SdoService
{
  //  using namespace SDOPackage;
  using namespace std;

  class SdoServiceTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(SdoServiceTests);
    CPPUNIT_TEST(test_getProfile);
    CPPUNIT_TEST(test_set_getName);
    CPPUNIT_TEST(test_set_getInterfaceType);
    CPPUNIT_TEST(test_set_getIdlDefinition);
    CPPUNIT_TEST(test_set_getProperties);
    CPPUNIT_TEST(test_set_getServiceRef);
    CPPUNIT_TEST_SUITE_END();
  
  private:
    //    SDOServiceProfile* m_pSSP;
  public:
  
    /*!
     * @brief Constructor
     */
    SdoServiceTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~SdoServiceTests()
    {
    }
  
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
      //      m_pSSP = new SDOServiceProfile();
    }
    
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    {
      //      delete m_pSSP; 
    }
    /* tests for */
    void test_getProfile() {
    }
    
    /* tests for */
    void test_set_getName() {
    }
    
    /* tests for */
    void test_set_getInterfaceType() {
    }
    
    /* tests for */
    void test_set_getIdlDefinition() {
    }
    
    /* tests for */
    void test_set_getProperties() {
    }
    
    /* tests for */
    void test_set_getServiceRef() {
    }
  };
}; // namespace SdoService

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(SdoService::SdoServiceTests);

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
#endif // SdoService_cpp
