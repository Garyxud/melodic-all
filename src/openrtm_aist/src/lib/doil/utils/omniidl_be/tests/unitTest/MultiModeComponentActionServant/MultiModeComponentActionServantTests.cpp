// -*- C++ -*-
/*!
 * @file   MultiModeComponentActionServantTests.cpp
 * @brief  MultiModeComponentActionServant test class
 * @date   $Date$ 
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id$ 
 *
 */

/*
 * $Log$
 *
 */

#ifndef MultiModeComponentActionServant_cpp
#define MultiModeComponentActionServant_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

/*!
 * @class MultiModeComponentActionServantTests class
 * @brief MultiModeComponentActionServant test
 */
namespace MultiModeComponentActionServant
{
  class MultiModeComponentActionServantTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(MultiModeComponentActionServantTests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    MultiModeComponentActionServantTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~MultiModeComponentActionServantTests()
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
  
    /* test case */
    void test_case0()
    {
      CPPUNIT_FAIL("Automatic failue.");
    }
  };
}; // namespace MultiModeComponentActionServant

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(MultiModeComponentActionServant::MultiModeComponentActionServantTests);

#ifdef LOCAL_MAIN
int main(int argc, char* argv[])
{
    CppUnit::TextUi::TestRunner runner;
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry().makeTest());
    CppUnit::Outputter* outputter = 
      new CppUnit::TextOutputter(&runner.result(), std::cout);
    runner.setOutputter(outputter);
    bool retcode = runner.run();
    return !retcode;
}
#endif // MAIN
#endif // MultiModeComponentActionServant_cpp
