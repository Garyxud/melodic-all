// -*- C++ -*-
/*!
 * @file   MultiModeObjectServantTests.cpp
 * @brief  MultiModeObjectServant test class
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

#ifndef MultiModeObjectServant_cpp
#define MultiModeObjectServant_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

/*!
 * @class MultiModeObjectServantTests class
 * @brief MultiModeObjectServant test
 */
namespace MultiModeObjectServant
{
  class MultiModeObjectServantTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(MultiModeObjectServantTests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    MultiModeObjectServantTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~MultiModeObjectServantTests()
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
}; // namespace MultiModeObjectServant

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(MultiModeObjectServant::MultiModeObjectServantTests);

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
#endif // MultiModeObjectServant_cpp
