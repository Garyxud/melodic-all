// -*- C++ -*-
/*!
 * @file   LightweightRTObjectServantTests.cpp
 * @brief  LightweightRTObjectServant test class
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

#ifndef LightweightRTObjectServant_cpp
#define LightweightRTObjectServant_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

/*!
 * @class LightweightRTObjectServantTests class
 * @brief LightweightRTObjectServant test
 */
namespace LightweightRTObjectServant
{
  class LightweightRTObjectServantTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(LightweightRTObjectServantTests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    LightweightRTObjectServantTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~LightweightRTObjectServantTests()
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
}; // namespace LightweightRTObjectServant

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(LightweightRTObjectServant::LightweightRTObjectServantTests);

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
#endif // LightweightRTObjectServant_cpp
