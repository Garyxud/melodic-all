// -*- C++ -*-
/*!
 * @file   FsmServiceServantTests.cpp
 * @brief  FsmServiceServant test class
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

#ifndef FsmServiceServant_cpp
#define FsmServiceServant_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

/*!
 * @class FsmServiceServantTests class
 * @brief FsmServiceServant test
 */
namespace FsmServiceServant
{
  class FsmServiceServantTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(FsmServiceServantTests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    FsmServiceServantTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~FsmServiceServantTests()
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
}; // namespace FsmServiceServant

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(FsmServiceServant::FsmServiceServantTests);

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
#endif // FsmServiceServant_cpp
