// -*- C++ -*-
/*!
 * @file   ExecutionContextServantTests.cpp
 * @brief  ExecutionContextServant test class
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

#ifndef ExecutionContextServant_cpp
#define ExecutionContextServant_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

/*!
 * @class ExecutionContextServantTests class
 * @brief ExecutionContextServant test
 */
namespace ExecutionContextServant
{
  class ExecutionContextServantTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ExecutionContextServantTests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    ExecutionContextServantTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~ExecutionContextServantTests()
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
}; // namespace ExecutionContextServant

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ExecutionContextServant::ExecutionContextServantTests);

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
#endif // ExecutionContextServant_cpp
