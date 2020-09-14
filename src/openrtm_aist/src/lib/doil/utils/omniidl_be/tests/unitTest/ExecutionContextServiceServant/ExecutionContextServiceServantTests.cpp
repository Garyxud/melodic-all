// -*- C++ -*-
/*!
 * @file   ExecutionContextServiceServantTests.cpp
 * @brief  ExecutionContextServiceServant test class
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

#ifndef ExecutionContextServiceServant_cpp
#define ExecutionContextServiceServant_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

/*!
 * @class ExecutionContextServiceServantTests class
 * @brief ExecutionContextServiceServant test
 */
namespace ExecutionContextServiceServant
{
  class ExecutionContextServiceServantTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ExecutionContextServiceServantTests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    ExecutionContextServiceServantTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~ExecutionContextServiceServantTests()
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
}; // namespace ExecutionContextServiceServant

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ExecutionContextServiceServant::ExecutionContextServiceServantTests);

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
#endif // ExecutionContextServiceServant_cpp
