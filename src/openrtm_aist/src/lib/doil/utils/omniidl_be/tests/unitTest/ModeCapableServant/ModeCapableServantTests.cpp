// -*- C++ -*-
/*!
 * @file   ModeCapableServantTests.cpp
 * @brief  ModeCapableServant test class
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

#ifndef ModeCapableServant_cpp
#define ModeCapableServant_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

/*!
 * @class ModeCapableServantTests class
 * @brief ModeCapableServant test
 */
namespace ModeCapableServant
{
  class ModeCapableServantTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ModeCapableServantTests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    ModeCapableServantTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~ModeCapableServantTests()
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
}; // namespace ModeCapableServant

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ModeCapableServant::ModeCapableServantTests);

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
#endif // ModeCapableServant_cpp
