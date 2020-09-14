// -*- C++ -*-
/*!
 * @file   SDOSystemElementServantTests.cpp
 * @brief  SDOSystemElementServant test class
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

#ifndef SDOSystemElementServant_cpp
#define SDOSystemElementServant_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

/*!
 * @class SDOSystemElementServantTests class
 * @brief SDOSystemElementServant test
 */
namespace SDOSystemElementServant
{
  class SDOSystemElementServantTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(SDOSystemElementServantTests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    SDOSystemElementServantTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~SDOSystemElementServantTests()
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
}; // namespace SDOSystemElementServant

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(SDOSystemElementServant::SDOSystemElementServantTests);

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
#endif // SDOSystemElementServant_cpp
