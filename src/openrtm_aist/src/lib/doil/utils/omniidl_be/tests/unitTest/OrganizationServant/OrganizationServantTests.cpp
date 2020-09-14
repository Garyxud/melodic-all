// -*- C++ -*-
/*!
 * @file   OrganizationServantTests.cpp
 * @brief  OrganizationServant test class
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

#ifndef OrganizationServant_cpp
#define OrganizationServant_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

/*!
 * @class OrganizationServantTests class
 * @brief OrganizationServant test
 */
namespace OrganizationServant
{
  class OrganizationServantTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(OrganizationServantTests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    OrganizationServantTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~OrganizationServantTests()
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
}; // namespace OrganizationServant

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(OrganizationServant::OrganizationServantTests);

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
#endif // OrganizationServant_cpp
