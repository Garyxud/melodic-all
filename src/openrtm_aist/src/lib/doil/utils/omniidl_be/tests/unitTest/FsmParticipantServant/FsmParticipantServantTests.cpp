// -*- C++ -*-
/*!
 * @file   FsmParticipantServantTests.cpp
 * @brief  FsmParticipantServant test class
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

#ifndef FsmParticipantServant_cpp
#define FsmParticipantServant_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

/*!
 * @class FsmParticipantServantTests class
 * @brief FsmParticipantServant test
 */
namespace FsmParticipantServant
{
  class FsmParticipantServantTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(FsmParticipantServantTests);
    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    FsmParticipantServantTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~FsmParticipantServantTests()
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
}; // namespace FsmParticipantServant

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(FsmParticipantServant::FsmParticipantServantTests);

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
#endif // FsmParticipantServant_cpp
