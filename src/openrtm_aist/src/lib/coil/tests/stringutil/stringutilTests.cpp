// -*- C++ -*-
/*!
 * @file   stringutilTests.cpp
 * @brief  stringutil test class
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

#ifndef stringutil_cpp
#define stringutil_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/stringutil.h>

/*!
 * @class stringutilTests class
 * @brief stringutil test
 */
namespace stringutil
{
  class stringutilTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(stringutilTests);
    CPPUNIT_TEST(test_toUpper);
    CPPUNIT_TEST(test_toLower);
    CPPUNIT_TEST(test_togetlinePortable);
    CPPUNIT_TEST(test_isEscaped);
    CPPUNIT_TEST(test_escape);
    CPPUNIT_TEST(test_unescape);
    CPPUNIT_TEST(test_eraseHeadBlank);
    CPPUNIT_TEST(test_eraseTailBlank);
    CPPUNIT_TEST(test_replaceString);
    CPPUNIT_TEST(test_split);
    CPPUNIT_TEST(test_toBool);
    CPPUNIT_TEST(test_isAbsolutePath);
    CPPUNIT_TEST(test_isURL);
    CPPUNIT_TEST(test_otos);
    CPPUNIT_TEST(test_stringTo);
    CPPUNIT_TEST(test_unique_sv);
    CPPUNIT_TEST(test_flatten);
    CPPUNIT_TEST(test_toArgv);
    CPPUNIT_TEST(test_sprintf);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    stringutilTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~stringutilTests()
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
    void test_toUpper()
    {
    }
    void test_toLower()
    {
    }
    void test_togetlinePortable()
    {
    }
    void test_isEscaped()
    {
    }
    void test_escape()
    {
    }
    void test_unescape()
    {
    }
    void test_eraseHeadBlank()
    {
    }
    void test_eraseTailBlank()
    {
    }
    void test_replaceString()
    {
    }
    void test_split()
    {
    }
    void test_toBool()
    {
    }
    void test_isAbsolutePath()
    {
    }
    void test_isURL()
    {
    }
    void test_otos()
    {
    }
    void test_stringTo()
    {
    }
    void test_unique_sv()
    {
    }
    void test_flatten()
    {
    }
    void test_toArgv()
    {
    }
    void test_sprintf()
    {
    }

  };
}; // namespace stringutil

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(stringutil::stringutilTests);

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
#endif // stringutil_cpp
