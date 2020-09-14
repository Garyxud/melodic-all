// -*- C++ -*-
/*!
 * @file   FileTests.cpp
 * @brief  File test class
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

#ifndef File_cpp
#define File_cpp

#include <string.h>
#include <string>
#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <coil/File.h>

static const std::string Str01("/usr/include/coil");
static const std::string Str02("/usr/include/coil/");
static const std::string Str03("coil");
static const std::string Str04("/");
static const std::string Str05(".");
static const std::string Str06("..");
static const std::string Str07("/usr");

/*!
 * @class FileTests class
 * @brief File test
 */
namespace File
{
  class FileTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(FileTests);
    CPPUNIT_TEST(test_dirname_01);
    CPPUNIT_TEST(test_basename_01);
    CPPUNIT_TEST(test_dirname_02);
    CPPUNIT_TEST(test_basename_02);
    CPPUNIT_TEST(test_dirname_03);
    CPPUNIT_TEST(test_basename_03);
    CPPUNIT_TEST(test_dirname_04);
    CPPUNIT_TEST(test_basename_04);
    CPPUNIT_TEST(test_dirname_05);
    CPPUNIT_TEST(test_basename_05);
    CPPUNIT_TEST(test_dirname_06);
    CPPUNIT_TEST(test_basename_06);
    CPPUNIT_TEST(test_dirname_07);
    CPPUNIT_TEST(test_basename_07);
    CPPUNIT_TEST_SUITE_END();
  
  private:

  public:
  
    /*!
     * @brief Constructor
     */
    FileTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~FileTests()
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
    void test_dirname_01()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str01.c_str());

      std::string result = coil::dirname(buff);
      std::string expected("/usr/include");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_basename_01()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str01.c_str());

      std::string result = coil::basename(buff);
      std::string expected("coil");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_dirname_02()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str02.c_str());

      std::string result = coil::dirname(buff);
      std::string expected("/usr/include");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_basename_02()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str02.c_str());

      std::string result = coil::basename(buff);
      std::string expected("coil");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_dirname_03()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str03.c_str());

      std::string result = coil::dirname(buff);
      std::string expected(".");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_basename_03()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str03.c_str());

      std::string result = coil::basename(buff);
      std::string expected("coil");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_dirname_04()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str04.c_str());

      std::string result = coil::dirname(buff);
      std::string expected("/");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_basename_04()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str04.c_str());

      std::string result = coil::basename(buff);
      std::string expected("/");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_dirname_05()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str05.c_str());

      std::string result = coil::dirname(buff);
      std::string expected(".");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_basename_05()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str05.c_str());

      std::string result = coil::basename(buff);
      std::string expected(".");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_dirname_06()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str06.c_str());

      std::string result = coil::dirname(buff);
      std::string expected(".");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_basename_06()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str06.c_str());

      std::string result = coil::basename(buff);
      std::string expected("..");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_dirname_07()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str07.c_str());

      std::string result = coil::dirname(buff);
      std::string expected("/");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

    void test_basename_07()
    {
      char buff[BUFSIZ];
      strcpy(buff, Str07.c_str());

      std::string result = coil::basename(buff);
      std::string expected("usr");
      CPPUNIT_ASSERT_EQUAL(expected, result);
    }

  };
}; // namespace File

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(File::FileTests);

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
#endif // File_cpp
