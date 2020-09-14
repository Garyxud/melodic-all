// -*- C++ -*-
/*!
 * @file   DynamicLibTests.cpp
 * @brief  DynamicLib test class
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

#ifndef DynamicLib_cpp
#define DynamicLib_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/DynamicLib.h>

/*!
 * @class DynamicLibTests class
 * @brief DynamicLib test
 */
namespace DynamicLib
{
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
// #error WIN32!!
  const char * LibName = "DynamicLibTestDll.dll";
//  const char * LibName = "coil.dll";
  const char * SymbolName = "ForExternTest";

#else
// #error POSIX.
//  const char * LibName = "libPluginC.so";
  const char * LibName = ".libs/libDynamicLib.so";
//  const char * LibName = "libcoil.so.0";
//  const char * SymbolName = "svc_run";
  const char * SymbolName = "ForExternTest";
//  const char * SymbolName = "PluginCInit";

#endif

  class DynamicLibTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(DynamicLibTests);
    CPPUNIT_TEST(test_DynamicLib_1);
    CPPUNIT_TEST(test_DynamicLib_2);
    CPPUNIT_TEST(test_DynamicLib_3);
    CPPUNIT_TEST(test_DynamicLib_4);
    CPPUNIT_TEST(test_DynamicLib_open_failure);
    CPPUNIT_TEST(test_DynamicLib_open_and_close);
    CPPUNIT_TEST(test_DynamicLib_symbol_failure);
    CPPUNIT_TEST(test_DynamicLib_symbol);
    CPPUNIT_TEST(test_DynamicLib_error);
    CPPUNIT_TEST_SUITE_END();
  
  private:
    coil::DynamicLib * dl;

  public:
  
    /*!
     * @brief Constructor
     */
    DynamicLibTests()
    {

    }
    
    /*!
     * @brief Destructor
     */
    ~DynamicLibTests()
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

     /*!
     * @brief コンストラクタその１
     */
	void test_DynamicLib_1()
	{
	  /* 例外が起きなければＯＫとする */
      coil::DynamicLib * dl1 = new coil::DynamicLib(1);
      int result = dl1->open(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
      CPPUNIT_ASSERT_EQUAL(0, result);
      delete dl1;
	}

     /*!
     * @brief コンストラクタその２
     */
	void test_DynamicLib_2()
	{
	  /* 例外が起きなければＯＫとする */
      coil::DynamicLib * dl1 = new coil::DynamicLib(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
	  delete dl1;
	}

     /*!
     * @brief コンストラクタその３
     */
	void test_DynamicLib_3()
	{
	  /* 例外が起きなければＯＫとする */
	  coil::DynamicLib dl1(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
      coil::DynamicLib dl2(dl1);   // <--- ここで、std::bad_allocに落ちる Kz.080930 ← fix 081006.
	}

     /*!
     * @brief コンストラクタその４
     */
	void test_DynamicLib_4()
	{
	  /* 例外が起きなければＯＫとする */
      coil::DynamicLib * dl1 = new coil::DynamicLib(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
       coil::DynamicLib dl2 = *dl1;   // <--- ここで、std::bad_allocに落ちる Kz.080930 ← fix 081006.
	  delete dl1;
	}

     /*!
     * @brief Try to open nonExistence.
     */
	void test_DynamicLib_open_failure()
	{
      coil::DynamicLib * dl1 = new coil::DynamicLib(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
      int result = dl1->open("Hoge", COIL_DEFAULT_DYNLIB_MODE, 1);
      CPPUNIT_ASSERT_EQUAL(-1, result);
      delete dl1;
	}

     /*!
     * @brief Open and Close
     */
	void test_DynamicLib_open_and_close()
	{
      coil::DynamicLib * dl1 = new coil::DynamicLib(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
      int result = dl1->open(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
      CPPUNIT_ASSERT_EQUAL(0, result);

      dl1->close();
      
      delete dl1;
	}

     /*!
     * @brief do symbol
     * note 存在しないシンボルを読み出す。
     */
	void test_DynamicLib_symbol_failure()
	{
      coil::DynamicLib * dl1 = new coil::DynamicLib(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
      dl1->symbol("Hoge");

      int result = dl1->open(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
      CPPUNIT_ASSERT_EQUAL(0, result);

      char * p = (char *)dl1->symbol("HogeHoge");
//      std::cout << "Error : " << dl1->error() << "." << std::endl ;
      CPPUNIT_ASSERT(0 == p);

      dl1->close();
      delete dl1;
	}

     /*!
     * @brief do symbol
     * note 存在するシンボルを読み出す。
     */
	void test_DynamicLib_symbol()
	{
      coil::DynamicLib * dl1 = new coil::DynamicLib(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
      dl1->symbol("Hoge");

      int result = dl1->open(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
      CPPUNIT_ASSERT_EQUAL(0, result);

      typedef int (*exec)();

      exec func = NULL;
      func = (exec)dl1->symbol(SymbolName);

      CPPUNIT_ASSERT(func);

      int ic = func();

      CPPUNIT_ASSERT_EQUAL((int)0xdeadbeef, ic);

      dl1->close();
      delete dl1;
	}

     /*!
     * @brief do error
     * note エラー発生時にerror()が非０を返すこと
     */
	void test_DynamicLib_error()
	{
//      coil::DynamicLib * dl1 = new coil::DynamicLib(1);  // ↓どちらでも例外は起きない
      coil::DynamicLib * dl1 = new coil::DynamicLib(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
      dl1->error();

      dl1->open(LibName, COIL_DEFAULT_DYNLIB_MODE, 1);
      CPPUNIT_ASSERT(!dl1->error());    // 正常時には０を返す

      dl1->symbol("HogeHogeHoge");
      CPPUNIT_ASSERT(dl1->error());    // 非正常時には非０を返す
//      std::cout << "error() : " << err2 << "." << std::endl;

      dl1->close();
      delete dl1;
	}

  };
}; // namespace DynamicLib

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(DynamicLib::DynamicLibTests);

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
#endif // DynamicLib_cpp
