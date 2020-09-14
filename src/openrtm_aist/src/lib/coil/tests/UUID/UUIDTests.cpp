// -*- C++ -*-
/*!
 * @file   UUIDTests.cpp
 * @brief  UUID test class
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

#ifndef UUID_cpp
#define UUID_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/UUID.h>

/*!
 * @class UUIDTests class
 * @brief UUID test
 */

#ifdef __QNX__
using std::isxdigit;
#endif

namespace coilUUID
{
  class UUIDTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(UUIDTests);
    CPPUNIT_TEST(test_UUID_to_string);
    CPPUNIT_TEST(test_UUID_Generator_init);
    CPPUNIT_TEST(test_UUID_Generator_generateUUID);
    CPPUNIT_TEST_SUITE_END();
  
  private:
//    uuid_t uuid;
    coil::UUID * uu;
    coil::UUID_Generator *ug;
  
  public:
  
    /*!
     * @brief Constructor
     */
    UUIDTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~UUIDTests()
    {
    }
  
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
//      uu = new coil::UUID(&uuid);
      uu = new coil::UUID();
//      std::cout << "UUID : " << uu->to_string() << std::endl ;
      ug = new coil::UUID_Generator();
    }
    
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    {
      delete uu;
      delete ug;
    }
  
    /* test case */

    /*!
     * @brief test UUID::to_string()
     */
    void test_UUID_to_string()
    {
      CPPUNIT_ASSERT(isUuidString(uu->to_string()));
    }

    /*!
     * @brief test UUID_Generator::init()
     */
    void test_UUID_Generator_init()
    {
      // init()は呼ばれても何もしないメソドなので、テスト内容無し
    }

    /*!
     * @brief 
     */
    void test_UUID_Generator_generateUUID()
    {
      int n(1);
      int h(2);
      coil::UUID * p;
      
      p = ug->generateUUID(n, h);
//      std::cout << "UUID : " << p->to_string() << std::endl;
      CPPUNIT_ASSERT(isUuidString(p->to_string()));
      delete p;
    }

	/*!
	 *  @brief 文字列がUUIDフォーマットかどうかを判別する。
	 */
    bool isUuidString(const char* aString)
	{
	    bool result(false);
    
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (*aString != '-') return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (*aString != '-') return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (*aString != '-') return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (*aString != '-') return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
	    if (!isxdigit(*aString)) return result; aString++;
    
	    result = true;
    
	    return result;
	}

  };
}; // namespace UUID

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(coilUUID::UUIDTests);

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

/*
 *  Utility functions.
 */

/*!
 *  @brief 文字列がUUIDフォーマットかどうかを判別する。
 */
static bool isUuidString(const char* aString)
{
    bool result(false);
    
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (*aString != '-') return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (*aString != '-') return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (*aString != '-') return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (*aString != '-') return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    if (!isxdigit(*aString)) return result; aString++;
    
    result = true;
    
    return result;
}

#endif // UUID_cpp
