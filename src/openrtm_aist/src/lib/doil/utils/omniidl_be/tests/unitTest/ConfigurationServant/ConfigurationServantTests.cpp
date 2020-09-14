// -*- C++ -*-
/*!
 * @file   ConfigurationServantTests.cpp
 * @brief  ConfigurationServant test class
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

#ifndef ConfigurationServant_cpp
#define ConfigurationServant_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <ConfigurationServant.h>

#include <coil/Properties.h>
#include <doil/ImplBase.h>
#include <doil/ServantFactory.h>
#include <doil/corba/CORBAManager.h>
#include <doil/corba/CORBAServantBase.h>
#include <IConfiguration.h>
#include <stubs/ConfigurationImpl.h>
#include <stubs/Logger.h>

/*!
 * @class ConfigurationServantTests class
 * @brief ConfigurationServant test
 */
namespace ConfigurationServant
{
  class ConfigurationServantTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ConfigurationServantTests);
    CPPUNIT_TEST(test_call_set_device_profile);
    CPPUNIT_TEST(test_call_set_service_profile);
    CPPUNIT_TEST(test_call_add_organization);
    CPPUNIT_TEST(test_call_remove_service_profile);
    CPPUNIT_TEST(test_call_remove_organization);
    CPPUNIT_TEST(test_call_get_configuration_parameters);
    CPPUNIT_TEST(test_call_get_configuration_parameter_values);
    CPPUNIT_TEST(test_call_get_configuration_parameter_value);
    CPPUNIT_TEST(test_call_set_configuration_parameter);
    CPPUNIT_TEST(test_call_get_configuration_sets);
    CPPUNIT_TEST(test_call_get_configuration_set);
    CPPUNIT_TEST(test_call_set_configuration_set_values);
    CPPUNIT_TEST(test_call_get_active_configuration_set);
    CPPUNIT_TEST(test_call_add_configuration_set);
    CPPUNIT_TEST(test_call_remove_configuration_set);
    CPPUNIT_TEST(test_call_activate_configuration_set);
    CPPUNIT_TEST_SUITE_END();
  
  private:
    ::UnitTest::Servant::ConfigurationImpl* Impl;
    ::UnitTest::Servant::Logger Log;
    ::doil::ServantBase* Servant;
    ::SDOPackage::CORBA::ConfigurationServant * CServant;

  public:
  
    /*!
     * @brief Constructor
     */
    ConfigurationServantTests()
    {
        // registerFactory
        Impl = new UnitTest::Servant::ConfigurationImpl(Log);
        doil::CORBA::CORBAManager::instance().registerFactory(Impl->id(),
            doil::New<SDOPackage::CORBA::ConfigurationServant>,
            doil::Delete<SDOPackage::CORBA::ConfigurationServant>);
        doil::ReturnCode_t ret = doil::CORBA::CORBAManager::instance().activateObject(Impl);
        Servant = doil::CORBA::CORBAManager::instance().toServant(Impl);
        CServant = dynamic_cast<SDOPackage::CORBA::ConfigurationServant*>(Servant);
//      std::cout << "ConfigurationServantTests-constractor" << std::endl;
    }
    
    /*!
     * @brief Destructor
     */
    ~ConfigurationServantTests()
    {
      delete Impl;
      Impl = 0;
    }

    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
//      std::cout << "ConfigurationServantTests-setUp" << std::endl;
    }

    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    {
//      std::cout << "ConfigurationServantTests-tearDown" << std::endl;
    }

    /* test case */
    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_set_device_profile()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("set_device_profile");
      ::SDOPackage::DeviceProfile dp;
      ::CORBA::Boolean result;
      result = CServant->set_device_profile(dp);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not true", true, result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_set_service_profile()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("set_service_profile");
      ::SDOPackage::ServiceProfile sp;
      ::CORBA::Boolean result;
      result = CServant->set_service_profile(sp);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not true", true, result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_add_organization()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("add_organization");
      ::SDOPackage::Organization_ptr org;
      ::CORBA::Boolean result;
      result = CServant->add_organization(org);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not true", true, result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 引数がImplに渡されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_remove_service_profile()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("remove_service_profile");
      std::string id("Hoge");
      ::CORBA::Boolean result;
      result = CServant->remove_service_profile(id.c_str());
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not true", true, result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not argument", Log.pop(), id);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 引数がImplに渡されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_remove_organization()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("remove_organization");
      std::string id("Hoge");
      ::CORBA::Boolean result;
      result = CServant->remove_organization(id.c_str());
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not true", true, result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not argument", Log.pop(), id);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 戻り値がnullでないこと
     */
    void test_call_get_configuration_parameters()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("get_configuration_parameters");
      ::SDOPackage::ParameterList* result;
      result = CServant->get_configuration_parameters();
      CPPUNIT_ASSERT_MESSAGE("non-exist", result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 戻り値がnullでないこと
     */
    void test_call_get_configuration_parameter_values()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("get_configuration_parameter_values");
      ::SDOPackage::NVList* result;
      result = CServant->get_configuration_parameter_values();
      CPPUNIT_ASSERT_MESSAGE("non-exist", result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 戻り値がnullでないこと
     */
    void test_call_get_configuration_parameter_value()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("get_configuration_parameter_value");
      std::string name("Hoge");
      ::CORBA::Any* result;
      result = CServant->get_configuration_parameter_value(name.c_str());
      CPPUNIT_ASSERT_MESSAGE("non-exist", result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not argument", Log.pop(), name);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 引数がImplに渡されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_set_configuration_parameter()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("set_configuration_parameter");
      std::string name("hoge");
      ::CORBA::Any value;
      ::CORBA::Boolean result;
      result = CServant->set_configuration_parameter(name.c_str(), value);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not true", true, result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not argument", Log.pop(), name);
//      CPPUNIT_ASSERT_EQUAL_MESSAGE("not argument", Log.pop(), value);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_get_configuration_sets()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("get_configuration_sets");
      ::SDOPackage::ConfigurationSetList* result;
      result = CServant->get_configuration_sets();
      CPPUNIT_ASSERT_MESSAGE("non-exist", result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 引数がImplに渡されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_get_configuration_set()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("get_configuration_set");
      std::string config_id("hoge");
      ::SDOPackage::ConfigurationSet* result;
      result = CServant->get_configuration_set(config_id.c_str());
      CPPUNIT_ASSERT_MESSAGE("non-exist", result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not argument", Log.pop(), config_id);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 引数がImplに渡されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_set_configuration_set_values()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("set_configuration_set_values");
      std::string config_id("hoge");
      ::SDOPackage::ConfigurationSet set;
      ::CORBA::Boolean result;
      result = CServant->set_configuration_set_values(config_id.c_str(), set);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not true", true, result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not argument", Log.pop(), config_id);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_get_active_configuration_set()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("get_active_configuration_set");
      ::SDOPackage::ConfigurationSet* result;
      result = CServant->get_active_configuration_set();
      CPPUNIT_ASSERT_MESSAGE("non-exist", result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 引数がImplに渡されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_add_configuration_set()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("add_configuration_set");
      ::SDOPackage::ConfigurationSet set;
      ::CORBA::Boolean result;
      result = CServant->add_configuration_set(set);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not true", true, result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 引数がImplに渡されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_remove_configuration_set()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("remove_configuration_set");
      std::string config_id("hoge");
      ::CORBA::Boolean result;
      result = CServant->remove_configuration_set(config_id.c_str());
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not true", true, result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not argument", Log.pop(), config_id);
    }

    /*!
     * @brief 以下の要件を確認する
     * @brief Implのメソドが呼び出されていること
     * @brief 引数がImplに渡されていること
     * @brief 戻り値が妥当であること
     */
    void test_call_activate_configuration_set()
    {
      CPPUNIT_ASSERT(CServant);

      std::string str("activate_configuration_set");
      std::string str2("hoge");
      ::CORBA::Boolean result;
      result = CServant->activate_configuration_set(str2.c_str());
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not true", true, result);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not method name", Log.pop(), str);
      CPPUNIT_ASSERT_EQUAL_MESSAGE("not argument", Log.pop(), str2);
    }
  };
}; // namespace ConfigurationServant

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ConfigurationServant::ConfigurationServantTests);

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
#endif // ConfigurationServant_cpp
