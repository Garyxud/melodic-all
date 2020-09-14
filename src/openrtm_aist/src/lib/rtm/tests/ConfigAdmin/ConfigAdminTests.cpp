// -*- C++ -*-
/*!
 * @file   ConfigAdminTests.cpp
 * @brief  ConfigAdmin test class
 * @date   $Date: 2008/05/12 01:47:32 $
 *
 * $Id: ConfigAdminTests.cpp,v 1.1 2008/05/12 01:47:32 arafune Exp $
 *
 */

/*
 * $Log: ConfigAdminTests.cpp,v $
 * Revision 1.1  2008/05/12 01:47:32  arafune
 * The first commitment.
 *
 *
 */

#ifndef ConfigAdmin_cpp
#define ConfigAdmin_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <strstream>
#include <rtm/ConfigAdmin.h>

/*!
 * @class ConfigTests class
 * @brief Config test
 */
namespace Config
{
  class ConfigTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ConfigTests);
    CPPUNIT_TEST(test_constructor);
    CPPUNIT_TEST(test_update);
    CPPUNIT_TEST(test_update_with_illegal_value);
    CPPUNIT_TEST(test_update_with_illegal_default_value);
    CPPUNIT_TEST_SUITE_END();
		
  private:
	
	
  public:
    /*!
     * @brief Constructor
     */
    ConfigTests()
    {
    }
		
    /*!
     * @brief Destructor
     */
    ~ConfigTests()
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
		
    /*!
     * @brief コンストラクタのテスト
     */
    void test_constructor()
    {
      std::string name = "name of parameter";
      double value = 0.0;
      std::string default_value = "3.14159";
			
      RTC::Config<double> config(name.c_str(), value, default_value.c_str());
			
      // 指定した名称が正しく設定されているか？
      CPPUNIT_ASSERT_EQUAL(name, std::string(config.name));
			
      // 指定したデフォルト値（の文字列表現）が正しく設定されているか？
      CPPUNIT_ASSERT_EQUAL(default_value, std::string(config.default_value));
    }
		
    /*!
     * @brief update()メソッドのテスト
     * 
     * - バインドした変数が、update()で指定された値に正しく更新されるか？
     */
    void test_update()
    {
      std::string name = "name of parameter";
      double value = 0.0;
      std::string default_value = "3.14159";
			
      RTC::Config<double> config(name.c_str(), value, default_value.c_str());
			
      // update()前は、変数値が初期状態のままのはず
      CPPUNIT_ASSERT_EQUAL(0.0, value);
			
      // update()する
      CPPUNIT_ASSERT_EQUAL(true, config.update("1.41421356"));
			
      // バインドした変数が、update()で指定された値に正しく更新されるか？
      CPPUNIT_ASSERT_EQUAL(1.41421356, value);
    }
		
    /*!
     * @brief update()メソッドのテスト
     * 
     * - 指定の型に変換できない内容でupdate()を呼出し、意図どおり失敗するか？
     * - バインドした変数の値が、コンストラクタで指定したデフォルト値に正しく更新されるか？
     */
    void test_update_with_illegal_value()
    {
      std::string name = "name of parameter";
      double value = 0.0;
      std::string default_value = "3.14159";
			
      RTC::Config<double> config(name.c_str(), value, default_value.c_str());
			
      // update()前は、変数値が初期状態のままのはず
      CPPUNIT_ASSERT_EQUAL(0.0, value);
			
      // 浮動小数点値に変換できない内容でupdate()を呼出し、意図どおり失敗するか？
      CPPUNIT_ASSERT_EQUAL(false, config.update("Not float value"));
			
      // バインドした変数の値が、コンストラクタで指定したデフォルト値に正しく更新されるか？
      CPPUNIT_ASSERT_EQUAL(3.14159, value);
    }
		
    /*!
     * @brief update()メソッドのテスト
     * 
     * - 浮動小数点値に変換できないデフォルト値が指定され、かつ浮動小数点値に変換できない内容でupdate()を呼出した場合、
     * バインドした変数の値は、更新されることなく元の値に留まるか？
     */
    void test_update_with_illegal_default_value()
    {
      std::string name = "name of parameter";
      double value = 0.0;
      std::string illegal_default_value = "Not float value";
			
      RTC::Config<double> config(name.c_str(), value, illegal_default_value.c_str());

      // update()前は、変数値が初期状態のままのはず
      CPPUNIT_ASSERT_EQUAL(0.0, value);
			
      // 浮動小数点値に変換できないデフォルト値が指定され、
      // かつ浮動小数点値に変換できない内容でupdate()を呼出した場合、
      // バインドした変数の値は、更新されることなく元の値に留まるか？
      CPPUNIT_ASSERT_EQUAL(false, config.update("Not float value too"));
      CPPUNIT_ASSERT_EQUAL(0.0, value);
    }
		
  };
}; // namespace Config

/*!
 * @class ConfigAdminTests class
 * @brief ConfigAdmin test
 */
namespace ConfigAdmin
{
  class OnUpdateCallbackMock : public RTC::OnUpdateCallback
  {
  public:
    OnUpdateCallbackMock(void) : result(false) {}
    virtual ~OnUpdateCallbackMock(void){}
    void operator()(const char* config_set)
      {
        // この出力があれば正しく呼ばれている
//      std::cout << "OnUpdateCallbackMock1 config_set=" << config_set << std::endl;
        result = true;
      }
    bool result;
  };

  class OnUpdateParamCallbackMock : public RTC::OnUpdateParamCallback
  {
  public:
    OnUpdateParamCallbackMock(void) : result(false) {}
    virtual ~OnUpdateParamCallbackMock(void){}
    void operator()(const char* config_set, const char* config_param)
      {
//      std::cout << "OnUpdateParamCallbackMock2 config_set=" << config_set << std::endl;
//      std::cout << "OnUpdateParamCallbackMock2 config_param=" << config_param << std::endl;
        result = true;
      }
    bool result;
  };

  class OnSetConfigurationSetCallbackMock : public RTC::OnSetConfigurationSetCallback
  {
  public:
    OnSetConfigurationSetCallbackMock(void) : result(false) {}
    virtual ~OnSetConfigurationSetCallbackMock(void){}
    void operator()(const coil::Properties& config_set)
      {
//      std::cout << "OnSetConfigurationSetCallbackMock3 config_set=" << std::endl << config_set << std::endl;
        result = true;
      }
    bool result;
  };

  class OnAddConfigurationAddCallbackMock : public RTC::OnAddConfigurationAddCallback
  {
  public:
    OnAddConfigurationAddCallbackMock(void) : result(false) {}
    virtual ~OnAddConfigurationAddCallbackMock(void){}
    void operator()(const coil::Properties& config_set)
      {
//      std::cout << "OnAddConfigurationAddCallbackMock4 config_set=" << std::endl << config_set << std::endl;
        result = true;
      }
    bool result;
  };

  class OnRemoveConfigurationSetCallbackMock : public RTC::OnRemoveConfigurationSetCallback
  {
  public:
    OnRemoveConfigurationSetCallbackMock(void) : result(false) {}
    virtual ~OnRemoveConfigurationSetCallbackMock(void){}
    void operator()(const char* config_set)
      {
//      std::cout << "OnRemoveConfigurationSetCallbackMock5 config_set=" << config_set << std::endl;
        result = true;
      }
    bool result;
  };

  class OnActivateSetCallbackMock : public RTC::OnActivateSetCallback
  {
  public:
    OnActivateSetCallbackMock(void) : result(false) {}
    virtual ~OnActivateSetCallbackMock(void){}
    void operator()(const char* config_id)
      {
//      std::cout << "OnActivateSetCallbackMock6 config_id=" << config_id << std::endl;
        result = true;
      }
    bool result;
  };

  // ConfigAdmin を継承して、protected: 関数をオーバーロードする
  class ConfigAdminMock : public RTC::ConfigAdmin
  {
  public:
    ConfigAdminMock(coil::Properties& configsets)
      : RTC::ConfigAdmin(configsets) {}
    virtual ~ConfigAdminMock(void){}

    void onUpdateMock(const char* config_set)
      {
//      std::cout << "ConfigAdmin::onUpdate() 1 before" << std::endl;
        RTC::ConfigAdmin::onUpdate(config_set);
//      std::cout << "ConfigAdmin::onUpdate() 1 after" << std::endl;
      }
    void onUpdateParamMock(const char* config_set, const char* config_param)
      {
//      std::cout << "ConfigAdmin::onUpdateParam() 2 before" << std::endl;
        RTC::ConfigAdmin::onUpdateParam(config_set, config_param);
//      std::cout << "ConfigAdmin::onUpdateParam() 2 after" << std::endl;
      }
    void onSetConfigurationSetMock(const coil::Properties& config_set)
      {
//      std::cout << "ConfigAdmin::onSetConfigurationSet() 3 before" << std::endl;
        RTC::ConfigAdmin::onSetConfigurationSet(config_set);
//      std::cout << "ConfigAdmin::onSetConfigurationSet() 3 after" << std::endl;
      }
    void onAddConfigurationSetMock(const coil::Properties& config_set)
      {
//      std::cout << "ConfigAdmin::onAddConfigurationSet() 4 before" << std::endl;
        RTC::ConfigAdmin::onAddConfigurationSet(config_set);
//      std::cout << "ConfigAdmin::onAddConfigurationSet() 4 after" << std::endl;
      }
    void onRemoveConfigurationSetMock(const char* config_id)
      {
//      std::cout << "ConfigAdmin::onRemoveConfigurationSet() 5 before" << std::endl;
        RTC::ConfigAdmin::onRemoveConfigurationSet(config_id);
//      std::cout << "ConfigAdmin::onRemoveConfigurationSet() 5 after" << std::endl;
      }
    void onActivateSetMock(const char* config_id)
      {
//      std::cout << "ConfigAdmin::onActivateSet() 6 before" << std::endl;
        RTC::ConfigAdmin::onActivateSet(config_id);
//      std::cout << "ConfigAdmin::onActivateSet() 6 after" << std::endl;
      }
  };


  class ConfigAdminTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ConfigAdminTests);

    CPPUNIT_TEST(test_setOnUpdate);
    CPPUNIT_TEST(test_setOnUpdateParam);
    CPPUNIT_TEST(test_setOnSetConfigurationSet);
    CPPUNIT_TEST(test_setOnAddConfigurationSet);
    CPPUNIT_TEST(test_setOnRemoveConfigurationSet);
    CPPUNIT_TEST(test_setOnActivateSet);
    CPPUNIT_TEST(test_constructor);
    CPPUNIT_TEST(test_bindParameter);
    CPPUNIT_TEST(test_bindParameter_already_exist);
    CPPUNIT_TEST(test_bindParameter_illegal_default_value);
    CPPUNIT_TEST(test_update);
    CPPUNIT_TEST(test_update_by_inexist_configuration_set);
    CPPUNIT_TEST(test_update_with_specified_parameter_name);
    CPPUNIT_TEST(test_update_by_active_configuration_set);
    CPPUNIT_TEST(test_isExist);
    CPPUNIT_TEST(test_isChanged_on_addConfigurationSet);
    CPPUNIT_TEST(test_isChanged_on_removeConfigurationSet);
    CPPUNIT_TEST(test_isChanged_on_activateConfigurationSet);
    CPPUNIT_TEST(test_getActiveId);
    CPPUNIT_TEST(test_haveConfig);
    CPPUNIT_TEST(test_isActive_on_addConfigurationSet);
    CPPUNIT_TEST(test_isActive_on_removeConfigurationSet);
    CPPUNIT_TEST(test_getConfigurationSets);
    CPPUNIT_TEST(test_addConfigurationSet_and_getConfigurationSet);
    CPPUNIT_TEST(test_setConfigurationSetValues);
    CPPUNIT_TEST(test_setConfigurationSetValues_with_inexist_configuration_set);
    CPPUNIT_TEST(test_getActiveConfigurationSet);
    CPPUNIT_TEST(test_removeConfigurationSet);
    CPPUNIT_TEST(test_removeConfigurationSet_with_inexist_configuration_id);

    CPPUNIT_TEST_SUITE_END();
		
  private:
    // helper
    const coil::Properties* getPropertiesBy(
					   const std::string& name,
					   const std::vector<coil::Properties*>& propertiesSet) const
    {
      for (int i = 0; i < (int) propertiesSet.size(); ++i)
	{
	  if (std::string(propertiesSet[i]->getName()) == name)
	    {
	      return propertiesSet[i];
	    }
	}
			
      return 0; // not found
    }
	
  public:
    /*!
     * @brief Constructor
     */
    ConfigAdminTests()
    {
    }
		
    /*!
     * @brief Destructor
     */
    ~ConfigAdminTests()
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
		
    /*!
     * @brief コンストラクタのテスト
     * 
     * - 引数で指定したコンフィグレーションセットが、正しく設定されるか？
     */
    void test_constructor()
    {
      coil::Properties configSet("config_id");
      configSet.setProperty("config_id.key", "value");
			
      RTC::ConfigAdmin configAdmin(configSet);
			
      // 引数で指定したコンフィグレーションセットが、正しく設定されるか？
      const coil::Properties& activeConfigSet = configAdmin.getConfigurationSet("config_id");
      CPPUNIT_ASSERT_EQUAL(std::string("value"), activeConfigSet.getProperty("key"));
    }
		
    /*!
     * @brief bindParameter()メソッドのテスト
     * 
     * - bindParameter()で、正常に変数をバインドできるか？
     * - バインドした変数は、指定したデフォルト値に正しく更新されているか？
     */		
    void test_bindParameter()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // bindParameter()で、正常に変数をバインドできるか？
      const char* varName = "name";
      double var = 0.0;
      const char* default_value = "3.14159";
      CPPUNIT_ASSERT_EQUAL(
			   true, configAdmin.bindParameter(varName, var, default_value));
			
      // バインドした変数は、指定したデフォルト値に正しく更新されているか？
      CPPUNIT_ASSERT_EQUAL(3.14159, var);
    }
		
    /*!
     * @brief bindParameter()メソッドのテスト
     * 
     * - 同一名称の変数バインドを試みて、意図どおり失敗するか？
     */
    void test_bindParameter_already_exist()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // まずは、初回のバインドを行う
      const char* varName1 = "name";
      double var1 = 0.0;
      const char* default_value1 = "3.14159";
      CPPUNIT_ASSERT_EQUAL(
			   true, configAdmin.bindParameter(varName1, var1, default_value1));
      CPPUNIT_ASSERT_EQUAL(3.14159, var1);
			
      // 同一名称の変数バインドを試みて、意図どおり失敗するか？
      const char* varName2 = varName1;
      double var2 = 1.0;
      const char* default_value2 = "1.41421356";
      CPPUNIT_ASSERT_EQUAL(
			   false, configAdmin.bindParameter(varName2, var2, default_value2));
				
      // バインド変数の値は更新されることなく保持されているか？
      CPPUNIT_ASSERT_EQUAL(3.14159, var1);
    }
		
    /*!
     * @brief bindParameter()メソッドのテスト
     * 
     * - 指定のデータ型に変換できないデフォルト値を指定して、意図どおり失敗するか？
     */
    void test_bindParameter_illegal_default_value()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // 指定のデータ型に変換できないデフォルト値を指定して、意図どおり失敗するか？
      const char* varName = "name";
      double var = 0.0;
      const char* default_value = "Illegal default value";
      CPPUNIT_ASSERT_EQUAL(
			   false, configAdmin.bindParameter(varName, var, default_value));
    }
		
    /*!
     * @brief update()メソッドのテスト
     * 
     * - コンフィグレーションセットを指定してupdate()し、その内容でバインド変数値が正しく更新されるか？
     */
    void test_update()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // コンフィグレーションセット１を準備する
      coil::Properties configSet1("set 1");
      configSet1.setProperty("name", "1.41421356");
			
      // コンフィグレーションセット２を準備する
      coil::Properties configSet2("set 2");
      configSet2.setProperty("name", "1.7320508");
			
      // 準備した２つのコンフィグレーションセットを追加しておく
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet1));
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet2));
			
      // 変数をバインドする
      const char* varName = "name";
      double var = 0.0;
      const char* default_value = "3.14159";
      CPPUNIT_ASSERT_EQUAL(
			   true, configAdmin.bindParameter(varName, var, default_value));
			
      // update()前は、まだ変数がデフォルト値のままであることを確認する
      CPPUNIT_ASSERT_EQUAL(3.14159, var);
			
      // コンフィグレーションセット１を指定してupdate()し、その内容でバインド変数値が正しく更新されるか？
      configAdmin.update("set 1");
      CPPUNIT_ASSERT_EQUAL(1.41421356, var);

      // コンフィグレーションセット２を指定してupdate()し、その内容でバインド変数値が正しく更新されるか？
      configAdmin.update("set 2");
      CPPUNIT_ASSERT_EQUAL(1.7320508, var);
    }
		
    /*!
     * @brief update()メソッドのテスト
     * 
     * - 存在しないコンフィグレーションIDを指定してupdate()を呼出した場合に、
     * バインド変数が更新されずに、そのまま保持されるか？
     */
    void test_update_by_inexist_configuration_set()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // コンフィグレーションセットを準備し、追加しておく
      coil::Properties configSet("set");
      configSet.setProperty("name", "1.41421356");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet));
			
      // 変数をバインドする
      const char* varName = "name";
      double var = 0.0;
      const char* default_value = "3.14159";
      CPPUNIT_ASSERT_EQUAL(
			   true, configAdmin.bindParameter(varName, var, default_value));
      CPPUNIT_ASSERT_EQUAL(3.14159, var);
			
      // 存在しないコンフィグレーションIDを指定してupdate()を呼出した場合に、
      // バインド変数が更新されずに、そのまま保持されるか？
      configAdmin.update("inexist set");
      CPPUNIT_ASSERT_EQUAL(3.14159, var);
    }
		
    /*!
     * @brief update()メソッド（名称指定）のテスト
     * 
     * - 指定したコンフィグレーションセットの指定した名称の変数だけが、正しく更新されるか？
     */
    void test_update_with_specified_parameter_name()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // ２つのパラメータ値を含むコンフィグレーションセットを準備し、追加しておく
      coil::Properties configSet1("set 1");
      configSet1.setProperty("name 1", "1.41421356");
      configSet1.setProperty("name 2", "1.7320508");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet1));

      coil::Properties configSet2("set 2");
      configSet2.setProperty("name 1", "3.14159");
      configSet2.setProperty("name 2", "2.71828");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet2));
			
      // ２つの変数をバインドする
      const char* varName1 = "name 1";
      double var1 = 0.0;
      const char* default_value1 = "2.23620679";
      CPPUNIT_ASSERT_EQUAL(
			   true, configAdmin.bindParameter(varName1, var1, default_value1));
      CPPUNIT_ASSERT_EQUAL(2.23620679, var1);
			
      const char* varName2 = "name 2";
      double var2 = 0.0;
      const char* default_value2 = "2.4494897";
      CPPUNIT_ASSERT_EQUAL(
			   true, configAdmin.bindParameter(varName2, var2, default_value2));
      CPPUNIT_ASSERT_EQUAL(2.4494897, var2);
			
      // ２つのうち、片方の変数のみを名称指定してupdate()を行い、指定した変数だけが正しく更新されるか？
      configAdmin.update("set 2", "name 1");
      CPPUNIT_ASSERT_EQUAL(3.14159, var1);
      CPPUNIT_ASSERT_EQUAL(2.4494897, var2);
    }
		
    /*!
     * @brief update()メソッド（アクティブコンフィグレーションセット）のテスト
     * 
     * - update()呼出しにより、バインド変数がアクティブなコンフィグレーションセットの値で更新されるか？
     */
    void test_update_by_active_configuration_set()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // コンフィグレーションセットを準備し、追加しておく
      coil::Properties configSet1("set 1");
      configSet1.setProperty("name", "1.41421356");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet1));

      coil::Properties configSet2("set 2");
      configSet2.setProperty("name", "1.7320508");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet2));
			
      // 変数をバインドする
      const char* varName = "name";
      double var = 0.0;
      const char* default_value = "3.14159";
      CPPUNIT_ASSERT_EQUAL(
			   true, configAdmin.bindParameter(varName, var, default_value));
      CPPUNIT_ASSERT_EQUAL(3.14159, var);
			
      // "set 1"のほうをアクティブにする
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.activateConfigurationSet("set 1"));
			
      // アクティブにしただけでは、まだバインド変数は更新されていないはず
      CPPUNIT_ASSERT_EQUAL(3.14159, var);
			
      // update()呼出しにより、バインド変数がアクティブなコンフィグレーションセットの値で更新されるか？
      configAdmin.update();
      CPPUNIT_ASSERT_EQUAL(1.41421356, var);

      // "set 2"のほうをアクティブにする
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.activateConfigurationSet("set 2"));
			
      // アクティブにしただけでは、まだバインド変数は更新されていないはず
      CPPUNIT_ASSERT_EQUAL(1.41421356, var);

      // update()呼出しにより、バインド変数がアクティブなコンフィグレーションセットの値で更新されるか？
      configAdmin.update();
      CPPUNIT_ASSERT_EQUAL(1.7320508, var);
    }
		
    /*!
     * @brief isExist()メソッドのテスト
     * 
     * - バインドした変数の名称でisExist()を呼出し、真値が得られるか？
     * - バインドしていない名称でisExist()を呼出し、偽値が得られるか？
     */
    void test_isExist()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // 変数をバインドする
      const char* varName = "name";
      double var = 0.0;
      const char* default_value = "3.14159";
      CPPUNIT_ASSERT_EQUAL(
			   true, configAdmin.bindParameter(varName, var, default_value));
      CPPUNIT_ASSERT_EQUAL(3.14159, var);
			
      // バインドした変数の名称でisExist()を呼出し、真値が得られるか？
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.isExist("name"));
			
      // バインドしていない名称でisExist()を呼出し、偽値が得られるか？
      CPPUNIT_ASSERT_EQUAL(false, configAdmin.isExist("inexist name"));
    }
		
    /*!
     * @brief isChanged()メソッドのテスト
     * 
     * - addConfigurationSet()呼出後は、isChanged()は真値となるか？
     */
    void test_isChanged_on_addConfigurationSet()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // update()を呼出してバインド変数と同期を行い、isChanged()が偽となる状態にする
      configAdmin.update();
      CPPUNIT_ASSERT_EQUAL(false, configAdmin.isChanged());
			
      // addConfigurationSet()を呼出す
      coil::Properties configSet("id");
      configSet.setProperty("key", "value");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet));

      // addConfigurationSet()呼出後は、isChanged()は真値となるか？
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.isChanged());
    }
		
    /*!
     * @brief isChanged()メソッドのテスト
     * 
     * - removeConfigurationSet()呼出後は、isChanged()は真値となるか？
     */
    void test_isChanged_on_removeConfigurationSet()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // addConfigurationSet()を呼出す
      coil::Properties configSet("id");
      configSet.setProperty("key", "value");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet));

      // update()を呼出してバインド変数と同期を行い、isChanged()が偽となる状態にする
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.activateConfigurationSet("id"));
      configAdmin.update();
      CPPUNIT_ASSERT_EQUAL(false, configAdmin.isChanged());
			
      // removeConfigurationSet()の呼出後は、isChanged()は真値となるか？
      //CPPUNIT_ASSERT_EQUAL(true, configAdmin.removeConfigurationSet("id"));
      //CPPUNIT_ASSERT_EQUAL(true, configAdmin.isChanged());
    }
		
    /*!
     * @brief isChanged()メソッドのテスト
     * 
     * - activateConfigurationSet()呼出後は、isChanged()は真値となるか？
     */
    void test_isChanged_on_activateConfigurationSet()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // コンフィグレーションセットを準備し、追加しておく
      coil::Properties configSet1("set 1");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet1));

      coil::Properties configSet2("set 2");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet2));
			
      // "set 1"のほうをアクティブにする
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.activateConfigurationSet("set 1"));
			
      // update()を行い、isChanged()が偽値の状態にしておく
      configAdmin.update();
      CPPUNIT_ASSERT_EQUAL(false, configAdmin.isChanged());

      // "set 2"のほうをアクティブにすると、isChanged()が真値となるか？
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.activateConfigurationSet("set 2"));
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.isChanged());
    }
		
    void test_isChanged_on_setConfigurationSetValues()
    {
      // TODO 実装すること
    }
		
    /*!
     * @brief getActiveId()メソッドのテスト
     * 
     * - アクティブ化したコンフィグレーションセットのIDを正しく取得できるか？
     */
    void test_getActiveId()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // コンフィグレーションセットを準備し、追加しておく
      coil::Properties configSet1("set 1");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet1));

      coil::Properties configSet2("set 2");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet2));
			
      // 初期状態では、アクティブIDは"default"のはず
      CPPUNIT_ASSERT_EQUAL(std::string("default"), std::string(configAdmin.getActiveId()));
			
      // "set 1"をアクティブにした後、意図どおりのアクティブIDを取得できるか？
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.activateConfigurationSet("set 1"));
      CPPUNIT_ASSERT_EQUAL(std::string("set 1"), std::string(configAdmin.getActiveId()));
			
      // "set 2"をアクティブにした後、意図どおりのアクティブIDを取得できるか？
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.activateConfigurationSet("set 2"));
      CPPUNIT_ASSERT_EQUAL(std::string("set 2"), std::string(configAdmin.getActiveId()));
    }
		
    /*!
     * @brief haveConfig()メソッドのテスト
     * 
     * - 存在するコンフィグレーションセットIDを指定した場合に、haveConfig()で正しく真値を得るか？
     * - 存在しないコンフィグレーションセットIDを指定した場合に、haveConfig()で正しく偽値を得るか？
     */
    void test_haveConfig()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // コンフィグレーションセットを準備し、追加しておく
      coil::Properties configSet1("id");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet1));
			
      // 存在するコンフィグレーションセットIDを指定した場合に、haveConfig()で正しく真値を得るか？
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.haveConfig("id"));
			
      // 存在しないコンフィグレーションセットIDを指定した場合に、haveConfig()で正しく偽値を得るか？
      CPPUNIT_ASSERT_EQUAL(false, configAdmin.haveConfig("inexist id"));
    }
		
    /*!
     * @brief isActive()メソッドのテスト
     * 
     * - addConfigurationSet()呼出後は、isActive()は正しく偽値を得るか？
     * - activateConfigurationSet()でアクティブ化した後は、isActive()は正しく真値を得るか？
     */
    void test_isActive_on_addConfigurationSet()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // addConfigurationSet()呼出後は、isActive()は正しく偽値を得るか？
      coil::Properties configSet("id");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet));
      CPPUNIT_ASSERT_EQUAL(false, configAdmin.isActive());
			
      // activateConfigurationSet()でアクティブ化した後は、isActive()は正しく真値を得るか？
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.activateConfigurationSet("id"));
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.isActive());
    }
		
    /*!
     * @brief isActive()メソッドのテスト
     * 
     * - removeConfigurationSet()呼出後は、isActive()は正しく偽値を得るか？
     */
    void test_isActive_on_removeConfigurationSet()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);

      // コンフィグレーションセットを追加してアクティブ化しておく
      coil::Properties configSet("id");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet));
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.activateConfigurationSet("id"));
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.isActive());
			
      // 登録されているコンフィグレーションセットをremoveConfigurationSet()で解除した場合、
      // isActive()は正しく偽値を得るか？
      //CPPUNIT_ASSERT_EQUAL(true, configAdmin.removeConfigurationSet("id"));
      //CPPUNIT_ASSERT_EQUAL(false, configAdmin.isActive());
    }
		
    /*!
     * @brief isActive()メソッドのテスト
     * 
     * - activateConfigurationSet()でアクティブ化した後は、isActive()は正しく真値を得るか？
     */
    void test_isActive_on_activateConfigurationSet()
    {
      // test_isActive_on_addConfigurationSet()で兼用
    }
		
    void test_isActive_on_setConfigurationSetValues()
    {
      // TODO 実装すること
    }
		
    /*!
     * @brief getConfigurationSets()メソッドのテスト
     * 
     * - 登録されている全てのコンフィグレーションセットを正しく取得できるか？
     */
    void test_getConfigurationSets()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // ２つのパラメータ値を含むコンフィグレーションセットを準備し、追加しておく
      coil::Properties configSet1("set 1");
      configSet1.setProperty("name 1", "1.41421356");
      configSet1.setProperty("name 2", "1.7320508");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet1));
			
      coil::Properties configSet2("set 2");
      configSet2.setProperty("name 1", "3.14159");
      configSet2.setProperty("name 2", "2.71828");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet2));
			
      // getConfigurationSets()で登録されている全てのコンフィグレーションセットを取得し、
      // 登録したものと一致していることを確認する
      std::vector<coil::Properties*> expectedConfigSets;
      expectedConfigSets.push_back(&configSet1);
      expectedConfigSets.push_back(&configSet2);
			
      const std::vector<coil::Properties*>& configSets
	= configAdmin.getConfigurationSets();
			
      CPPUNIT_ASSERT_EQUAL(
			   std::string("1.41421356"),
			   getPropertiesBy("set 1", configSets)->getProperty("name 1"));
      CPPUNIT_ASSERT_EQUAL(
			   std::string("1.7320508"),
			   getPropertiesBy("set 1", configSets)->getProperty("name 2"));

      CPPUNIT_ASSERT_EQUAL(
			   std::string("3.14159"),
			   getPropertiesBy("set 2", configSets)->getProperty("name 1"));
      CPPUNIT_ASSERT_EQUAL(
			   std::string("2.71828"),
			   getPropertiesBy("set 2", configSets)->getProperty("name 2"));
    }
		
    /*!
     * addConfigurationSet()メソッドとgetConfigurationSet()メソッドのテスト
     * 
     * - addConfigurationSet()で、コンフィグレーションセットを追加できるか？
     * - getConfigurationSet()で、追加したコンフィグレーションセットを正しく取得できるか？
     */
    void test_addConfigurationSet_and_getConfigurationSet()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // addConfigurationSet()で、コンフィグレーションセットを追加できるか？
      coil::Properties configSet("id");
      configSet.setProperty("key", "value");
      configAdmin.addConfigurationSet(configSet);
			
      // getConfigurationSet()で、追加したコンフィグレーションセットを正しく取得できるか？
      const coil::Properties& configSetRet = configAdmin.getConfigurationSet("id");
      CPPUNIT_ASSERT_EQUAL(std::string("value"), configSetRet.getProperty("key"));
    }
		
    /*!
     * setConfigurationSetValues()メソッドのテスト
     * 
     * - 指定したプロパティが、正しく指定したコンフィグレーションセットに追加されるか？
     */
    void test_setConfigurationSetValues()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      coil::Properties configSet1("id");
      configSet1.setProperty("name 1", "1.41421356");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet1));
			
      // 登録済みのコンフィグレーションセットに対して、プロパティを追加する
      coil::Properties configSet2("id");
      configSet2.setProperty("name 2", "1.7320508");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.setConfigurationSetValues(configSet2));
			
      // 当該コンフィグレーションセットを取得して、プロパティが意図どおり追加されていることを確認する
      const coil::Properties& configSetRet = configAdmin.getConfigurationSet("id");
      CPPUNIT_ASSERT_EQUAL(std::string("1.41421356"), configSetRet.getProperty("name 1"));
      CPPUNIT_ASSERT_EQUAL(std::string("1.7320508"), configSetRet.getProperty("name 2"));
    }
		
    /*!
     * @brief setConfigurationSetValues()メソッドのテスト
     * 
     * - 存在しないコンフィグレーションセットに対してプロパティ追加を試みて、意図どおり失敗するか？
     * - 失敗後に、登録済みのコンフィグレーションセットが影響を受けていないか？
     */
    void test_setConfigurationSetValues_with_inexist_configuration_set()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      coil::Properties configSet1("id");
      configSet1.setProperty("name 1", "1.41421356");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet1));
			
      // 存在しないコンフィグレーションセットに対してプロパティ追加を試みて、意図どおり失敗するか？
      coil::Properties configSet2("inexist id");
      configSet2.setProperty("name 2", "1.7320508");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.setConfigurationSetValues(configSet2));
			
      // 失敗後に、登録済みのコンフィグレーションセットが影響を受けていないか？
      const coil::Properties& configSetRet = configAdmin.getConfigurationSet("id");
      CPPUNIT_ASSERT_EQUAL(std::string("1.41421356"), configSetRet.getProperty("name 1"));
      CPPUNIT_ASSERT_EQUAL(std::string(""), configSetRet.getProperty("name 2"));
    }
		
    /*!
     * @brief getActiveConfigurationSet()メソッドのテスト
     * 
     * - アクティブコンフィグレーションセットを正しく取得できるか？
     */
    void test_getActiveConfigurationSet()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // コンフィグレーションセットを準備し、追加しておく
      coil::Properties configSet1("set 1");
      configSet1.setProperty("name 1", "1.41421356");
      configSet1.setProperty("name 2", "1.7320508");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet1));
			
      coil::Properties configSet2("set 2");
      configSet2.setProperty("name 1", "3.14159");
      configSet2.setProperty("name 2", "2.71828");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet2));
			
      // "set 1"をアクティブ化した後、アクティブコンフィグレーションセットとして正しく取得できるか？
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.activateConfigurationSet("set 1"));
      const coil::Properties& activeConfigSet1 = configAdmin.getActiveConfigurationSet();
      CPPUNIT_ASSERT_EQUAL(std::string("1.41421356"), activeConfigSet1.getProperty("name 1"));
      CPPUNIT_ASSERT_EQUAL(std::string("1.7320508"), activeConfigSet1.getProperty("name 2"));

      // "set 2"をアクティブ化した後、アクティブコンフィグレーションセットとして正しく取得できるか？
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.activateConfigurationSet("set 2"));
      const coil::Properties& activeConfigSet2 = configAdmin.getActiveConfigurationSet();
      CPPUNIT_ASSERT_EQUAL(std::string("3.14159"), activeConfigSet2.getProperty("name 1"));
      CPPUNIT_ASSERT_EQUAL(std::string("2.71828"), activeConfigSet2.getProperty("name 2"));
    }
		
    /*!
     * @brief removeConfigurationSet()のテスト
     * 
     * - 登録されているコンフィグレーションセットを正しく登録解除できるか？
     */
    void test_removeConfigurationSet()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // コンフィグレーションセットを追加しておく
      coil::Properties configSet("id");
      configSet.setProperty("key", "value");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet));
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.haveConfig("id"));
			
      // いったん登録したコンフィグレーションセットを登録解除する
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.removeConfigurationSet("id"));
			
      // 当該コンフィグレーションセットが存在しないことを確認する
      CPPUNIT_ASSERT_EQUAL(false, configAdmin.haveConfig("id"));
    }
		
    /*!
     * @brief removeConfigurationSet()のテスト
     * 
     * - 存在しないコンフィグレーションセットIDを指定した場合に、意図どおりに失敗するか？
     */
    void test_removeConfigurationSet_with_inexist_configuration_id()
    {
      coil::Properties nullProp;
      RTC::ConfigAdmin configAdmin(nullProp);
			
      // コンフィグレーションセットを追加しておく
      coil::Properties configSet("id");
      configSet.setProperty("key", "value");
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.addConfigurationSet(configSet));
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.haveConfig("id"));
			
      // 存在しないコンフィグレーションセットIDを指定した場合に、意図どおりに失敗するか？
      CPPUNIT_ASSERT_EQUAL(false, configAdmin.removeConfigurationSet("inexist id"));
			
      // 登録されているコンフィグレーションセットは、元どおり存在しているか？
      CPPUNIT_ASSERT_EQUAL(true, configAdmin.haveConfig("id"));
    }
		
    void test_activateConfigurationSet()
    {
      // test_getActiveConfigurationSet()で兼ねる
    }
		
    /*!
     * @brief setOnUpdate()メソッドのテスト
     * 
     * - 
     */
    void test_setOnUpdate()
    {
      coil::Properties configSet("config_id");
      configSet.setProperty("config_id.key", "value");
      ConfigAdminMock configAdmin(configSet);

      OnUpdateCallbackMock* cdm1 = new OnUpdateCallbackMock();
      OnUpdateCallbackMock* cdm2 = new OnUpdateCallbackMock();

      // 1回目のsetでメンバー変数へ設定
      configAdmin.setOnUpdate(cdm1);
      CPPUNIT_ASSERT(!cdm1->result);
      configAdmin.onUpdateMock("config_id");
      CPPUNIT_ASSERT(cdm1->result);

      // 2回目のsetでdeleteが呼ばれる
      configAdmin.setOnUpdate(cdm2);
      CPPUNIT_ASSERT(!cdm2->result);
      configAdmin.onUpdateMock("config_id");
      CPPUNIT_ASSERT(cdm2->result);

      // delete cdm1; delete cdm2 は不要です。
      // ConfigAdmin::setOn*()の中で、delete m_* を実行しています。
    }
		
    /*!
     * @brief setOnUpdateParam()メソッドのテスト
     * 
     * - 
     */
    void test_setOnUpdateParam()
    {
      coil::Properties configSet("config_id");
      configSet.setProperty("config_id.key", "value");
      ConfigAdminMock configAdmin(configSet);

      OnUpdateParamCallbackMock* cdm1 = new OnUpdateParamCallbackMock();
      OnUpdateParamCallbackMock* cdm2 = new OnUpdateParamCallbackMock();

      // 1回目のsetでメンバー変数へ設定
      configAdmin.setOnUpdateParam(cdm1);
      CPPUNIT_ASSERT(!cdm1->result);
      configAdmin.onUpdateParamMock("config_id", "param1");
      CPPUNIT_ASSERT(cdm1->result);

      // 2回目のsetでdeleteが呼ばれる
      configAdmin.setOnUpdateParam(cdm2);
      CPPUNIT_ASSERT(!cdm2->result);
      configAdmin.onUpdateParamMock("config_id", "param2");
      CPPUNIT_ASSERT(cdm2->result);

      // delete cdm1; delete cdm2 は不要です。
      // ConfigAdmin::setOn*()の中で、delete m_* を実行しています。
    }
		
    /*!
     * @brief setOnSetConfigurationSet()メソッドのテスト
     * 
     * - 
     */
    void test_setOnSetConfigurationSet()
    {
      coil::Properties configSet("config_id");
      configSet.setProperty("config_id.key", "value");
      ConfigAdminMock configAdmin(configSet);
      coil::Properties configSet2("config_id2");
      configSet2.setProperty("config_id2.key", "value2");

      OnSetConfigurationSetCallbackMock* cdm1 = new OnSetConfigurationSetCallbackMock();
      OnSetConfigurationSetCallbackMock* cdm2 = new OnSetConfigurationSetCallbackMock();

      // 1回目のsetでメンバー変数へ設定
      configAdmin.setOnSetConfigurationSet(cdm1);
      CPPUNIT_ASSERT(!cdm1->result);
      configAdmin.onSetConfigurationSetMock(configSet);
      CPPUNIT_ASSERT(cdm1->result);

      // 2回目のsetでdeleteが呼ばれる
      configAdmin.setOnSetConfigurationSet(cdm2);
      CPPUNIT_ASSERT(!cdm2->result);
      configAdmin.onSetConfigurationSetMock(configSet2);
      CPPUNIT_ASSERT(cdm2->result);

      // delete cdm1; delete cdm2 は不要です。
      // ConfigAdmin::setOn*()の中で、delete m_* を実行しています。
    }
		
    /*!
     * @brief setOnAddConfigurationSet()メソッドのテスト
     * 
     * - 
     */
    void test_setOnAddConfigurationSet()
    {
      coil::Properties configSet("config_id");
      configSet.setProperty("config_id.key", "value");
      ConfigAdminMock configAdmin(configSet);
      coil::Properties configSet2("config_id2");
      configSet2.setProperty("config_id2.key", "value2");

      OnAddConfigurationAddCallbackMock* cdm1 = new OnAddConfigurationAddCallbackMock();
      OnAddConfigurationAddCallbackMock* cdm2 = new OnAddConfigurationAddCallbackMock();

      // 1回目のsetでメンバー変数へ設定
      configAdmin.setOnAddConfigurationSet(cdm1);
      CPPUNIT_ASSERT(!cdm1->result);
      configAdmin.onAddConfigurationSetMock(configSet);
      CPPUNIT_ASSERT(cdm1->result);

      // 2回目のsetでdeleteが呼ばれる
      configAdmin.setOnAddConfigurationSet(cdm2);
      CPPUNIT_ASSERT(!cdm2->result);
      configAdmin.onAddConfigurationSetMock(configSet2);
      CPPUNIT_ASSERT(cdm2->result);

      // delete cdm1; delete cdm2 は不要です。
      // ConfigAdmin::setOn*()の中で、delete m_* を実行しています。
    }
		
    /*!
     * @brief setOnRemoveConfigurationSet()メソッドのテスト
     * 
     * - 
     */
    void test_setOnRemoveConfigurationSet()
    {
      coil::Properties configSet("config_id");
      configSet.setProperty("config_id.key", "value");
      ConfigAdminMock configAdmin(configSet);

      OnRemoveConfigurationSetCallbackMock* cdm1 = new OnRemoveConfigurationSetCallbackMock();
      OnRemoveConfigurationSetCallbackMock* cdm2 = new OnRemoveConfigurationSetCallbackMock();

      // 1回目のsetでメンバー変数へ設定
      configAdmin.setOnRemoveConfigurationSet(cdm1);
      CPPUNIT_ASSERT(!cdm1->result);
      configAdmin.onRemoveConfigurationSetMock("config_id");
      CPPUNIT_ASSERT(cdm1->result);

      // 2回目のsetでdeleteが呼ばれる
      configAdmin.setOnRemoveConfigurationSet(cdm2);
      CPPUNIT_ASSERT(!cdm2->result);
      configAdmin.onRemoveConfigurationSetMock("config_id2");
      CPPUNIT_ASSERT(cdm2->result);

      // delete cdm1; delete cdm2 は不要です。
      // ConfigAdmin::setOn*()の中で、delete m_* を実行しています。
    }
		
    /*!
     * @brief setOnActivateSet()メソッドのテスト
     * 
     * - 
     */
    void test_setOnActivateSet()
    {
      coil::Properties configSet("config_id");
      configSet.setProperty("config_id.key", "value");
      ConfigAdminMock configAdmin(configSet);

      OnActivateSetCallbackMock* cdm1 = new OnActivateSetCallbackMock();
      OnActivateSetCallbackMock* cdm2 = new OnActivateSetCallbackMock();

      // 1回目のsetでメンバー変数へ設定
      configAdmin.setOnActivateSet(cdm1);
      CPPUNIT_ASSERT(!cdm1->result);
      configAdmin.onActivateSetMock("config_id");
      CPPUNIT_ASSERT(cdm1->result);

      // 2回目のsetでdeleteが呼ばれる
      configAdmin.setOnActivateSet(cdm2);
      CPPUNIT_ASSERT(!cdm2->result);
      configAdmin.onActivateSetMock("config_id2");
      CPPUNIT_ASSERT(cdm2->result);

      // delete cdm1; delete cdm2 は不要です。
      // ConfigAdmin::setOn*()の中で、delete m_* を実行しています。
    }
		

  };
}; // namespace ConfigAdmin


/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Config::ConfigTests);
CPPUNIT_TEST_SUITE_REGISTRATION(ConfigAdmin::ConfigAdminTests);

#ifdef LOCAL_MAIN
int main(int argc, char* argv[])
{

  FORMAT format = TEXT_OUT;
  int target = 0;
  std::string xsl;
  std::string ns;
  std::string fname;
  std::ofstream ofs;

  int i(1);
  while (i < argc)
    {
      std::string arg(argv[i]);
      std::string next_arg;
      if (i + 1 < argc) next_arg = argv[i + 1];
      else              next_arg = "";

      if (arg == "--text") { format = TEXT_OUT; break; }
      if (arg == "--xml")
	{
	  if (next_arg == "")
	    {
	      fname = argv[0];
	      fname += ".xml";
	    }
	  else
	    {
	      fname = next_arg;
	    }
	  format = XML_OUT;
	  ofs.open(fname.c_str());
	}
      if ( arg == "--compiler"  ) { format = COMPILER_OUT; break; }
      if ( arg == "--cerr"      ) { target = 1; break; }
      if ( arg == "--xsl"       )
	{
	  if (next_arg == "") xsl = "default.xsl"; 
	  else                xsl = next_arg;
	}
      if ( arg == "--namespace" )
	{
	  if (next_arg == "")
	    {
	      std::cerr << "no namespace specified" << std::endl;
	      exit(1); 
	    }
	  else
	    {
	      xsl = next_arg;
	    }
	}
      ++i;
    }
  CppUnit::TextUi::TestRunner runner;
  if ( ns.empty() )
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry().makeTest());
  else
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry(ns).makeTest());
  CppUnit::Outputter* outputter = 0;
  std::ostream* stream = target ? &std::cerr : &std::cout;
  switch ( format )
    {
    case TEXT_OUT :
      outputter = new CppUnit::TextOutputter(&runner.result(),*stream);
      break;
    case XML_OUT :
      std::cout << "XML_OUT" << std::endl;
      outputter = new CppUnit::XmlOutputter(&runner.result(),
					    ofs, "shift_jis");
      static_cast<CppUnit::XmlOutputter*>(outputter)->setStyleSheet(xsl);
      break;
    case COMPILER_OUT :
      outputter = new CppUnit::CompilerOutputter(&runner.result(),*stream);
      break;
    }
  runner.setOutputter(outputter);
  runner.run();
  return 0; // runner.run() ? 0 : 1;
}
#endif // MAIN
#endif // ConfigAdmin_cpp
