// -*- C++ -*-
/*!
 * @file   PropertiesTests.cpp
 * @brief  Properties test class
 * @date   $Date: 2008/01/09 04:12:46 $
 * @author Noriaki Ando <n-ando@aist.go.jp>,
 *         Shinji Kurihara
 *
 *  $Id$
 *
 */

/*
 * $Log: PropertiesTests.cpp,v $
 * Revision 1.3  2008/01/09 04:12:46  arafune
 * *** empty log message ***
 *
 * Revision 1.2  2008/01/08 07:53:11  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2007/12/20 07:50:18  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2006/11/27 08:23:25  n-ando
 * TestSuites are devided into each directory.
 *
 * Revision 1.3  2006/10/18 05:59:55  kurihara
 *
 * test_list(),test_load(),test_store() added and It is a revision generally.
 *
 */

#ifndef Properties_cpp
#define Properties_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <fstream>
#include <coil/Properties.h>
#include <algorithm>

/*!
 * @class PropertiesTests class
 * @brief Properties test
 */
using namespace std;

typedef std::map<std::string, std::string>::iterator Itr;

namespace Properties
{
  class PropertiesTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(PropertiesTests);
		
    CPPUNIT_TEST(test_ctor_map);
    CPPUNIT_TEST(test_Properties_key_value);
    CPPUNIT_TEST(test_Properties_char);
    CPPUNIT_TEST(test_Properties_copy);
    CPPUNIT_TEST(test_Properties_destructor);
    CPPUNIT_TEST(test_substitute);
    CPPUNIT_TEST(test_Properties_destructor);

    CPPUNIT_TEST(test_getName);
    CPPUNIT_TEST(test_getValue);
    CPPUNIT_TEST(test_getDefaultValue);
    CPPUNIT_TEST(test_getLeaf);
    CPPUNIT_TEST(test_getRoot);

    CPPUNIT_TEST(test_getProperty);
    CPPUNIT_TEST(test_getDefault);   
    CPPUNIT_TEST(test_setProperty);
    CPPUNIT_TEST(test_setDefaults);

    CPPUNIT_TEST(test_list);
    CPPUNIT_TEST(test_load);
    CPPUNIT_TEST(test_store);

    CPPUNIT_TEST(test_propertyNames);
    CPPUNIT_TEST(test_size);
    CPPUNIT_TEST(test_findNode);
    CPPUNIT_TEST(test_getNode);
    CPPUNIT_TEST(test_createNode);
    CPPUNIT_TEST(test_removeNode);
    CPPUNIT_TEST(test_hasKey);
    CPPUNIT_TEST(test_clear);
    CPPUNIT_TEST(test_streamInput);
    CPPUNIT_TEST(test_splitKeyValue);


	
    CPPUNIT_TEST_SUITE_END();
		
  private:
    string EMPTY_STRING;
    map<string, string> DEFAULTS_CONF;
		
//    RTC::Properties* m_prop;
    coil::Properties* m_prop;
    
  public:
    /*!
     * @brief Constructor
     */
    PropertiesTests()
    {
    }
		
    /*!
     * @brief Destructor
     */
    ~PropertiesTests()
    {
    }
		
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
      EMPTY_STRING = "";
			
      DEFAULTS_CONF["rtc.openrtm.version"] = "0.4.0";
      DEFAULTS_CONF["rtc.openrtm.release"] = "aist";
      DEFAULTS_CONF["rtc.openrtm.vendor"] = "AIST";
      DEFAULTS_CONF["rtc.openrtm.author"] = "Noriaki Ando";
      DEFAULTS_CONF["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      DEFAULTS_CONF["rtc.manager.debug.level"] = "PARANOID";
      DEFAULTS_CONF["rtc.manager.orb"] = "omniORB";
      DEFAULTS_CONF["rtc.manager.orb.options"] =
        "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      DEFAULTS_CONF["rtc.manager.arch"] = "i386";
      DEFAULTS_CONF["rtc.manager.os"] = "FreeBSD";
      DEFAULTS_CONF["rtc.manager.os.release"] = "6.1-RELEASE";
      DEFAULTS_CONF["rtc.manager.language"] = "C++";
      DEFAULTS_CONF["rtc.manager.subsystems"] =
        "Camera, Manipulator, Force Sensor";
      DEFAULTS_CONF["rtc.component.conf.path"] =
        "C:\\\\Program\\\\ Files\\\\OpenRTM-aist";
      DEFAULTS_CONF["rtc.manager.opening_message"] = "\"Hello \" World\"";
    }
		
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    { 
    }

		
    /*!
     * @brief Properties(const char* key = "", const char* value = "")コンストラクタのテスト
     * 
     * <ul>
     * <li>keyとvalueで指定した値が正しく取得されるか？</li>
     * </ul>
     */
    void test_Properties_key_value()
    {
      /* key,valueによるデフォルト値指定でPropertiesオブジェクトを生成する */
      string value("test");

      /*
      // プロパティの名称と値が、コンストラクタで指定した値で取得されることを確認する
      // (1) キーを階層化しないでテスト
      */
      string key0("manager");
      coil::Properties prop0(key0.c_str(),value.c_str());
      string retkey(prop0.getName());
      CPPUNIT_ASSERT_EQUAL(key0, retkey);
      string retval(prop0.getValue());
      CPPUNIT_ASSERT_EQUAL(value, retval);


      /*
      // (2) キーを階層化してテスト
      */
      string key1("manager.test.test");
      coil::Properties prop1(key1.c_str(),value.c_str());
			
      retkey = prop1.getName();
      CPPUNIT_ASSERT_EQUAL(key1, retkey);
      retval = prop1.getValue();
      CPPUNIT_ASSERT_EQUAL(value, retval);
    }

    /*!
     * @brief Properties(std::map<string, string>&)コンストラクタのテスト
     * 
     * <ul>
     * <li>std::mapで指定したデフォルト値が、デフォルト値として正しく取
     * 得されるか？</li>
     * </ul>
     */
    void test_ctor_map()
    {
      map<string, string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.debug.level"] = "PARANOID";
      defaults["rtc.manager.language"] = "C++";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";
      defaults["rtc.manager.orb.name"] = "omniORB";
      defaults["rtc.manager.orb.options"] =
        "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      defaults["rtc.manager.os.name"] = "FreeBSD";
      defaults["rtc.manager.os.release"] = "6.1-RELEASE";
      defaults["rtc.manager.subsystems"] = "Camera, Manipulator, Force Sensor";
      defaults["rtc.openrtm.author"] = "Noriaki Ando";
      defaults["rtc.openrtm.release"] = "aist";
      defaults["rtc.openrtm.vendor"] = "AIST TaskIntelligence";
      defaults["rtc.openrtm.version"] = "0.4.0";
			
      /* mapによるデフォルト値指定でPropertiesオブジェクトを生成する    */
      coil::Properties prop(defaults);
			
      /* 各プロパティのデフォルト値および通常値がいずれも、コンストラク */
      /* タで指定した値で取得されることを確認する                       */
      map<string, string>::iterator it;
      it = defaults.begin();
      for (; it != defaults.end(); ++it) {
	string key = it->first;
	string value = it->second;
	CPPUNIT_ASSERT_EQUAL(value, prop.getDefault(key));
	CPPUNIT_ASSERT_EQUAL(value, prop.getProperty(key));
      }
    }
		

    void test_Properties_char()
    {
      const char* defaults[] = {
	"rtc.component.conf.path", "C:\\Program\\ Files\\OpenRTM-aist",
	"rtc.manager.arch", "i386",
	"rtc.manager.debug.level", "PARANOID",
	"rtc.manager.language", "C++",
	"rtc.manager.nameserver", "zonu.a02.aist.go.jp",
	"rtc.manager.opening_message", "Hello World",
	"rtc.manager.orb.name", "omniORB",
	"rtc.manager.orb.options", "IIOPAddrPort, -ORBendPoint, giop:tcp:",
	"rtc.manager.os.name", "FreeBSD",
	"rtc.manager.os.release", "6.1-RELEASE",
	"rtc.manager.subsystems", "Camera, Manipulator, Force Sensor",
	"rtc.openrtm.author", "Noriaki Ando",
	"rtc.openrtm.release", "aist",
	"rtc.openrtm.vendor", "AIST TaskIntelligence",
	"rtc.openrtm.version", "0.4.0",
	""
      };
      
      coil::Properties prop(defaults);
      /*
      // 各プロパティのデフォルト値および通常値がいずれも、コンストラクタで指定した値で取得されることを確認する
      */
      for (int i = 0; defaults[i] != ""; i += 2) {
	string key = defaults[i];
	string value = defaults[i + 1];
	CPPUNIT_ASSERT_EQUAL(value, prop.getDefault(key));
	CPPUNIT_ASSERT_EQUAL(value, prop.getProperty(key));
      }
    }
		
    /*!
     * @brief Properties(const Properties& prop)コピーコンストラクタのテスト
     * 
     * <ul>
     * <li>propで指定したデフォルト値が、コピーオブジェクトから正しく取得されるか？</li>
     * </ul>
     */
    void test_Properties_copy()
    {
      map<string, string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.debug.level"] = "PARANOID";
      defaults["rtc.manager.language"] = "C++";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";
      defaults["rtc.manager.orb.name"] = "omniORB";
      defaults["rtc.manager.orb.options"] = "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      defaults["rtc.manager.os.name"] = "FreeBSD";
      defaults["rtc.manager.os.release"] = "6.1-RELEASE";
      defaults["rtc.manager.subsystems"] = "Camera, Manipulator, Force Sensor";
      defaults["rtc.openrtm.author"] = "Noriaki Ando";
      defaults["rtc.openrtm.release"] = "aist";
      defaults["rtc.openrtm.vendor"] = "AIST TaskIntelligence";
      defaults["rtc.openrtm.version"] = "0.4.0";
			
      /* mapによるデフォルト値指定でPropertiesオブジェクトを生成する */
      coil::Properties prop(defaults);
      coil::Properties cprop(prop);


      /* 各プロパティのデフォルト値および通常値がいずれも、コンストラクタで指定した値で取得されることを確認する */
      for (map<string, string>::iterator it = defaults.begin(); it != defaults.end(); it++) {
	string key = it->first;
	string value = it->second;
	CPPUNIT_ASSERT_EQUAL(value, cprop.getDefault(key));
	CPPUNIT_ASSERT_EQUAL(value, cprop.getProperty(key));
      }
    }


    /*!
     * @brief ~Properties(void)デストラクタのテスト
     * 
     * <ul>
     * <li>デストラクタによりオブジェクトからデータが正しく削除されるか？</li>
     * </ul>
     */
    void test_Properties_destructor()
    {
      map<string, string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.debug.level"] = "PARANOID";
      defaults["rtc.manager.language"] = "C++";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";
      defaults["rtc.manager.orb.name"] = "omniORB";
      defaults["rtc.manager.orb.options"] = "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      defaults["rtc.manager.os.name"] = "FreeBSD";
      defaults["rtc.manager.os.release"] = "6.1-RELEASE";
      defaults["rtc.manager.subsystems"] = "Camera, Manipulator, Force Sensor";
      defaults["rtc.openrtm.author"] = "Noriaki Ando";
      defaults["rtc.openrtm.release"] = "aist";
      defaults["rtc.openrtm.vendor"] = "AIST TaskIntelligence";
      defaults["rtc.openrtm.version"] = "0.4.0";
			
      /* mapによるデフォルト値指定でPropertiesオブジェクトを生成する */
      coil::Properties prop(defaults);
      std::vector<coil::Properties*> leaf0(prop.getNode("rtc").getLeaf());
      CPPUNIT_ASSERT(leaf0.size() == 3);

      /* rtc.managerノードを削除する */
      delete &prop.getNode("rtc.manager");

      /* rtcノードからmanagerが削除されている事を確認する。*/
      std::vector<coil::Properties*> leaf1(prop.getNode("rtc").getLeaf());
      CPPUNIT_ASSERT(leaf1.size() == 2);
      std::string getval(prop.getNode("rtc.manager").getValue());
      CPPUNIT_ASSERT("" == getval);
    }

    /*!
     * @brief =演算子（代入演算子）のテスト
     * 
     * <ul>
     * <li>デフォルト値、通常値のいずれも設定されている場合に、それら両
     * 方が正しく代入されるか？</li>
     * <li>デフォルト値のみ設定されており、通常値が設定されていない場合
     * に、それら両方が正しく代入されるか？</li>
     * <li>デフォルト値が設定されていないが、通常値が設定されている場合
     * に、それら両方が正しく代入されるか？</li>
     * </ul>
     */
    void test_substitute()
    {
      /* 代入元となるPropertiesを作成する */
      coil::Properties propSrc;
			
      /* (1) デフォルト値、通常値のいずれも設定されている場合 */
      string key1 = "key1";
      string key1DefaultValue = "key1-default-value";
      string key1Value = "key1-value";
      propSrc.setDefault(key1, key1DefaultValue);
      propSrc.setProperty(key1, key1Value);
			
      /* (2) デフォルト値のみ設定されており、通常値が設定されていない場合 */
      string key2 = "key2";
      string key2DefaultValue = "key2-default-value";
      propSrc.setDefault(key2, key2DefaultValue);
			
      /* (3) デフォルト値が設定されていないが、通常値が設定されている場合 */
      string key3 = "key3";
      string key3Value = "key3-value";
      propSrc.setProperty(key3, key3Value);
			
      /* 代入を行い、それぞれの場合で、正しく代入されたことを確認する */
      coil::Properties prop;
      prop = propSrc;
			
      CPPUNIT_ASSERT_EQUAL(key1DefaultValue, prop.getDefault(key1));
      CPPUNIT_ASSERT_EQUAL(key1Value, prop.getProperty(key1));
      CPPUNIT_ASSERT_EQUAL(key2DefaultValue, prop.getDefault(key2));
      CPPUNIT_ASSERT_EQUAL(key2DefaultValue, prop.getProperty(key2));
      CPPUNIT_ASSERT_EQUAL(EMPTY_STRING, prop.getDefault(key3));
      CPPUNIT_ASSERT_EQUAL(key3Value, prop.getProperty(key3));
    }


    /*!
     * @brief getProperty()メソッドのテスト
     * 
     * <ul>
     * <li>デフォルト値が設定されており、かつ通常の値も設定されている場
     *     合に、プロパティ値として正しく通常の値が取得されるか？</li>
     * <li>デフォルト値が設定されているが、通常の値は設定されていない場
     *     合に、プロパティ値として正しくデフォルト値が取得されるか？</li>
     * <li>デフォルト値は設定されていないが、通常の値は設定されている場
     *     合に、プロパティ値として正しく通常の値が取得されるか？</li>
     * <li>デフォルト値、通常の値のいずれも設定されていない場合に、プロ
     *     パティ値として正しく空文字列が取得されるか？</li>
     * </ul>
     */		
    void test_getProperty()
    {
      coil::Properties prop;
			
      prop.setDefault("property_1", "default_1");
      prop.setProperty("property_1", "value_1");
      prop.setDefault("property_2", "default_2");
      prop.setProperty("property_3", "value_3");
			
      /* (1) デフォルト値が設定されており、かつ通常の値も設定されている */
      /* 場合に、プロパティ値として正しく通常の値が取得されるか？       */
      string expected_1 = "value_1";
      CPPUNIT_ASSERT_EQUAL(expected_1, prop.getProperty("property_1"));
			
      /* (2) デフォルト値が設定されているが、通常の値は設定されていない */
      /*/ 場合に、プロパティ値として正しくデフォルト値が取得されるか？  */
      string expected_2 = "default_2";
      CPPUNIT_ASSERT_EQUAL(expected_2, prop.getProperty("property_2"));
			
      /* (3) デフォルト値は設定されていないが、通常の値は設定されている */
      /* 場合に、プロパティ値として正しく通常の値が取得されるか？       */
      string expected_3 = "value_3";
      CPPUNIT_ASSERT_EQUAL(expected_3, prop.getProperty("property_3"));
			
      /* (4) デフォルト値、通常の値のいずれも設定されていない場合に、プ */
      /* ロパティ値として正しく空文字列が取得されるか？                 */
      string expected_4 = "";
      CPPUNIT_ASSERT_EQUAL(expected_4, prop.getProperty("property_4"));
    }


    /*!
     * @brief getDefault()のテスト
     * 
     * <ul>
     *
     * <li>デフォルト値が設定されており、かつ通常の値も設定されている場
     * 合に、正しく設定されているデフォルト値を取得できるか？</li>
     *
     * <li>デフォルト値が設定されているが、通常の値は設定されていない場
     * 合に、正しく設定されているデフォルト値を取得できるか？</li>
     * 
     * <li>デフォルト値は設定されていないが、通常の値は設定されている場
     * 合に、デフォルト値として空文字列が取得されるか？</li>
     * 
     * <li>デフォルト値、通常の値のいずれも設定されていない場合に、デフォ
     * ルト値として空文字列が取得されるか？</li>
     *
     * </ul>
     */
    void test_getDefault()
    {
      coil::Properties prop;
			
      string key1 = "key1";
      string key1DefaultValue = "key1-default-value";
      string key1Value = "key1-value";
      prop.setDefault(key1, key1DefaultValue);
      prop.setProperty(key1, key1Value);
			
      string key2 = "key2";
      string key2DefaultValue = "key2-default-value";
      prop.setDefault(key2, key2DefaultValue);
			
      string key3 = "key3";
      string key3Value = "key3-value";
      prop.setProperty(key3, key3Value);
			
      /* (1) デフォルト値が設定されており、かつ通常の値も設定されている */
      /* 場合に、正しく設定されているデフォルト値を取得できるか？       */
      CPPUNIT_ASSERT_EQUAL(key1DefaultValue, prop.getDefault(key1));
			
      /* (2) デフォルト値が設定されているが、通常の値は設定されていない */
      /* 場合に、正しく設定されているデフォルト値を取得できるか？       */
      CPPUNIT_ASSERT_EQUAL(key2DefaultValue, prop.getDefault(key2));
			
      /* (3) デフォルト値は設定されていないが、通常の値は設定されている */
      /* 場合に、デフォルト値として空文字列が取得されるか？             */
      CPPUNIT_ASSERT_EQUAL(EMPTY_STRING, prop.getDefault(key3));
			
      /* (4) デフォルト値、通常の値のいずれも設定されていない場合に、デ */
      /* フォルト値として空文字列が取得されるか？                       */
      string keyNonExist = "key-non-exist";
      CPPUNIT_ASSERT_EQUAL(EMPTY_STRING, prop.getDefault(keyNonExist));
    }
		
    /*!
     * @brief setProperty()メソッドのテスト
     * 
     * <ul>
     * <li>設定時に指定した値が、正しく設定されるか？</li>
     * <li>設定時の戻り値として、元の設定値が正しく取得されるか？</li>
     * </ul>
     */
    void test_setProperty()
    {
      string key = "key";
      string oldValue = "old-value";
      string newValue = "new-value";
			
      coil::Properties prop;
      prop.setProperty(key, oldValue);
			
      /* (1) 設定時に指定した値が、正しく設定されるか？ */
      CPPUNIT_ASSERT_EQUAL(oldValue, prop.getProperty(key));

      /* (2) 設定時の戻り値として、元の設定値が正しく取得されるか？ */
      CPPUNIT_ASSERT_EQUAL(oldValue, prop.setProperty(key, newValue));

      /* (1) 設定時に指定した値が、正しく設定されるか？（その２） */
      CPPUNIT_ASSERT_EQUAL(newValue, prop.getProperty(key));
    }


    /*!
     * @brief list()メソッドのテスト
     * 
     * <ul>
     *
     * <li>設定されているプロパティ値が、正しく出力されるか？</li>
     *
     * <li>\（バックスラッシュ）を含む値が、正しくエスケープ処理されて
     * 出力されるか？</li>
     *
     * </ul>
     */
    void test_list()
    {
      coil::Properties prop;
      prop.setProperty("rtc.component.conf.path",
                       "C:\\Program\\ Files\\OpenRTM-aist");
      prop.setProperty("rtc.manager.arch", "i386");
      prop.setProperty("rtc.manager.debug.level", "PARANOID");
      prop.setProperty("rtc.manager.language", "C++");
      prop.setProperty("rtc.manager.nameserver", "zonu.a02.aist.go.jp");
      prop.setProperty("rtc.manager.opening_message", "Hello World");
      prop.setProperty("rtc.manager.orb.name", "omniORB");
      prop.setProperty("rtc.manager.orb.options",
                       "IIOPAddrPort, -ORBendPoint, giop:tcp:");
      prop.setProperty("rtc.manager.os.name", "FreeBSD");
      prop.setProperty("rtc.manager.os.release", "6.1-RELEASE");
      prop.setProperty("rtc.manager.subsystems",
                       "Camera, Manipulator, Force Sensor");
      prop.setProperty("rtc.openrtm.author", "Noriaki Ando");
      prop.setProperty("rtc.openrtm.release", "aist");
      prop.setProperty("rtc.openrtm.vendor", "AIST TaskIntelligence");
      prop.setProperty("rtc.openrtm.version", "0.4.0");
			
      std::ofstream outFile("listed.conf");
      prop.list(outFile);
      outFile.close();
			
      vector<string> expectedLines;
      expectedLines.push_back("rtc.component.conf.path: C:\\\\Program\\\\ Files\\\\OpenRTM-aist");
      expectedLines.push_back("rtc.manager.arch: i386");
      expectedLines.push_back("rtc.manager.debug.level: PARANOID");
      expectedLines.push_back("rtc.manager.language: C++");
      expectedLines.push_back("rtc.manager.nameserver: zonu.a02.aist.go.jp");
      expectedLines.push_back("rtc.manager.opening_message: Hello World");
      expectedLines.push_back("rtc.manager.orb.name: omniORB");
      expectedLines.push_back("rtc.manager.orb.options: IIOPAddrPort, -ORBendPoint, giop:tcp:");
      expectedLines.push_back("rtc.manager.os.name: FreeBSD");
      expectedLines.push_back("rtc.manager.os.release: 6.1-RELEASE");
      expectedLines.push_back("rtc.manager.subsystems: Camera, Manipulator, Force Sensor");
      expectedLines.push_back("rtc.openrtm.author: Noriaki Ando");
      expectedLines.push_back("rtc.openrtm.release: aist");
      expectedLines.push_back("rtc.openrtm.vendor: AIST TaskIntelligence");
      expectedLines.push_back("rtc.openrtm.version: 0.4.0");
			
      vector<string> storedLines;
      std::ifstream inFile("listed.conf");
      string str;
      while (getline(inFile, str))
	{
          storedLines.push_back(str);
        }
      inFile.close();
			
      /* ファイルに書き込まれた内容と期待値を比較する */
      sort(expectedLines.begin(), expectedLines.end());
      sort(storedLines.begin(), storedLines.end());
      CPPUNIT_ASSERT_EQUAL(expectedLines.size(), storedLines.size());
      for (vector<string>::size_type i = 0; i < expectedLines.size(); i++)
        {
          CPPUNIT_ASSERT_EQUAL(expectedLines[i], storedLines[i]);
        }
    }
		
    /*!
     * @brief load()メソッドのテスト
     * <ul>
     *
     * <li>プロパティリストをファイルからPropertiesオブジェクトへ正しく
     * 読み込めるか？</li>
     *
     * </ul>
     */
    void test_load()
    {
      /* プロパティリストをファイルからPropertiesオブジェクトへ読み込む */
      coil::Properties prop;
      std::ifstream ifl("defaults.conf");
      prop.load(ifl);
      ifl.close();
			
      /* 期待値と比較して、正しく読み込まれたことを確認する */
      map<string, string>::iterator expected;
      for (expected = DEFAULTS_CONF.begin();
           expected != DEFAULTS_CONF.end(); expected++) {
	string key = expected->first;
	string value = expected->second;
	CPPUNIT_ASSERT_EQUAL(value, prop.getProperty(key));
      }
    }
		
    /*!
     * @brief store()メソッドのテスト
     * 
     * <ul>
     *
     * <li>設定されているプロパティ値が、正しく出力されるか？</li>
     *
     * <li>指定したヘッダ文字列が、正しく出力されるか？</li>
     *
     * <li>\（バックスラッシュ）を含む値が、正しくエスケープ処理されて
     * 出力されるか？</li>
     *
     * </ul>
     */
    void test_store()
    {
      /* テスト用のプロパティを設定する */
      coil::Properties prop;
      prop.setProperty("rtc.component.conf.path",
                       "C:\\Program\\ Files\\OpenRTM-aist");
      prop.setProperty("rtc.manager.arch", "i386");
      prop.setProperty("rtc.manager.debug.level", "PARANOID");
      prop.setProperty("rtc.manager.language", "C++");
      prop.setProperty("rtc.manager.nameserver", "zonu.a02.aist.go.jp");
      prop.setProperty("rtc.manager.opening_message", "Hello World");
      prop.setProperty("rtc.manager.orb.name", "omniORB");
      prop.setProperty("rtc.manager.orb.options",
                       "IIOPAddrPort, -ORBendPoint, giop:tcp:");
      prop.setProperty("rtc.manager.os.name", "FreeBSD");
      prop.setProperty("rtc.manager.os.release", "6.1-RELEASE");
      prop.setProperty("rtc.manager.subsystems",
                       "Camera, Manipulator, Force Sensor");
      prop.setProperty("rtc.openrtm.author", "Noriaki Ando");
      prop.setProperty("rtc.openrtm.release", "aist");
      prop.setProperty("rtc.openrtm.vendor", "AIST TaskIntelligence");
      prop.setProperty("rtc.openrtm.version", "0.4.0");
      prop.setProperty("hello", "\tHello\t");
			
      std::ofstream outFile("stored.conf");
      prop.store(outFile, "stored data");
      outFile.close();
			
      /* 比較のための期待値を準備する */
      vector<string> expectedLines;
      expectedLines.push_back("# stored data");
      expectedLines.push_back("rtc.component.conf.path: C:\\\\Program\\\\ Files\\\\OpenRTM-aist");
      expectedLines.push_back("rtc.manager.arch: i386");
      expectedLines.push_back("rtc.manager.debug.level: PARANOID");
      expectedLines.push_back("rtc.manager.language: C++");
      expectedLines.push_back("rtc.manager.nameserver: zonu.a02.aist.go.jp");
      expectedLines.push_back("rtc.manager.opening_message: Hello World");
      expectedLines.push_back("rtc.manager.orb.name: omniORB");
      expectedLines.push_back("rtc.manager.orb.options: IIOPAddrPort, -ORBendPoint, giop:tcp:");
      expectedLines.push_back("rtc.manager.os.name: FreeBSD");
      expectedLines.push_back("rtc.manager.os.release: 6.1-RELEASE");
      expectedLines.push_back("rtc.manager.subsystems: Camera, Manipulator, Force Sensor");
      expectedLines.push_back("rtc.openrtm.author: Noriaki Ando");
      expectedLines.push_back("rtc.openrtm.release: aist");
      expectedLines.push_back("rtc.openrtm.vendor: AIST TaskIntelligence");
      expectedLines.push_back("rtc.openrtm.version: 0.4.0");
      expectedLines.push_back("hello: \\tHello\\t");
			
      /* 出力されたファイルの中身を１行づつ読み込む */
      vector<string> storedLines;
      std::ifstream inFile("stored.conf");
      string str;
      while (getline(inFile, str))	{
	storedLines.push_back(str);
      }
      inFile.close();
			
      /* ファイルに書き込まれた内容と期待値を比較する */
      sort(expectedLines.begin(), expectedLines.end());
      sort(storedLines.begin(), storedLines.end());
      CPPUNIT_ASSERT_EQUAL(expectedLines.size(), storedLines.size());
      for (vector<string>::size_type i = 0; i < expectedLines.size(); i++) {
	CPPUNIT_ASSERT_EQUAL(expectedLines[i], storedLines[i]);
      }
    }


    /*!
     * @brief propertyNames()メソッドのテスト
     * 
     * <ul>
     *
     * <li>通常のプロパティ値とデフォルト値の両方が設定されているプロパ
     * ティについて、キー名が取得されるか？</li>
     *
     * <li>通常のプロパティ値のみが設定されているプロパティについて、キー
     * 名が取得されるか？</li>
     *
     * <li>デフォルト値のみが設定されているプロパティについて、キー名が
     * 取得されるか？</li>
     *
     * </ul>
     */
    void test_propertyNames()
    {
      coil::Properties prop;
			
      /* (1) 通常のプロパティ値とデフォルト値の両方を設定する */
      prop.setProperty("property_01", "value_01");
      prop.setDefault("property_01", "default_01");
			
      /* (2) 通常のプロパティ値のみを設定する */
      prop.setProperty("property_02", "value_02");
			
      /* (3) デフォルト値のみを設定する */
      prop.setDefault("property_03", "default_03");
			
      /* (1),(2),(3)いずれの場合についてもキー名が取得されることを確認する */
      vector<string> keys = prop.propertyNames();
      CPPUNIT_ASSERT_EQUAL(3, (int) keys.size());
      CPPUNIT_ASSERT(find(keys.begin(), keys.end(), "property_01")
                     != keys.end());
      CPPUNIT_ASSERT(find(keys.begin(), keys.end(), "property_02")
                     != keys.end());
      CPPUNIT_ASSERT(find(keys.begin(), keys.end(), "property_03")
                     != keys.end());
    }


    /*!
     * @brief getName(void)メソッドのテスト
     * 
     * <ul>
     * <li>コンストラクタで指定したプロパティの名称が正しく取得されるか？</li>
     * </ul>
     */
    void test_getName()
    {

      string value("test");

      /* (1) キーを階層化していない場合 */
      /* key,valueによるデフォルト値指定でPropertiesオブジェクトを生成する */
      string key0("manager");
      coil::Properties prop0(key0.c_str(),value.c_str());
      string retkey(prop0.getName());
      CPPUNIT_ASSERT_EQUAL(key0, retkey);

      /* (1) キーを階層化した場合 */
      /* key,valueによるデフォルト値指定でPropertiesオブジェクトを生成する */
      string key1("manager.hoge");
      coil::Properties prop1(key1.c_str(),value.c_str());
      retkey= prop1.getName();
      CPPUNIT_ASSERT_EQUAL(key1, retkey);
    }

    /*!
     * @brief getValue(void)メソッドのテスト
     * 
     * <ul>
     * <li>コンストラクタで指定したプロパティの値が正しく取得されるか？</li>
     * </ul>
     */
    void test_getValue()
    {
      /* key,valueによるデフォルト値指定でPropertiesオブジェクトを生成する */
      string key("manager");
      string value("test.test");
      coil::Properties prop(key.c_str(),value.c_str());
			
      /* コンストラクタで指定したプロパティの値が取得されることを確認する */
      string retval(prop.getValue());
      CPPUNIT_ASSERT_EQUAL(value, retval);
    }

    /*!
     * @brief getDefaultValue(void)メソッドのテスト
     * 
     * <ul>
     * <li>setDefaultで指定したプロパティの値が正しく取得されるか？</li>
     * </ul>
     */
    void test_getDefaultValue()
    {
      coil::Properties prop;

      string key("property_1");
      string value("default_1");
      /* デフォルト値を設定する */
      prop.setDefault(key.c_str(), value.c_str());

      /* デフォルト値を取得する */
      string retval(prop.getNode(key.c_str()).getDefaultValue());
      CPPUNIT_ASSERT_EQUAL(value, retval);
    }

    /*!
     * @brief getLeaf(void)メソッドのテスト
     * 
     * <ul>
     * <li>getLeafで子要素が正しく取得されるか？</li>
     * </ul>
     */
    void test_getLeaf()
    {
      map<string, string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.debug.level"] = "PARANOID";
      defaults["rtc.manager.language"] = "C++";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";
      defaults["rtc.manager.orb.name"] = "omniORB";
      defaults["rtc.manager.orb.options"] = "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      defaults["rtc.manager.os.name"] = "FreeBSD";
      defaults["rtc.manager.os.release"] = "6.1-RELEASE";
      defaults["rtc.manager.subsystems"] = "Camera, Manipulator, Force Sensor";
      defaults["rtc.openrtm.author"] = "Noriaki Ando";
      defaults["rtc.openrtm.release"] = "aist";
      defaults["rtc.openrtm.vendor"] = "AIST TaskIntelligence";
      defaults["rtc.openrtm.version"] = "0.4.0";
			
      /* mapによるデフォルト値指定でPropertiesオブジェクトを生成する */
      coil::Properties prop(defaults);

      /* getLeaf()で子要素を取得 */
      std::vector<coil::Properties*> leaf(prop.getNode("rtc").getLeaf());
      CPPUNIT_ASSERT(leaf[1]->getProperty("arch") == "i386");
      CPPUNIT_ASSERT(leaf[1]->getProperty("debug.level") == "PARANOID");
    }

    /*!
     * @brief getRoot(void)メソッドのテスト
     * 
     * <ul>
     * <li>getRootでルート要素が正しく取得されるか？</li>
     * </ul>
     */
    void test_getRoot()
    {
      map<string, string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.debug.level"] = "PARANOID";
      defaults["rtc.manager.language"] = "C++";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";
      defaults["rtc.manager.orb.name"] = "omniORB";
      defaults["rtc.manager.orb.options"] = "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      defaults["rtc.manager.os.name"] = "FreeBSD";
      defaults["rtc.manager.os.release"] = "6.1-RELEASE";
      defaults["rtc.manager.subsystems"] = "Camera, Manipulator, Force Sensor";
      defaults["rtc.openrtm.author"] = "Noriaki Ando";
      defaults["rtc.openrtm.release"] = "aist";
      defaults["rtc.openrtm.vendor"] = "AIST TaskIntelligence";
      defaults["rtc.openrtm.version"] = "0.4.0";
			
      /* mapによるデフォルト値指定でPropertiesオブジェクトを生成する */
      coil::Properties prop(defaults);

      /* getLeaf()でルート要素を取得 */
      const coil::Properties* root(prop.getNode("rtc.manager.os").getRoot());
      CPPUNIT_ASSERT(root->getProperty("arch") == "i386");
      CPPUNIT_ASSERT(root->getProperty("debug.level") == "PARANOID");
    }

    /*!
     * @brief operator[]()メソッドのテスト
     * 
     * <ul>
     * <li>指定されたキーを持つプロパティを、プロパティリストから探せるか？</li>
     * </ul>
     */
    void test_operator()
    {
      map<string, string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.debug.level"] = "PARANOID";
      defaults["rtc.manager.language"] = "C++";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";
      defaults["rtc.manager.orb.name"] = "omniORB";
      defaults["rtc.manager.orb.options"] = "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      defaults["rtc.manager.os.name"] = "FreeBSD";
      defaults["rtc.manager.os.release"] = "6.1-RELEASE";
      defaults["rtc.manager.subsystems"] = "Camera, Manipulator, Force Sensor";
      defaults["rtc.openrtm.author"] = "Noriaki Ando";
      defaults["rtc.openrtm.release"] = "aist";
      defaults["rtc.openrtm.vendor"] = "AIST TaskIntelligence";
      defaults["rtc.openrtm.version"] = "0.4.0";
			
      //  const std::string& 
      //     Properties::operator[](const std::string& key) const
      //  のテスト
      const coil::Properties prop0(defaults);
      const string& retval0(prop0["rtc.manager.arch"]);
      CPPUNIT_ASSERT(retval0 == "i386");

      //  std::string& 
      //     Properties::operator[](const std::string& key)
      //  のテスト
      coil::Properties prop1(defaults);
      string retval1(prop1["rtc.manager.arch"]);
      CPPUNIT_ASSERT(retval1 == "i386");
      
    }

    /*!
     * @brief setDefaults()メソッドのテスト
     * 
     * <ul>
     * <li>デフォルト値を登録し、その値が正しく取得できるか？</li>
     * </ul>
     */
    void test_setDefaults()
    {
      const char* defaults[] = {
	"rtc.component.conf.path", "C:\\Program\\ Files\\OpenRTM-aist",
	"rtc.manager.arch", "i386",
	"rtc.manager.debug.level", "PARANOID",
	"rtc.manager.language", "C++",
	"rtc.manager.nameserver", "zonu.a02.aist.go.jp",
	"rtc.manager.opening_message", "Hello World",
	"rtc.manager.orb.name", "omniORB",
	"rtc.manager.orb.options", "IIOPAddrPort, -ORBendPoint, giop:tcp:",
	"rtc.manager.os.name", "FreeBSD",
	"rtc.manager.os.release", "6.1-RELEASE",
	"rtc.manager.subsystems", "Camera, Manipulator, Force Sensor",
	"rtc.openrtm.author", "Noriaki Ando",
	"rtc.openrtm.release", "aist",
	"rtc.openrtm.vendor", "AIST TaskIntelligence",
	"rtc.openrtm.version", "0.4.0",
	""
      };
      
      coil::Properties prop;
      prop.setDefaults(defaults);

      /*
      // 各プロパティのデフォルト値がsetDefaultsで指定した値で取得されることを確認する
      */
      for (int i = 0; defaults[i] != ""; i += 2) {
	string key = defaults[i];
	string value = defaults[i + 1];
	CPPUNIT_ASSERT_EQUAL(value, prop.getDefault(key));
      }
    }

    /*!
     * @brief size()メソッドのテスト
     * 
     * <ul>
     * <li>プロパティの数が正しく取得できるか？</li>
     * </ul>
     */
    void test_size()
    {
      map<string, string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.debug.level"] = "PARANOID";
      defaults["rtc.manager.language"] = "C++";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";
      defaults["rtc.manager.orb.name"] = "omniORB";
      defaults["rtc.manager.orb.options"] = "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      defaults["rtc.manager.os.name"] = "FreeBSD";
      defaults["rtc.manager.os.release"] = "6.1-RELEASE";
      defaults["rtc.manager.subsystems"] = "Camera, Manipulator, Force Sensor";
      defaults["rtc.openrtm.author"] = "Noriaki Ando";
      defaults["rtc.openrtm.release"] = "aist";
      defaults["rtc.openrtm.vendor"] = "AIST TaskIntelligence";
      defaults["rtc.openrtm.version"] = "0.4.0";
			
      coil::Properties prop(defaults);

      /* (1) コンストラクタ引数にて指定したプロパティの数が正しく取得できるか？ */
      CPPUNIT_ASSERT(15 == prop.size());

      /* (2) 新たに追加後のプロパティの数が正しく取得できるか？ */
      string key = "key";
      string newValue = "new-value";
      prop.setProperty(key, newValue);
      CPPUNIT_ASSERT(16 == prop.size());
    }

    /*!
     * @brief findNode()メソッドのテスト
     * 
     * <ul>
     * <li>キーで指定したノードが正しく検索できるか？</li>
     * </ul>
     */
    void test_findNode()
    {
      map<string, string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.debug.level"] = "PARANOID";
      defaults["rtc.manager.language"] = "C++";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";
      defaults["rtc.manager.orb.name"] = "omniORB";
      defaults["rtc.manager.orb.options"] = "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      defaults["rtc.manager.os.name"] = "FreeBSD";
      defaults["rtc.manager.os.release"] = "6.1-RELEASE";
      defaults["rtc.manager.subsystems"] = "Camera, Manipulator, Force Sensor";
      defaults["rtc.openrtm.author"] = "Noriaki Ando";
      defaults["rtc.openrtm.release"] = "aist";
      defaults["rtc.openrtm.vendor"] = "AIST TaskIntelligence";
      defaults["rtc.openrtm.version"] = "0.4.0";
			
      /* mapによるデフォルト値指定でPropertiesオブジェクトを生成する */
      coil::Properties prop(defaults);

      /* (1) 存在するノードを指定しノードを取得 */
      CPPUNIT_ASSERT(prop.findNode("rtc.manager")->getProperty("arch") == "i386");
      CPPUNIT_ASSERT(prop.findNode("rtc.manager")->getProperty("debug.level") == "PARANOID");
      CPPUNIT_ASSERT(prop.findNode("rtc.manager") != NULL);

      /* (2) 存在しないノードを指定しノードを取得 */
      CPPUNIT_ASSERT(prop.findNode("manager") == NULL);
      CPPUNIT_ASSERT(prop.findNode("rtc.MANAGER") == NULL);
    }

    /*!
     * @brief getNode()メソッドのテスト
     * 
     * <ul>
     * <li>キーで指定したノードが正しく取得できるか？</li>
     * </ul>
     */
    void test_getNode()
    {
      map<string, string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.debug.level"] = "PARANOID";
      defaults["rtc.manager.language"] = "C++";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";
      defaults["rtc.manager.orb.name"] = "omniORB";
      defaults["rtc.manager.orb.options"] = "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      defaults["rtc.manager.os.name"] = "FreeBSD";
      defaults["rtc.manager.os.release"] = "6.1-RELEASE";
      defaults["rtc.manager.subsystems"] = "Camera, Manipulator, Force Sensor";
      defaults["rtc.openrtm.author"] = "Noriaki Ando";
      defaults["rtc.openrtm.release"] = "aist";
      defaults["rtc.openrtm.vendor"] = "AIST TaskIntelligence";
      defaults["rtc.openrtm.version"] = "0.4.0";
			
      /* mapによるデフォルト値指定でPropertiesオブジェクトを生成する */
      coil::Properties prop(defaults);

      /* (1) 既に存在するノードを取得 */
      CPPUNIT_ASSERT(prop.getNode("rtc.manager").getProperty("arch") == "i386");
      CPPUNIT_ASSERT(prop.getNode("rtc.manager").getProperty("debug.level") == "PARANOID");

      /* (2) 存在しないノードを取得 */
      std::string getval(prop.getNode("manager").getValue());
      CPPUNIT_ASSERT(getval == "");
//      CPPUNIT_ASSERT(prop.getNode("rtc.manager") != NULL);
    }

    /*!
     * @brief removeNode()メソッドのテスト
     * 
     * <ul>
     * <li>指定したノードが正しく削除できるか？</li>
     * </ul>
     */
    void test_removeNode()
    {
      map<string, string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.debug.level"] = "PARANOID";
      defaults["rtc.manager.language"] = "C++";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";
      defaults["rtc.manager.orb.name"] = "omniORB";
      defaults["rtc.manager.orb.options"] = "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      defaults["rtc.manager.os.name"] = "FreeBSD";
      defaults["rtc.manager.os.release"] = "6.1-RELEASE";
      defaults["rtc.manager.subsystems"] = "Camera, Manipulator, Force Sensor";
      defaults["rtc.openrtm.author"] = "Noriaki Ando";
      defaults["rtc.openrtm.release"] = "aist";
      defaults["rtc.openrtm.vendor"] = "AIST TaskIntelligence";
      defaults["rtc.openrtm.version"] = "0.4.0";
			
      /* mapによるデフォルト値指定でPropertiesオブジェクトを生成する */
      coil::Properties prop(defaults);

      /* "rtc.manager"のように指定した場合、削除できない。 */
      CPPUNIT_ASSERT(prop.removeNode("rtc.manager") == NULL);

      /* "rtc.manager"ノードの削除 */
      CPPUNIT_ASSERT(prop.getNode("rtc").removeNode("manager") != NULL);

      /* (1) 削除したノードを検索 */
      CPPUNIT_ASSERT(prop.findNode("rtc.manager") == NULL);

      /* (2) 削除したノードを取得 */
      std::string getval0(prop.getNode("rtc.manager").getValue());
      CPPUNIT_ASSERT(getval0 == "");
    }

    /*!
     * @brief hasKey()メソッドのテスト
     * 
     * <ul>
     * <li>子ノードにkeyがあるかどうか？</li>
     * </ul>
     */
    void test_hasKey()
    {
      map<string, string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.debug.level"] = "PARANOID";
      defaults["rtc.manager.language"] = "C++";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";
      defaults["rtc.manager.orb.name"] = "omniORB";
      defaults["rtc.manager.orb.options"] = "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      defaults["rtc.manager.os.name"] = "FreeBSD";
      defaults["rtc.manager.os.release"] = "6.1-RELEASE";
      defaults["rtc.manager.subsystems"] = "Camera, Manipulator, Force Sensor";
      defaults["rtc.openrtm.author"] = "Noriaki Ando";
      defaults["rtc.openrtm.release"] = "aist";
      defaults["rtc.openrtm.vendor"] = "AIST TaskIntelligence";
      defaults["rtc.openrtm.version"] = "0.4.0";
			
      /* mapによるデフォルト値指定でPropertiesオブジェクトを生成する */
      coil::Properties prop(defaults);

      /* (1) 存在するキーでテスト */
      CPPUNIT_ASSERT(prop.hasKey("rtc") != NULL);

      /* (2) 存在しないキーでテスト */
      CPPUNIT_ASSERT(prop.hasKey("hoge") == NULL);

      /* (3) 存在するキーでテスト */
      CPPUNIT_ASSERT(prop.getNode("rtc").hasKey("manager") != NULL);

      /* (4) 存在しないキーでテスト */
      CPPUNIT_ASSERT(prop.getNode("rtc").hasKey("hoge") == NULL);
    }

    /*!
     * @brief clear()メソッドのテスト
     * 
     * <ul>
     * <li>子ノードがすべて削除されるかどうか？</li>
     * </ul>
     */
    void test_clear()
    {
      map<string, string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.debug.level"] = "PARANOID";
      defaults["rtc.manager.language"] = "C++";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";
      defaults["rtc.manager.orb.name"] = "omniORB";
      defaults["rtc.manager.orb.options"] = "IIOPAddrPort, -ORBendPoint, giop:tcp:";
      defaults["rtc.manager.os.name"] = "FreeBSD";
      defaults["rtc.manager.os.release"] = "6.1-RELEASE";
      defaults["rtc.manager.subsystems"] = "Camera, Manipulator, Force Sensor";
      defaults["rtc.openrtm.author"] = "Noriaki Ando";
      defaults["rtc.openrtm.release"] = "aist";
      defaults["rtc.openrtm.vendor"] = "AIST TaskIntelligence";
      defaults["rtc.openrtm.version"] = "0.4.0";
			
      /* mapによるデフォルト値指定でPropertiesオブジェクトを生成する */
      coil::Properties prop(defaults);

      prop.getNode("rtc.manager.os").clear();

      /* (1) "rtc.manager.os"以下のノードが削除されているか？ */
      CPPUNIT_ASSERT(prop.findNode("rtc.manager.os.name") == NULL);
      CPPUNIT_ASSERT(prop.findNode("rtc.manager.os.release") == NULL);
      
      /* (2) "rtc.manager.os"以外のノードは削除されていないか？ */
      CPPUNIT_ASSERT(prop.findNode("rtc.manager.arch") != NULL);
      CPPUNIT_ASSERT(prop.findNode("rtc.manager.orb") != NULL);

      /* (3) すべてのノードが削除されているか？ */
      prop.clear();
      CPPUNIT_ASSERT(prop.findNode("rtc.manager.os") == NULL);
      CPPUNIT_ASSERT(prop.findNode("rtc.manager") == NULL);
      CPPUNIT_ASSERT(prop.findNode("rtc.openrtm") == NULL);
      CPPUNIT_ASSERT(prop.findNode("rtc.component") == NULL);
      CPPUNIT_ASSERT(prop.findNode("rtc") == NULL);
    }


    /*!
     * @brief createNode()のテスト
     * 
     * <ul>
     *
     * <li>デフォルト値が設定されており、かつ通常の値も設定されている場
     * 合に、当該キー名の新規ノード作成が、意図どおり失敗するか？</li>
     *
     * <li>デフォルト値が設定されているが、通常の値は設定されていない場
     * 合に、当該キー名の新規ノード作成が、意図どおり失敗するか？</li>
     *
     * <li>デフォルト値は設定されていないが、通常の値は設定されている場
     * 合に、当該キー名の新規ノード作成が、意図どおり失敗するか？</li>
     *
     * <li>デフォルト値、通常の値のいずれも設定されていない場合に、新規
     * ノード作成が成功するか？</li>
     *
     * </ul>
     */
    void test_createNode()
    {
      coil::Properties prop;
			
      string key1 = "key1";
      string key1DefaultValue = "key1-default-value";
      string key1Value = "key1-value";
      prop.setDefault(key1, key1DefaultValue);
      prop.setProperty(key1, key1Value);
			
      string key2 = "key2";
      string key2DefaultValue = "key2-default-value";
      prop.setDefault(key2, key2DefaultValue);
			
      string key3 = "key3";
      string key3Value = "key3-value";
      prop.setProperty(key3, key3Value);
			
      /*
      // (1) デフォルト値が設定されており、かつ通常の値も設定されている
      // 場合に、当該キー名の新規ノード作成が、意図どおり失敗するか？
      */
      CPPUNIT_ASSERT(! prop.createNode(key1.c_str()));
			
      /*
      // (2) デフォルト値が設定されているが、通常の値は設定されていない
      // 場合に、当該キー名の新規ノード作成が、意図どおり失敗するか？
      */
      CPPUNIT_ASSERT(! prop.createNode(key2.c_str()));
			
      /*
      // (3) デフォルト値は設定されていないが、通常の値は設定されている
      // 場合に、当該キー名の新規ノード作成が、意図どおり失敗するか？
      */
      CPPUNIT_ASSERT(! prop.createNode(key3.c_str()));
			
      /*
      // (4) デフォルト値、通常の値のいずれも設定されていない場合に、新
      // 規ノード作成が成功するか？
      */
      string keyNonExist = "key-non-exist";
      CPPUNIT_ASSERT(prop.createNode(keyNonExist.c_str()));
    }


    /*!
     * @brief <<演算子のテスト
     * 
     * <ul>
     *
     * <li>デフォルト値は、入力されないことを確認する</li>
     *
     * <li>２つのPropertiesのうち片方だけに設定されているキーに関して、
     * 設定されている通常値が、入力前後で変化しないか？</li>
     *
     * <li>２つのPropertiesの両方に共通しているキーに関して、設定されて
     * いる通常値が入力した値で上書きされるか？</li>
     *
     * </ul>
     */
    void test_streamInput()
    {
      /* 入力元となるPropertiesの１つ目を作成する */
      coil::Properties propA;
			
      string keyA1 = "keyA1";
      string keyA1DefaultValue = "keyA1-default-value";
      string keyA1Value = "keyA1-value";
      propA.setDefault(keyA1, keyA1DefaultValue);
      propA.setProperty(keyA1, keyA1Value);
			
      string keyA2 = "keyA2";
      string keyA2DefaultValue = "keyA2-default-value";
      propA.setDefault(keyA2, keyA2DefaultValue);
			
      string keyA3 = "keyA3";
      string keyA3Value = "keyA3-value";
      propA.setProperty(keyA3, keyA3Value);
			
      /* 入力元となるPropertiesの２つ目を作成する */
      coil::Properties propB;
			
      string keyB1 = "keyB1";
      string keyB1DefaultValue = "keyB1-default-value";
      string keyB1Value = "keyB1-value";
      propB.setDefault(keyB1, keyB1DefaultValue);
      propB.setProperty(keyB1, keyB1Value);
			
      string keyB2 = "keyB2";
      string keyB2DefaultValue = "keyB2-default-value";
      propB.setDefault(keyB2, keyB2DefaultValue);

      string keyB3 = "keyB3";
      string keyB3Value = "keyB3-value";
      propB.setProperty(keyB3, keyB3Value);
			
      /*
      // ２つのPropertiesに、共通するキー名で互いに異なる値を設定しておく
      */
      string keyCommon = "keyCommon";
      string keyCommonValueA = "keyCommon-value-A";
      string keyCommonValueB = "keyCommon-value-B";
      propA.setProperty(keyCommon, keyCommonValueA);
      propB.setProperty(keyCommon, keyCommonValueB);

      /* propBをpropAに入力する */
      propA << propB;
			
      /* 正しくマージされたことを確認する */
      /* (1) 入力前から設定されており、共通キーでないものは、もとのまま */
      /* であることを確認する                                           */
      CPPUNIT_ASSERT_EQUAL(keyA1DefaultValue, propA.getDefault(keyA1));
      CPPUNIT_ASSERT_EQUAL(keyA1Value, propA.getProperty(keyA1));
      CPPUNIT_ASSERT_EQUAL(keyA2DefaultValue, propA.getDefault(keyA2));
      CPPUNIT_ASSERT_EQUAL(keyA2DefaultValue, propA.getProperty(keyA2));
      CPPUNIT_ASSERT_EQUAL(EMPTY_STRING, propA.getDefault(keyA3));
      CPPUNIT_ASSERT_EQUAL(keyA3Value, propA.getProperty(keyA3));

      /* (2) 通常値は入力され、デフォルト値は入力されないことを確認する */
      CPPUNIT_ASSERT_EQUAL(EMPTY_STRING, propA.getDefault(keyB1));
      CPPUNIT_ASSERT_EQUAL(keyB1Value, propA.getProperty(keyB1));
      CPPUNIT_ASSERT_EQUAL(EMPTY_STRING, propA.getDefault(keyB2));
      CPPUNIT_ASSERT_EQUAL(keyB2DefaultValue, propA.getProperty(keyB2));
      CPPUNIT_ASSERT_EQUAL(EMPTY_STRING, propA.getDefault(keyB3));
      CPPUNIT_ASSERT_EQUAL(keyB3Value, propA.getProperty(keyB3));
			
      /* (3) 共通キー名について、入力した値で上書きされていることを確認する */
      CPPUNIT_ASSERT_EQUAL(keyCommonValueB, propA.getProperty(keyCommon));
    }

    /*!
     * @brief splitKeyValue()メソッドのテスト
     * 
     * <ul>
     *
     * <li>キーの前に空白文字を含む場合について、キーと値を正しく分離で
     * きるか？</li>
     *
     * <li>キーとデリミタの間に空白文字を含む場合について、キーと値を正
     * しく分離できるか？</li>
     *
     * <li>デリミタと値の間に空白文字を含む場合について、キーと値を正し
     * く分離できるか？</li>
     *
     *a <li>値の後ろに空白文字を含む場合について、キーと値を正しく分離で
     * きるか？</li>
     *
     * </ul>
     */
    void test_splitKeyValue()
    {
      class P : public coil::Properties
      {
      public:
	void splitKeyValue_protected(const std::string& str,
                                     std::string& key, std::string& value)
	{
	  splitKeyValue(str, key, value);
	}
      };
			
      string keyAndValue = " property_name : C:\\abc\\pqr.xyz ";
			
      string key, value;
      P prop;
      prop.splitKeyValue_protected(keyAndValue, key, value);
			
      /* キーと値が、余分な空白文字が除去されたうえで分離されていること */
      /* を確認する                                                     */
      string expectedKey("property_name");
      CPPUNIT_ASSERT_EQUAL(expectedKey, key);

      string expectedValue("C:\\abc\\pqr.xyz");
      CPPUNIT_ASSERT_EQUAL(expectedValue, value);
    }
    

		
  };
}; // namespace Properties

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Properties::PropertiesTests);

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
#endif // Properties_cpp
