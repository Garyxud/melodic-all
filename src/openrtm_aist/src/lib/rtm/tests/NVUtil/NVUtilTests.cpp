// -*- C++ -*-
/*!
 * @file   NVUtilTests.cpp
 * @brief  NVUtil test class
 * @date   $Date: 2008/03/03 12:24:24 $
 * @author Shinji Kurihara
 *         Noriaki Ando <n-ando@aist.go.jp>
 * 
 * Copyright (C) 2006-2008
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

/*
 * $Log: NVUtilTests.cpp,v $
 * Revision 1.5  2008/03/03 12:24:24  arafune
 * Added some tests.
 *
 * Revision 1.4  2008/01/24 01:51:58  tsakamoto
 * *** empty log message ***
 *
 * Revision 1.3  2007/12/27 10:28:16  arafune
 * *** empty log message ***
 *
 * Revision 1.2  2007/12/25 12:13:38  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2007/12/20 07:50:18  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2006/11/27 08:26:13  n-ando
 * TestSuites are devided into each directory.
 *
 * Revision 1.3  2006/11/15 09:47:18  kurihara
 * tests for find(),isString(),toString() are added.
 *
 * Revision 1.1  2006/11/14 07:23:16  kurihara
 * test program for NVUtil module. first commit.
 *
 */

#ifndef NVUtil_cpp
#define NVUtil_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/NVUtil.h>

namespace NVUtil
{
  using namespace NVUtil;
  using namespace std;

  int g_argc;
  vector<string> g_argv;
  
  /*!
   * @class NVUtilTests class
   * @brief NVUtil Test
   */
  class NVUtilTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(NVUtilTests);
    CPPUNIT_TEST(test_newNV_Short);
    CPPUNIT_TEST(test_newNV_Long);
    CPPUNIT_TEST(test_newNV_Float);
    CPPUNIT_TEST(test_newNV_Double);
    CPPUNIT_TEST(test_newNV_Str);
    CPPUNIT_TEST(test_newNVChar);
    CPPUNIT_TEST(test_newNVBool);
    CPPUNIT_TEST(test_newNVOctet);
    CPPUNIT_TEST(test_copy);
    CPPUNIT_TEST(test_toProperties);
    CPPUNIT_TEST(test_copyToProperties);
    CPPUNIT_TEST(test_find);
    CPPUNIT_TEST(test_find_index);
    CPPUNIT_TEST(test_isString);
    CPPUNIT_TEST(test_isStringValue);
    CPPUNIT_TEST(test_toString);
    CPPUNIT_TEST(test_appendStringValue);
    CPPUNIT_TEST(test_append);
    CPPUNIT_TEST(test_dump);
    CPPUNIT_TEST(test_toStringNV);
    CPPUNIT_TEST_SUITE_END();

  private:

  public:
    NVUtilTests()
    {
      CORBA::ORB_var orb;
      char* argv[g_argc];
      for (int i = 0; i < g_argc; i++) {
	argv[i] = (char *)g_argv[i].c_str();
      }
	      
      orb = CORBA::ORB_init(g_argc, argv);
    }

    ~NVUtilTests()
    {
    }

    virtual void setUp()
    {
    }
    
    virtual void tearDown()
    { 
    }

    /*!
     * @brief newNV(char*,Value)のテスト
     * 
     * - CORBA::Short型データのNameValueを正しく生成できるか？
     */
    void test_newNV_Short()
    {
      // CORBA::Short型のデータを持つNameValueを生成し、値を比較して正しく生成されていることを確認する
      CORBA::Short value = 1;
      string name = "short";
      SDOPackage::NameValue nv = newNV(name.c_str(), value);
      
      string nvName(nv.name);
      CPPUNIT_ASSERT_EQUAL(name, nvName);

      CORBA::Short nvValue;
      nv.value >>= nvValue;
      CPPUNIT_ASSERT_EQUAL(value, nvValue);
    }

    /*!
     * @brief newNV(char*,Value)のテスト
     * 
     * - CORBA::Long型データのNameValueを正しく生成できるか？
     */
    void test_newNV_Long()
    {
      // CORBA::Long型のデータを持つNameValueを生成し、値を比較して正しく生成されていることを確認する
      CORBA::Long value = 999999999;
      string name = "long";
      SDOPackage::NameValue nv = newNV(name.c_str(), value);
      
      string nvName(nv.name);
      CPPUNIT_ASSERT_EQUAL(name, nvName);

      CORBA::Long nvValue;
      nv.value >>= nvValue;
      CPPUNIT_ASSERT_EQUAL(value, nvValue);
    }

    /*!
     * @brief newNV(char*,Value)のテスト
     * 
     *  - CORBA::Float型データのNameValueを正しく生成できるか？
     */
    void test_newNV_Float()
    {
      // CORBA::Float型のデータを持つNameValueを生成し、値を比較して正しく生成されていることを確認する
      CORBA::Float value = 99999.9;
      string name = "float";
      SDOPackage::NameValue nv = newNV(name.c_str(), value);
      
      string nvName(nv.name);
      CPPUNIT_ASSERT_EQUAL(name, nvName);

      CORBA::Float nvValue;
      nv.value >>= nvValue;
      CPPUNIT_ASSERT_EQUAL(value, nvValue);
    }

    /*!
     * @brief newNV(char*,Value)のテスト
     * 
     * - CORBA::Double型データのNameValueを正しく生成できるか？
     */
    void test_newNV_Double()
    {
      // CORBA::Double型のデータを持つNameValueを生成し、値を比較して正しく生成されていることを確認する
      CORBA::Double value = 9999999.999;
      string name = "double";
      SDOPackage::NameValue nv = newNV(name.c_str(), value);
      
      string nvName(nv.name);
      CPPUNIT_ASSERT_EQUAL(name, nvName);

      CORBA::Double nvValue;
      nv.value >>= nvValue;
      CPPUNIT_ASSERT_EQUAL(value, nvValue);
    }
    
    /*!
     * @brief newNV(const char*, const char*)のテスト
     * 
     * - const char*型データのNameValueを正しく生成できるか？
     */
    void test_newNV_Str()
    {
      // (1) CORBA::String_var型のvalueとchar*型のnameをnewNV()に渡し、NameValueを取得する
      string name = "string";
      CORBA::String_var value = CORBA::string_dup("Hello, world!");
      SDOPackage::NameValue nv = newNV(name.c_str(), value);
    	
      // (2) newNV()にセットしたnameと、取得したNameValue.nameを比較する
      string nvName(nv.name);
      CPPUNIT_ASSERT_EQUAL(name, nvName);
    	
      // (3) newNV()にセットしたvalueと、取得したNameValue.valueを比較する
      //    	CORBA::String_var nvValue;
      const char* nvValue;
      nv.value >>= nvValue;
      CPPUNIT_ASSERT_EQUAL(string("Hello, world!"), string(nvValue));
    }
    
    /*!
     * @brief newNVChar()のテスト
     * 
     * - CORBA::Char型データのNameValueを正しく生成できるか？
     */
    void test_newNVChar()
    {
      // (1) CORBA::Char型のvalueとchar*型のnameをnewNVChar()に渡し、NameValueを取得する。
      string name = "char";
      CORBA::Char value = 'A';
      SDOPackage::NameValue nv = newNVChar(name.c_str(), value);
      
      // (2) newNVChar()にセットしたnameと取得したNameValue.nameの比較。
      string nvName(nv.name);
      CPPUNIT_ASSERT_EQUAL(name, nvName);
      
      // (3) newNVChar()にセットしたvalueと取得したNameValue.valueの比較。
      CORBA::Char nvValue;
      nv.value >>= CORBA::Any::to_char(nvValue);
      CPPUNIT_ASSERT_EQUAL(value, nvValue);
    }
    
    /*!
     * @brief newNVBool()のテスト
     * 
     * - CORBA::Boolean型データのNameValueを正しく生成できるか？
     */
    void test_newNVBool()
    {
      string name = "bool";
      CORBA::Boolean value = false;
      SDOPackage::NameValue nv = newNVBool(name.c_str(), value);
      
      string nvName(nv.name);
      CPPUNIT_ASSERT_EQUAL(name, nvName);
      
      CORBA::Boolean nvValue;
      nv.value >>= CORBA::Any::to_boolean(nvValue);
      CPPUNIT_ASSERT_EQUAL(value, nvValue);
    }

    /*!
     * @brief newNVOctet()のテスト
     * 
     * - CORBA::Octet型データのNameValueを正しく生成できるか？
     */
    void test_newNVOctet()
    {
      string name = "octet";
      CORBA::Octet value = 030;
      SDOPackage::NameValue nv = newNVOctet(name.c_str(), value);
      
      string nvName(nv.name);
      CPPUNIT_ASSERT_EQUAL(name, nvName);
      
      CORBA::Octet nvValue;
      nv.value >>= CORBA::Any::to_octet(nvValue);
      CPPUNIT_ASSERT_EQUAL(value, nvValue);
    }
    
    /*!
     * @brief copy()のテスト
     * 
     * - RTC::Propertiesの内容を正しくをNVListにコピーできるか？
     */
    void test_copy()
    {
      // (1) RTC::Propertiesオブジェクトの生成
      // ※ Propertiesのコンストラクタでは引数で与えられたmapのキー値だけが保存される。
      string name = "port-type";
      string value = "port-type-value";
      map<string, string> mProp;
      mProp[name] = value;
      coil::Properties prop(mProp);
      
      // (2）copy()にてPropertiesオブジェクトをnvlistにコピー
      SDOPackage::NVList nvlist;
      copyFromProperties(nvlist, prop);
      
      // (3) copy()により引数で与えたnvlistが書き換えられているかを確認。
      string nvName(nvlist[0].name);
      CPPUNIT_ASSERT_EQUAL(name, nvName);
      
      const char* getval;
      nvlist[0].value >>= getval;
      string nvValue(getval);
      CPPUNIT_ASSERT_EQUAL(value, nvValue);
    }

    /*!
     * @brief toProperties()のテスト
     * 
     * - NVListの内容を正しくRTC::Propertiesに変換できるか？
     */
    void test_toProperties()
    {
      // (1) 変換元となるNVListオブジェクトを生成する
      SDOPackage::NVList nvlist;		    	
      nvlist.length(2);
    	
      string name1 = "testname.test1";
      string value1 = "testval1";
      nvlist[0].name = name1.c_str();
      nvlist[0].value <<= value1.c_str();
			
      string name2 = "testname.test2";
      string value2 = "testval2";
      nvlist[1].name = name2.c_str();
      nvlist[1].value <<= value2.c_str();
			
      // (2) RTC::Propertiesへ変換する
      coil::Properties prop = toProperties(nvlist);
			
      // (3) 正しく変換されていることを確認する
      string propValue1 = prop.getProperty(name1);
      CPPUNIT_ASSERT_EQUAL(value1, propValue1);
			
      string propValue2 = prop.getProperty(name2);
      CPPUNIT_ASSERT_EQUAL(value2, propValue2);
    }

    /*!
     * @brief copyToProperties()のテスト
     * 
     * - NVListの内容を正しくRTC::Propertiesにコピーできるか？
     */
    void test_copyToProperties()
    {
      // (1) コピー元となるNVListオブジェクトを生成する
      SDOPackage::NVList nvlist;
      nvlist.length(2);

      string name1 = "testname.test1";
      string value1 = "testval1";
      nvlist[0].name = name1.c_str();
      nvlist[0].value <<= value1.c_str();
			
      string name2 = "testname.test2";
      string value2 = "testval2";
      nvlist[1].name = name2.c_str();
      nvlist[1].value <<= value2.c_str();

      // (2) RTC::Propertiesへコピーする
      coil::Properties prop;
      copyToProperties(prop, nvlist);
    	
      // (3) 値を比較して、正しくコピーされていることを確認する
      string propValue1 = prop.getProperty(name1);
      CPPUNIT_ASSERT_EQUAL(value1, propValue1);
      string propValue2 = prop.getProperty(name2);
      CPPUNIT_ASSERT_EQUAL(value2, propValue2);
    }
    
    /*!
     * @brief find()のテスト
     * 
     * - 指定した名称でNVList内の値を正しく検索できるか？
     */
    void test_find()
    {
      SDOPackage::NVList nvlist;
      nvlist.length(2);
      
      // (1) NVList要素のnameに"short",valueにshort型のデータをセット。
      string name1 = "short";
      CORBA::Short value1 = 1;
      nvlist[0].name = name1.c_str();
      nvlist[0].value <<= value1;
      
      // (2) NVList要素のnameに"long",valueにlong型のデータをセット。
      string name2 = "long";
      CORBA::Long value2 = 111;
      nvlist[1].name = name2.c_str();
      nvlist[1].value <<= value2;

      // (3) nvlistの中からNameValue.nameが"long"のNameValue.valueを検索して、正しい値を取得できることを確認する。
      CORBA::Short foundValue1;
      (find(nvlist, name1.c_str())) >>= foundValue1;
      CPPUNIT_ASSERT_EQUAL(value1, foundValue1);
      
      // (4) nvlistの中からNameValue.nameが"short"のNameValue.valueを検索して、正しい値を取得できることを確認する。
      CORBA::Long foundValue2;
      (find(nvlist, name2.c_str())) >>= foundValue2;
      CPPUNIT_ASSERT_EQUAL(value2, foundValue2);
    }
    
    /*!
     * @brief find_index()のテスト
     * 
     * - 指定した名称でNVList内の要素番号を正しく検索できるか？
     */
    void test_find_index()
    {
      SDOPackage::NVList nvlist;
      nvlist.length(2);
      
      // (1) NVList要素のnameに"short",valueにshort型のデータをセット。
      string name1 = "short";
      CORBA::Short value1 = 1;
      nvlist[0].name = name1.c_str();
      nvlist[0].value <<= value1;
      
      // (2) NVList要素のnameに"long",valueにlong型のデータをセット。
      string name2 = "long";
      CORBA::Long value2 = 111;
      nvlist[1].name = name2.c_str();
      nvlist[1].value <<= value2;

      // (3) nvlistの中からNameValue.nameが"long"のNameValue.valueを検索して、要素番号を取得できることを確認する。
      CORBA::Long ret;
      ret = find_index(nvlist, name1.c_str());
      CPPUNIT_ASSERT_EQUAL((CORBA::Long)0, ret);
      
      // (4) nvlistの中からNameValue.nameが"short"のNameValue.valueを検索して、要素番号を取得できることを確認する。
      ret = find_index(nvlist, name2.c_str());
      CPPUNIT_ASSERT_EQUAL((CORBA::Long)1, ret);
    }
    
    /*!
     * @brief isString()のテスト
     * 
     *  - NVList内の指定した名称を持つ値がstring型かどうかを正しく判定できるか？
     */
    void test_isString()
    {
      SDOPackage::NVList nvlist;
      nvlist.length(2);
      
      // (1) NVList要素のnameに"short",valueにshort型のデータをセット。
      string name1 = "short";
      CORBA::Short value1 = 1;
      nvlist[0].name = name1.c_str();
      nvlist[0].value <<= value1;
      
      // (2) NVList要素のnameに"string",valueにstring型のデータをセット。
      string name2 = "string";
      string value2 = "test";
      nvlist[1].name = name2.c_str();
      nvlist[1].value <<= value2.c_str();
      
      // (3) isString(nvlist,name)にて,指定されたnameのvalueの型がstringかどうかを判定。
      CPPUNIT_ASSERT(!isString(nvlist, name1.c_str()));
      CPPUNIT_ASSERT(isString(nvlist, name2.c_str()));
    }

    /*!
     * @brief isStringValue()のテスト
     * 
     *  - NVList内の指定した名称を持つ値がstring型かどうかを正しく判定できるか？
     */
    void test_isStringValue()
    {
      SDOPackage::NVList nvlist;
      nvlist.length(2);
      
      // (1) NVList要素のnameに"short",valueにshort型のデータをセット。
      string name1 = "short";
      CORBA::Short value1 = 1;
      nvlist[0].name = name1.c_str();
      nvlist[0].value <<= value1;
      
      // (2) NVList要素のnameに"string",valueにstring型のデータをセット。
      string name2 = "string";
      string value2 = "test";
      nvlist[1].name = name2.c_str();
      nvlist[1].value <<= value2.c_str();
      
      // (3) isString(nvlist,name)にて,指定されたnameのvalueの型がstringかどうかを判定。
      CPPUNIT_ASSERT(!isStringValue(nvlist, name1.c_str(), "1"));
      CPPUNIT_ASSERT(isStringValue(nvlist, name2.c_str(), "test"));
    }

    /*!
     * @brief toString()のテスト
     * 
     * - NVList内の指定した名称を持つ値がstring型である場合に、その値を正しく取得できるか？
     */
    void test_toString() {

			
      SDOPackage::NVList nvlist;
      nvlist.length(2);
			
      // (1) NVList要素のnameに"short",valueにshort型のデータをセット。
      string name1 = "short";
      CORBA::Short value1 = 1;
      nvlist[0].name = name1.c_str();
      nvlist[0].value <<= value1;
			
      // (2) NVList要素のnameに"string",valueにstring型のデータをセット。
      string name2 = "string";
      string value2 = "test";
      nvlist[1].name = name2.c_str();
      nvlist[1].value <<= value2.c_str();
			
      // (3) toString(nvlist,name)にて,指定されたnameのvalueをstring型で意図とおりに取得できるか？
      string empty("");
      CPPUNIT_ASSERT_EQUAL(empty, toString(nvlist, name1.c_str()));
      CPPUNIT_ASSERT_EQUAL(value2, toString(nvlist, name2.c_str()));
    }

    /*!
     * @brief appendStringValue()のテスト
     * 
     * - 指定した名称と値を持つNameValueを、NVListに正しく追加できるか？
     */
    void test_appendStringValue()
    {

      SDOPackage::NVList nvlist;
      nvlist.length(3);
			
      // (1) 追加対象となるNVListを作成する
      string name1 = "language";
      string value1 = "japanese";
      nvlist[0].name = name1.c_str();
      nvlist[0].value <<= value1.c_str();
			
      string name2 = "fruit";
      string value2 = "apple";
      nvlist[1].name = name2.c_str();
      nvlist[1].value <<= value2.c_str();
			
      string name3 = "drink";
      string value3 = "coffee, coke";
      nvlist[2].name = name3.c_str();
      nvlist[2].value <<= value3.c_str();
			
      // (2) 作成したNVListに、名称"os", 値"unix"のNameValueを正しく追加できるか？
      string name4 = "os";
      string value4 = "unix";
      CPPUNIT_ASSERT(appendStringValue(nvlist, name4.c_str(), value4.c_str()));
      CPPUNIT_ASSERT_EQUAL(value4, toString(nvlist, name4.c_str()));
			
      // (3) 既存の名称である"language"に、値"english"を正しく追加できるか？
      string name5 = name1;
      string value5 = "english";
      CPPUNIT_ASSERT(appendStringValue(nvlist, name5.c_str(), value5.c_str()));
      string expectedValueLanguage = "japanese,english";
      CPPUNIT_ASSERT_EQUAL(expectedValueLanguage, toString(nvlist, name5.c_str()));
			
      // (4) 既存の名称・値と全く同一のNameValueを追加しようとしたときに、意図どおり何も追加せずに終了するか？
      string name6 = name2;
      string value6 = value2;
      CPPUNIT_ASSERT(appendStringValue(nvlist, name6.c_str(), value6.c_str()));
      CPPUNIT_ASSERT_EQUAL(value2, toString(nvlist, name6.c_str()));
			
      // (5) 一部の値が、既存の値と重複するNameValueを追加しようとした時に、意図どおりにマージされるか？
      string name7 = name3;
      string value7 = "coke, beer";
      CPPUNIT_ASSERT(appendStringValue(nvlist, name7.c_str(), value7.c_str()));
      string expectedValueDrink = "coffee, coke,coke, beer";
      CPPUNIT_ASSERT_EQUAL(expectedValueDrink, toString(nvlist, name7.c_str()));
    }

    /*!
     * @brief append()のテスト
     * 
     * - NVListの内容を、他のNVListに正しく追加できるか？
     */
    void test_append()
    {
      // (1) 1つ目のNVListを作成する
      SDOPackage::NVList nvlistA;
      nvlistA.length(2);
			
      string nameA1 = "nameA1";
      string valueA1 = "valueA1";
      nvlistA[0].name = nameA1.c_str();
      nvlistA[0].value <<= valueA1.c_str();
			
      string nameA2 = "nameA2";
      string valueA2 = "valueA2";
      nvlistA[1].name = nameA2.c_str();
      nvlistA[1].value <<= valueA2.c_str();
			
      // (2) 2つ目のNVListを作成する
      SDOPackage::NVList nvlistB;
      nvlistB.length(2);
			
      string nameB1 = "nameB1";
      string valueB1 = "valueB1";
      nvlistB[0].name = nameB1.c_str();
      nvlistB[0].value <<= valueB1.c_str();
			
      string nameB2 = "nameB2";
      string valueB2 = "valueB2";
      nvlistB[1].name = nameB2.c_str();
      nvlistB[1].value <<= valueB2.c_str();
			
      // (3) 1つ目のNVListに、2つ目のNVListを追加する
      append(nvlistA, nvlistB);
			
      // (4) 正しく追加されたことを確認する
      CPPUNIT_ASSERT_EQUAL(valueA1, toString(nvlistA, nameA1.c_str()));
      CPPUNIT_ASSERT_EQUAL(valueA2, toString(nvlistA, nameA2.c_str()));
      CPPUNIT_ASSERT_EQUAL(valueB1, toString(nvlistA, nameB1.c_str()));
      CPPUNIT_ASSERT_EQUAL(valueB2, toString(nvlistA, nameB2.c_str()));
    }
    void test_dump()
    { 
      SDOPackage::NVList nvlistC;
      nvlistC.length(3);
			
      string nameC1 = "nameC1";
      string valueC1 = "valueC1";
      nvlistC[0].name = nameC1.c_str();
      nvlistC[0].value <<= valueC1.c_str();
			
      string nameC2 = "nameC2";
      string valueC2 = "valueC2";
      nvlistC[1].name = nameC2.c_str();
      nvlistC[1].value <<= valueC2.c_str();

      string nameC3 = "name_hoge";
      string valueC3 = "value_hoge";
      nvlistC[2].name = nameC3.c_str();
      nvlistC[2].value <<= valueC3.c_str();
     

      dump(nvlistC);
    }	

    // std::string toString(const SDOPackage::NVList& nv);のテストです。
    void test_toStringNV()
    {
      SDOPackage::NVList nvlistD;
      nvlistD.length(2);
			
      string nameD1 = "nameD1";
      string valueD1 = "valueD1";
      nvlistD[0].name = nameD1.c_str();
      nvlistD[0].value <<= valueD1.c_str();
			
      string nameD2 = "nameD2";
      string valueD2 = "valueD2";
      nvlistD[1].name = nameD2.c_str();
      nvlistD[1].value <<= valueD2.c_str();
 
      string strD ="\n\nnameD1: valueD1\nnameD2: valueD2";
//      std::cout << strD << std::endl;

      SDOPackage::NVList nvlistE;
      nvlistE.length(2);
			
      // (1) NVList要素のnameに"short",valueにshort型のデータをセット。
      string nameE1 = "short";
      CORBA::Short valueE1 = 1;
      nvlistE[0].name = nameE1.c_str();
      nvlistE[0].value <<= valueE1;
			
      // (2) NVList要素のnameに"string",valueにstring型のデータをセット。
      string nameE2 = "string";
      string valueE2 = "test";
      nvlistE[1].name = nameE2.c_str();
      nvlistE[1].value <<= valueE2.c_str();
      			
      // (3) ストリング以外のデータがセットされたときのメッセージを確認。

      std::string empty = "short: not a string value\nstring: test\n";
      std::string str_nvlistE = toString(nvlistE);
      /*
      if (empty == str_nvlistE)
	{
	  std::cout << "OKK";
	}
      else 
	{
	  std::cout << "not match !!" << std::endl;
	}
      */  
      CPPUNIT_ASSERT(toString(nvlistE) == empty);


    }	
  };
}; // namespace NVUtil

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(NVUtil::NVUtilTests);

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
#endif // NVUtil_cpp
