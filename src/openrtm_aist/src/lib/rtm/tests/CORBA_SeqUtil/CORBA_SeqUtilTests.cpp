// -*- C++ -*-
/*!
 * @file   CORBA_SeqUtilTests.cpp
 * @brief  CORBA_SeqUtil test class
 * @date   $Date: 2008/01/10 11:02:06 $
 * @author Shinji Kurihara
 *         Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006
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
 * $Log: CORBA_SeqUtilTests.cpp,v $
 * Revision 1.2  2008/01/10 11:02:06  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2007/12/20 07:50:18  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2006/11/27 08:26:10  n-ando
 * TestSuites are devided into each directory.
 *
 * Revision 1.1  2006/10/31 13:10:51  kurihara
 *
 * test program for CORBA_SeqUtil.h
 *
 */

#ifndef CORBA_SeqUtil_cpp
#define CORBA_SeqUtil_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/CORBA_SeqUtil.h>
#include <rtm/NVUtil.h>
#include <rtm/PortAdmin.h>
#include <rtm/ConnectorBase.h>
#include "SeqUtilTestsSkel.h"

/*!
 * @class CORBA_SeqUtilTests class
 * @brief CORBA_SeqUtil test
 */
namespace CORBA_SeqUtil
{

  class Logger
  {
  public:
    void log(const std::string& msg)
    {
      m_log.push_back(msg);
    }

    int countLog(const std::string& msg)
    {
      int count = 0;
      for (int i = 0; i < (int) m_log.size(); ++i)
        {
          if (m_log[i] == msg) ++count;
        }
      return count;
    }
		
    void clearLog(void)
    {
        m_log.clear();
    }
  private:
    std::vector<std::string> m_log;
  };

  class PortMock
    : public RTC::PortBase
  {
  protected:
    virtual RTC::ReturnCode_t publishInterfaces(RTC::ConnectorProfile&)
    {
      return RTC::RTC_OK;
    }
    virtual RTC::ReturnCode_t subscribeInterfaces(const RTC::ConnectorProfile&)
    {
      return RTC::RTC_OK;
    }
    virtual void unsubscribeInterfaces(const RTC::ConnectorProfile&)
    {
    }
    virtual void activateInterfaces()
    {
    }
    virtual void deactivateInterfaces()
    {
    }
  };


  using namespace std;
  
  class CORBA_SeqUtilTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(CORBA_SeqUtilTests);
		
    CPPUNIT_TEST(test_for_each);
    CPPUNIT_TEST(test_find);
    CPPUNIT_TEST(test_push_back);
    CPPUNIT_TEST(test_push_back_list);
    CPPUNIT_TEST(test_insert);
    CPPUNIT_TEST(test_front);
    CPPUNIT_TEST(test_back);
    CPPUNIT_TEST(test_erase);
    CPPUNIT_TEST(test_erase_if);
    CPPUNIT_TEST(test_clear);
    CPPUNIT_TEST(test_refToVstring);

    CPPUNIT_TEST_SUITE_END();
		
  private:
    NameValue nv;
    NVList nvlist;
    CORBA::Short  st, rst;
    CORBA::Long   lg, rlg;
    CORBA::Float  ft, rft;
    CORBA::Double dl, rdl;
		
  public:
    /*!
     * @brief Constructor
     */
    CORBA_SeqUtilTests()
    {
    }
		    
    /*!
     * @brief Destructor
     */
    ~CORBA_SeqUtilTests()
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
     * @brief test_for_each()で使用するファンクタ
     */
    struct functor_for_each
    {
      functor_for_each(vector<NameValue>& receivedNameValues)
	: _receivedNameValues(receivedNameValues) {}
			
      bool operator()(const NameValue& nv) {
	_receivedNameValues.push_back(nv);
	return true;
      }
			
      vector<NameValue>& _receivedNameValues;
    };
		
    /*!
     * @brief for_each()メソッドのテスト
     * 
     * <ul>
     * <li>引数で指定されたNVList内のすべての要素について、正しい順序でファンクタが呼び出されるか？</li>
     * <li>ファンクタ呼出時に引数で渡されるNameValueは正しいものか？</li>
     * </ul>
     */
    void test_for_each()
    {
      // テスト用のNVListを作成する
      NVList nvlist;
      nvlist.length(4);
			
      NameValue nvShort;
      nvShort.name = "short";
      nvShort.value <<= (CORBA::Short) 123;
      nvlist[0] = nvShort;
			
      NameValue nvLong;
      nvLong.name = "long";
      nvLong.value <<= (CORBA::Long) 123456;
      nvlist[1] = nvLong;
			
      NameValue nvFloat;
      nvFloat.name = "float";
      nvFloat.value <<= (CORBA::Float) 987.654;
      nvlist[2] = nvFloat;
			
      NameValue nvDouble;
      nvDouble.name = "double";
      nvDouble.value <<= (CORBA::Double) 987654.321987;
      nvlist[3] = nvDouble;
			
      // for_each()を呼び出す
      vector<NameValue> receivedNameValues;
      CORBA_SeqUtil::for_each(nvlist, functor_for_each(receivedNameValues));
			
      // ファンクタが呼び出された時に内部に記録したNameValueのベクタを取得し、期待値と比較する
      CPPUNIT_ASSERT_EQUAL(4, (int) receivedNameValues.size());
      CPPUNIT_ASSERT_EQUAL((string) nvlist[0].name, (string) receivedNameValues[0].name);
      CPPUNIT_ASSERT_EQUAL((string) nvlist[1].name, (string) receivedNameValues[1].name);
      CPPUNIT_ASSERT_EQUAL((string) nvlist[2].name, (string) receivedNameValues[2].name);
      CPPUNIT_ASSERT_EQUAL((string) nvlist[3].name, (string) receivedNameValues[3].name);
			
      CORBA::Short expectedShort, actualShort;
      nvlist[0].value >>= expectedShort;
      receivedNameValues[0].value >>= actualShort;
      CPPUNIT_ASSERT_EQUAL(expectedShort, actualShort);
			
      CORBA::Long expectedLong, actualLong;
      nvlist[1].value >>= expectedLong;
      receivedNameValues[1].value >>= actualLong;
      CPPUNIT_ASSERT_EQUAL(expectedLong, actualLong);

      CORBA::Float expectedFloat, actualFloat;
      nvlist[2].value >>= expectedFloat;
      receivedNameValues[2].value >>= actualFloat;
      CPPUNIT_ASSERT_EQUAL(expectedFloat, actualFloat);

      CORBA::Double expectedDouble, actualDouble;
      nvlist[3].value >>= expectedDouble;
      receivedNameValues[3].value >>= actualDouble;
      CPPUNIT_ASSERT_EQUAL(expectedDouble, actualDouble);
    }
		
    /*!
     * @brief test_find()で使用するファンクタ
     */
    struct functor_find
    {
      functor_find(const string& name)
	: _name(name) {}
			
      bool operator()(const NameValue& nv) {
	return _name == string(nv.name);
      }
			
      const string _name;
    };
		
    /*!
     * @brief find()メソッドのテスト
     * 
     * <ul>
     * <li>ファンクタを用いたNVList内の要素を正しく検索できるか？</li>
     * </ul>
     */
    void test_find()
    {
      // テスト用のNVListを作成する
      NVList nvlist;
      nvlist.length(4);
			
      NameValue nvShort;
      nvShort.name = "short";
      nvShort.value <<= (CORBA::Short) 123;
      nvlist[0] = nvShort;
			
      NameValue nvLong;
      nvLong.name = "long";
      nvLong.value <<= (CORBA::Long) 123456;
      nvlist[1] = nvLong;
			
      NameValue nvFloat;
      nvFloat.name = "float";
      nvFloat.value <<= (CORBA::Float) 987.654;
      nvlist[2] = nvFloat;
			
      NameValue nvDouble;
      nvDouble.name = "double";
      nvDouble.value <<= (CORBA::Double) 987654.321987;
      nvlist[3] = nvDouble;

      // 名称"short"を持つNameValueのインデクスを正しく検索できるか？
      CPPUNIT_ASSERT_EQUAL((CORBA::Long) 0, CORBA_SeqUtil::find(nvlist, functor_find("short")));
			
      // 名称"long"を持つNameValueのインデクスを正しく検索できるか？
      CPPUNIT_ASSERT_EQUAL((CORBA::Long) 1, CORBA_SeqUtil::find(nvlist, functor_find("long")));

      // 名称"float"を持つNameValueのインデクスを正しく検索できるか？
      CPPUNIT_ASSERT_EQUAL((CORBA::Long) 2, CORBA_SeqUtil::find(nvlist, functor_find("float")));
			
      // 名称"double"を持つNameValueのインデクスを正しく検索できるか？
      CPPUNIT_ASSERT_EQUAL((CORBA::Long) 3, CORBA_SeqUtil::find(nvlist, functor_find("double")));
    }

    /*!
     * @brief push_back()メソッドのテスト
     * 
     * <ul>
     * <li>push_backにより追加した各要素の内容が、それぞれ正しく格納されるか？</li>
     * </ul>
     */
    void test_push_back()
    {
      NVList nvlist;

      // １つめの要素をpush_backして、要素数が正しいかチェックする
      NameValue nvShort;
      nvShort.name = "short";
      nvShort.value <<= (CORBA::Short) 123;
      CORBA_SeqUtil::push_back(nvlist, nvShort);
      CPPUNIT_ASSERT_EQUAL((CORBA::ULong) 1, nvlist.length());
			
      // ２つめの要素をpush_backして、要素数が正しいかチェックする
      NameValue nvLong;
      nvLong.name = "long";
      nvLong.value <<= (CORBA::Long) 123456;
      CORBA_SeqUtil::push_back(nvlist, nvLong);
      CPPUNIT_ASSERT_EQUAL((CORBA::ULong) 2, nvlist.length());
			
      // ３つめの要素をpush_backして、要素数が正しいかチェックする
      NameValue nvFloat;
      nvFloat.name = "float";
      nvFloat.value <<= (CORBA::Float) 987.654;
      CORBA_SeqUtil::push_back(nvlist, nvFloat);
      CPPUNIT_ASSERT_EQUAL((CORBA::ULong) 3, nvlist.length());
			
      // ４つめの要素をpush_backして、要素数が正しいかチェックする
      NameValue nvDouble;
      nvDouble.name = "double";
      nvDouble.value <<= (CORBA::Double) 987654.321987;
      CORBA_SeqUtil::push_back(nvlist, nvDouble);
      CPPUNIT_ASSERT_EQUAL((CORBA::ULong) 4, nvlist.length());
			
      // push_backした各要素の内容が、NVList内に正しく格納されていることを確認する
      CPPUNIT_ASSERT_EQUAL((string) "short", (string) nvlist[0].name);
      CPPUNIT_ASSERT_EQUAL((string) "long", (string) nvlist[1].name);
      CPPUNIT_ASSERT_EQUAL((string) "float", (string) nvlist[2].name);
      CPPUNIT_ASSERT_EQUAL((string) "double", (string) nvlist[3].name);
			
      CORBA::Short actualShort;
      nvlist[0].value >>= actualShort;
      CPPUNIT_ASSERT_EQUAL((CORBA::Short) 123, actualShort);
			
      CORBA::Long actualLong;
      nvlist[1].value >>= actualLong;
      CPPUNIT_ASSERT_EQUAL((CORBA::Long) 123456, actualLong);

      CORBA::Float actualFloat;
      nvlist[2].value >>= actualFloat;
      CPPUNIT_ASSERT_EQUAL((CORBA::Float) 987.654, actualFloat);

      CORBA::Double actualDouble;
      nvlist[3].value >>= actualDouble;
      CPPUNIT_ASSERT_EQUAL((CORBA::Double) 987654.321987, actualDouble);
    }
		
    /*!
     * @brief push_back_list()メソッドのテスト
     * 
     * <ul>
     * <li>一方のNVListの内容を、他方のNVListの後ろに正しくアペンドできるか？</li>
     * </ul>
     */
    void test_push_back_list()
    {
      // １つめのNVListを作成する
      NVList nvlist1;
      nvlist1.length(2);
			
      NameValue nvShort;
      nvShort.name = "short";
      nvShort.value <<= (CORBA::Short) 123;
      nvlist1[0] = nvShort;
			
      NameValue nvLong;
      nvLong.name = "long";
      nvLong.value <<= (CORBA::Long) 123456;
      nvlist1[1] = nvLong;
			
      // ２つめのNVListを作成する
      NVList nvlist2;
      nvlist2.length(2);
			
      NameValue nvFloat;
      nvFloat.name = "float";
      nvFloat.value <<= (CORBA::Float) 987.654;
      nvlist2[0] = nvFloat;
			
      NameValue nvDouble;
      nvDouble.name = "double";
      nvDouble.value <<= (CORBA::Double) 987654.321987;
      nvlist2[1] = nvDouble;
			
      // push_back_listして、１つめのNVListに２つめのNVListをアペンドする
      CORBA_SeqUtil::push_back_list(nvlist1, nvlist2);
			
      // 正しくアペンドされたことを確認する
      CPPUNIT_ASSERT_EQUAL((string) "short", (string) nvlist1[0].name);
      CPPUNIT_ASSERT_EQUAL((string) "long", (string) nvlist1[1].name);
      CPPUNIT_ASSERT_EQUAL((string) "float", (string) nvlist1[2].name);
      CPPUNIT_ASSERT_EQUAL((string) "double", (string) nvlist1[3].name);
			
      CORBA::Short actualShort;
      nvlist1[0].value >>= actualShort;
      CPPUNIT_ASSERT_EQUAL((CORBA::Short) 123, actualShort);
			
      CORBA::Long actualLong;
      nvlist1[1].value >>= actualLong;
      CPPUNIT_ASSERT_EQUAL((CORBA::Long) 123456, actualLong);

      CORBA::Float actualFloat;
      nvlist1[2].value >>= actualFloat;
      CPPUNIT_ASSERT_EQUAL((CORBA::Float) 987.654, actualFloat);

      CORBA::Double actualDouble;
      nvlist1[3].value >>= actualDouble;
      CPPUNIT_ASSERT_EQUAL((CORBA::Double) 987654.321987, actualDouble);
    }
		
    /*!
     * @brief insert()メソッドのテスト
     * 
     * <ul>
     * <li>先頭位置への挿入を正しく行えるか？</li>
     * <li>最後尾位置への挿入を正しく行えるか？</li>
     * <li>中間位置への挿入を正しく行えるか？</li>
     * </ul>
     */
    void test_insert()
    {
      // 挿入対象となるNVListを作成する
      NVList nvlist;
      nvlist.length(1);

      NameValue nvLong;
      nvLong.name = "long";
      nvLong.value <<= (CORBA::Long) 123456;
      nvlist[0] = nvLong;
			
      // (1) 先頭への挿入を行う
      NameValue nvShort;
      nvShort.name = "short";
      nvShort.value <<= (CORBA::Short) 123;
      CORBA_SeqUtil::insert(nvlist, nvShort, 0);
			
      // (2) 最後尾への挿入を行う
      NameValue nvDouble;
      nvDouble.name = "double";
      nvDouble.value <<= (CORBA::Double) 987654.321987;
      CORBA_SeqUtil::insert(nvlist, nvDouble, 2);
			
      // (3) 中間への挿入を行う
      NameValue nvFloat;
      nvFloat.name = "float";
      nvFloat.value <<= (CORBA::Float) 987.654;
      CORBA_SeqUtil::insert(nvlist, nvFloat, 2);
			
      // 挿入結果が正しいことを確認する
      CPPUNIT_ASSERT_EQUAL((string) "short", (string) nvlist[0].name);
      CPPUNIT_ASSERT_EQUAL((string) "long", (string) nvlist[1].name);
      CPPUNIT_ASSERT_EQUAL((string) "float", (string) nvlist[2].name);
      CPPUNIT_ASSERT_EQUAL((string) "double", (string) nvlist[3].name);
			
      CORBA::Short actualShort;
      nvlist[0].value >>= actualShort;
      CPPUNIT_ASSERT_EQUAL((CORBA::Short) 123, actualShort);
			
      CORBA::Long actualLong;
      nvlist[1].value >>= actualLong;
      CPPUNIT_ASSERT_EQUAL((CORBA::Long) 123456, actualLong);

      CORBA::Float actualFloat;
      nvlist[2].value >>= actualFloat;
      CPPUNIT_ASSERT_EQUAL((CORBA::Float) 987.654, actualFloat);

      CORBA::Double actualDouble;
      nvlist[3].value >>= actualDouble;
      CPPUNIT_ASSERT_EQUAL((CORBA::Double) 987654.321987, actualDouble);
    }
		
    /*!
     * @brief front()メソッドのテスト
     * 
     * <ul>
     * <li>取得対象のNVListの要素数が０の場合、例外がスローされるか？</li>
     * <li>取得対象のNVListの要素数が１以上の場合、先頭の要素を取得できるか？</li>
     * </ul>
     */
    void test_front()
    {
      // (1) 取得対象のNVListの要素数が０の場合、例外がスローされるか
      NVList nvlistEmpty;
      try {
	CORBA_SeqUtil::front<NVList, NameValue>(nvlistEmpty);
	CPPUNIT_FAIL("Exception must be thrown.");
      } catch (...) {
	// expected
      }
			
      // (2) 取得対象のNVListの要素数が１以上の場合、先頭の要素を取得できるか？
      NVList nvlist;
      nvlist.length(4);
			
      NameValue nvShort;
      nvShort.name = "short";
      nvShort.value <<= (CORBA::Short) 123;
      nvlist[0] = nvShort;
			
      NameValue nvLong;
      nvLong.name = "long";
      nvLong.value <<= (CORBA::Long) 123456;
      nvlist[1] = nvLong;
			
      NameValue nvFloat;
      nvFloat.name = "float";
      nvFloat.value <<= (CORBA::Float) 987.654;
      nvlist[2] = nvFloat;
			
      NameValue nvDouble;
      nvDouble.name = "double";
      nvDouble.value <<= (CORBA::Double) 987654.321987;
      nvlist[3] = nvDouble;
			
      // 先頭の要素を取得して、期待値と比較してチェックする
      NameValue nvFront = CORBA_SeqUtil::front<NVList, NameValue>(nvlist);
			
      CPPUNIT_ASSERT_EQUAL((string) "short", (string) nvFront.name);
			
      CORBA::Short actualValue;
      nvFront.value >>= actualValue;
      CPPUNIT_ASSERT_EQUAL((CORBA::Short) 123, actualValue);
    }
		
    /*!
     * @brief back()メソッドのテスト
     * 
     * <ul>
     * <li>取得対象のNVListの要素数が０の場合、例外がスローされるか？</li>
     * <li>取得対象のNVListの要素数が１以上の場合、最後尾の要素を取得できるか？</li>
     * </ul>
     */
    void test_back()
    {
      // (1) 取得対象のNVListの要素数が０の場合、例外がスローされるか？
      NVList nvlistEmpty;
      try {
	CORBA_SeqUtil::back<NVList, NameValue>(nvlistEmpty);
	CPPUNIT_FAIL("Exception must be thrown.");
      } catch (...) {
	// expected
      }
			
      // (2) 取得対象のNVListの要素数が１以上の場合、最後尾の要素を取得できるか？
      NVList nvlist;
      nvlist.length(4);
			
      NameValue nvShort;
      nvShort.name = "short";
      nvShort.value <<= (CORBA::Short) 123;
      nvlist[0] = nvShort;
			
      NameValue nvLong;
      nvLong.name = "long";
      nvLong.value <<= (CORBA::Long) 123456;
      nvlist[1] = nvLong;
			
      NameValue nvFloat;
      nvFloat.name = "float";
      nvFloat.value <<= (CORBA::Float) 987.654;
      nvlist[2] = nvFloat;
			
      NameValue nvDouble;
      nvDouble.name = "double";
      nvDouble.value <<= (CORBA::Double) 987654.321987;
      nvlist[3] = nvDouble;

      // 最後の要素を取得して、期待値と比較してチェックする
      NameValue nvBack = CORBA_SeqUtil::back<NVList, NameValue>(nvlist);
			
      CPPUNIT_ASSERT_EQUAL((string) "double", (string) nvBack.name);
			
      CORBA::Double actualValue;
      nvBack.value >>= actualValue;
      CPPUNIT_ASSERT_EQUAL((CORBA::Double) 987654.321987, actualValue);
    }
		
    /*!
     * @brief erase()メソッドのテスト
     * 
     * <ul>
     * <li>先頭の要素のみを削除した場合、他要素はそのまま保たれるか？</li>
     * <li>中間の要素のみを削除した場合、他要素はそのまま保たれるか？</li>
     * <li>最後尾の要素のみを削除した場合、他要素はそのまま保たれるか？</li>
     * </ul>
     */
    void test_erase()
    {
      // テスト用のNVListを作成する
      NVList nvlist1, nvlist2, nvlist3;
      nvlist1.length(4);
      nvlist2.length(4);
      nvlist3.length(4);
			
      NameValue nvShort;
      nvShort.name = "short";
      nvShort.value <<= (CORBA::Short) 123;
      nvlist1[0] = nvShort;
      nvlist2[0] = nvShort;
      nvlist3[0] = nvShort;
			
      NameValue nvLong;
      nvLong.name = "long";
      nvLong.value <<= (CORBA::Long) 123456;
      nvlist1[1] = nvLong;
      nvlist2[1] = nvLong;
      nvlist3[1] = nvLong;
			
      NameValue nvFloat;
      nvFloat.name = "float";
      nvFloat.value <<= (CORBA::Float) 987.654;
      nvlist1[2] = nvFloat;
      nvlist2[2] = nvFloat;
      nvlist3[2] = nvFloat;
			
      NameValue nvDouble;
      nvDouble.name = "double";
      nvDouble.value <<= (CORBA::Double) 987654.321987;
      nvlist1[3] = nvDouble;
      nvlist2[3] = nvDouble;
      nvlist3[3] = nvDouble;
			
      // (1) 先頭の要素のみを削除した場合、他要素はそのまま保たれるか？
      CORBA_SeqUtil::erase(nvlist1, 0);
      CPPUNIT_ASSERT_EQUAL((CORBA::ULong) 3, nvlist1.length());
      CPPUNIT_ASSERT_EQUAL((string) "long", (string) nvlist1[0].name);
      CPPUNIT_ASSERT_EQUAL((string) "float", (string) nvlist1[1].name);
      CPPUNIT_ASSERT_EQUAL((string) "double", (string) nvlist1[2].name);
			
      CORBA::Long actualLong1;
      nvlist1[0].value >>= actualLong1;
      CPPUNIT_ASSERT_EQUAL((CORBA::Long) 123456, actualLong1);

      CORBA::Float actualFloat1;
      nvlist1[1].value >>= actualFloat1;
      CPPUNIT_ASSERT_EQUAL((CORBA::Float) 987.654, actualFloat1);

      CORBA::Double actualDouble1;
      nvlist1[2].value >>= actualDouble1;
      CPPUNIT_ASSERT_EQUAL((CORBA::Double) 987654.321987, actualDouble1);
			
      // (2) 中間の要素のみを削除した場合、他要素はそのまま保たれるか？
      CORBA_SeqUtil::erase(nvlist2, 1);
      CPPUNIT_ASSERT_EQUAL((CORBA::ULong) 3, nvlist2.length());
      CPPUNIT_ASSERT_EQUAL((string) "short", (string) nvlist2[0].name);
      CPPUNIT_ASSERT_EQUAL((string) "float", (string) nvlist2[1].name);
      CPPUNIT_ASSERT_EQUAL((string) "double", (string) nvlist2[2].name);
			
      CORBA::Short actualShort2;
      nvlist2[0].value >>= actualShort2;
      CPPUNIT_ASSERT_EQUAL((CORBA::Short) 123, actualShort2);

      CORBA::Float actualFloat2;
      nvlist2[1].value >>= actualFloat2;
      CPPUNIT_ASSERT_EQUAL((CORBA::Float) 987.654, actualFloat2);

      CORBA::Double actualDouble2;
      nvlist2[2].value >>= actualDouble2;
      CPPUNIT_ASSERT_EQUAL((CORBA::Double) 987654.321987, actualDouble2);

      // (3) 最後尾の要素のみを削除した場合、他要素はそのまま保たれるか？
      CORBA_SeqUtil::erase(nvlist3, 3);
      CPPUNIT_ASSERT_EQUAL((CORBA::ULong) 3, nvlist3.length());
      CPPUNIT_ASSERT_EQUAL((string) "short", (string) nvlist3[0].name);
      CPPUNIT_ASSERT_EQUAL((string) "long", (string) nvlist3[1].name);
      CPPUNIT_ASSERT_EQUAL((string) "float", (string) nvlist3[2].name);
			
      CORBA::Short actualShort3;
      nvlist3[0].value >>= actualShort3;
      CPPUNIT_ASSERT_EQUAL((CORBA::Short) 123, actualShort3);

      CORBA::Long actualLong3;
      nvlist3[1].value >>= actualLong3;
      CPPUNIT_ASSERT_EQUAL((CORBA::Long) 123456, actualLong3);

      CORBA::Float actualFloat3;
      nvlist3[2].value >>= actualFloat3;
      CPPUNIT_ASSERT_EQUAL((CORBA::Float) 987.654, actualFloat3);
    }

    /*!
     * @brief test_erase_if()で使用するファンクタ
     */
    struct functor_erase_if
    {
      functor_erase_if(const char* name)
	: _name(name) {}
			
      bool operator()(const NameValue& nv)
      {
	return _name == std::string(nv.name);
      }
			
      string _name;
    };
		
    /*!
     * @brief erase_if()メソッドのテスト
     * 
     * <ul>
     * <li>条件に合致する要素がない場合、何も削除されずに保たれるか？</li>
     * <li>条件に合致する要素がある場合、その要素が削除され、他要素は保たれるか？</li>
     * </ul>
     */
    void test_erase_if()
    {
      NVList nvlist1, nvlist2;
      nvlist1.length(4);
      nvlist2.length(4);
			
      NameValue nvShort;
      nvShort.name = "short";
      nvShort.value <<= (CORBA::Short) 123;
      nvlist1[0] = nvShort;
      nvlist2[0] = nvShort;
			
      NameValue nvLong;
      nvLong.name = "long";
      nvLong.value <<= (CORBA::Long) 123456;
      nvlist1[1] = nvLong;
      nvlist2[1] = nvLong;
			
      NameValue nvFloat;
      nvFloat.name = "float";
      nvFloat.value <<= (CORBA::Float) 987.654;
      nvlist1[2] = nvFloat;
      nvlist2[2] = nvFloat;
			
      NameValue nvDouble;
      nvDouble.name = "double";
      nvDouble.value <<= (CORBA::Double) 987654.321987;
      nvlist1[3] = nvDouble;
      nvlist2[3] = nvDouble;
			
      // (1) 条件に合致する要素がない場合、何も削除されずに保たれるか？
      CORBA_SeqUtil::erase_if(nvlist1, functor_erase_if("no-match-name"));
			
      CPPUNIT_ASSERT_EQUAL((string) "short", (string) nvlist1[0].name);
      CPPUNIT_ASSERT_EQUAL((string) "long", (string) nvlist1[1].name);
      CPPUNIT_ASSERT_EQUAL((string) "float", (string) nvlist1[2].name);
      CPPUNIT_ASSERT_EQUAL((string) "double", (string) nvlist1[3].name);
			
      CORBA::Short actualShort1;
      nvlist1[0].value >>= actualShort1;
      CPPUNIT_ASSERT_EQUAL((CORBA::Short) 123, actualShort1);
			
      CORBA::Long actualLong1;
      nvlist1[1].value >>= actualLong1;
      CPPUNIT_ASSERT_EQUAL((CORBA::Long) 123456, actualLong1);

      CORBA::Float actualFloat1;
      nvlist1[2].value >>= actualFloat1;
      CPPUNIT_ASSERT_EQUAL((CORBA::Float) 987.654, actualFloat1);

      CORBA::Double actualDouble1;
      nvlist1[3].value >>= actualDouble1;
      CPPUNIT_ASSERT_EQUAL((CORBA::Double) 987654.321987, actualDouble1);
			
      // (2) 条件に合致する要素がある場合、その要素が削除され、他要素は保たれるか？
      CORBA_SeqUtil::erase_if(nvlist2, functor_erase_if("float"));
			
      CPPUNIT_ASSERT_EQUAL((string) "short", (string) nvlist2[0].name);
      CPPUNIT_ASSERT_EQUAL((string) "long", (string) nvlist2[1].name);
      CPPUNIT_ASSERT_EQUAL((string) "double", (string) nvlist2[2].name);
			
      CORBA::Short actualShort2;
      nvlist2[0].value >>= actualShort2;
      CPPUNIT_ASSERT_EQUAL((CORBA::Short) 123, actualShort2);
			
      CORBA::Long actualLong2;
      nvlist2[1].value >>= actualLong2;
      CPPUNIT_ASSERT_EQUAL((CORBA::Long) 123456, actualLong2);

      CORBA::Double actualDouble2;
      nvlist2[2].value >>= actualDouble2;
      CPPUNIT_ASSERT_EQUAL((CORBA::Double) 987654.321987, actualDouble2);
    }
		
    /*!
     * @brief clear()メソッドのテスト
     * 
     * <ul>
     * <li>クリアにより要素数が０になるか？</li>
     * </ul>
     */
    void test_clear()
    {
      // テスト用のNVListを作成する
      NVList nvlist;
      nvlist.length(4);
			
      NameValue nvShort;
      nvShort.name = "short";
      nvShort.value <<= (CORBA::Short) 123;
      nvlist[0] = nvShort;
			
      NameValue nvLong;
      nvLong.name = "long";
      nvLong.value <<= (CORBA::Long) 123456;
      nvlist[1] = nvLong;
			
      NameValue nvFloat;
      nvFloat.name = "float";
      nvFloat.value <<= (CORBA::Float) 987.654;
      nvlist[2] = nvFloat;
			
      NameValue nvDouble;
      nvDouble.name = "double";
      nvDouble.value <<= (CORBA::Double) 987654.321987;
      nvlist[3] = nvDouble;
			
      // clear()を呼出し、クリアされたこと（要素数０になったこと）を確認する
      CORBA_SeqUtil::clear(nvlist);
      CPPUNIT_ASSERT_EQUAL((CORBA::ULong) 0, nvlist.length());
    }
    
    /*!
     * @brief refToVstring()メソッドのテスト
     * 
     * <ul>
     * <li>オブジェクトリファレンスの文字列が返されるか？</li>
     * </ul>
     */
    void test_refToVstring()
    {
      int argc(0);
      char** argv(NULL);
      CORBA::ORB_ptr m_pORB = CORBA::ORB_init(argc, argv);
      PortableServer::POA_ptr m_pPOA = 
          PortableServer::POA::_narrow(m_pORB->resolve_initial_references("RootPOA"));
      m_pPOA->the_POAManager()->activate();

      RTC::PortAdmin portAdmin(m_pORB, m_pPOA);
      PortMock* port0 = new PortMock();
      port0->setName("port0");
      CPPUNIT_ASSERT_EQUAL(true, portAdmin.addPort(*port0));

      RTC::PortService_var portRef0 = portAdmin.getPortRef("port0");
      RTC::ConnectorProfile cprof;
      cprof.connector_id = "id";
      cprof.name = CORBA::string_dup("Test");

      CORBA_SeqUtil::push_back(cprof.properties,
                               NVUtil::newNV("dataport.interface_type",
					     "corba_cdr"));
      CORBA_SeqUtil::push_back(cprof.properties,
                               NVUtil::newNV("dataport.dataflow_type",
					     "push"));
      CORBA_SeqUtil::push_back(cprof.properties,
                               NVUtil::newNV("dataport.subscription_type",
					     "flush"));
      cprof.ports.length(1);
      cprof.ports[0] = portRef0;
      coil::vstring str;
      str = CORBA_SeqUtil::refToVstring(cprof.ports);
      int len = str.size();
      int pos = str[0].find("IOR", 0);
      CPPUNIT_ASSERT_EQUAL(1, len);
      CPPUNIT_ASSERT(pos != string::npos);

//      delete port0;
      // PortBase デストラクタで、deactivate_object を実行しています。
    }
    
  };
}; // namespace CORBA_SeqUtil

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(CORBA_SeqUtil::CORBA_SeqUtilTests);

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
#endif // CORBA_SeqUtil_cpp
