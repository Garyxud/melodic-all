// -*- C++ -*-
/*!
 * @file   CorbaNamingTests.cpp
 * @brief  CorbaNaming test class
 * @date   $Date: 2008/04/29 09:54:28 $
 *
 * $Id$
 *
 */

/*
 * $Log: CorbaNamingTests.cpp,v $
 * Revision 1.1  2008/04/29 09:54:28  arafune
 * The first commitment.
 *
 *
 */

#ifndef CorbaNaming_cpp
#define CorbaNaming_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/RTC.h>
#include <rtm/RTObject.h>
#include <rtm/CorbaNaming.h>

namespace RTC
{
  class CorbaObjectManager
  {
  public:
    CorbaObjectManager(::CORBA::ORB_ptr orb, ::PortableServer::POA_ptr poa)
      : m_orb(orb), m_poa(poa)
    {
    }

    void activate(::PortableServer::ServantBase* servant)
    {
      try
        {
          m_poa->activate_object(servant);
        }
      catch(const ::PortableServer::POA::ServantAlreadyActive &)
        {
        }
    }
  protected:
    ::CORBA::ORB_ptr m_orb;
    ::PortableServer::POA_ptr m_poa;
  };
};

/*!
 * @class CorbaNamingTests class
 * @brief CorbaNaming test
 */
namespace CorbaNaming
{
  class CorbaNamingTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(CorbaNamingTests);
    CPPUNIT_TEST(test_resolveStr);
    CPPUNIT_TEST(test_bindOrResolve);
    CPPUNIT_TEST(test_bindOrResolveContext);
    CPPUNIT_TEST(test_bind);
    CPPUNIT_TEST(test_bindByString);
    CPPUNIT_TEST(test_bindRecursive);
    CPPUNIT_TEST(test_bind_already_bound);
    CPPUNIT_TEST(test_rebind);
    CPPUNIT_TEST(test_rebindByString);
    CPPUNIT_TEST(tset_rebindRecursive);
    CPPUNIT_TEST(test_bindContext);
    CPPUNIT_TEST(test_bindContextRecursive);
    CPPUNIT_TEST(test_rebindContext);
    CPPUNIT_TEST(test_rebindContextRecursive);
    CPPUNIT_TEST(test_unbind);
    CPPUNIT_TEST(test_bindNewContext);
    CPPUNIT_TEST(test_destroyRecursive);
    CPPUNIT_TEST_SUITE_END();
	
  private:
    // helper
    CORBA::Object_ptr resolveRecursive(const char* fullName)
    {
      CORBA::Object_ptr context
	= CosNaming::NamingContext::_duplicate(m_pNaming->getRootContext());
			
      CosNaming::Name name = m_pNaming->toName(fullName);
			
      CORBA::ULong len(name.length());
      for (CORBA::ULong i = 0; i < len; ++i)
	{
	  if (m_pNaming->isNamingContext(context))
	    {
	      CosNaming::NamingContext_var nc
		= CosNaming::NamingContext::_narrow(context);
	      CPPUNIT_ASSERT(! CORBA::is_nil(nc));

	      CosNaming::Name subName = m_pNaming->subName(name, i, i);
				
	      CORBA::Object_ptr resolvedObj = nc->resolve(subName);
				
	      context =	resolvedObj;
					
	      if (i == len - 1)
	        {
                  return context;
	        }
	    }
	  else
	    {
	      CosNaming::NamingContext_var nc
		= CosNaming::NamingContext::_narrow(context);

	      throw CosNaming::NamingContext::CannotProceed(
							    nc, m_pNaming->subName(name, i));
	    }
	}
    }

    void printName(CosNaming::Name name)
    {
      CORBA::ULong len(name.length());
      for (CORBA::ULong i = 0; i < len; i++)
	{
	  std::cout << name[i].id << "." << name[i].kind;
	  if (i < len - 1)
	    {
	      std::cout << "/";
	    }
	}
      std::cout << std::endl;
    }
	
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
    RTC::CorbaNaming* m_pNaming;
	
  public:
    /*!
     * @brief Constructor
     */
    CorbaNamingTests()
    {
    }
		    
    /*!
     * @brief Destructor
     */
    virtual ~CorbaNamingTests()
    {
    }
		  
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
      int argc(0);
      char** argv(NULL);
			
      m_pORB = CORBA::ORB_init(argc, argv);
      m_pPOA = PortableServer::POA::_narrow(
					    m_pORB->resolve_initial_references("RootPOA"));
      m_pPOA->the_POAManager()->activate();
      m_pNaming = new RTC::CorbaNaming(m_pORB);

      m_pNaming->init("localhost:2809");
      m_pNaming->clearAll();
    }
    
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    {
      if (m_pNaming != NULL) delete m_pNaming;
    }
		
    /*!
     * @brief getNameServer(), toString(), toUrl(), resolveStr()メソッドのテスト
     * 
     * - 
     */
    void test_resolveStr()
    {
      std::string set_name("corbaloc::localhost:2809/NameService");
      m_pNaming->init("localhost:2809");

      // ネームサーバの名前を正しく取得できるか
      std::string get_name(m_pNaming->getNameServer());
      CPPUNIT_ASSERT_EQUAL(set_name, get_name);

      CosNaming::Name name;
      name.length(1);
      name[0].id = "id";
      name[0].kind = "kind";
      // 与えられた NameComponent の文字列表現を正しく返すか
      std::string str(m_pNaming->toString(name));
      CPPUNIT_ASSERT("id.kind" == str);

//      std::string url(m_pNaming->toUrl("", "" ));

      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
      RTC::RTObject_impl* rto = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto->getObjRef()));
      m_pNaming->bindByString("id.kind", rto->getObjRef());

      // 与えられた文字列表現を resolve しオブジェクトを返すか
      CORBA::Object_ptr obj = m_pNaming->resolveStr("id.kind");
      CPPUNIT_ASSERT(obj->_is_equivalent(rto->_this()));

      rto->_remove_ref();
    }
		
    /*!
     * @brief bindOrResolve()メソッドのテスト
     * 
     * - オブジェクトの名前をバインドまたは解決できるか
     */
    void test_bindOrResolve()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // バインド先となるコンテキストを登録しておく
      RTC::RTObject_impl* rto0 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto0);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto0->getObjRef()));

      const char* fullName0 = "id0-lv0.kind0-lv0/id0-lv1.kind0-lv1";
      m_pNaming->bind(m_pNaming->toName(fullName0), rto0->getObjRef());
      CORBA::Object_ptr obj0 = resolveRecursive("id0-lv0.kind0-lv0");
      CosNaming::NamingContext_var nc0 = CosNaming::NamingContextExt::_narrow(obj0);
      CPPUNIT_ASSERT(! CORBA::is_nil(nc0));
			
      // アクティブ化する
      RTC::RTObject_impl* rto = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto->getObjRef()));

      // オブジェクトをバインドし、それが正しく設定されるか？
      const char* fullName = "lv1-id.lv1-kind";
      CORBA::Object_ptr obj1 = m_pNaming->bindOrResolve(nc0._retn(), 
                                          m_pNaming->toName(fullName), rto->getObjRef());
      CORBA::Object_ptr obj = resolveRecursive("id0-lv0.kind0-lv0/lv1-id.lv1-kind");
      CPPUNIT_ASSERT(obj->_is_equivalent(rto->_this()));

      rto->_remove_ref();
      rto0->_remove_ref();
    }
		
    /*!
     * @brief bindOrResolveContext()メソッドのテスト
     * 
     * - 名前をバインドまたは解決できるか
     */
    void test_bindOrResolveContext()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // バインド先となるコンテキストを登録しておく
      RTC::RTObject_impl* rto0 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto0);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto0->getObjRef()));

      const char* fullName0 = "id0-lv0.kind0-lv0/id0-lv1.kind0-lv1";
      m_pNaming->bind(m_pNaming->toName(fullName0), rto0->getObjRef());
      CORBA::Object_ptr obj0 = resolveRecursive("id0-lv0.kind0-lv0");
      CosNaming::NamingContext_var nc0 = CosNaming::NamingContextExt::_narrow(obj0);
      CPPUNIT_ASSERT(! CORBA::is_nil(nc0));
			
      // アクティブ化する
      RTC::RTObject_impl* rto = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto->getObjRef()));

      // オブジェクトをバインドし、それが正しく設定されるか？
      const char* fullName = "lv1-id.lv1-kind";
      CosNaming::NamingContext_var nc1 = m_pNaming->bindOrResolveContext(nc0._retn(), 
                                          m_pNaming->toName(fullName) );
      CPPUNIT_ASSERT(! CORBA::is_nil(nc1));

      CosNaming::NamingContext_var nc2 = m_pNaming->bindOrResolveContext(nc1._retn(), 
                                          m_pNaming->toName(fullName), m_pNaming->newContext() );
      CPPUNIT_ASSERT(! CORBA::is_nil(nc2));

      rto->_remove_ref();
      rto0->_remove_ref();
    }
		
    /*!
     * @brief bind()メソッドのテスト
     * 
     * - オブジェクトをバインドし、それが正しく設定されるか？
     */
    void test_bind()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // アクティブ化する
      RTC::RTObject_impl* rto = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto->getObjRef()));

      // オブジェクトをバインドし、それが正しく設定されるか？
      const char* fullName = "lv1-id.lv1-kind/lv2-id.lv2-kind/lv3-id.lv3-kind";
      m_pNaming->bind(m_pNaming->toName(fullName), rto->getObjRef());
      CORBA::Object_ptr obj = resolveRecursive(fullName);
      CPPUNIT_ASSERT(obj->_is_equivalent(rto->_this()));
      rto->_remove_ref();
    }
		
    /*!
     * @brief bindByString()メソッドのテスト
     * 
     * - オブジェクトをバインドし、それが正しく設定されるか？
     */
    void test_bindByString()
    {

      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // アクティブ化する
      RTC::RTObject_impl* rto = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto->getObjRef()));

      // オブジェクトをバインドし、それが正しく設定されるか？
      const char* fullName = "lv1-id.lv1-kind/lv2-id.lv2-kind/lv3-id.lv3-kind";
      m_pNaming->bindByString(fullName, rto->getObjRef());
      CORBA::Object_ptr obj = resolveRecursive(fullName);
      CPPUNIT_ASSERT(obj->_is_equivalent(rto->_this()));
      rto->_remove_ref();
    }
		
    /*!
     * @brief bindRecursive()メソッドのテスト
     * 
     * - オブジェクトをバインドし、それが正しく設定されるか？
     */
    void test_bindRecursive()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // バインド先となるコンテキストを登録しておく
      RTC::RTObject_impl* rto0 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto0);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto0->getObjRef()));

      const char* fullName0 = "id0-lv0.kind0-lv0/id0-lv1.kind0-lv1";
      m_pNaming->bind(m_pNaming->toName(fullName0), rto0->getObjRef());

      CORBA::Object_ptr obj0 = resolveRecursive("id0-lv0.kind0-lv0");
      CosNaming::NamingContext_var nc0 = CosNaming::NamingContextExt::_narrow(obj0);
      CPPUNIT_ASSERT(! CORBA::is_nil(nc0));
			
      // アクティブ化する
      RTC::RTObject_impl* rto = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto->getObjRef()));

      // オブジェクトをバインドし、それが正しく設定されるか？
      const char* fullName = "lv1-id.lv1-kind/lv2-id.lv2-kind/lv3-id.lv3-kind";
      m_pNaming->bindRecursive(nc0._retn(), m_pNaming->toName(fullName), rto->getObjRef());
      CORBA::Object_ptr obj = resolveRecursive("id0-lv0.kind0-lv0/lv1-id.lv1-kind/lv2-id.lv2-kind/lv3-id.lv3-kind");
      CPPUNIT_ASSERT(obj->_is_equivalent(rto->_this()));
      rto->_remove_ref();
      rto0->_remove_ref();
    }
		
    /*!
     * @brief bind()メソッドのテスト
     * 
     * - すでにバインド済みの名称を指定してバインドを試みた場合、意図どおりの例外がスローされるか？
     */
    void test_bind_already_bound()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // アクティブ化する
      RTC::RTObject_impl* rto = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto->getObjRef()));

      // オブジェクトをバインドし、それが正しく設定されるか？
      const char* fullName = "lv1-id.lv1-kind/lv2-id.lv2-kind/lv3-id.lv3-kind";
      m_pNaming->bind(m_pNaming->toName(fullName), rto->getObjRef());
      CORBA::Object_ptr obj = resolveRecursive(fullName);
      CPPUNIT_ASSERT(obj->_is_equivalent(rto->_this()));
			
      // すでにバインド済みの名称を指定してバインドを試みた場合、意図どおりの例外がスローされるか？
      try
	{
	  m_pNaming->bind(m_pNaming->toName(fullName), rto->getObjRef());
	  CPPUNIT_FAIL("Expected exception not thrown.");
	}
      catch (CosNaming::NamingContext::AlreadyBound expected) {}
      rto->_remove_ref();
    }
		
    /*!
     * @brief rebind()メソッドのテスト
     * 
     * - すでにバインド済みの名称を指定してリバインドを試みた場合、新しいバインディングに置き換わるか？
     */
    void test_rebind()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // アクティブ化する
      RTC::RTObject_impl* rto1 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto1);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto1->getObjRef()));
      RTC::RTObject_impl* rto2 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto2);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto2->getObjRef()));

      // オブジェクトをバインドし、それが正しく設定されるか？
      const char* fullName = "lv1-id.lv1-kind/lv2-id.lv2-kind/lv3-id.lv3-kind";
      m_pNaming->bind(m_pNaming->toName(fullName), rto1->getObjRef());
      CORBA::Object_ptr obj1 = resolveRecursive(fullName);
      CPPUNIT_ASSERT(obj1->_is_equivalent(rto1->_this()));
			
      // すでにバインド済みの名称を指定してリバインドを試みた場合、新しいバインディングに置き換わるか？
      m_pNaming->rebind(m_pNaming->toName(fullName), rto2->getObjRef());
      CORBA::Object_ptr obj2 = resolveRecursive(fullName);
      CPPUNIT_ASSERT(obj2->_is_equivalent(rto2->_this()));
      rto2->_remove_ref();
      rto1->_remove_ref();
    }
		
    /*!
     * @brief rebindByString()メソッドのテスト
     * 
     * - すでにバインド済みの名称を指定してリバインドを試みた場合、新しいバインディングに置き換わるか？
     */
    void test_rebindByString()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // アクティブ化する
      RTC::RTObject_impl* rto1 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto1);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto1->getObjRef()));
      RTC::RTObject_impl* rto2 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto2);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto2->getObjRef()));

      // オブジェクトをバインドし、それが正しく設定されるか？
      const char* fullName = "lv1-id.lv1-kind/lv2-id.lv2-kind/lv3-id.lv3-kind";
      m_pNaming->bind(m_pNaming->toName(fullName), rto1->getObjRef());
      CORBA::Object_ptr obj1 = resolveRecursive(fullName);
      CPPUNIT_ASSERT(obj1->_is_equivalent(rto1->_this()));
			
      // すでにバインド済みの名称を指定してリバインドを試みた場合、新しいバインディングに置き換わるか？
      m_pNaming->rebindByString(fullName, rto2->getObjRef());
      CORBA::Object_ptr obj2 = resolveRecursive(fullName);
      CPPUNIT_ASSERT(obj2->_is_equivalent(rto2->_this()));
      rto2->_remove_ref();
      rto1->_remove_ref();
    }
		
    /*!
     * @brief rebindRecursive()メソッドのテスト
     * 
     * - すでにバインド済みの名称を指定してリバインドを試みた場合、正しく新しいバインディングに置き換わるか？
     */
    void tset_rebindRecursive()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // バインド先となるコンテキストを作成し、バインドしておく
      CosNaming::NamingContext_ptr ncParent = m_pNaming->newContext();
      CPPUNIT_ASSERT(! CORBA::is_nil(ncParent));
			
      m_pNaming->bindContext("id-parent.kind-parent", ncParent);
      CORBA::Object_ptr objParent = resolveRecursive("id-parent.kind-parent");
      CPPUNIT_ASSERT(objParent->_is_equivalent(ncParent));
			
      // テスト用にバインドするオブジェクトを作成しておく
      RTC::RTObject_impl* rto1 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto1);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto1->getObjRef()));
			
      RTC::RTObject_impl* rto2 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto2);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto2->getObjRef()));
			
      // １つ目のオブジェクトをバインドする
      m_pNaming->bindRecursive(
			       ncParent, m_pNaming->toName("id-rto.kind-rto"), rto1->getObjRef());
      CORBA::Object_ptr objRto1 = resolveRecursive(
						   "id-parent.kind-parent/id-rto.kind-rto");
      CPPUNIT_ASSERT(objRto1->_is_equivalent(rto1->getObjRef()));
			
      // ２つ目のオブジェクトを、１つ目と同じ位置に正しくリバインドできるか？
      m_pNaming->rebindRecursive(
				 ncParent, m_pNaming->toName("id-rto.kind-rto"), rto2->getObjRef());
      CORBA::Object_ptr objRto2 = resolveRecursive(
						   "id-parent.kind-parent/id-rto.kind-rto");
      CPPUNIT_ASSERT(objRto2->_is_equivalent(rto2->getObjRef()));
      rto2->_remove_ref();
      rto1->_remove_ref();
    }
		
    /*!
     * @brief bindContext()メソッドのテスト
     * 
     * - NamingContextを正しくバインドできるか？
     */
    void test_bindContext()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);

      // バインド先となるコンテキストを登録しておく
      RTC::RTObject_impl* rto = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto->getObjRef()));

      // テストに用いるコンテキストを準備しておく
      CosNaming::NamingContext_ptr nc = m_pNaming->newContext();
      CPPUNIT_ASSERT(! CORBA::is_nil(nc));
			
      nc->bind(m_pNaming->toName("id.kind"), rto->getObjRef());
			
      // コンテキストをバインドし、正しくバインドできたことを確認する
      m_pNaming->bindContext("id-lv0.kind-lv0/id-lv1.kind-lv1", nc);
			
      CORBA::Object_ptr objNc = resolveRecursive("id-lv0.kind-lv0/id-lv1.kind-lv1");
      CPPUNIT_ASSERT(objNc->_is_equivalent(nc));
			
      CORBA::Object_ptr objRto = resolveRecursive("id-lv0.kind-lv0/id-lv1.kind-lv1/id.kind");
      CPPUNIT_ASSERT(objRto->_is_equivalent(rto->getObjRef()));
      rto->_remove_ref();
    }

    /*!
     * @brief bindContextRecursive()メソッドのテスト
     * 
     * - バインド先となるコンテキストを指定して、バインド対象となるコンテキストをバインドできるか？
     */
    void test_bindContextRecursive()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);

      // バインド先となるコンテキストを作成し、あらかじめバインドしておく
      CosNaming::NamingContext_ptr ncParent = m_pNaming->newContext();
      CPPUNIT_ASSERT(! CORBA::is_nil(ncParent));
			
      m_pNaming->bindContext("id-parent.kind-parent", ncParent);
      CORBA::Object_ptr objParent = resolveRecursive("id-parent.kind-parent");
      CPPUNIT_ASSERT(objParent->_is_equivalent(ncParent));
			
      // バインドするコンテキストを作成しておく
      RTC::RTObject_impl* rto = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto->getObjRef()));

      CosNaming::NamingContext_ptr nc = m_pNaming->newContext();
      CPPUNIT_ASSERT(! CORBA::is_nil(nc));
      nc->bind(m_pNaming->toName("id.kind"), rto->getObjRef());
			
      // バインド先となるコンテキストを指定して、バインド対象となるコンテキストをバインドできるか？
      m_pNaming->bindContextRecursive(
				      ncParent, m_pNaming->toName("id-lv0.kind-lv0/id-lv1.kind-lv1"), nc);
			
      CORBA::Object_ptr objNc = resolveRecursive(
						 "id-parent.kind-parent/id-lv0.kind-lv0/id-lv1.kind-lv1");
      CPPUNIT_ASSERT(objNc->_is_equivalent(nc));
			
      CORBA::Object_ptr objRto = resolveRecursive(
						  "id-parent.kind-parent/id-lv0.kind-lv0/id-lv1.kind-lv1/id.kind");
      CPPUNIT_ASSERT(objRto->_is_equivalent(rto->getObjRef()));
      rto->_remove_ref();
    }
		
    /*!
     * @brief rebindContext()メソッドのテスト
     * 
     * - すでにバインド済みのコンテキストを同じ位置に別のコンテキストのバインドし、正しくリバインドできることを確認する
     */
    void test_rebindContext()
    {

      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);

      // テストに用いるコンテキストを準備しておく
      RTC::RTObject_impl* rto1 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto1);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto1->getObjRef()));
			
      RTC::RTObject_impl* rto2 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto2);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto2->getObjRef()));
			
      CosNaming::NamingContext_ptr nc1 = m_pNaming->newContext();
      CPPUNIT_ASSERT(! CORBA::is_nil(nc1));
      nc1->bind(m_pNaming->toName("id-rto.kind-rto"), rto1->getObjRef());
			
      CosNaming::NamingContext_ptr nc2 = m_pNaming->newContext();
      CPPUNIT_ASSERT(! CORBA::is_nil(nc2));
      nc2->bind(m_pNaming->toName("id-rto.kind-rto"), rto2->getObjRef());
			
      // １つ目のコンテキストをバインドする
      m_pNaming->bindContext(m_pNaming->toName("id-nc.kind-nc"), nc1);
      CORBA::Object_ptr objRto1 = resolveRecursive("id-nc.kind-nc/id-rto.kind-rto");
      CPPUNIT_ASSERT(objRto1->_is_equivalent(rto1->getObjRef()));
			
      // ２つ目のコンテキストを同じ位置にバインドし、正しくリバインドできることを確認する
      m_pNaming->rebindContext(m_pNaming->toName("id-nc.kind-nc"), nc2);
      CORBA::Object_ptr objRto2 = resolveRecursive("id-nc.kind-nc/id-rto.kind-rto");
      CPPUNIT_ASSERT(objRto2->_is_equivalent(rto2->getObjRef()));
      rto2->_remove_ref();
      rto1->_remove_ref();
    }

    /*!
     * @brief rebindContextRecursive()メソッドのテスト
     * 
     * - すでにバインド済みのコンテキストと同じ位置に、異なるコンテキストを正しくリバインドできるか？
     */
    void test_rebindContextRecursive()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // バインド先となるコンテキストを作成し、バインドしておく
      CosNaming::NamingContext_ptr ncParent = m_pNaming->newContext();
      CPPUNIT_ASSERT(! CORBA::is_nil(ncParent));
			
      m_pNaming->bindContext("id-parent.kind-parent", ncParent);
      CORBA::Object_ptr objParent = resolveRecursive("id-parent.kind-parent");
      CPPUNIT_ASSERT(objParent->_is_equivalent(ncParent));
			
      // テストに用いるコンテキストを準備しておく
      RTC::RTObject_impl* rto1 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto1);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto1->getObjRef()));
			
      RTC::RTObject_impl* rto2 = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto2);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto2->getObjRef()));
			
      CosNaming::NamingContext_ptr nc1 = m_pNaming->newContext();
      CPPUNIT_ASSERT(! CORBA::is_nil(nc1));
      nc1->bind(m_pNaming->toName("id-rto.kind-rto"), rto1->getObjRef());
			
      CosNaming::NamingContext_ptr nc2 = m_pNaming->newContext();
      CPPUNIT_ASSERT(! CORBA::is_nil(nc2));
      nc2->bind(m_pNaming->toName("id-rto.kind-rto"), rto2->getObjRef());
			
      // １つ目のコンテキストをバインドする
      m_pNaming->bindContextRecursive(
				      ncParent, m_pNaming->toName("id-nc.kind-nc"), nc1);
      CORBA::Object_ptr objRto1 = resolveRecursive(
						   "id-parent.kind-parent/id-nc.kind-nc/id-rto.kind-rto");
      CPPUNIT_ASSERT(objRto1->_is_equivalent(rto1->getObjRef()));
			
      // ２つ目のコンテキストを同じ位置にバインドし、正しくリバインドできることを確認する
      m_pNaming->rebindContextRecursive(
					ncParent, m_pNaming->toName("id-nc.kind-nc"), nc2);
      CORBA::Object_ptr objRto2 = resolveRecursive(
						   "id-parent.kind-parent/id-nc.kind-nc/id-rto.kind-rto");
      CPPUNIT_ASSERT(objRto2->_is_equivalent(rto2->getObjRef()));
      rto2->_remove_ref();
      rto1->_remove_ref();
    }
		
    void test_resolve()
    {
      // 他テスト群にて兼ねる
    }
		
    /*!
     * @brief unbind()メソッドのテスト
     * 
     * - バインド済みのオブジェクトを正常にアンバインドできるか？
     * - アンバインド済みのオブジェクトの解決を試みて、意図どおりの例外がスローされるか？
     */
    void test_unbind()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // アクティブ化する
      RTC::RTObject_impl* rto = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto->getObjRef()));

      // オブジェクトをバインドする
      m_pNaming->bindByString("id.kind", rto->getObjRef());
      CORBA::Object_ptr obj = m_pNaming->resolve("id.kind");
      CPPUNIT_ASSERT(obj->_is_equivalent(rto->_this()));
			
      // オブジェクトをアンバインドする
      m_pNaming->unbind("id.kind");
			
      // アンバインドしたオブジェクトの解決を試みて、意図どおりの例外がスローされるか？
      try
	{
	  m_pNaming->resolve("id.kind");
	  CPPUNIT_FAIL("Expected exception not thrown.");
	}
      catch (CosNaming::NamingContext::NotFound expected)
	{
	  // do nothing
	}
      catch (...)
	{
	  CPPUNIT_FAIL("Unexpected exception catched.");
	}
      rto->_remove_ref();
    }
		
    /*!
     * @brief bindNewContext()メソッドのテスト
     * 
     * - 新しいコンテキストをバインドできるか？
     * - バインドしたコンテキストを利用してオブジェクト参照を正しくバインドできるか？
     */
    void test_bindNewContext()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // アクティブ化する
      RTC::RTObject_impl* rto = new RTC::RTObject_impl(m_pORB, m_pPOA);
      objMgr.activate(rto);
      CPPUNIT_ASSERT(! CORBA::is_nil(rto->getObjRef()));

      // 新しいコンテキストをバインドできるか？
      CosNaming::NamingContext_ptr nc
	= m_pNaming->bindNewContext(m_pNaming->toName("id-lv0.kind-lv0"));
      CPPUNIT_ASSERT(! CORBA::is_nil(nc));
			
      // バインドしたコンテキストを利用してオブジェクト参照をバインドできるか？
      nc->bind(m_pNaming->toName("id.kind"), rto->getObjRef());
      CORBA::Object_ptr obj = resolveRecursive("id-lv0.kind-lv0/id.kind");
      CPPUNIT_ASSERT(! CORBA::is_nil(obj));

      rto->_remove_ref();
    }
		
    void test_destroy()
    {
      // CosNaming::NamingContext::destroy()への単純な委譲なので、テスト省略
    }
		
    void test_destroyRecursive()
    {
      RTC::CorbaObjectManager objMgr(m_pORB, m_pPOA);
			
      // destroy対象となるコンテキストのツリーを構築する（親１：子２）
      // (親)
      CosNaming::NamingContext_ptr ncParent = m_pNaming->newContext();
      CPPUNIT_ASSERT(! CORBA::is_nil(ncParent));
			
      m_pNaming->bindContext("id-parent.kind-parent", ncParent);
      CORBA::Object_ptr objNcParent = resolveRecursive("id-parent.kind-parent");
      CPPUNIT_ASSERT(objNcParent->_is_equivalent(ncParent));
			
      // (子１)
      CosNaming::NamingContext_ptr nc1 = m_pNaming->newContext();
      CPPUNIT_ASSERT(! CORBA::is_nil(nc1));
			
      m_pNaming->bindContextRecursive(ncParent, m_pNaming->toName("id1.kind1"), nc1);
      CORBA::Object_ptr objNc1 = resolveRecursive("id-parent.kind-parent/id1.kind1");
      CPPUNIT_ASSERT(objNc1->_is_equivalent(nc1));

      // (子２)
      CosNaming::NamingContext_ptr nc2 = m_pNaming->newContext();
      CPPUNIT_ASSERT(! CORBA::is_nil(nc2));
			
      m_pNaming->bindContextRecursive(ncParent, m_pNaming->toName("id2.kind2"), nc2);
      CORBA::Object_ptr objNc2 = resolveRecursive("id-parent.kind-parent/id2.kind2");
      CPPUNIT_ASSERT(objNc2->_is_equivalent(nc2));
			
      // destroyRecursive()を呼び出す
      m_pNaming->destroyRecursive(m_pNaming->getRootContext());
			
      // 各コンテキストがdestroyされているか？
      // （各コンテキストへのメソッド呼出しを行い、意図どおり例外がスローされるか？）
      // ※CORBA実装によってはCORBA::OBJECT_NOT_EXIST例外ではないかも知れない？
      //   そこで、CORBA::OBJECT_NOT_EXISTでない場合も考慮してチェックしている。

      //new_context doesn't throw out the exception. 
      //Therefore, the following tests are omitted.  
/*
      {

	bool thrown = false;
	try
	  {
	    ncParent->new_context();
	  }
	catch (CORBA::OBJECT_NOT_EXIST expected)
	  {
	    // expected
	    thrown = true;
	  }
	catch (...)
	  {
	    // expected
	    thrown = true;
	  }
	if (! thrown) CPPUNIT_FAIL("Exception not thrown.");

      }
			
      {
	bool thrown = false;
	try
	  {
	    nc1->new_context();
	  }
	catch (CORBA::OBJECT_NOT_EXIST expected)
	  {
	    // expected
	    thrown = true;
	  }
	catch (...)
	  {
	    // expected
	    thrown = true;
	  }
	if (! thrown) CPPUNIT_FAIL("Exception not thrown.");
      }
			
      {
	bool thrown = false;
	try
	  {
	    nc2->new_context();
	  }
	catch (CORBA::OBJECT_NOT_EXIST expected)
	  {
	    // expected
	    thrown = true;
	  }
	catch (...)
	  {
	    // expected
	    thrown = true;
	  }
	if (! thrown) CPPUNIT_FAIL("Exception not thrown.");
      }
*/
    }
		
  };
}; // namespace CorbaNaming

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(CorbaNaming::CorbaNamingTests);

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
#endif // CorbaNaming_cpp
