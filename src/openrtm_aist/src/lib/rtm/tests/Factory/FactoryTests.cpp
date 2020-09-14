// -*- C++ -*-
/*!
 * @file   FactoryTests.cpp
 * @brief  Factory test class
 * @date   $Date: 2008/05/02 12:30:29 $
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
 * $Log: FactoryTests.cpp,v $
 * Revision 1.2  2008/05/02 12:30:29  arafune
 * Modified some tests.
 *
 * Revision 1.1  2007/12/20 07:50:18  arafune
 * *** empty log message ***
 *
 * Revision 1.2  2007/01/12 14:50:35  n-ando
 * A trivial fix.
 *
 * Revision 1.1  2006/11/27 08:31:38  n-ando
 * TestSuites are devided into each directory.
 *
 *
 */

#ifndef Factory_cpp
#define Factory_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/RTObject.h>
#include <rtm/Factory.h>
#include <rtm/Manager.h>
#include <coil/Properties.h>


namespace Tests
{
  class Logger
  {
  public:
    void log(const std::string& msg)
    {
      m_log.push_back(msg);
    }
		
    int countLog(const std::string& line)
    {
      int count = 0;
      for (int i = 0; i < (int) m_log.size(); ++i)
	{
	  if (m_log[i] == line) ++count;
	}
      return count;
    }
		
  private:
    std::vector<std::string> m_log;
  };
	
  class RTObjectMock
    : public virtual RTC::RTObject_impl
  {
  public:
    RTObjectMock(CORBA::ORB_ptr orb, PortableServer::POA_ptr poa)
      : RTC::RTObject_impl(orb, poa), m_logger(NULL)
    {
    }
		
  public: // helper for test
    void setLogger(Logger* logger)
    {
      m_logger = logger;
    }
		
  private:
    Logger* m_logger;
	
  private:
    void log(const std::string& msg)
    {
      if (m_logger != NULL) m_logger->log(msg);
    }
  };

  RTC::RtcBase* CreateRTObjectMock(RTC::Manager* manager)
  {
    CORBA::ORB_ptr orb = manager->getORB();
    PortableServer::POA_ptr poa = manager->getPOA();
    return new RTObjectMock(orb, poa);
  }

  void DeleteRTObjectMock(RTC::RtcBase* rtc)
  {
    if (rtc != NULL) rtc->_remove_ref();
  }
	
  class FactoryTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(FactoryTests);
    CPPUNIT_TEST(test_create_and_destroy);
    CPPUNIT_TEST(test_profile);
    CPPUNIT_TEST(test_number);
    CPPUNIT_TEST_SUITE_END();

  private:
    RTC::Manager* m_mgr;
	
  public:
    /*!
     * @brief Constructor
     */
    FactoryTests()
    {
    }
		
    /*!
     * @brief Destructor
     */
    ~FactoryTests()
    {
    }
		
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
      m_mgr = RTC::Manager::init(0, NULL);
    }
		
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    {
      // m_mgr->terminate();
    }
		
    /*!
     * @brief create()メソッドとdestroy()メソッドのテスト
     * 
     * - 正常にコンポーネントを生成できるか？
     * - 生成されたコンポーネントには、正しくプロパティが設定されているか？
     * - 正常にコンポーネントを破棄できるか？（デストラクタが呼ばれるか？）
     */
    void test_create_and_destroy()
    {
      coil::Properties properties;
      properties.setProperty("name", "NAME");
			
      RTC::FactoryCXX factory(
			      properties, CreateRTObjectMock, DeleteRTObjectMock);
			
      // 正常にコンポーネントを生成できるか？
      RTC::RtcBase* rtc = factory.create(m_mgr);
      CPPUNIT_ASSERT(rtc != NULL);
			
      RTObjectMock* mock = dynamic_cast<RTObjectMock*>(rtc);
      CPPUNIT_ASSERT(mock != NULL);
			
      Logger logger;
      mock->setLogger(&logger);
			
      // 生成されたコンポーネントには、正しくプロパティが設定されているか？
      coil::Properties propertiesRet = rtc->getProperties();
      CPPUNIT_ASSERT_EQUAL(std::string("NAME"), propertiesRet.getProperty("name"));
			
      // 正常にコンポーネントを破棄できるか？
      factory.destroy(rtc);
    }
		
    /*!
     * @brief profile()メソッドのテスト
     * 
     * - コンストラクタで指定したプロパティを取得できるか？
     */
    void test_profile()
    {
      coil::Properties properties;
      properties.setProperty("name", "NAME");
			
      RTC::FactoryCXX factory(
			      properties, CreateRTObjectMock, DeleteRTObjectMock);
			
      // コンストラクタで指定したプロパティを取得できるか？
      coil::Properties propertiesRet = factory.profile();
      CPPUNIT_ASSERT_EQUAL(std::string("NAME"), propertiesRet.getProperty("name"));
    }
		
    /*!
     * @brief number()メソッドのテスト
     * 
     * - 生成したインスタンス数が正しく得られるか？
     */
    void test_number()
    {
			
      coil::Properties properties;
      properties.setProperty("name", "NAME");
			
      RTC::FactoryCXX factory(
			      properties, CreateRTObjectMock, DeleteRTObjectMock);
			
      int MAX_NUM = 1;
			
      std::vector<RTC::RtcBase*> rtcList;
      for (int i = 0; i < MAX_NUM; ++i)
	{
	  // create()呼出前のインスタンス数は期待どおりか？
	  CPPUNIT_ASSERT_EQUAL(i-1, factory.number());
				
	  // createする
	  RTC::RtcBase* rtc = factory.create(m_mgr);
	  CPPUNIT_ASSERT(rtc != NULL);
				
	  // create()呼出後のインスタンス数は期待どおりか？
	  CPPUNIT_ASSERT_EQUAL(i, factory.number());
				
	  rtcList.push_back(rtc);
	}
			
      for (int i = 0; i < MAX_NUM; ++i)
	{
	  // destroy()呼出前のインスタンス数は期待どおりか？
	  CPPUNIT_ASSERT_EQUAL(i, factory.number());
				
	  try {
	    // destroyする
	    factory.destroy(rtcList[i]);
	  }
	  catch (...) {}
				
	  // destroy()呼出後のインスタンス数は期待どおりか？
	  CPPUNIT_ASSERT_EQUAL(i-1, factory.number());
	}

    }
		
  };
}; // namespace Factory

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Tests::FactoryTests);

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
#endif // Factory_cpp
