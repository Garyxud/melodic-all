// -*- C++ -*-
/*!
 * @file   SystemLoggerTests.cpp
 * @brief  SystemLogger test class
 * @date   $Date: 2008/05/12 03:58:45 $
 *
 * $Id$
 *
 */

/*
 * $Log: SystemLoggerTests.cpp,v $
 * Revision 1.2  2008/05/12 03:58:45  arafune
 * Added some tests.
 * Rearranged tests in a different order.
 *
 * Revision 1.1  2008/05/09 12:01:44  arafune
 * The first commitment.
 *
 *
 */

#ifndef SystemLoggerTests_cpp
#define SystemLoggerTests_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/Task.h>
#include <coil/DynamicLib.h>

#include <rtm/SystemLogger.h>
#include <coil/TimeMeasure.h>
#include <rtm/Manager.h>

/*!
 * @class SystemLoggerTests class
 * @brief SystemLogger test
 */
namespace Tests
{
  // protected: 関数のテスト用
  class LoggerMock : public RTC::Logger
  {
  public:
    // コンストラクタ
    LoggerMock(RTC::LogStreamBuf* streambuf)
      : RTC::Logger(streambuf) {}
    virtual ~LoggerMock(void) {}


    void setDateFormat(const char* format)
    {
//      std::cout << "LoggerMock::setDateFormat() IN" << std::endl;
      RTC::Logger::setDateFormat(format);
      mock_m_dateFormat = std::string(format);
//      std::cout << "LoggerMock::setDateFormat() OUT" << std::endl;
    }

    void setName(const char* name)
    {
//      std::cout << "LoggerMock::setName() IN" << std::endl;
      RTC::Logger::setName(name);
      mock_m_name = name;
//      std::cout << "LoggerMock::setName() OUT" << std::endl;
    }

    void header(int level)
    {
//      std::cout << "LoggerMock::header() IN" << std::endl;
      RTC::Logger::header(level);
//      std::cout << "LoggerMock::header() OUT" << std::endl;
    }

    std::string getDate(void)
    {
//      std::cout << "LoggerMock::getDate() IN" << std::endl;
      return RTC::Logger::getDate();
    }

    int strToLevel(const char* level)
    {
//      std::cout << "LoggerMock::strToLevel() IN" << std::endl;
      return RTC::Logger::strToLevel(level);
    }

    // デッドロック確認用
    std::string test_string(void)
    {
//      std::cout << "LoggerMock::test_string() IN" << std::endl;
      this->lock();
      this->level(RTC::Logger::RTL_TRACE) << "RTC_TRACE2 test_string()" << std::endl;
      this->unlock();
//      std::cout << "LoggerMock::test_string() OUT" << std::endl;
      return std::string("TestString");
    }

    std::string mock_m_name;
    std::string mock_m_dateFormat;
  };


  /*!
   * 
   * 
   *
   */
  class SystemLoggerTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(SystemLoggerTests);

    CPPUNIT_TEST(test_setLevel);
    CPPUNIT_TEST(test_setDateFormat);
    CPPUNIT_TEST(test_setName);
    CPPUNIT_TEST(test_header);
    CPPUNIT_TEST(test_getDate);
    CPPUNIT_TEST(test_strToLevel);
    CPPUNIT_TEST(test_logfile_PARANOID);
    CPPUNIT_TEST(test_logfile_VERBOSE);
    CPPUNIT_TEST(test_logfile_TRACE);
    CPPUNIT_TEST(test_logfile_DEBUG);
    CPPUNIT_TEST(test_logfile_INFO);
    CPPUNIT_TEST(test_logfile_WARNING);
    CPPUNIT_TEST(test_logfile_ERROR);
    CPPUNIT_TEST(test_logfile_FATAL);
    CPPUNIT_TEST(test_logfile_SILENT);
    CPPUNIT_TEST(test_constract_name);
    CPPUNIT_TEST(test_deadlock);

    CPPUNIT_TEST_SUITE_END();

  private:

  protected:

  public:
    /*!
     * @brief Constructor
     */
    SystemLoggerTests()
    {
    }

    /*!
     * @brief Destructor
     */
    virtual ~SystemLoggerTests()
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
     * @brief setLevel()メソッドのテスト
     * 
     * - ログレベルの文字列を正しく設定できるか？
     */
    void test_setLevel(void)
    {
//      std::cout << "test_setLevel() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::stringstream s0;

      logger.addStream(s0.rdbuf());
      LoggerMock rtclog(&logger);

      // ログレベル設定で、引数に規定値・規定外を設定して正しく動作するか？
      CPPUNIT_ASSERT(rtclog.setLevel("SILENT"));  //規定値
      CPPUNIT_ASSERT(rtclog.setLevel("FATAL"));
      CPPUNIT_ASSERT(rtclog.setLevel("ERROR"));
      CPPUNIT_ASSERT(rtclog.setLevel("WARN"));
      CPPUNIT_ASSERT(rtclog.setLevel("INFO"));
      CPPUNIT_ASSERT(rtclog.setLevel("DEBUG"));
      CPPUNIT_ASSERT(rtclog.setLevel("TRACE"));
      CPPUNIT_ASSERT(rtclog.setLevel("VERBOSE"));
      CPPUNIT_ASSERT(rtclog.setLevel("PARANOID"));
      CPPUNIT_ASSERT(rtclog.setLevel("other"));  //規定外
//      std::cout << "test_setLevel() OUT" << std::endl;
    }

    /*!
     * @brief setDateFormat()メソッドのテスト
     * 
     * - ヘッダに付加する日時フォーマットを正しく指定できるか？
     */
    void test_setDateFormat(void)
    {
//      std::cout << "test_setDateFormat() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::stringstream s0;
      std::string rstr;

      logger.addStream(s0.rdbuf());
      LoggerMock rtclog(&logger);

      // ヘッダに付加する日時フォーマットを正しく指定できるか？
      rtclog.setDateFormat("");
      rstr = rtclog.getDate();
      CPPUNIT_ASSERT(rstr.size() == 0);
      CPPUNIT_ASSERT_EQUAL(std::string(""), rtclog.mock_m_dateFormat);

      rtclog.setDateFormat("%b %d %H:%M:%S");
      rstr = rtclog.getDate();
      CPPUNIT_ASSERT(rstr.size() > 0);
      CPPUNIT_ASSERT_EQUAL(std::string("%b %d %H:%M:%S"), rtclog.mock_m_dateFormat);
//      std::cout << "test_setDateFormat() OUT" << std::endl;
    }

    /*!
     * @brief setName()メソッドのテスト
     * 
     * - ヘッダの日時の後に付加する文字列を正しく設定できるか？
     */
    void test_setName(void)
    {
//      std::cout << "test_setName() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::stringstream s0;

      logger.addStream(s0.rdbuf());
      LoggerMock rtclog(&logger);
      rtclog.setDateFormat("");
      // ヘッダの日時の後に付加する文字列を正しく設定できるか？
      rtclog.setName("");  		// 付加文字列：なし
      rtclog.header(rtclog.RTL_DEBUG);
      CPPUNIT_ASSERT_EQUAL(std::string(""), rtclog.mock_m_name);
      CPPUNIT_ASSERT_EQUAL(std::string(" DEBUG: : "), s0.str());

      s0.str("");
      rtclog.setName("TestName");  	// 付加文字列：あり
      rtclog.header(rtclog.RTL_DEBUG);
      CPPUNIT_ASSERT_EQUAL(std::string("TestName"), rtclog.mock_m_name);
      CPPUNIT_ASSERT_EQUAL(std::string(" DEBUG: TestName: "), s0.str());
//      std::cout << "test_setName() OUT" << std::endl;
    }

    /*!
     * @brief header()メソッドのテスト
     * 
     * - メッセージのプリフィックス追加を正しく設定できるか？
     */
    void test_header(void)
    {
//      std::cout << "test_header() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::stringstream s0;

      logger.addStream(s0.rdbuf());
      LoggerMock rtclog(&logger);

      // メッセージのプリフィックス追加を正しく設定できるか？
      rtclog.setDateFormat("");
      rtclog.setName("");
      rtclog.header(rtclog.RTL_SILENT);
      CPPUNIT_ASSERT_EQUAL(std::string(" SILENT: : "), s0.str());
      s0.str("");
      rtclog.header(rtclog.RTL_FATAL);
      CPPUNIT_ASSERT_EQUAL(std::string(" FATAL: : "), s0.str());
      s0.str("");
      rtclog.header(rtclog.RTL_ERROR);
      CPPUNIT_ASSERT_EQUAL(std::string(" ERROR: : "), s0.str());
      s0.str("");
      rtclog.header(rtclog.RTL_WARN);
      CPPUNIT_ASSERT_EQUAL(std::string(" WARNING: : "), s0.str());
      s0.str("");
      rtclog.header(rtclog.RTL_INFO);
      CPPUNIT_ASSERT_EQUAL(std::string(" INFO: : "), s0.str());
      s0.str("");
      rtclog.header(rtclog.RTL_DEBUG);
      CPPUNIT_ASSERT_EQUAL(std::string(" DEBUG: : "), s0.str());
      s0.str("");
      rtclog.header(rtclog.RTL_TRACE);
      CPPUNIT_ASSERT_EQUAL(std::string(" TRACE: : "), s0.str());
      s0.str("");
      rtclog.header(rtclog.RTL_VERBOSE);
      CPPUNIT_ASSERT_EQUAL(std::string(" VERBOSE: : "), s0.str());
      s0.str("");
      rtclog.header(rtclog.RTL_PARANOID);
      CPPUNIT_ASSERT_EQUAL(std::string(" PARANOID: : "), s0.str());
//      std::cout << "test_header() OUT" << std::endl;
    }

    /*!
     * @brief getDate()メソッドのテスト
     * 
     * - フォーマットされた現在日時文字列を正しく取得できるか？
     */
    void test_getDate(void)
    {
//      std::cout << "test_getDate() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::stringstream s0;
      std::string rstr;

      logger.addStream(s0.rdbuf());
      LoggerMock rtclog(&logger);

      // フォーマットされた現在日時文字列を正しく取得できるか？
      rtclog.setDateFormat("");
      rstr = rtclog.getDate();
      CPPUNIT_ASSERT(rstr.size() == 0);

      rtclog.setDateFormat("%b %d %H:%M:%S");
      rstr = rtclog.getDate();
      CPPUNIT_ASSERT(rstr.size() > 0);
//      std::cout << "test_getDate() OUT" << std::endl;
    }

    /*!
     * @brief strToLevel()メソッドのテスト
     * 
     * - ログレベルを正しく設定できるか？
     */
    void test_strToLevel(void)
    {
//      std::cout << "test_strToLevel() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::stringstream s0;

      logger.addStream(s0.rdbuf());
      LoggerMock rtclog(&logger);

      // ログレベル設定で、引数に規定値・規定外を設定して正しく動作するか？
      CPPUNIT_ASSERT(rtclog.strToLevel("SILENT") == rtclog.RTL_SILENT);  //規定値
      CPPUNIT_ASSERT(rtclog.strToLevel("FATAL") == rtclog.RTL_FATAL);
      CPPUNIT_ASSERT(rtclog.strToLevel("ERROR") == rtclog.RTL_ERROR);
      CPPUNIT_ASSERT(rtclog.strToLevel("WARN") == rtclog.RTL_WARN);
      CPPUNIT_ASSERT(rtclog.strToLevel("INFO") == rtclog.RTL_INFO);
      CPPUNIT_ASSERT(rtclog.strToLevel("DEBUG") == rtclog.RTL_DEBUG);
      CPPUNIT_ASSERT(rtclog.strToLevel("TRACE") == rtclog.RTL_TRACE);
      CPPUNIT_ASSERT(rtclog.strToLevel("VERBOSE") == rtclog.RTL_VERBOSE);
      CPPUNIT_ASSERT(rtclog.strToLevel("PARANOID") == rtclog.RTL_PARANOID);
      CPPUNIT_ASSERT(rtclog.strToLevel("other") == rtclog.RTL_SILENT);  //規定外
//      std::cout << "test_strToLevel() OUT" << std::endl;
    }

    /*!
     * @brief logfile出力のテスト
     * 
     * - ログレベルを PARANOID にした場合のファイル出力が正しく行われるか？
     */
    void test_logfile_PARANOID(void)
    {
//      std::cout << "test_logfile_PARANOID() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::string logfile("rtcPARANOID.log");

      std::filebuf of;
      of.open(logfile.c_str(), std::ios::out);
      if (!of.is_open())
        {
          std::cerr << "Error: cannot open logfile: "
                    << logfile << std::endl;
        }
      logger.addStream(&of, true);

      RTC::Logger rtclog(&logger);
      rtclog.setName("Test");
      rtclog.setDateFormat("%b %d %H:%M:%S");
      rtclog.setLevel("PARANOID");

      // 汎用ログ出力マクロ、各種ログ出力マクロで正しくファイル出力されるか？
      RTC_LOG(    ::RTC::Logger::RTL_PARANOID,("RTL_PARANOID tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_PARANOID, "RTL_PARANOID tests str");
      RTC_PARANOID(   ("Macro RTL_PARANOID tests %s","fmt"));
      RTC_PARANOID_STR("Macro RTL_PARANOID tests str");

      RTC_LOG(    ::RTC::Logger::RTL_VERBOSE,("RTL_VERBOSE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_VERBOSE, "RTL_VERBOSE tests str");
      RTC_VERBOSE(   ("Macro RTL_VERBOSE tests %s","fmt"));
      RTC_VERBOSE_STR("Macro RTL_VERBOSE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_TRACE,("RTL_TRACE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_TRACE, "RTL_TRACE tests str");
      RTC_TRACE(   ("Macro RTL_TRACE tests %s","fmt"));
      RTC_TRACE_STR("Macro RTL_TRACE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_DEBUG,("RTL_DEBUG tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_DEBUG, "RTL_DEBUG tests str");
      RTC_DEBUG(   ("Macro RTL_DEBUG tests %s","fmt"));
      RTC_DEBUG_STR("Macro RTL_DEBUG tests str");

      RTC_LOG(    ::RTC::Logger::RTL_INFO,("RTL_INFO tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_INFO, "RTL_INFO tests str");
      RTC_INFO(   ("Macro RTL_INFO tests %s","fmt"));
      RTC_INFO_STR("Macro RTL_INFO tests str");

      RTC_LOG(    ::RTC::Logger::RTL_WARN,("RTL_WARN tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_WARN, "RTL_WARN tests str");
      RTC_WARN(   ("Macro RTL_WARN tests %s","fmt"));
      RTC_WARN_STR("Macro RTL_WARN tests str");

      RTC_LOG(    ::RTC::Logger::RTL_ERROR,("RTL_ERROR tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_ERROR, "RTL_ERROR tests str");
      RTC_ERROR(   ("Macro RTL_ERROR tests %s","fmt"));
      RTC_ERROR_STR("Macro RTL_ERROR tests str");

      RTC_LOG(    ::RTC::Logger::RTL_FATAL,("RTL_FATAL tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_FATAL, "RTL_FATAL tests str");
      RTC_FATAL(   ("Macro RTL_FATAL tests %s","fmt"));
      RTC_FATAL_STR("Macro RTL_FATAL tests str");

      RTC_LOG(    ::RTC::Logger::RTL_SILENT,("RTL_SILENT tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_SILENT, "RTL_SILENT tests str");

      of.close();

      // ファイル出力があるか？
      std::string rstr;
      std::ifstream ifs(logfile.c_str());
      ifs >> rstr;
      CPPUNIT_ASSERT(rstr.size() > 0);
      ifs >> rstr;
      ifs >> rstr;
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("PARANOID:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("Test:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("RTL_PARANOID"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("tests"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("fmt"), rstr);

//      std::cout << "test_logfile_PARANOID() OUT" << std::endl;
    }

    /*!
     * @brief logfile出力のテスト
     * 
     * - ログレベルを VERBOSE にした場合のファイル出力が正しく行われるか？
     */
    void test_logfile_VERBOSE(void)
    {
//      std::cout << "test_logfile_VERBOSE() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::string logfile("rtcVERBOSE.log");

      std::filebuf of;
      of.open(logfile.c_str(), std::ios::out);
      if (!of.is_open())
        {
          std::cerr << "Error: cannot open logfile: "
                    << logfile << std::endl;
        }
      logger.addStream(&of, true);

      RTC::Logger rtclog(&logger);
      rtclog.setName("Test");
      rtclog.setDateFormat("%b %d %H:%M:%S");
      rtclog.setLevel("VERBOSE");

      // 汎用ログ出力マクロ、各種ログ出力マクロで正しくファイル出力されるか？
      RTC_LOG(    ::RTC::Logger::RTL_PARANOID,("RTL_PARANOID tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_PARANOID, "RTL_PARANOID tests str");
      RTC_PARANOID(   ("Macro RTL_PARANOID tests %s","fmt"));
      RTC_PARANOID_STR("Macro RTL_PARANOID tests str");

      RTC_LOG(    ::RTC::Logger::RTL_VERBOSE,("RTL_VERBOSE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_VERBOSE, "RTL_VERBOSE tests str");
      RTC_VERBOSE(   ("Macro RTL_VERBOSE tests %s","fmt"));
      RTC_VERBOSE_STR("Macro RTL_VERBOSE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_TRACE,("RTL_TRACE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_TRACE, "RTL_TRACE tests str");
      RTC_TRACE(   ("Macro RTL_TRACE tests %s","fmt"));
      RTC_TRACE_STR("Macro RTL_TRACE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_DEBUG,("RTL_DEBUG tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_DEBUG, "RTL_DEBUG tests str");
      RTC_DEBUG(   ("Macro RTL_DEBUG tests %s","fmt"));
      RTC_DEBUG_STR("Macro RTL_DEBUG tests str");

      RTC_LOG(    ::RTC::Logger::RTL_INFO,("RTL_INFO tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_INFO, "RTL_INFO tests str");
      RTC_INFO(   ("Macro RTL_INFO tests %s","fmt"));
      RTC_INFO_STR("Macro RTL_INFO tests str");

      RTC_LOG(    ::RTC::Logger::RTL_WARN,("RTL_WARN tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_WARN, "RTL_WARN tests str");
      RTC_WARN(   ("Macro RTL_WARN tests %s","fmt"));
      RTC_WARN_STR("Macro RTL_WARN tests str");

      RTC_LOG(    ::RTC::Logger::RTL_ERROR,("RTL_ERROR tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_ERROR, "RTL_ERROR tests str");
      RTC_ERROR(   ("Macro RTL_ERROR tests %s","fmt"));
      RTC_ERROR_STR("Macro RTL_ERROR tests str");

      RTC_LOG(    ::RTC::Logger::RTL_FATAL,("RTL_FATAL tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_FATAL, "RTL_FATAL tests str");
      RTC_FATAL(   ("Macro RTL_FATAL tests %s","fmt"));
      RTC_FATAL_STR("Macro RTL_FATAL tests str");

      RTC_LOG(    ::RTC::Logger::RTL_SILENT,("RTL_SILENT tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_SILENT, "RTL_SILENT tests str");

      of.close();

      // ファイル出力があるか？
      std::string rstr;
      std::ifstream ifs(logfile.c_str());
      ifs >> rstr;
      CPPUNIT_ASSERT(rstr.size() > 0);
      ifs >> rstr;
      ifs >> rstr;
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("VERBOSE:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("Test:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("RTL_VERBOSE"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("tests"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("fmt"), rstr);

//      std::cout << "test_logfile_VERBOSE() OUT" << std::endl;
    }

    /*!
     * @brief logfile出力のテスト
     * 
     * - ログレベルを TRACE にした場合のファイル出力が正しく行われるか？
     */
    void test_logfile_TRACE(void)
    {
//      std::cout << "test_logfile_TRACE() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::string logfile("rtcTRACE.log");

      std::filebuf of;
      of.open(logfile.c_str(), std::ios::out);
      if (!of.is_open())
        {
          std::cerr << "Error: cannot open logfile: "
                    << logfile << std::endl;
        }
      logger.addStream(&of, true);

      RTC::Logger rtclog(&logger);
      rtclog.setName("Test");
      rtclog.setDateFormat("%b %d %H:%M:%S");
      rtclog.setLevel("TRACE");

      // 汎用ログ出力マクロ、各種ログ出力マクロで正しくファイル出力されるか？
      RTC_LOG(    ::RTC::Logger::RTL_PARANOID,("RTL_PARANOID tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_PARANOID, "RTL_PARANOID tests str");
      RTC_PARANOID(   ("Macro RTL_PARANOID tests %s","fmt"));
      RTC_PARANOID_STR("Macro RTL_PARANOID tests str");

      RTC_LOG(    ::RTC::Logger::RTL_VERBOSE,("RTL_VERBOSE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_VERBOSE, "RTL_VERBOSE tests str");
      RTC_VERBOSE(   ("Macro RTL_VERBOSE tests %s","fmt"));
      RTC_VERBOSE_STR("Macro RTL_VERBOSE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_TRACE,("RTL_TRACE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_TRACE, "RTL_TRACE tests str");
      RTC_TRACE(   ("Macro RTL_TRACE tests %s","fmt"));
      RTC_TRACE_STR("Macro RTL_TRACE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_DEBUG,("RTL_DEBUG tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_DEBUG, "RTL_DEBUG tests str");
      RTC_DEBUG(   ("Macro RTL_DEBUG tests %s","fmt"));
      RTC_DEBUG_STR("Macro RTL_DEBUG tests str");

      RTC_LOG(    ::RTC::Logger::RTL_INFO,("RTL_INFO tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_INFO, "RTL_INFO tests str");
      RTC_INFO(   ("Macro RTL_INFO tests %s","fmt"));
      RTC_INFO_STR("Macro RTL_INFO tests str");

      RTC_LOG(    ::RTC::Logger::RTL_WARN,("RTL_WARN tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_WARN, "RTL_WARN tests str");
      RTC_WARN(   ("Macro RTL_WARN tests %s","fmt"));
      RTC_WARN_STR("Macro RTL_WARN tests str");

      RTC_LOG(    ::RTC::Logger::RTL_ERROR,("RTL_ERROR tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_ERROR, "RTL_ERROR tests str");
      RTC_ERROR(   ("Macro RTL_ERROR tests %s","fmt"));
      RTC_ERROR_STR("Macro RTL_ERROR tests str");

      RTC_LOG(    ::RTC::Logger::RTL_FATAL,("RTL_FATAL tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_FATAL, "RTL_FATAL tests str");
      RTC_FATAL(   ("Macro RTL_FATAL tests %s","fmt"));
      RTC_FATAL_STR("Macro RTL_FATAL tests str");

      RTC_LOG(    ::RTC::Logger::RTL_SILENT,("RTL_SILENT tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_SILENT, "RTL_SILENT tests str");

      of.close();

      // ファイル出力があるか？
      std::string rstr;
      std::ifstream ifs(logfile.c_str());
      ifs >> rstr;
      CPPUNIT_ASSERT(rstr.size() > 0);
      ifs >> rstr;
      ifs >> rstr;
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("TRACE:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("Test:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("RTL_TRACE"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("tests"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("fmt"), rstr);

//      std::cout << "test_logfile_TRACE() OUT" << std::endl;
    }

    /*!
     * @brief logfile出力のテスト
     * 
     * - ログレベルを DEBUG にした場合のファイル出力が正しく行われるか？
     */
    void test_logfile_DEBUG(void)
    {
//      std::cout << "test_logfile_DEBUG() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::string logfile("rtcDEBUG.log");

      std::filebuf of;
      of.open(logfile.c_str(), std::ios::out);
      if (!of.is_open())
        {
          std::cerr << "Error: cannot open logfile: "
                    << logfile << std::endl;
        }
      logger.addStream(&of, true);

      RTC::Logger rtclog(&logger);
      rtclog.setName("Test");
      rtclog.setDateFormat("%b %d %H:%M:%S");
      rtclog.setLevel("DEBUG");

      // 汎用ログ出力マクロ、各種ログ出力マクロで正しくファイル出力されるか？
      RTC_LOG(    ::RTC::Logger::RTL_PARANOID,("RTL_PARANOID tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_PARANOID, "RTL_PARANOID tests str");
      RTC_PARANOID(   ("Macro RTL_PARANOID tests %s","fmt"));
      RTC_PARANOID_STR("Macro RTL_PARANOID tests str");

      RTC_LOG(    ::RTC::Logger::RTL_VERBOSE,("RTL_VERBOSE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_VERBOSE, "RTL_VERBOSE tests str");
      RTC_VERBOSE(   ("Macro RTL_VERBOSE tests %s","fmt"));
      RTC_VERBOSE_STR("Macro RTL_VERBOSE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_TRACE,("RTL_TRACE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_TRACE, "RTL_TRACE tests str");
      RTC_TRACE(   ("Macro RTL_TRACE tests %s","fmt"));
      RTC_TRACE_STR("Macro RTL_TRACE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_DEBUG,("RTL_DEBUG tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_DEBUG, "RTL_DEBUG tests str");
      RTC_DEBUG(   ("Macro RTL_DEBUG tests %s","fmt"));
      RTC_DEBUG_STR("Macro RTL_DEBUG tests str");

      RTC_LOG(    ::RTC::Logger::RTL_INFO,("RTL_INFO tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_INFO, "RTL_INFO tests str");
      RTC_INFO(   ("Macro RTL_INFO tests %s","fmt"));
      RTC_INFO_STR("Macro RTL_INFO tests str");

      RTC_LOG(    ::RTC::Logger::RTL_WARN,("RTL_WARN tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_WARN, "RTL_WARN tests str");
      RTC_WARN(   ("Macro RTL_WARN tests %s","fmt"));
      RTC_WARN_STR("Macro RTL_WARN tests str");

      RTC_LOG(    ::RTC::Logger::RTL_ERROR,("RTL_ERROR tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_ERROR, "RTL_ERROR tests str");
      RTC_ERROR(   ("Macro RTL_ERROR tests %s","fmt"));
      RTC_ERROR_STR("Macro RTL_ERROR tests str");

      RTC_LOG(    ::RTC::Logger::RTL_FATAL,("RTL_FATAL tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_FATAL, "RTL_FATAL tests str");
      RTC_FATAL(   ("Macro RTL_FATAL tests %s","fmt"));
      RTC_FATAL_STR("Macro RTL_FATAL tests str");

      RTC_LOG(    ::RTC::Logger::RTL_SILENT,("RTL_SILENT tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_SILENT, "RTL_SILENT tests str");

      of.close();

      // ファイル出力があるか？
      std::string rstr;
      std::ifstream ifs(logfile.c_str());
      ifs >> rstr;
      CPPUNIT_ASSERT(rstr.size() > 0);
      ifs >> rstr;
      ifs >> rstr;
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("DEBUG:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("Test:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("RTL_DEBUG"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("tests"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("fmt"), rstr);

//      std::cout << "test_logfile_DEBUG() OUT" << std::endl;
    }

    /*!
     * @brief logfile出力のテスト
     * 
     * - ログレベルを INFO にした場合のファイル出力が正しく行われるか？
     */
    void test_logfile_INFO(void)
    {
//      std::cout << "test_logfile_INFO() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::string logfile("rtcINFO.log");

      std::filebuf of;
      of.open(logfile.c_str(), std::ios::out);
      if (!of.is_open())
        {
          std::cerr << "Error: cannot open logfile: "
                    << logfile << std::endl;
        }
      logger.addStream(&of, true);

      RTC::Logger rtclog(&logger);
      rtclog.setName("Test");
      rtclog.setDateFormat("%b %d %H:%M:%S");
      rtclog.setLevel("INFO");

      // 汎用ログ出力マクロ、各種ログ出力マクロで正しくファイル出力されるか？
      RTC_LOG(    ::RTC::Logger::RTL_PARANOID,("RTL_PARANOID tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_PARANOID, "RTL_PARANOID tests str");
      RTC_PARANOID(   ("Macro RTL_PARANOID tests %s","fmt"));
      RTC_PARANOID_STR("Macro RTL_PARANOID tests str");

      RTC_LOG(    ::RTC::Logger::RTL_VERBOSE,("RTL_VERBOSE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_VERBOSE, "RTL_VERBOSE tests str");
      RTC_VERBOSE(   ("Macro RTL_VERBOSE tests %s","fmt"));
      RTC_VERBOSE_STR("Macro RTL_VERBOSE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_TRACE,("RTL_TRACE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_TRACE, "RTL_TRACE tests str");
      RTC_TRACE(   ("Macro RTL_TRACE tests %s","fmt"));
      RTC_TRACE_STR("Macro RTL_TRACE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_DEBUG,("RTL_DEBUG tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_DEBUG, "RTL_DEBUG tests str");
      RTC_DEBUG(   ("Macro RTL_DEBUG tests %s","fmt"));
      RTC_DEBUG_STR("Macro RTL_DEBUG tests str");

      RTC_LOG(    ::RTC::Logger::RTL_INFO,("RTL_INFO tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_INFO, "RTL_INFO tests str");
      RTC_INFO(   ("Macro RTL_INFO tests %s","fmt"));
      RTC_INFO_STR("Macro RTL_INFO tests str");

      RTC_LOG(    ::RTC::Logger::RTL_WARN,("RTL_WARN tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_WARN, "RTL_WARN tests str");
      RTC_WARN(   ("Macro RTL_WARN tests %s","fmt"));
      RTC_WARN_STR("Macro RTL_WARN tests str");

      RTC_LOG(    ::RTC::Logger::RTL_ERROR,("RTL_ERROR tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_ERROR, "RTL_ERROR tests str");
      RTC_ERROR(   ("Macro RTL_ERROR tests %s","fmt"));
      RTC_ERROR_STR("Macro RTL_ERROR tests str");

      RTC_LOG(    ::RTC::Logger::RTL_FATAL,("RTL_FATAL tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_FATAL, "RTL_FATAL tests str");
      RTC_FATAL(   ("Macro RTL_FATAL tests %s","fmt"));
      RTC_FATAL_STR("Macro RTL_FATAL tests str");

      RTC_LOG(    ::RTC::Logger::RTL_SILENT,("RTL_SILENT tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_SILENT, "RTL_SILENT tests str");

      of.close();

      // ファイル出力があるか？
      std::string rstr;
      std::ifstream ifs(logfile.c_str());
      ifs >> rstr;
      CPPUNIT_ASSERT(rstr.size() > 0);
      ifs >> rstr;
      ifs >> rstr;
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("INFO:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("Test:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("RTL_INFO"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("tests"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("fmt"), rstr);

//      std::cout << "test_logfile_INFO() OUT" << std::endl;
    }

    /*!
     * @brief logfile出力のテスト
     * 
     * - ログレベルを WARNING にした場合のファイル出力が正しく行われるか？
     */
    void test_logfile_WARNING(void)
    {
//      std::cout << "test_logfile_WARNING() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::string logfile("rtcWARNING.log");

      std::filebuf of;
      of.open(logfile.c_str(), std::ios::out);
      if (!of.is_open())
        {
          std::cerr << "Error: cannot open logfile: "
                    << logfile << std::endl;
        }
      logger.addStream(&of, true);

      RTC::Logger rtclog(&logger);
      rtclog.setName("Test");
      rtclog.setDateFormat("%b %d %H:%M:%S");
      rtclog.setLevel("WARN");

      // 汎用ログ出力マクロ、各種ログ出力マクロで正しくファイル出力されるか？
      RTC_LOG(    ::RTC::Logger::RTL_PARANOID,("RTL_PARANOID tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_PARANOID, "RTL_PARANOID tests str");
      RTC_PARANOID(   ("Macro RTL_PARANOID tests %s","fmt"));
      RTC_PARANOID_STR("Macro RTL_PARANOID tests str");

      RTC_LOG(    ::RTC::Logger::RTL_VERBOSE,("RTL_VERBOSE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_VERBOSE, "RTL_VERBOSE tests str");
      RTC_VERBOSE(   ("Macro RTL_VERBOSE tests %s","fmt"));
      RTC_VERBOSE_STR("Macro RTL_VERBOSE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_TRACE,("RTL_TRACE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_TRACE, "RTL_TRACE tests str");
      RTC_TRACE(   ("Macro RTL_TRACE tests %s","fmt"));
      RTC_TRACE_STR("Macro RTL_TRACE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_DEBUG,("RTL_DEBUG tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_DEBUG, "RTL_DEBUG tests str");
      RTC_DEBUG(   ("Macro RTL_DEBUG tests %s","fmt"));
      RTC_DEBUG_STR("Macro RTL_DEBUG tests str");

      RTC_LOG(    ::RTC::Logger::RTL_INFO,("RTL_INFO tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_INFO, "RTL_INFO tests str");
      RTC_INFO(   ("Macro RTL_INFO tests %s","fmt"));
      RTC_INFO_STR("Macro RTL_INFO tests str");

      RTC_LOG(    ::RTC::Logger::RTL_WARN,("RTL_WARN tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_WARN, "RTL_WARN tests str");
      RTC_WARN(   ("Macro RTL_WARN tests %s","fmt"));
      RTC_WARN_STR("Macro RTL_WARN tests str");

      RTC_LOG(    ::RTC::Logger::RTL_ERROR,("RTL_ERROR tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_ERROR, "RTL_ERROR tests str");
      RTC_ERROR(   ("Macro RTL_ERROR tests %s","fmt"));
      RTC_ERROR_STR("Macro RTL_ERROR tests str");

      RTC_LOG(    ::RTC::Logger::RTL_FATAL,("RTL_FATAL tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_FATAL, "RTL_FATAL tests str");
      RTC_FATAL(   ("Macro RTL_FATAL tests %s","fmt"));
      RTC_FATAL_STR("Macro RTL_FATAL tests str");

      RTC_LOG(    ::RTC::Logger::RTL_SILENT,("RTL_SILENT tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_SILENT, "RTL_SILENT tests str");

      of.close();

      // ファイル出力があるか？
      std::string rstr;
      std::ifstream ifs(logfile.c_str());
      ifs >> rstr;
      CPPUNIT_ASSERT(rstr.size() > 0);
      ifs >> rstr;
      ifs >> rstr;
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("WARNING:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("Test:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("RTL_WARN"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("tests"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("fmt"), rstr);

//      std::cout << "test_logfile_WARNING() OUT" << std::endl;
    }

    /*!
     * @brief logfile出力のテスト
     * 
     * - ログレベルを ERROR にした場合のファイル出力が正しく行われるか？
     */
    void test_logfile_ERROR(void)
    {
//      std::cout << "test_logfile_ERROR() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::string logfile("rtcERROR.log");

      std::filebuf of;
      of.open(logfile.c_str(), std::ios::out);
      if (!of.is_open())
        {
          std::cerr << "Error: cannot open logfile: "
                    << logfile << std::endl;
        }
      logger.addStream(&of, true);

      RTC::Logger rtclog(&logger);
      rtclog.setName("Test");
      rtclog.setDateFormat("%b %d %H:%M:%S");
      rtclog.setLevel("ERROR");

      // 汎用ログ出力マクロ、各種ログ出力マクロで正しくファイル出力されるか？
      RTC_LOG(    ::RTC::Logger::RTL_PARANOID,("RTL_PARANOID tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_PARANOID, "RTL_PARANOID tests str");
      RTC_PARANOID(   ("Macro RTL_PARANOID tests %s","fmt"));
      RTC_PARANOID_STR("Macro RTL_PARANOID tests str");

      RTC_LOG(    ::RTC::Logger::RTL_VERBOSE,("RTL_VERBOSE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_VERBOSE, "RTL_VERBOSE tests str");
      RTC_VERBOSE(   ("Macro RTL_VERBOSE tests %s","fmt"));
      RTC_VERBOSE_STR("Macro RTL_VERBOSE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_TRACE,("RTL_TRACE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_TRACE, "RTL_TRACE tests str");
      RTC_TRACE(   ("Macro RTL_TRACE tests %s","fmt"));
      RTC_TRACE_STR("Macro RTL_TRACE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_DEBUG,("RTL_DEBUG tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_DEBUG, "RTL_DEBUG tests str");
      RTC_DEBUG(   ("Macro RTL_DEBUG tests %s","fmt"));
      RTC_DEBUG_STR("Macro RTL_DEBUG tests str");

      RTC_LOG(    ::RTC::Logger::RTL_INFO,("RTL_INFO tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_INFO, "RTL_INFO tests str");
      RTC_INFO(   ("Macro RTL_INFO tests %s","fmt"));
      RTC_INFO_STR("Macro RTL_INFO tests str");

      RTC_LOG(    ::RTC::Logger::RTL_WARN,("RTL_WARN tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_WARN, "RTL_WARN tests str");
      RTC_WARN(   ("Macro RTL_WARN tests %s","fmt"));
      RTC_WARN_STR("Macro RTL_WARN tests str");

      RTC_LOG(    ::RTC::Logger::RTL_ERROR,("RTL_ERROR tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_ERROR, "RTL_ERROR tests str");
      RTC_ERROR(   ("Macro RTL_ERROR tests %s","fmt"));
      RTC_ERROR_STR("Macro RTL_ERROR tests str");

      RTC_LOG(    ::RTC::Logger::RTL_FATAL,("RTL_FATAL tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_FATAL, "RTL_FATAL tests str");
      RTC_FATAL(   ("Macro RTL_FATAL tests %s","fmt"));
      RTC_FATAL_STR("Macro RTL_FATAL tests str");

      RTC_LOG(    ::RTC::Logger::RTL_SILENT,("RTL_SILENT tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_SILENT, "RTL_SILENT tests str");

      of.close();

      // ファイル出力があるか？
      std::string rstr;
      std::ifstream ifs(logfile.c_str());
      ifs >> rstr;
      CPPUNIT_ASSERT(rstr.size() > 0);
      ifs >> rstr;
      ifs >> rstr;
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("ERROR:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("Test:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("RTL_ERROR"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("tests"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("fmt"), rstr);

//      std::cout << "test_logfile_ERROR() OUT" << std::endl;
    }

    /*!
     * @brief logfile出力のテスト
     * 
     * - ログレベルを FATAL にした場合のファイル出力が正しく行われるか？
     */
    void test_logfile_FATAL(void)
    {
//      std::cout << "test_logfile_FATAL() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::string logfile("rtcFATAL.log");

      std::filebuf of;
      of.open(logfile.c_str(), std::ios::out);
      if (!of.is_open())
        {
          std::cerr << "Error: cannot open logfile: "
                    << logfile << std::endl;
        }
      logger.addStream(&of, true);

      RTC::Logger rtclog(&logger);
      rtclog.setName("Test");
      rtclog.setDateFormat("%b %d %H:%M:%S");
      rtclog.setLevel("FATAL");

      // 汎用ログ出力マクロ、各種ログ出力マクロで正しくファイル出力されるか？
      RTC_LOG(    ::RTC::Logger::RTL_PARANOID,("RTL_PARANOID tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_PARANOID, "RTL_PARANOID tests str");
      RTC_PARANOID(   ("Macro RTL_PARANOID tests %s","fmt"));
      RTC_PARANOID_STR("Macro RTL_PARANOID tests str");

      RTC_LOG(    ::RTC::Logger::RTL_VERBOSE,("RTL_VERBOSE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_VERBOSE, "RTL_VERBOSE tests str");
      RTC_VERBOSE(   ("Macro RTL_VERBOSE tests %s","fmt"));
      RTC_VERBOSE_STR("Macro RTL_VERBOSE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_TRACE,("RTL_TRACE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_TRACE, "RTL_TRACE tests str");
      RTC_TRACE(   ("Macro RTL_TRACE tests %s","fmt"));
      RTC_TRACE_STR("Macro RTL_TRACE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_DEBUG,("RTL_DEBUG tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_DEBUG, "RTL_DEBUG tests str");
      RTC_DEBUG(   ("Macro RTL_DEBUG tests %s","fmt"));
      RTC_DEBUG_STR("Macro RTL_DEBUG tests str");

      RTC_LOG(    ::RTC::Logger::RTL_INFO,("RTL_INFO tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_INFO, "RTL_INFO tests str");
      RTC_INFO(   ("Macro RTL_INFO tests %s","fmt"));
      RTC_INFO_STR("Macro RTL_INFO tests str");

      RTC_LOG(    ::RTC::Logger::RTL_WARN,("RTL_WARN tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_WARN, "RTL_WARN tests str");
      RTC_WARN(   ("Macro RTL_WARN tests %s","fmt"));
      RTC_WARN_STR("Macro RTL_WARN tests str");

      RTC_LOG(    ::RTC::Logger::RTL_ERROR,("RTL_ERROR tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_ERROR, "RTL_ERROR tests str");
      RTC_ERROR(   ("Macro RTL_ERROR tests %s","fmt"));
      RTC_ERROR_STR("Macro RTL_ERROR tests str");

      RTC_LOG(    ::RTC::Logger::RTL_FATAL,("RTL_FATAL tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_FATAL, "RTL_FATAL tests str");
      RTC_FATAL(   ("Macro RTL_FATAL tests %s","fmt"));
      RTC_FATAL_STR("Macro RTL_FATAL tests str");

      RTC_LOG(    ::RTC::Logger::RTL_SILENT,("RTL_SILENT tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_SILENT, "RTL_SILENT tests str");

      of.close();

      // ファイル出力があるか？
      std::string rstr;
      std::ifstream ifs(logfile.c_str());
      ifs >> rstr;
      CPPUNIT_ASSERT(rstr.size() > 0);
      ifs >> rstr;
      ifs >> rstr;
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("FATAL:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("Test:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("RTL_FATAL"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("tests"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("fmt"), rstr);

//      std::cout << "test_logfile_FATAL() OUT" << std::endl;
    }

    /*!
     * @brief logfile出力のテスト
     * 
     * - ログレベルを SILENT にした場合のファイル出力が正しく行われるか？
     */
    void test_logfile_SILENT(void)
    {
//      std::cout << "test_logfile_SILENT() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::string logfile("rtcSILENT.log");

      std::filebuf of;
      of.open(logfile.c_str(), std::ios::out);
      if (!of.is_open())
        {
          std::cerr << "Error: cannot open logfile: "
                    << logfile << std::endl;
        }
      logger.addStream(&of, true);

      RTC::Logger rtclog(&logger);
      rtclog.setName("Test");
      rtclog.setDateFormat("%b %d %H:%M:%S");
      rtclog.setLevel("SILENT");

      // 汎用ログ出力マクロ、各種ログ出力マクロで正しくファイル出力されるか？
      RTC_LOG(    ::RTC::Logger::RTL_PARANOID,("RTL_PARANOID tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_PARANOID, "RTL_PARANOID tests str");
      RTC_PARANOID(   ("Macro RTL_PARANOID tests %s","fmt"));
      RTC_PARANOID_STR("Macro RTL_PARANOID tests str");

      RTC_LOG(    ::RTC::Logger::RTL_VERBOSE,("RTL_VERBOSE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_VERBOSE, "RTL_VERBOSE tests str");
      RTC_VERBOSE(   ("Macro RTL_VERBOSE tests %s","fmt"));
      RTC_VERBOSE_STR("Macro RTL_VERBOSE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_TRACE,("RTL_TRACE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_TRACE, "RTL_TRACE tests str");
      RTC_TRACE(   ("Macro RTL_TRACE tests %s","fmt"));
      RTC_TRACE_STR("Macro RTL_TRACE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_DEBUG,("RTL_DEBUG tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_DEBUG, "RTL_DEBUG tests str");
      RTC_DEBUG(   ("Macro RTL_DEBUG tests %s","fmt"));
      RTC_DEBUG_STR("Macro RTL_DEBUG tests str");

      RTC_LOG(    ::RTC::Logger::RTL_INFO,("RTL_INFO tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_INFO, "RTL_INFO tests str");
      RTC_INFO(   ("Macro RTL_INFO tests %s","fmt"));
      RTC_INFO_STR("Macro RTL_INFO tests str");

      RTC_LOG(    ::RTC::Logger::RTL_WARN,("RTL_WARN tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_WARN, "RTL_WARN tests str");
      RTC_WARN(   ("Macro RTL_WARN tests %s","fmt"));
      RTC_WARN_STR("Macro RTL_WARN tests str");

      RTC_LOG(    ::RTC::Logger::RTL_ERROR,("RTL_ERROR tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_ERROR, "RTL_ERROR tests str");
      RTC_ERROR(   ("Macro RTL_ERROR tests %s","fmt"));
      RTC_ERROR_STR("Macro RTL_ERROR tests str");

      RTC_LOG(    ::RTC::Logger::RTL_FATAL,("RTL_FATAL tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_FATAL, "RTL_FATAL tests str");
      RTC_FATAL(   ("Macro RTL_FATAL tests %s","fmt"));
      RTC_FATAL_STR("Macro RTL_FATAL tests str");

      RTC_LOG(    ::RTC::Logger::RTL_SILENT,("RTL_SILENT tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_SILENT, "RTL_SILENT tests str");

      of.close();

      // ファイル出力があるか？
      std::string rstr;
      std::ifstream ifs(logfile.c_str());
      ifs >> rstr;
      CPPUNIT_ASSERT(rstr.size() > 0);
      ifs >> rstr;
      ifs >> rstr;
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("SILENT:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("Test:"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("RTL_SILENT"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("tests"), rstr);
      ifs >> rstr;
      CPPUNIT_ASSERT_EQUAL(std::string("fmt"), rstr);

//      std::cout << "test_logfile_SILENT() OUT" << std::endl;
    }

    /*!
     * @brief コンストラクタログレベルのテスト
     * 
     * - コンストラクタ（name)の場合、Managerの設定ログレベル(INFO)で動作するか？
     */
    void test_constract_name(void)
    {
//      std::cout << "test_constract_name() IN" << std::endl;
      RTC::Manager* m_mgr;
      m_mgr = RTC::Manager::init(0, NULL);
      CPPUNIT_ASSERT(m_mgr != NULL);

      RTC::Logger rtclog("TestName");
      std::string log_level = m_mgr->getLogLevel();
      CPPUNIT_ASSERT_EQUAL(std::string("INFO"), log_level);

      coil::Properties m_config = m_mgr->getConfig();
      std::vector<std::string> logouts = coil::split(m_config["logger.file_name"], ",");

      // 汎用ログ出力マクロ、各種ログ出力マクロでファイル出力
      RTC_LOG(    ::RTC::Logger::RTL_PARANOID,("RTL_PARANOID tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_PARANOID, "RTL_PARANOID tests str");
      RTC_PARANOID(   ("Macro RTL_PARANOID tests %s","fmt"));
      RTC_PARANOID_STR("Macro RTL_PARANOID tests str");

      RTC_LOG(    ::RTC::Logger::RTL_VERBOSE,("RTL_VERBOSE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_VERBOSE, "RTL_VERBOSE tests str");
      RTC_VERBOSE(   ("Macro RTL_VERBOSE tests %s","fmt"));
      RTC_VERBOSE_STR("Macro RTL_VERBOSE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_TRACE,("RTL_TRACE tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_TRACE, "RTL_TRACE tests str");
      RTC_TRACE(   ("Macro RTL_TRACE tests %s","fmt"));
      RTC_TRACE_STR("Macro RTL_TRACE tests str");

      RTC_LOG(    ::RTC::Logger::RTL_DEBUG,("RTL_DEBUG tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_DEBUG, "RTL_DEBUG tests str");
      RTC_DEBUG(   ("Macro RTL_DEBUG tests %s","fmt"));
      RTC_DEBUG_STR("Macro RTL_DEBUG tests str");

      RTC_LOG(    ::RTC::Logger::RTL_INFO,("RTL_INFO tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_INFO, "RTL_INFO tests str");
      RTC_INFO(   ("Macro RTL_INFO tests %s","fmt"));
      RTC_INFO_STR("Macro RTL_INFO tests str");

      RTC_LOG(    ::RTC::Logger::RTL_WARN,("RTL_WARN tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_WARN, "RTL_WARN tests str");
      RTC_WARN(   ("Macro RTL_WARN tests %s","fmt"));
      RTC_WARN_STR("Macro RTL_WARN tests str");

      RTC_LOG(    ::RTC::Logger::RTL_ERROR,("RTL_ERROR tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_ERROR, "RTL_ERROR tests str");
      RTC_ERROR(   ("Macro RTL_ERROR tests %s","fmt"));
      RTC_ERROR_STR("Macro RTL_ERROR tests str");

      RTC_LOG(    ::RTC::Logger::RTL_FATAL,("RTL_FATAL tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_FATAL, "RTL_FATAL tests str");
      RTC_FATAL(   ("Macro RTL_FATAL tests %s","fmt"));
      RTC_FATAL_STR("Macro RTL_FATAL tests str");

      RTC_LOG(    ::RTC::Logger::RTL_SILENT,("RTL_SILENT tests %s","fmt"));
      RTC_LOG_STR(::RTC::Logger::RTL_SILENT, "RTL_SILENT tests str");

      m_mgr->terminate();

      // rtc*.log ファイルが作成され、４列目のログレベルが INFO以下か？
      // INFO WARNING ERROR FATAL SILENT だけが記録されているか？
      // Aug 03 14:03:09 INFO: manager: OpenRTM-aist-1.0.0
      // [0] 1  2        3     4        5
      std::string rstr;
      std::vector<std::string> vstr;
      bool bret;
      std::ifstream ifs(logouts[0].c_str());
      while(getline(ifs, rstr))
        {
          if(rstr.size() == 0) break;
          vstr = coil::split(rstr, " ");
          // ログレベル判定
          bret = false;
          if( (vstr[3] == "INFO:") || (vstr[3] == "WARNING:") || (vstr[3] == "ERROR:") ||
              (vstr[3] == "FATAL:") || (vstr[3] == "SILENT:") )
              bret = true;
          CPPUNIT_ASSERT(bret);

          // name判定
          bret = false;
          if( (vstr[4] == "manager:") || (vstr[4] == "TestName:") ||
              (vstr[4] == "NamingOnCorba:") || (vstr[4] == "NamingManager:") ||
              (vstr[4] == "ManagerServant:") )
              bret = true;
          CPPUNIT_ASSERT(bret);
        }
//      std::cout << "test_constract_name() OUT" << std::endl;
    }

    /*!
     * @brief ログ出力のデッドロックテスト
     * 
     * - RTC_LOG出力時、引数の関数内でログ出力がある場合にデッドロックしないか？
     */
    void test_deadlock(void)
    {
//      std::cout << "test_deadlock() IN" << std::endl;
      coil::LogStreamBuffer logger;
      std::stringstream s0;

      logger.addStream(s0.rdbuf());
      LoggerMock rtclog(&logger);
      rtclog.setName("Test");
      rtclog.setDateFormat("");
      rtclog.setLevel("TRACE");
      s0.str("");
      // ロックモード設定
      rtclog.enableLock();
      RTC_TRACE(("RTC_TRACE1 %s", rtclog.test_string().c_str()));

      std::string rstr;
      getline(s0, rstr);
      CPPUNIT_ASSERT_EQUAL(std::string(" TRACE: Test: RTC_TRACE2 test_string()"), rstr);
      getline(s0, rstr);
      CPPUNIT_ASSERT_EQUAL(std::string(" TRACE: Test: RTC_TRACE1 TestString"), rstr);

      // ロックモード解除
      rtclog.disableLock();
//      std::cout << "test_deadlock() OUT" << std::endl;
    }

  };
}; // namespace Tests

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Tests::SystemLoggerTests);

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
#endif // SystemLoggerTests_cpp
