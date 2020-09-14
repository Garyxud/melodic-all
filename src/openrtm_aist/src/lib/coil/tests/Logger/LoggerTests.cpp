// -*- C++ -*-
/*!
 * @file   LoggerTests.cpp
 * @brief  Logger test class
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

#ifndef Logger_cpp
#define Logger_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <string>
#include <map>
#include <iostream>
#include <cstdlib>

#include <coil/Logger.h>
#include <coil/Task.h>
#include <coil/TimeMeasure.h>
#include <coil/stringutil.h>
#include <coil/Time.h>
#include <coil/Properties.h>

#include <coil/Mutex.h>
#include <coil/Guard.h>

#ifdef __QNX__
using std::rand;
#endif

namespace coil 
{
};

/*!
 * @class LoggerTests class
 * @brief Logger test
 */
namespace Logger
{
class LogCreator
  : public coil::Task
{
public:
  LogCreator(const char* name, std::basic_streambuf<char>* streambuf)
    : m_name(name),
      m_out(streambuf)
      //      logger(name)
  {
  }

  virtual int svc()
  {
    coil::TimeMeasure tm;
    for (int i(0); i < 100; ++i)
      {
        double r(rand() / (double)RAND_MAX * 100.0);
        tm.tick();
        {
        coil::Guard<coil::Mutex> guard(m_mutex);
        m_out << coil::sprintf("%s %03d %6.2f", m_name.c_str(), i, r) << std::endl;
        }
        tm.tack();
        coil::usleep((int)r);
      }

#if 0
    double max, min, mean, stddev;
    tm.getStatistics(max, min, mean, stddev);
    std::cout << m_name << std::endl;
    printf("max   : %04.2f [us]\n", max * 1000000);
    printf("min   : %04.2f [us]\n", min * 1000000);
    printf("mean  : %04.2f [us]\n", mean * 1000000);
    printf("stddev: %04.2f [us]\n", stddev * 1000000);
#endif
    return 0;
  }

private:
  std::string m_name;
  std::basic_ostream<char> m_out;
//  coil::Mutex m_mutex;
public:
  static coil::Mutex m_mutex;
};
coil::Mutex LogCreator::m_mutex;

enum LogLevel
  {
    SILENT_LEVEL,
    INFO_LEVEL,
    ERROR_LEVEL,
    TRACE_LEVEL,
    PARANOID_LEVEL
  };

class LogOut
  : public coil::LogStream
{

  
public:
  LogOut(::coil::LogStream::streambuf_type* streambuf)
    : ::coil::LogStream(streambuf, /* "test", */
                        SILENT_LEVEL, PARANOID_LEVEL, SILENT_LEVEL)
  {
  }
  virtual ~LogOut(){}

  virtual std::string& prefix(std::string& prefix, int level)
  {
    const char* level_str[] =
      {
        "SILENT   ",
        "INFO     ",
        "ERROR    ",
        "TRACE    ",
        "PARANOID "
      };
    prefix = getFmtDate() + level_str[level];
    return prefix;
  }

  void setDateFormat(const char* format)
  {
    m_dateFormat = std::string(format);
  }
  void header(int level)
  {
    const char* level_str[] = {
        " SILENT  :",
        " INFO    :",
        " ERROR   :",
        " TRACE   :",
        " PARANOID:"
    };
    *this << getDate() + level_str[level] + m_name + ": ";
  }
  std::string getDate(void)
  {
    const int maxsize = 256;
    char buf[maxsize];

    time_t timer;
    struct tm* date;
      
    timer = time(NULL);
    date = localtime(&timer);
    strftime(buf, sizeof(buf), m_dateFormat.c_str(), date);

    return std::string(buf);
  }
  void setName(const char* name)
  {
    m_name = name;
  }
protected:
  std::string getFmtDate()
  {
    const int maxsize = 256;
    char buf[maxsize];

    time_t timer;
    struct tm* date;
    
    timer = time(NULL);
    date = localtime(&timer);
    strftime(buf, maxsize, "%b %d %H:%M:%S ", date);
    
    return std::string(buf);
  }
  std::string m_name;
  std::string m_dateFormat;
};

class LogOut2
  : public coil::LogStream
{

  
public:
  LogOut2(::coil::LogStream::streambuf_type* streambuf)
    : ::coil::LogStream(streambuf, /* "test", */
                        SILENT_LEVEL, PARANOID_LEVEL, SILENT_LEVEL)
  {
  }
  virtual ~LogOut2(){}


protected:
};



#define RTC_LOG(LV, fmt)			           \
  if (m_out.isValid(LV))                                   \
    {                                                      \
      m_out.lock();                                        \
      m_out.level(LV) << ::coil::sprintf fmt << std::endl; \
      m_out.unlock();                                      \
    }

#define RTC_TRACE(fmt) RTC_LOG(TRACE_LEVEL, fmt)


class LogOutCreator
  : public coil::Task
{
public:
  LogOutCreator(const char* name, coil::LogStreamBuffer *streambuf)
    : m_name(name),
      m_out(streambuf)
      //      logger(name)
  {
      m_out.setName(m_name.c_str());
      m_out.setDateFormat("%b %d %H:%M:%S");
      m_out.setLevel(PARANOID_LEVEL);
      m_out.enableLock();
  }
  ~LogOutCreator()
  {
      m_out.disableLock();
  }

  virtual int svc()
  {
    coil::TimeMeasure tm;
    for (int i(0); i < 200; ++i)
      {
        double r(rand() / (double)RAND_MAX * 100.0);
        tm.tick();
        std::string str = coil::sprintf("svc() %03d %6.2f",  i, r);
        RTC_TRACE((str.c_str()));
        tm.tack();
        coil::usleep((int)r);
      }

#if 0
    double max, min, mean, stddev;
    tm.getStatistics(max, min, mean, stddev);
    std::cout << m_name << std::endl;
    printf("max   : %04.2f [us]\n", max * 1000000);
    printf("min   : %04.2f [us]\n", min * 1000000);
    printf("mean  : %04.2f [us]\n", mean * 1000000);
    printf("stddev: %04.2f [us]\n", stddev * 1000000);
#endif
    return 0;
  }

private:
  std::string m_name;
  LogOut m_out;
  int dummy0(int ic)
  {
    RTC_TRACE(("IN  dummy0"));
    RTC_TRACE(("    ic=%d",ic));
    RTC_TRACE(("OUT dummy0"));
    return ic;
  }

};


/*!
 * @class LoggerTests class
 * @brief Logger test
 */
  class LoggerTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(LoggerTests);

    CPPUNIT_TEST(test_log_streambuf);
    CPPUNIT_TEST(test_log_streambuf2);
    CPPUNIT_TEST(test_log_stream);
    CPPUNIT_TEST(test_log_stream2);
    CPPUNIT_TEST(test_log_stream_properties);
    CPPUNIT_TEST(test_log_stream3);

    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    LoggerTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~LoggerTests()
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
    void test_log_streambuf()
    {
      coil::LogStreamBuffer logger;

      std::stringstream s0;
      std::stringstream s1;
      std::stringstream s2;
      std::filebuf f;
      f.open("log.log", std::ios::out);

      logger.addStream(s0.rdbuf());
      logger.addStream(s1.rdbuf());
      logger.addStream(s2.rdbuf());
      logger.addStream(&f);

      LogCreator l0("log0", &logger);
      LogCreator l1("log1", &logger);
      LogCreator l2("log2", &logger);
      LogCreator l3("log3", &logger);

      l0.activate();
      l1.activate();
      l2.activate();
      l3.activate();

      l0.wait();
      l1.wait();
      l2.wait();
      l3.wait();

      std::ofstream f0("log0.log");
      std::ofstream f1("log1.log");
      std::ofstream f2("log2.log");
      f0 << s0.str() << std::endl;
      f1 << s1.str() << std::endl;
      f2 << s2.str() << std::endl;
      f0.close();
      f1.close();
      f2.close();
      CPPUNIT_ASSERT(s0.str() == s1.str());
      CPPUNIT_ASSERT(s1.str() == s2.str());


      std::string s;
      getline(s0, s);
      size_t len(s.size());

      while (getline(s0, s))
        {
          CPPUNIT_ASSERT(len == s.size());
        }
    }

    void test_log_streambuf2()
    {
      coil::LogStreamBuffer logger;
      std::stringstream s0;
//      logger.addStream(std::cout.rdbuf());
      logger.addStream(s0.rdbuf());

      std::basic_ostream<char> out(&logger);
      std::string str("::");
      int ic(2);
      out <<"Logger"<<str<<"test_log_streambuf"<<ic<<std::endl;
      std::ostringstream os,osm;
      os <<"Logger"<<str<<"test_log_streambuf"<<ic<<std::endl;
      osm <<"s0.str():"<<s0.str()<<" os.str():"<<"Logger"<<str<<"test_log_streambuf"<<ic<<std::endl;
      CPPUNIT_ASSERT_MESSAGE(osm.str(),s0.str() == os.str());
       
    }

    void test_log_stream()
    {
      coil::LogStreamBuffer logbuf;
      std::stringstream s0;
      logbuf.addStream(s0.rdbuf());
//      logbuf.addStream(std::cout.rdbuf());

      LogOut log(&logbuf);
      log.setLevel(PARANOID_LEVEL);
//      std::cout << std::endl;
      log.level(SILENT_LEVEL) << coil::sprintf("This is silent message.") << std::endl;
      log.level(INFO_LEVEL) << coil::sprintf("This is info message.") << std::endl;
      log.level(ERROR_LEVEL) << coil::sprintf("This is error message.") << std::endl;
      log.level(PARANOID_LEVEL) << coil::sprintf("This is paranoid message.") << std::endl;

//      std::cout << std::endl;
      log.setLevel(INFO_LEVEL);
      log.level(SILENT_LEVEL) << coil::sprintf("This is silent message.") << std::endl;
      log.level(INFO_LEVEL) << coil::sprintf("This is info message.") << std::endl;
      log.level(ERROR_LEVEL) << coil::sprintf("This is error message.") << std::endl;
      log.level(PARANOID_LEVEL) << coil::sprintf("This is paranoid message.") << std::endl;

//      std::cout << std::endl;
      log.setLevel(SILENT_LEVEL);
      log.level(SILENT_LEVEL) << coil::sprintf("This is silent message.") << std::endl;
      log.level(INFO_LEVEL) << coil::sprintf("This is info message.") << std::endl;
      log.level(ERROR_LEVEL) << coil::sprintf("This is error message.") << std::endl;
      log.level(PARANOID_LEVEL) << coil::sprintf("This is paranoid message.") << std::endl;

      std::ofstream f0("test_log_stream.log");
      f0 << s0.str() << std::endl;
      f0.close();


    }

    void test_log_stream2()
    {
      coil::LogStreamBuffer logger;

      std::stringstream s0;
      std::stringstream s1;
      std::stringstream s2;
      std::filebuf f;
      f.open("log.log", std::ios::out);

      logger.addStream(s0.rdbuf());
      logger.addStream(s1.rdbuf());
      logger.addStream(s2.rdbuf());
      logger.addStream(&f);

      LogOutCreator l0("test_log_stream_log0", &logger);
      LogOutCreator l1("test_log_stream_log1", &logger);
      LogOutCreator l2("test_log_stream_log2", &logger);
      LogOutCreator l3("test_log_stream_log3", &logger);

      l0.activate();
      l1.activate();
      l2.activate();
      l3.activate();

      l0.wait();
      l1.wait();
      l2.wait();
      l3.wait();

      std::ofstream f0("log4.log");
      std::ofstream f1("log5.log");
      std::ofstream f2("log6.log");
      f0 << s0.str() << std::endl;
      f1 << s1.str() << std::endl;
      f2 << s2.str() << std::endl;
      f0.close();
      f1.close();
      f2.close();
      CPPUNIT_ASSERT(s0.str() == s1.str());
      CPPUNIT_ASSERT(s1.str() == s2.str());


      std::string s;
      getline(s0, s);
      size_t len(s.size());

      while (getline(s0, s))
        {
          CPPUNIT_ASSERT(len == s.size());
        }
    }

    void test_log_stream3()
    {
      //
      //
      //
      coil::LogStreamBuffer logbuf;

      LogOut log(&logbuf);
      std::filebuf* of = new std::filebuf();
      of->open("test_log_stream3-1.log", std::ios::out | std::ios::app);
      logbuf.addStream(of);

      std::filebuf f;
      f.open("test_log_stream3-2.log", std::ios::out | std::ios::app);
      logbuf.addStream(&f);


      log.setLevel(INFO_LEVEL);
      log.level(SILENT_LEVEL) << coil::sprintf("This is silent message.") << std::endl;
      log.level(INFO_LEVEL) << coil::sprintf("This is info message.") << std::endl;
      log.level(ERROR_LEVEL) << coil::sprintf("This is error message.") << std::endl;
      log.level(PARANOID_LEVEL) << coil::sprintf("This is paranoid message.") << std::endl;

      CPPUNIT_ASSERT_EQUAL(true, logbuf.removeStream(&f));
      std::vector< ::coil::LogStream::streambuf_type* > vt;
      vt =  logbuf.getBuffers();
      CPPUNIT_ASSERT_EQUAL(1, (int)vt.size());
      for (int i(0), len(vt.size()); i < len; ++i)
        {
          try
          {
            CPPUNIT_ASSERT(of==vt[i]);
            delete vt[i];
           }
           catch(...)
           {
               CPPUNIT_FAIL( "Exception thrown." );
           }
          
        }

    }
    void test_log_stream_properties()
    {
      std::map<std::string, std::string> defaults;
      defaults["rtc.component.conf.path"] = "C:\\Program\\ Files\\OpenRTM-aist";
      defaults["rtc.manager.arch"] = "i386";
      defaults["rtc.manager.nameserver"] = "zonu.a02.aist.go.jp";
      defaults["rtc.manager.opening_message"] = "Hello World";

      coil::Properties prop(defaults);

      coil::LogStreamBuffer logbuf;
      std::stringstream s0;
      logbuf.addStream(s0.rdbuf());
//      logbuf.addStream(std::cout.rdbuf());

      LogOut2 log(&logbuf);
      log.setLevel(PARANOID_LEVEL);

      log.level(PARANOID_LEVEL) << prop; 

      std::ostringstream os,osm;
      os << prop;
      osm<<"---s0.str---"<<std::endl;
      osm<<s0.str()<<std::endl;
      osm<<"---os.str---"<<std::endl;
      osm<<os.str()<<std::endl;
      CPPUNIT_ASSERT_MESSAGE(osm.str(),s0.str() == os.str());

    }

  };
}; // namespace Logger

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Logger::LoggerTests);

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
#endif // Logger_cpp
