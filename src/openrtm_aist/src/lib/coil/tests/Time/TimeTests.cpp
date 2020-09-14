// -*- C++ -*-
/*!
 * @file   TimeTests.cpp
 * @brief  Time test class
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

#ifndef Time_cpp
#define Time_cpp

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <stdio.h>
#include <time.h>
#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/Time.h>
#include <coil/Task.h>
#include <coil/TimeValue.h>


/*!
 * @class TimeTests class
 * @brief Time test
 */
namespace Time
{
  class TimeTests
   : public CppUnit::TestFixture ,
     public coil::Task
  {
    CPPUNIT_TEST_SUITE(TimeTests);
    CPPUNIT_TEST(test_sleep);
//    CPPUNIT_TEST(test_sleep1);
//    CPPUNIT_TEST(test_sleep2);
//    CPPUNIT_TEST(test_sleep3);
    CPPUNIT_TEST(test_usleep);
//    CPPUNIT_TEST(test_usleep1);
    CPPUNIT_TEST(test_gettimeofday);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  public:
    class TestTimeTask
       : public coil::Task 
    {
    public:
      short m_count_time;
      short m_svccmd;
      short m_mainstop;
      short m_svcstop;
      int svc(void)
      {
        coil::TimeValue tv(0,1000000);
        unsigned int usec;
        unsigned int utime;
        short ic;
        switch(m_svccmd)
        {
          case 0:
            utime = 1;                   //1sec          
		    for (ic=0; ic<10; ic++)
            {
		  	  coil::sleep(utime);        //unsigned int sleep(unsigned int seconds)
              m_count_time++;
//              std::cout<<m_count_time<<std::endl;
              if (m_svcstop==1)
              {
                m_svcstop = 0;
                break;
              }
            }
            m_mainstop = 1;
            break;
          case 1:
//            tv = 1.0;                         //1s
		    for (ic=0; ic<10; ic++)
            {
			  coil::sleep(tv);                //int sleep(TimeValue& interval)
			  m_count_time++;
//              std::cout<<m_count_time<<std::endl;
              if (m_svcstop==1)
              {
                m_svcstop = 0;
                break;
              }
            }
            m_mainstop = 1;
            break;
          case 2:
            usec = 1000000;                      //1s
		    for (ic=0; ic<10; ic++)
            {
			  coil::usleep(usec);                //int usleep(useconds_t usec)
              m_count_time++;
//              std::cout<<m_count_time<<std::endl;
              if (m_svcstop==1)
              {
                m_svcstop = 0;
                break;
              }
            }
            m_mainstop = 1;
            break;
          case 3:
            tv = 0.1;                         //100ms
		    for (ic=0; ic<100; ic++)
            {
			  coil::sleep(tv);                //int sleep(TimeValue& interval)
			  m_count_time++;
//              std::cout<<m_count_time<<std::endl;
              if (m_svcstop==1)
              {
                m_svcstop = 0;
                break;
              }
            }
            m_mainstop = 1;
            break;
          default:
           break;
        }
        return 0;
      }
    };  
    /*!
     * @brief Constructor
     */
    TimeTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~TimeTests()
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
    int svc(void)
    {
      return 0;
    }
    virtual void tearDown()
    { 
    }
  
    /* test case */
    /*
    ---------------------------------------------------------------------------
    This function tests coil::sleep. 
     1.unsigned int sleep(unsigned int seconds)
     2.int sleep(TimeValue& interval)
    Check that it measures for ten seconds by the time function and coil::sleep(1) 
    ends ten times between those. 
    ---------------------------------------------------------------------------
    */
    void test_sleep()
    {

      time_t tmstart, tmend;
      short ic, icnt;
      char cstr[256];
      TimeTests::TestTimeTask *tta;

      tta = new TimeTests::TestTimeTask;

      tta->m_mainstop = 0;
      tta->m_svcstop = 0;
      tta->m_svccmd = 0;
      tta->m_count_time = 0;
      tta->activate();

      for(ic=0; ic<10; ic++)
      {
        ::time(&tmstart);
        for(;;)
        {
          ::time(&tmend);
		  if(::difftime(tmend,tmstart)>=1.0)
		  {
            break;
		  }
	    }
        if(tta->m_mainstop == 1)
        {
          break;
        }
      }
      tta->m_svcstop = 1;
      tta->wait();
      icnt = tta->m_count_time;
      delete tta;
      sprintf(cstr,"main:%d svc:%d", ic, icnt);
      //カウンタをチェック
      CPPUNIT_ASSERT_MESSAGE( cstr, (ic == icnt) );

      tta = new TimeTests::TestTimeTask;
      tta->m_svcstop = 0;
      tta->m_mainstop = 0;
      tta->m_svccmd = 1;
      tta->m_count_time = 0;
      tta->activate();
      for(ic=0; ic<10; ic++)
      {
        ::time(&tmstart);
        for(;;)
        {
          ::time(&tmend);
		  if(::difftime(tmend,tmstart)>=1.0)
		  {
            break;
		  }
	    }
        if(tta->m_mainstop == 1)
        {
          break;
        }
      }
      tta->m_svcstop = 1;
      tta->wait();
      icnt = tta->m_count_time;
      delete tta;
      sprintf(cstr,"main:%d svc:%d", ic, icnt);
      //カウンタをチェック
      CPPUNIT_ASSERT_MESSAGE( cstr, (ic == icnt) );
    }
    /*
    ---------------------------------------------------------------------------
    This function was made for the debugging of win32. 
    Time is measured by using the clock function. 
    ---------------------------------------------------------------------------
    */
    void test_sleep1()
    {
	  clock_t start, end;
      coil::TimeValue tv;
      TimeTests::TestTimeTask *tta;
      double dbcnt;
      char cstr[256];

      tta = new TimeTests::TestTimeTask;

      tta->m_mainstop = 0;
      tta->m_svcstop = 0;
      tta->m_svccmd = 0;
      tta->m_count_time = 0;
      tta->activate();
      start = ::clock();
      tta->wait();
      end = ::clock();
      delete tta;
      dbcnt = double(end - start) / CLOCKS_PER_SEC;
      sprintf(cstr,"time:%f", dbcnt);
      std::cout<<cstr<<std::endl;
      CPPUNIT_ASSERT_MESSAGE( cstr, (dbcnt>9.7)&(dbcnt<10.3) );

      tta = new TimeTests::TestTimeTask;

      tta->m_svcstop = 0;
      tta->m_mainstop = 0;
      tta->m_svccmd = 1;
      tta->m_count_time = 0;
      tta->activate();
      start = ::clock();
      tta->wait();
      end = ::clock();
      delete tta;
      dbcnt = double(end - start) / CLOCKS_PER_SEC;
      sprintf(cstr,"time:%f", dbcnt);
      std::cout<<cstr<<std::endl;
      CPPUNIT_ASSERT_MESSAGE( cstr, (dbcnt>9.7)&(dbcnt<10.3) );
    }
    /*
    ---------------------------------------------------------------------------
    This function was made for the debugging of win32. 
    Time is measured by using the clock function. 
    ---------------------------------------------------------------------------
    */
    void test_sleep2()
    {
	  clock_t start, end;
      coil::TimeValue tv;
      TimeTests::TestTimeTask *tta;
      double dbcnt;
      char cstr[256];

      tta = new TimeTests::TestTimeTask;

      tta->m_mainstop = 0;
      tta->m_svcstop = 0;
      tta->m_svccmd = 3;
      tta->m_count_time = 0;
      tta->activate();
      start = ::clock();
      tta->wait();
      end = ::clock();
      delete tta;
      dbcnt = double(end - start) / CLOCKS_PER_SEC;
      sprintf(cstr,"time:%f", dbcnt);
      std::cout<<cstr<<std::endl;
      CPPUNIT_ASSERT_MESSAGE( cstr, (dbcnt>9.7)&(dbcnt<10.3) );

    }
    /*
    ---------------------------------------------------------------------------
    Measurement at time of sleep(TimeValue& interval)
    ---------------------------------------------------------------------------
    */
    void test_sleep3()
    {
      
	  clock_t start, end;
      coil::TimeValue tv;
      double dbcnt;
      double dbcntmax,dbcntmin,dbcntave;
      short ic,icc;

      //0.0
      tv = 0;
      for(icc=0;icc<=10;icc++)
      {
        dbcntave = 0.0;
        dbcntmax = 0.0;
        dbcntmin = 10.0;
        for(ic=0;ic<100;ic++)
        {
          start = ::clock();
          coil::sleep(tv);
          end = ::clock();
          dbcnt = double(end - start) / CLOCKS_PER_SEC;
          dbcntave = dbcntave + dbcnt;
          if(dbcnt>dbcntmax)
          {
            dbcntmax = dbcnt;
          }
          if(dbcnt<dbcntmin)
          {
            dbcntmin = dbcnt;
          }
        }
        std::cout<<"sleep :"<<tv<<std::endl;
        std::cout<<"sleep time ave :"<<dbcntave/100<<std::endl;
        std::cout<<"sleep time max :"<<dbcntmax<<std::endl;
        std::cout<<"sleep time min :"<<dbcntmin<<std::endl;
        tv = tv + 0.01;
      }
    }
    /*
    ---------------------------------------------------------------------------
    This function tests coil::usleep. 
    Check that it measures for ten seconds by the time function and coil::usleep(1000000)
    ends ten times between those. 
    ---------------------------------------------------------------------------
    */
    void test_usleep()
    {

      time_t tmstart, tmend;
      coil::TimeValue tv;
      short ic,icnt;
      char cstr[256];
      TimeTests::TestTimeTask *tta;

      tta = new TimeTests::TestTimeTask;
      
      tta->m_svccmd = 2;
      tta->m_count_time = 0;
      tta->m_svcstop = 0;
      tta->m_mainstop = 0;
      tta->activate();
      for(ic=0; ic<10; ic++)
      {
        ::time(&tmstart);
        for(;;)
        {
          ::time(&tmend);
		  if(::difftime(tmend,tmstart)>=1.0)
		  {
            break;
		  }
	    }
        if(tta->m_mainstop == 1)
        {
          break;
        }
      }
      tta->m_svcstop = 1;
      tta->wait();
      icnt = tta->m_count_time;
      delete tta;
      sprintf(cstr,"main:%d svc:%d", ic, icnt);
      //カウンタをチェック
      CPPUNIT_ASSERT_MESSAGE( cstr, (ic == icnt) );
    }
    /*
    ---------------------------------------------------------------------------
    This function was made for the debugging of win32. 
    Time is measured by using the clock function. 
    ---------------------------------------------------------------------------
    */
    void test_usleep1()
    {

	  clock_t start, end;
      double dbcnt;
      char cstr[256];

      TimeTests::TestTimeTask *tta;

      tta = new TimeTests::TestTimeTask;
      
      tta->m_svccmd = 2;
      tta->m_count_time = 0;
      tta->m_svcstop = 0;
      tta->m_mainstop = 0;
      tta->activate();
      start = ::clock();
      tta->wait();
      end = ::clock();
      delete tta;
      dbcnt = double(end - start) / CLOCKS_PER_SEC;
      sprintf(cstr,"time:%f", dbcnt);
      std::cout<<cstr<<std::endl;
      CPPUNIT_ASSERT_MESSAGE( cstr, (dbcnt>9.7)&(dbcnt<10.3) );
    }
    /*
    ---------------------------------------------------------------------------
    This function tests coil::gettimeofday().
    ---------------------------------------------------------------------------
    */
    void test_gettimeofday()
    {
      long l1,l2;
	  time_t tmstart, tmend;
	  clock_t start, end;
      coil::TimeValue tv;
      char cstr[256];
      //check sec

      //get time
      tv = coil::gettimeofday();
      l1 = tv.sec();
      // 10sec wait
	  ::time(&tmstart);
      for(;;)
      {
        ::time(&tmend);
		if(::difftime(tmend,tmstart)>=10)
		{
          break;
		}
	  }
      //get time
      tv = coil::gettimeofday();
      l2 = tv.sec();
      //check
	  sprintf(cstr ,"sec%ld ",l2-l1);
      std::cout<<cstr<<std::endl;
      CPPUNIT_ASSERT_MESSAGE( cstr, (l2- l1)==10 );

	  //check usec
      //get time
	  tv = coil::gettimeofday();
      l1 = tv.usec();
      //100msec wait
      start = ::clock();
      for(;;)
      {
        end = ::clock();
        if((double)(end-start)/CLOCKS_PER_SEC>0.1)
	    {
          break;
        }
      }
      //get time
      tv = coil::gettimeofday();
      l2 = tv.usec();
      //check usec
	  sprintf(cstr ,"usec%ld ",l2-l1);
      std::cout<<cstr<<std::endl;
      CPPUNIT_ASSERT_MESSAGE( cstr, ((l2 - l1)>=80000) & ((l2 - l1)<=120000) );

    }
  };
}; // namespace Time

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Time::TimeTests);

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
#if 0
    CppUnit::TextUi::TestRunner runner;
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry().makeTest());
    CppUnit::Outputter* outputter = 
      new CppUnit::TextOutputter(&runner.result(), std::cout);
    runner.setOutputter(outputter);
    bool retcode = runner.run();
    return !retcode;
#endif
}
#endif // MAIN
#endif // Time_cpp
