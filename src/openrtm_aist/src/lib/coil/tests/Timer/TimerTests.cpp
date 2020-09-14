// -*- C++ -*-
/*!
 * @file   TimerTests.cpp
 * @brief  Timer test class
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

#ifndef Timer_cpp
#define Timer_cpp

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

//#include <../../include/coil/Timer.h>
#include <coil/Timer.h>
#include <coil/TimeValue.h>

#define JUDGEMAX 12
#define JUDGEMIN 8
/*!
 * @class TimerTests class
 * @brief Timer test
 */
namespace Timer
{
  class TimerTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(TimerTests);

    CPPUNIT_TEST(test_registerListener);
    CPPUNIT_TEST(test_activate_multi_timers_continuously);
    CPPUNIT_TEST(test_activate_multi_timers_concurrently);
 

    CPPUNIT_TEST_SUITE_END();
  
  private:
    class Listener : public ListenerBase
    {
    public:
      Listener(const char* name = "", bool printMsg = false)
        : _name(name), _printMsg(printMsg), _count(0)
      {
      }

      virtual void invoke()
      {
        _count++;

        if (_printMsg) {
          std::cout
            << std::endl
            << _name << ":invoked. (count = " << _count << ")"
            << std::endl;
        }
      }

      const char* _name;
      bool _printMsg;
      int _count;
    };
   
  public:
  
    /*!
     * @brief Constructor
     */
    TimerTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~TimerTests()
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
    void test_case0()
    {
    }
    /*!
     * @brief registerListener()•·•Ω•√•…§Œ•∆•π•»
     * 
     * - •ø•§•ﬁ§ÚµØ∆∞§∑°¢§¢§È§´§∏§·≈–œø§µ§ÅEø•ÅEπ• §¨∞’øﬁ§…§™§Í§Œª˛¥÷¥÷≥÷§«•≥°º•ÅE–•√•Ø§µ§ÅEÅE´°©
     * 
     */
    void test_registerListener()
    {
      time_t tmstart, tmend;
      char cstr[256];
      coil::TimeValue timerInterval(0, 100000); // 0.1 [sec]
      coil::Timer timer(timerInterval);

      Listener listener;
      coil::TimeValue listenerInterval(0, 1000000);
      timer.registerListener(&listener, listenerInterval);

      timer.start();
//      sleep(10);
      time(&tmstart);
      for(;;)
      {
        time(&tmend);
		if(difftime(tmend,tmstart)>=10)
		{
          break;
		}
	  }
	  timer.stop();
      sprintf(cstr, "count:%d", listener._count );
      // £±…√§À£±≤Û§Œ∏∆Ω–§ §Œ§«°¢£±£∞≤Û•´•¶•Û•»§µ§ÅE∆§§§ÅEœ§∫°£¿∫≈Ÿ§ÚπÕŒ∏§∑§∆°¢£π°¡£±£±≤Û§Œ»œ∞œ§«§¢§ÅE≥§»§Ú≥Œ«ß§π§ÅE£      
      CPPUNIT_ASSERT_MESSAGE(cstr ,(JUDGEMIN <= listener._count) && (listener._count <= JUDGEMAX));
    }

    /*!
     * @brief  £øÙ§Œ•ø•§•ﬁ°º§Úª˛¥÷≈™§ÀƒæŒÛ§À∆∞∫˚¿µ§ª§ÅE∆•π•»
     * 
     * - £≤§ƒ§Œ•ø•§•ﬁ°º§Œ∆∞∫˚¿¨∏ﬂ§§§À¥≥æƒ§π§ÅE≥§»§ §Ø°¢§¢§È§´§∏§·≈–œø§µ§ÅEø•ÅEπ• §¨∞’øﬁ§…§™§Í§Œª˛¥÷¥÷≥÷§«•≥°º•ÅE–•√•Ø§µ§ÅEÅE´°©
     * 
     */
    void test_activate_multi_timers_continuously()
    {
      time_t tmstart, tmend;
      char cstr[256];
	  //The first timer is started.
	  coil::TimeValue timerInterval1(0, 100000); // 0.1 [sec]
      coil::Timer timer1(timerInterval1);

      Listener listener1("listener-1");
      coil::TimeValue listenerInterval1(0, 1000000);
      timer1.registerListener(&listener1, listenerInterval1);

      timer1.start();
//      sleep(10);
      time(&tmstart);
      for(;;)
      {
        time(&tmend);
		if(difftime(tmend,tmstart)>=10)
		{
          break;
		}
	  }
      timer1.stop();

      //The second timer is started. 
	  coil::TimeValue timerInterval2(0, 100000); // 0.1 [sec]
      coil::Timer timer2(timerInterval2);

      Listener listener2("listener-2");
      coil::TimeValue listenerInterval2(0, 1000000);
      timer2.registerListener(&listener2, listenerInterval2);

      timer2.start();
//      sleep(10);
      time(&tmstart);
      for(;;)
      {
        time(&tmend);
		if(difftime(tmend,tmstart)>=10)
		{
          break;
		}
	  }
      timer2.stop();

      // £±…√§À£±≤Û§Œ∏∆Ω–§ §Œ§«°¢£±£∞≤Û•´•¶•Û•»§µ§ÅE∆§§§ÅEœ§∫°£¿∫≈Ÿ§ÚπÕŒ∏§∑§∆°¢£π°¡£±£±≤Û§Œ»œ∞œ§«§¢§ÅE≥§»§Ú≥Œ«ß§π§ÅE£
      sprintf(cstr,"count:%d", listener1._count);
      CPPUNIT_ASSERT_MESSAGE(cstr, (JUDGEMIN <= listener1._count) && (listener1._count <= JUDGEMAX));
      sprintf(cstr,"count:%d", listener2._count);
      CPPUNIT_ASSERT_MESSAGE(cstr, (JUDGEMIN <= listener2._count) && (listener2._count <= JUDGEMAX));
    }

    /*!
     * @brief  £øÙ§Œ•ø•§•ﬁ°º§Úª˛¥÷≈™§À ¬ŒÛ§À∆∞∫˚¿µ§ª§ÅE∆•π•»
     * 
     * - £≤§ƒ§Œ•ø•§•ﬁ°º§Œ∆∞∫˚¿¨∏ﬂ§§§À¥≥æƒ§π§ÅE≥§»§ §Ø°¢§¢§È§´§∏§·≈–œø§µ§ÅEø•ÅEπ• §¨∞’øﬁ§…§™§Í§Œª˛¥÷¥÷≥÷§«•≥°º•ÅE–•√•Ø§µ§ÅEÅE´°©
     * 
     */
    void test_activate_multi_timers_concurrently()
    {
      time_t tmstart, tmend;
      char cstr[256];
	  //The first timer is started. 
	  coil::TimeValue timerInterval1(0, 100000); // 0.1 [sec]
      coil::Timer timer1(timerInterval1);

      Listener listener1("listener-1");
      coil::TimeValue listenerInterval1(0, 1000000);
      timer1.registerListener(&listener1, listenerInterval1);

      //The second timer is started. 
	  coil::TimeValue timerInterval2(0, 100000); // 0.1 [sec]
      coil::Timer timer2(timerInterval2);

      Listener listener2("listener-2");
      coil::TimeValue listenerInterval2(0, 1000000);
      timer2.registerListener(&listener2, listenerInterval2);

      //Two timers be started are stopped simultaneously. 
	  timer1.start();
      timer2.start();
//      sleep(10);
      time(&tmstart);
      for(;;)
      {
        time(&tmend);
		if(difftime(tmend,tmstart)>=10)
		{
          break;
		}
	  }
      timer1.stop();
      timer2.stop();

      // £±…√§À£±≤Û§Œ∏∆Ω–§ §Œ§«°¢£±£∞≤Û•´•¶•Û•»§µ§ÅE∆§§§ÅEœ§∫°£¿∫≈Ÿ§ÚπÕŒ∏§∑§∆°¢£π°¡£±£±≤Û§Œ»œ∞œ§«§¢§ÅE≥§»§Ú≥Œ«ß§π§ÅE£
      sprintf(cstr,"count:%d", listener1._count);
      CPPUNIT_ASSERT_MESSAGE(cstr, (JUDGEMIN <= listener1._count) && (listener1._count <= JUDGEMAX));
      sprintf(cstr,"count:%d", listener2._count);
      CPPUNIT_ASSERT_MESSAGE(cstr, (JUDGEMIN <= listener2._count) && (listener2._count <= JUDGEMAX));
    }

  };
}; // namespace Timer

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Timer::TimerTests);

#ifdef LOCAL_MAIN
int main(int argc, char* argv[])
{
#if 0
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
#endif
    CppUnit::TextUi::TestRunner runner;
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry().makeTest());
    CppUnit::Outputter* outputter = 
      new CppUnit::TextOutputter(&runner.result(), std::cout);
    runner.setOutputter(outputter);
    bool retcode = runner.run();
    return !retcode;
}
#endif // MAIN
#endif // Timer_cpp
