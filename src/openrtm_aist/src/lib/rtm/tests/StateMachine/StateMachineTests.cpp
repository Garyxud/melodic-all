// -*- C++ -*-
/*!
 * @file   StateMachineTests.cpp
 * @brief  StateMachine test class
 * @date   $Date: 2008/04/08 02:21:39 $
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
 * $Log: StateMachineTests.cpp,v $
 * Revision 1.3  2008/04/08 02:21:39  arafune
 * Refactored test cases.
 *
 * Revision 1.2  2008/04/04 14:29:12  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2007/12/20 07:50:18  arafune
 * *** empty log message ***
 *
 * Revision 1.2  2007/01/12 14:56:19  n-ando
 * The getState() function is now getState().
 *
 * Revision 1.1  2006/11/27 08:26:03  n-ando
 * TestSuites are devided into each directory.
 *
 * Revision 1.2  2006/11/02 12:27:09  kurihara
 *
 * StateMachineTest is modified by kurihara.
 *
 * Revision 1.1  2006/10/26 08:56:56  n-ando
 * The first commitment.
 */

#ifndef StateMachine_cpp
#define StateMachine_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/TimeValue.h>
#include <rtm/StateMachine.h>

/*!
 * @class StateMachineTests class
 * @brief StateMachine test
 */
namespace StateMachineTests
{
  class StateMachineContext1
  {
  public:
    static const int STATE_1 = 0;
    static const int STATE_2 = 1;
    static const int STATE_3 = 2;
    static const int STATE_4 = 3;
    static const int STATE_5 = 4;
    static const int STATE_6 = 5;
    static const int SIZEOF_STATE = 6;
		
    static const int ENTRY  = 0;
    static const int PREDO  = 1;
    static const int DO     = 2;
    static const int POSTDO = 3;
    static const int EXIT   = 4;
    static const int SIZEOF_ACTION = 5;
		
    StateMachineContext1() : m_fsm(SIZEOF_STATE)
    {
      m_fsm.setListener(this);

      m_fsm.setEntryAction(STATE_1, &StateMachineContext1::onEntry_STATE_1);
      m_fsm.setPreDoAction(STATE_1, &StateMachineContext1::onPreDo_STATE_1);
      m_fsm.setDoAction(STATE_1, &StateMachineContext1::onDo_STATE_1);
      m_fsm.setPostDoAction(STATE_1, &StateMachineContext1::onPostDo_STATE_1);
      m_fsm.setExitAction(STATE_1, &StateMachineContext1::onExit_STATE_1);

      m_fsm.setEntryAction(STATE_2, &StateMachineContext1::onEntry_STATE_2);
      m_fsm.setPreDoAction(STATE_2, &StateMachineContext1::onPreDo_STATE_2);
      m_fsm.setDoAction(STATE_2, &StateMachineContext1::onDo_STATE_2);
      m_fsm.setPostDoAction(STATE_2, &StateMachineContext1::onPostDo_STATE_2);
      m_fsm.setExitAction(STATE_2, &StateMachineContext1::onExit_STATE_2);

      m_fsm.setEntryAction(STATE_3, &StateMachineContext1::onEntry_STATE_3);
      m_fsm.setPreDoAction(STATE_3, &StateMachineContext1::onPreDo_STATE_3);
      m_fsm.setDoAction(STATE_3, &StateMachineContext1::onDo_STATE_3);
      m_fsm.setPostDoAction(STATE_3, &StateMachineContext1::onPostDo_STATE_3);
      m_fsm.setExitAction(STATE_3, &StateMachineContext1::onExit_STATE_3);

      m_fsm.setEntryAction(STATE_4, &StateMachineContext1::onEntry_STATE_4);
      m_fsm.setPreDoAction(STATE_4, &StateMachineContext1::onPreDo_STATE_4);
      m_fsm.setDoAction(STATE_4, &StateMachineContext1::onDo_STATE_4);
      m_fsm.setPostDoAction(STATE_4, &StateMachineContext1::onPostDo_STATE_4);
      m_fsm.setExitAction(STATE_4, &StateMachineContext1::onExit_STATE_4);

      m_fsm.setEntryAction(STATE_5, &StateMachineContext1::onEntry_STATE_5);
      m_fsm.setPreDoAction(STATE_5, &StateMachineContext1::onPreDo_STATE_5);
      m_fsm.setDoAction(STATE_5, &StateMachineContext1::onDo_STATE_5);
      m_fsm.setPostDoAction(STATE_5, &StateMachineContext1::onPostDo_STATE_5);
      m_fsm.setExitAction(STATE_5, &StateMachineContext1::onExit_STATE_5);

      m_fsm.setEntryAction(STATE_6, &StateMachineContext1::onEntry_STATE_6);
      m_fsm.setPreDoAction(STATE_6, &StateMachineContext1::onPreDo_STATE_6);
      m_fsm.setDoAction(STATE_6, &StateMachineContext1::onDo_STATE_6);
      m_fsm.setPostDoAction(STATE_6, &StateMachineContext1::onPostDo_STATE_6);
      m_fsm.setExitAction(STATE_6, &StateMachineContext1::onExit_STATE_6);
			
      RTC_Utils::StateHolder<int> initialStates;
      initialStates.prev = STATE_1;
      initialStates.curr = STATE_1;
      initialStates.next = STATE_1;
      m_fsm.setStartState(initialStates);
    }
		
    void work()
    {
      for (int i = 0; i < 50; ++i)
	{
	  m_fsm.worker();
	}
    }
		
    void onEntry_STATE_1(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_1, ENTRY);
    }
    void onPreDo_STATE_1(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_1, PREDO);
      m_fsm.goTo(STATE_2);
    }
    void onDo_STATE_1(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_1, DO);
    }
    void onPostDo_STATE_1(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_1, POSTDO);
    }
    void onExit_STATE_1(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_1, EXIT);
    }

    void onEntry_STATE_2(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_2, ENTRY);
    }
    void onPreDo_STATE_2(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_2, PREDO);
    }
		
    void onDo_STATE_2(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_2, DO);
      m_fsm.goTo(STATE_3);
    }
    void onPostDo_STATE_2(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_2, POSTDO);
    }
    void onExit_STATE_2(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_2, EXIT);
    }

    void onEntry_STATE_3(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_3, ENTRY);
    }
    void onPreDo_STATE_3(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_3, PREDO);
    }
    void onDo_STATE_3(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_3, DO);
    }
    void onPostDo_STATE_3(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_3, POSTDO);
      m_fsm.goTo(STATE_4);
    }
    void onExit_STATE_3(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_3, EXIT);
    }

    void onEntry_STATE_4(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_4, ENTRY);
      m_fsm.goTo(STATE_5);
    }
    void onPreDo_STATE_4(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_4, PREDO);
    }
    void onDo_STATE_4(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_4, DO);
    }
    void onPostDo_STATE_4(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_4, POSTDO);
    }
    void onExit_STATE_4(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_4, EXIT);
    }

    void onEntry_STATE_5(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_5, ENTRY);
    }
    void onPreDo_STATE_5(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_5, PREDO);
    }
    void onDo_STATE_5(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_5, DO);
    }
    void onPostDo_STATE_5(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_5, POSTDO);
    }
    void onExit_STATE_5(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_5, EXIT);
      m_fsm.goTo(STATE_6);
    }

    void onEntry_STATE_6(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_6, ENTRY);
    }
    void onPreDo_STATE_6(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_6, PREDO);
    }
    void onDo_STATE_6(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_6, DO);
    }
    void onPostDo_STATE_6(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_6, POSTDO);
    }
    void onExit_STATE_6(const RTC_Utils::StateHolder<int>& states)
    {
      logCallback(STATE_6, EXIT);
    }
	
  public:
    struct StateAndAction
    {
      int state;
      int action;
    };
		
    const std::vector<StateAndAction>& getCallbackLog() const
    {
      return m_callbackLog;
    }
		
  private:
    RTC_Utils::StateMachine<int, StateMachineContext1> m_fsm;
    std::vector<StateAndAction> m_callbackLog;
	
  private:
    void logCallback(int state, int action)
    {
      StateAndAction sa;
      sa.state = state;
      sa.action = action;
      m_callbackLog.push_back(sa);
    }
  };
	
  class StateMachineTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(StateMachineTests);
    CPPUNIT_TEST(test_transition_story1);
    CPPUNIT_TEST_SUITE_END();
		
  private:
	
  public:
    /*!
     * @brief Constructor
     */
    StateMachineTests()
    {
    }
		
    /*!
     * @brief Destructor
     */
    ~StateMachineTests()
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
     * @brief StateMachineによる状態遷移のテスト
     * 
     * 次の場合に、意図どおりの順序でコールバックが呼び出されることを確認する；
     * - PreDo内で次状態を指定した場合
     * - Do内で次状態を指定した場合
     * - PostDo内で次状態を指定した場合
     * - Entry内で次状態を指定した場合
     * 
     * また、Entry/PreDo/Do/PostDoのいずれにおいても次状態を遷移しない場合に、現状態に留まることを確認する。
     */
    void test_transition_story1()
    {
      typedef StateMachineContext1 SMC;
			
      SMC::StateAndAction expected[] =
	{
	  { SMC::STATE_1, SMC::PREDO },
	  { SMC::STATE_1, SMC::EXIT },
	  { SMC::STATE_2, SMC::ENTRY },
	  { SMC::STATE_2, SMC::PREDO },
	  { SMC::STATE_2, SMC::DO },
	  { SMC::STATE_2, SMC::EXIT },
	  { SMC::STATE_3, SMC::ENTRY },
	  { SMC::STATE_3, SMC::PREDO },
	  { SMC::STATE_3, SMC::DO },
	  { SMC::STATE_3, SMC::POSTDO },
	  { SMC::STATE_3, SMC::EXIT },
	  { SMC::STATE_4, SMC::ENTRY },
	  { SMC::STATE_4, SMC::EXIT },
	  { SMC::STATE_5, SMC::ENTRY },
	  { SMC::STATE_5, SMC::PREDO },
	  { SMC::STATE_5, SMC::DO },
	  { SMC::STATE_5, SMC::POSTDO },
	  { SMC::STATE_5, SMC::PREDO }
	};
			
      SMC context;
      context.work();
			
      const std::vector<SMC::StateAndAction> log = context.getCallbackLog();
      for (int i = 0; i < sizeof(expected) / sizeof(SMC::StateAndAction); ++i)
	{
	  CPPUNIT_ASSERT_EQUAL(expected[i].state, log[i].state);
	  CPPUNIT_ASSERT_EQUAL(expected[i].action, log[i].action);
	}
    }
		
  };
}; // namespace StateMachine

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(StateMachineTests::StateMachineTests);

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
#endif // StateMachine_cpp
