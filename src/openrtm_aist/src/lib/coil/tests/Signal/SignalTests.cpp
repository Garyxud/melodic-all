// -*- C++ -*-
/*!
 * @file   SignalTests.cpp
 * @brief  Signal test class
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

#ifndef Signal_cpp
#define Signal_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/Signal.h>

/*!
 * @class SignalTests class
 * @brief Signal test
 */
namespace Signal
{
static int Signum;

  class SignalTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(SignalTests);
    CPPUNIT_TEST(test_SignalAction_constructor_1);
    CPPUNIT_TEST(test_SignalAction_constructor_2);
    CPPUNIT_TEST(test_SignalAction_test);
    CPPUNIT_TEST_SUITE_END();
  
  private:

  public:
  
    /*!
     * @brief Constructor
     */
    SignalTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~SignalTests()
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
    /*!
     *  @brief Constractor with no paramater.
     */
    void test_SignalAction_constructor_1()
    {
      coil::SignalAction * sa = new coil::SignalAction();
      delete sa;
    }

    /*!
     *  @brief Constractor with 2 paramater.
     */
    void test_SignalAction_constructor_2()
    {
      coil::SignalHandler handle(signalHandler);
      int signum(SIGINT);
      coil::SignalAction * sa = new coil::SignalAction(handle, signum);

      delete sa;
    }

    /*!
     *  @brief Constractor with 3 paramater.
     *  @note  Do not use. 20081003 Kojima.
     */
//    void test_SignalAction_constructor_3()
//    {
//      coil::SignalHandler handle(0);
//      int signum(0);
//      sigset_t *mask(0);
//      coil::SignalAction * sa = new coil::SignalAction(handle, signum, mask);

//      delete sa;
//    }

    /*!
     *  @brief Constractor with no paramater.
     */
    void test_SignalAction_test()
    {
      checkSignalAction();
    }

    /*!
     *  @brief Set SignalAction for Linux.
     */
    void checkSignalAction()
    {
      coil::SignalAction * signalAction;
      coil::SignalHandler handle(signalHandler);
//      int sn[] = {
//        SIGINT, SIGQUIT, SIGILL, SIGABRT, SIGFPE, SIGSEGV, SIGPIPE, SIGALRM,
//        SIGTERM,SIGUSR1, SIGUSR2, SIGCHLD, SIGCONT, SIGTSTP, SIGTTIN, SIGTTOU,
//        0
//      };
      int sn[] = {    // posix, win common.
        SIGINT, SIGILL, SIGABRT, SIGFPE, SIGSEGV, 
        0
      };
      int index;
      
      for (index = 0; sn[index]; index++) {
        Signum = sn[index];
//        signalAction = new coil::SignalAction(handle, Signum, mask);
        signalAction = new coil::SignalAction(handle, Signum);
        raise(Signum);
        delete signalAction;
      }
    }

    /* Signal Handler */
    /*!
     *  @brief ÇnÇrÇ©ÇÁìnÇ≥ÇÍÇΩÉVÉOÉiÉãî‘çÜÇ∆ÅAó\íËÇ≥ÇÍÇƒÇ¢ÇΩÉVÉOÉiÉãî‘çÜÇî‰äråüèÿÇ∑ÇÈÅB
     */
    static void signalHandler(int aSignum)
    {
//   std::cout << "Signum : " << Signum << std::endl << std::flush ;
      CPPUNIT_ASSERT_EQUAL(Signum, aSignum);
    }
  };
}; // namespace Signal

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Signal::SignalTests);

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

#if 0
__sighandler_t signal(int __sig, __sighandler_t __handler)
{
  __sighandler_t st;
  std::cout << "Hello, Signal" << std::endl;
  return st;
}
#endif

#endif // Signal_cpp
