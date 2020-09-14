// -*- C++ -*-
/*!
 * @file   ConditionTests.cpp
 * @brief  Condition test class
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

#ifndef Condition_cpp
#define Condition_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/Condition.h>
#include <coil/Mutex.h>
#include <coil/Guard.h>
#include <coil/Task.h>
#include <coil/Async.h>
#include <coil/Time.h>
#include <coil/TimeValue.h>

typedef coil::Guard<coil::Mutex> Guard;

class A
{
public:
  A(coil::Condition<coil::Mutex>& cond, coil::Mutex& mutex)
    : m_cond(cond), m_mutex(mutex) {}
  void signal(int usec)
  {
    coil::usleep(usec);
    m_mutex.lock();
    m_cond.signal();
    m_mutex.unlock();
  }
  coil::Condition<coil::Mutex>& m_cond;
  coil::Mutex& m_mutex;
};

/*!
 * @class ConditionTests class
 * @brief Condition test
 */
namespace Condition
{

/*!
 *  @brief 試験用タスク。起動するとwaitして、起こされると生成時に渡された値をConditionStatusに加算する。
 */
  class ConditionCheckTask : public coil::Task
  {
  public:
    ConditionCheckTask() : id(0) { }
    ConditionCheckTask(coil::Mutex & aMutex,
                       coil::Condition<coil::Mutex> & aCondition, int anId)
      : mutex(&aMutex), cond(&aCondition), id(anId)
    {
      //        std::cout << "Task(" << id << ") created." << std::endl;
    }

    virtual ~ConditionCheckTask()
    {
    };

    virtual int svc()
    {
      Guard guard(*mutex);
      //      mutex->lock();
      cond->wait();
      ConditionStatus += id;
      //      mutex->unlock();
      return ConditionStatus;
    }
    /*!
     *  @brief 結果情報のConditionStatusをクリアする
     */
    static void resteStatus() { ConditionStatus =  0; }
    /*!
     *  @brief 結果情報のConditionStatusの現在値を返す
     */
    static int getStatus() { return ConditionStatus; }
  private:
//    static int status;  // <- undefined reference
    coil::Mutex * mutex;
    coil::Condition<coil::Mutex> * cond;
    int id;
    static int ConditionStatus;
  };  // class ConditionCheckTask

  int ConditionCheckTask::ConditionStatus(0);

  
  class ConditionTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ConditionTests);
    CPPUNIT_TEST(test_Condition_wait_and_signal);
    CPPUNIT_TEST(test_Condition_wait_and_broadcast);
    CPPUNIT_TEST(test_Condition_wait_with_time);
    CPPUNIT_TEST_SUITE_END();
  
  private:
    coil::Mutex * mu;

  public:
    /*!
     * @brief Constructor
     */
    ConditionTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~ConditionTests()
    {
    }
 
		
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
      mu = new coil::Mutex;
    }
    
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    { 
      delete mu;
    }
  
    /* test case */

    /*!
     *  @brief wati with time.
     */
    void test_Condition_wait_and_signal()
    {
      coil::Mutex mu1;
      coil::Condition<coil::Mutex> cond1(mu1);
      coil::Mutex mu2;
      coil::Condition<coil::Mutex> cond2(mu2);
      int id1(0x02);
      int id2(0x08);

      ConditionCheckTask::resteStatus();
      ConditionCheckTask cct1(mu1, cond1, id1);
      ConditionCheckTask cct2(mu1, cond1, id2);

      cct1.activate();
      cct2.activate();

      coil::usleep(10000); // give cpu time to tasks

      CPPUNIT_ASSERT_EQUAL(0x00, ConditionCheckTask::getStatus());

      {
        Guard guard(mu1);
        //      mu1.lock();
        cond1.signal();
        //        mu1.unlock();
      }

      coil::usleep(10000); // give cpu time to tasks

      CPPUNIT_ASSERT_EQUAL(id1, ConditionCheckTask::getStatus());

      {
        Guard guard(mu1);
        //        mu1.lock();
        cond1.signal();
        //      mu1.unlock();
      }

      coil::usleep(10000); // give cpu time to tasks

      CPPUNIT_ASSERT_EQUAL(id1 + id2, ConditionCheckTask::getStatus());

      //      cct1.wait();
      //      cct2.wait();
    }

    /*!
     *  @brief wati with time.
     */
    void test_Condition_wait_and_broadcast()
    {
      coil::Condition<coil::Mutex> cd(*mu);
      coil::Mutex mu2;
      coil::Condition<coil::Mutex> cond2(mu2);

      ConditionCheckTask::resteStatus();
      ConditionCheckTask cct6(*mu, cd, 0x20);
      ConditionCheckTask cct5(*mu, cd, 0x10);
      ConditionCheckTask cct4(*mu, cd, 0x08);
      ConditionCheckTask cct3(*mu, cd, 0x04);
      ConditionCheckTask cct2(*mu, cd, 0x02);
      ConditionCheckTask cct1(*mu, cd, 0x01);

      cct6.activate();
      cct5.activate();
      cct4.activate();
      cct3.activate();
      cct2.activate();
      cct1.activate();

      CPPUNIT_ASSERT_EQUAL(0, ConditionCheckTask::getStatus());

      {
        Guard guard(mu2);
        // mu2.lock();
        cond2.wait(1);
        // mu2.unlock();
      }

      {
        Guard guard(*mu);
        // mu->lock();
        cd.broadcast();
        // mu->unlock();
      }

      {
        Guard guard(mu2);
        // mu2.lock();
        cond2.wait(1);
        // mu2.unlock();
      }

      CPPUNIT_ASSERT_EQUAL(0x3f, ConditionCheckTask::getStatus());

      CPPUNIT_ASSERT(true);
    }
  
    /*!
     *  @brief wati with time.
     */
    void test_Condition_wait_with_time()
    {
      int waitSec(2);
      coil::Condition<coil::Mutex> cd(*mu);
      //      std::cout << "Before wait " << waitSec << " sec." << std::endl << std::flush;
      bool result;
      {
        Guard guard(*mu);
        // mu->lock();
        result = cd.wait(waitSec);
        // mu->unlock();
      }

      // result = false (timeout)
      //      if (result) { std::cout << "signaled..ERROR" << std::endl; }
      //      else        { std::cout << "timeout...OK" << std::endl;  }
      CPPUNIT_ASSERT(!result);


      A a(cd, *mu);
      coil::Async*
        invoker(coil::AsyncInvoker(&a,
                                   std::bind2nd(std::mem_fun(&A::signal),
                                                1000000)));
      invoker->invoke();
      {
        Guard guard(*mu);
        // mu->lock();
        result = cd.wait(waitSec);
        // mu->unlock();
      }
      invoker->wait();

      // result = true (signal)
      //      if (result) { std::cout << "signaled..OK" << std::endl; }
      //      else        { std::cout << "timeout...ERROR" << std::endl;  }
      CPPUNIT_ASSERT(result);
    }

  };
}; // namespace Condition

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Condition::ConditionTests);

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
#endif // Condition_cpp
